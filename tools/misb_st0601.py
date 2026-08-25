#!/usr/bin/env python3
"""Minimal MISB ST 0601 (UAS Datalink Local Set) KLV encoder.

Stdlib-only, dependency-free — this is what `gw_klv_muxer.py` calls per frame
to turn GodotWings' telemetry JSON into a KLV byte string for GStreamer's
`mpegtsmux` to interleave with the video (STANAG 4609 = H.264 + this, in one
MPEG-TS program).

Implements SMPTE 336M KLV encoding (16-byte universal key + BER length +
Tag/Length/Value elements) and the ST 0601 checksum (Tag 1). The per-field
value mapping is a SIMPLIFIED IMAPB: values are clamped and linearly mapped
across the field's full integer range. The real ST 0601 IMAPB reserves a few
code points at the extremes for "value unavailable" / out-of-range flags,
which this does not reproduce — fine for carrying real geometry to any KLV
reader, not a claim of bit-exact certification compliance.

Only the tags GodotWings can actually populate are implemented: platform
attitude, sensor lat/lon/alt, sensor FOV, sensor position relative to the
platform (pan/tilt or gimbal angles), and frame center + corner points (Tags
23-25, 82-89) — the latter from a flat-ground-plane ray intersection computed
in `gw_klv_muxer.py`, not a real terrain raycast (Godot has the actual terrain
but this external process doesn't); see that module's docstring. Add more from
the ST 0601 tag table as needed — `pack_local_set` doesn't care which subset
you pass.
"""

from __future__ import annotations

import struct

UAS_LOCAL_SET_KEY = bytes.fromhex("060e2b34020b01010e01030101000000")

# tag number -> (byte width, encode(value) -> int-in-range, signed?)
# `range` is the real ST 0601 span for that tag; values outside it are clamped.
_FIELDS: dict[str, tuple[int, int, float, float, bool]] = {
    # name:            tag,  bytes, min,    max,   signed
    "checksum":        (1,   2,     0,      0,     False),  # handled specially
    "timestamp":       (2,   8,     0,      0,     False),  # raw µs, not IMAPB
    "platform_heading": (5,  2,     0.0,    360.0, False),
    "platform_pitch":  (6,   2,    -20.0,   20.0,  True),
    "platform_roll":   (7,   2,   -50.0,    50.0,  True),
    "sensor_lat":      (13,  4,   -90.0,    90.0,  True),
    "sensor_lon":      (14,  4,  -180.0,   180.0,  True),
    "sensor_true_alt": (15,  2,  -900.0, 19000.0,  False),
    "sensor_hfov":     (16,  2,     0.0,   180.0,  False),
    "sensor_vfov":     (17,  2,     0.0,   180.0,  False),
    "sensor_rel_az":   (18,  4,     0.0,   360.0,  False),
    "sensor_rel_el":   (19,  4,  -180.0,   180.0,  True),
    "sensor_rel_roll": (20,  4,     0.0,   360.0,  False),
    "frame_center_lat": (23, 4,   -90.0,    90.0,  True),
    "frame_center_lon": (24, 4,  -180.0,   180.0,  True),
    "frame_center_elevation": (25, 2, -900.0, 19000.0, False),
    "corner_lat_1":    (82,  4,   -90.0,    90.0,  True),  # full-precision "offset corner" tags
    "corner_lon_1":    (83,  4,  -180.0,   180.0,  True),  # (82-89): absolute lat/lon, not the
    "corner_lat_2":    (84,  4,   -90.0,    90.0,  True),  # older narrow-range Tag 26-33 encoding
    "corner_lon_2":    (85,  4,  -180.0,   180.0,  True),  # that's relative to frame center.
    "corner_lat_3":    (86,  4,   -90.0,    90.0,  True),
    "corner_lon_3":    (87,  4,  -180.0,   180.0,  True),
    "corner_lat_4":    (88,  4,   -90.0,    90.0,  True),
    "corner_lon_4":    (89,  4,  -180.0,   180.0,  True),
    "uas_ls_version":  (65,  1,     0,     255,    False),
}


def ber_length(n: int) -> bytes:
    """BER definite-length encoding (short form if it fits in 7 bits)."""
    if n < 0x80:
        return bytes([n])
    octets = n.to_bytes((n.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(octets)]) + octets


def _imapb(value: float, vmin: float, vmax: float, nbytes: int, signed: bool) -> bytes:
    """Simplified IMAPB: clamp to [vmin, vmax], map linearly onto the full
    [n]-byte integer range (signed or unsigned)."""
    value = max(vmin, min(vmax, value))
    if signed:
        half = 1 << (nbytes * 8 - 1)
        lo, hi = -half, half - 1
    else:
        lo, hi = 0, (1 << (nbytes * 8)) - 1
    t = (value - vmin) / (vmax - vmin) if vmax > vmin else 0.0
    code = max(lo, min(hi, round(lo + t * (hi - lo))))
    return int(code).to_bytes(nbytes, "big", signed=signed)


def _encode_field(name: str, value: float) -> bytes:
    tag, nbytes, vmin, vmax, signed = _FIELDS[name]
    if name == "timestamp":
        return struct.pack(">Q", int(value) & 0xFFFFFFFFFFFFFFFF)  # unix microseconds, raw
    if name == "uas_ls_version":
        return bytes([int(value) & 0xFF])
    return _imapb(float(value), vmin, vmax, nbytes, signed)


def _checksum16(data: bytes) -> int:
    """ST 0601 Tag 1: 16-bit sum of the packet as big-endian 16-bit words,
    zero-padded if odd length."""
    total = 0
    for i in range(0, len(data), 2):
        hi = data[i]
        lo = data[i + 1] if i + 1 < len(data) else 0
        total = (total + ((hi << 8) | lo)) & 0xFFFF
    return total


def pack_local_set(fields: dict) -> bytes:
    """Build one complete KLV packet: 16-byte UAS LS key + BER length + the
    given fields as Tag/Length/Value elements + a trailing Tag 1 checksum.

    `fields` maps names from `_FIELDS` (anything but "checksum") to values;
    unknown names raise. Field order in the dict is preserved in the packet.
    """
    body = bytearray()
    for name, value in fields.items():
        if name == "checksum":
            raise ValueError("checksum is computed automatically, don't pass it")
        if name not in _FIELDS:
            raise ValueError(f"unknown MISB ST 0601 field {name!r}")
        tag = _FIELDS[name][0]
        val = _encode_field(name, value)
        body += bytes([tag]) + ber_length(len(val)) + val

    length_field = ber_length(len(body) + 4)  # +4: checksum's own tag(1)+len(1)+value(2)
    # Compute the checksum over everything including its own tag+length and a
    # zeroed placeholder for its value, then splice the real value in — the
    # placeholder contributes 0 to the running sum, so this is equivalent to
    # "sum everything but the checksum value" per the ST 0601 definition.
    packet = bytearray(UAS_LOCAL_SET_KEY + length_field + bytes(body) + bytes([1, 2, 0, 0]))
    checksum = _checksum16(bytes(packet))
    packet[-2:] = checksum.to_bytes(2, "big")
    return bytes(packet)


def verify_checksum(packet: bytes) -> bool:
    """Recompute Tag 1 over `packet` and compare — for tests/round-trip checks."""
    if len(packet) < 2:
        return False
    zeroed = packet[:-2] + b"\x00\x00"
    return _checksum16(zeroed) == int.from_bytes(packet[-2:], "big")


def iter_tags(packet: bytes):
    """Yield (tag, value_bytes) for each element in a Local Set packet's body
    (skips the 16-byte key + outer BER length). Minimal — assumes short-form
    BER lengths throughout, true for every tag this module writes."""
    if packet[:16] != UAS_LOCAL_SET_KEY:
        raise ValueError("not a UAS Datalink Local Set packet")
    i = 16
    ln = packet[i]
    i += 1
    if ln & 0x80:  # long-form outer length (only used for very large packets)
        nbytes = ln & 0x7F
        ln = int.from_bytes(packet[i:i + nbytes], "big")
        i += nbytes
    end = i + ln
    while i < end:
        tag = packet[i]
        vlen = packet[i + 1]
        i += 2
        yield tag, packet[i:i + vlen]
        i += vlen


_TAG_TO_NAME = {tag: name for name, (tag, *_rest) in _FIELDS.items()}


def _imapb_decode(code_bytes: bytes, vmin: float, vmax: float, signed: bool) -> float:
    """Inverse of `_imapb`: full-range integer code -> value in [vmin, vmax]."""
    nbytes = len(code_bytes)
    if signed:
        half = 1 << (nbytes * 8 - 1)
        lo, hi = -half, half - 1
        code = int.from_bytes(code_bytes, "big", signed=True)
    else:
        lo, hi = 0, (1 << (nbytes * 8)) - 1
        code = int.from_bytes(code_bytes, "big")
    t = (code - lo) / (hi - lo) if hi > lo else 0.0
    return vmin + t * (vmax - vmin)


def unpack_local_set(packet: bytes) -> dict:
    """Inverse of `pack_local_set`: decode a KLV packet into {field_name:
    value}, plus `checksum_valid`. Best-effort — only decodes tags this module
    knows about (see `_FIELDS`); anything else is skipped, not raised."""
    result: dict = {"checksum_valid": verify_checksum(packet)}
    for tag, raw in iter_tags(packet):
        name = _TAG_TO_NAME.get(tag)
        if name is None or name == "checksum":
            continue
        _, nbytes, vmin, vmax, signed = _FIELDS[name]
        if name == "timestamp":
            result[name] = int.from_bytes(raw, "big") / 1e6  # back to unix seconds
        elif name == "uas_ls_version":
            result[name] = raw[0]
        else:
            result[name] = _imapb_decode(raw, vmin, vmax, signed)
    return result
