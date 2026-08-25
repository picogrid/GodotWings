#!/usr/bin/env python3
"""Watch a STANAG4609 UDP MPEG-TS stream (e.g. from `gw_klv_muxer.py`) and
print each KLV packet's decoded telemetry as it arrives.

This is the fastest way to confirm the KLV track is really there and to watch
values (pan/tilt/lat/lon/...) update live as you drive the camera over its
JSON control socket. It does its own byte-level scan for valid KLV units
rather than properly TS-demuxing, so it works with nothing but the stdlib.

    python3 gw_klv_dump.py --port 5700

Play the actual video separately, e.g.:

    ffplay udp://127.0.0.1:5700
"""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import misb_st0601 as klv  # noqa: E402

MAX_UNIT_LEN = 512  # generous upper bound for the tag set this project emits


def scan_and_print(buf: bytearray, count: int) -> int:
    """Consume every complete, checksum-valid KLV unit found at the front of
    `buf`, printing each one. Returns the updated running count."""
    while True:
        i = buf.find(klv.UAS_LOCAL_SET_KEY)
        if i < 0:
            if len(buf) > 4096:
                del buf[:-16]  # keep a tail in case a key straddles the next recv()
            return count

        matched = False
        for total_len in range(20, MAX_UNIT_LEN):
            end = i + total_len
            if end > len(buf):
                break  # not enough bytes yet to know either way — wait for more
            if klv.verify_checksum(bytes(buf[i:end])):
                count += 1
                values = klv.unpack_local_set(bytes(buf[i:end]))
                fields = ", ".join(
                    f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                    for k, v in values.items() if k != "checksum_valid"
                )
                print(f"[{count}] {fields}")
                del buf[:end]
                matched = True
                break

        if matched:
            continue
        if len(buf) - i > MAX_UNIT_LEN:
            del buf[:i + 1]  # false-positive key match (coincidental bytes) — skip past it
            continue
        return count  # genuine key, just waiting on more bytes to complete it


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, required=True, help="UDP port the MPEG-TS is arriving on.")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    print(f"gw_klv_dump: listening on udp://{args.host}:{args.port} ...", file=sys.stderr)

    buf = bytearray()
    count = 0
    try:
        while True:
            data, _ = sock.recvfrom(65536)
            buf += data
            count = scan_and_print(buf, count)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f"gw_klv_dump: {count} valid KLV packet(s) decoded.", file=sys.stderr)


if __name__ == "__main__":
    main()
