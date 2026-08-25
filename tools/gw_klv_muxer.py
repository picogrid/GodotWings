#!/usr/bin/env python3
"""STANAG 4609 muxer: GodotWings video + telemetry -> H.264 + MISB ST 0601 KLV
in one MPEG-TS, via GStreamer.

Godot (GWCamera / GWGroundPTZ, `launch_ffmpeg = false`) only renders and hosts
two dumb sockets — this tool is what ffmpeg would otherwise be, plus a KLV
metadata track ffmpeg's CLI can't cleanly interleave (see the README "Ground
PTZ" section for why: KLV needs exact packet-boundary control that a raw byte
pipe doesn't give you, which is exactly what GStreamer's buffer-based appsrc
does provide):

    GodotWings --raw RGBA frames (TCP)--> gw_klv_muxer --H.264+KLV MPEG-TS (UDP)--> any FMV/KLV client
              --telemetry JSON (UDP)---/

Point this at whichever camera's `raw_tcp_port` + metadata UDP port; it
auto-detects which telemetry schema arrived (GWCamera's vehicle-relative pose,
or GWGroundPTZ's absolute lat/lon + pan/tilt/zoom) and maps it onto MISB
ST 0601 fields: platform attitude, sensor lat/lon/altitude (properly
accounting for the camera's mount offset AND any gimbal rotation — see
`ypr_from_mount_basis`'s docstring), sensor FOV, and a frame-center + 4-corner
ground footprint from a flat-ground-plane ray intersection (`--ground-alt`) —
NOT a real terrain raycast, since Godot has the actual terrain but this
external process doesn't; see `ground_footprint_ned`. See `misb_st0601.py`
for the KLV encoding itself.

Requires GStreamer + its "good"/"bad" plugin sets and PyGObject:

    brew install gstreamer pygobject3          # macOS
    apt install python3-gi gstreamer1.0-plugins-{base,good,bad} \
                 gstreamer1.0-libav             # Debian/Ubuntu

Run GodotWings first (set `launch_ffmpeg = false` on the camera so it just
hosts the raw-frame server instead of launching its own ffmpeg), then e.g. for
GWGroundPTZ's defaults (1280x720 @30fps, raw frames on :5568, telemetry on
whatever `metadata_port` you set):

    python3 gw_klv_muxer.py --video-port 5568 --metadata-port 5611 \
        --width 1280 --height 720 --fps 30 --out-port 5700

Then point any STANAG4609-aware client (or `gst-launch-1.0 udpsrc port=5700 !
tsdemux ! ...`) at udp://<this host>:5700.

NOT imported by the Godot addon — this is a standalone companion tool, same
role as `gw_camera_client.py`.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from fractions import Fraction
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import misb_st0601 as klv  # noqa: E402

import gi  # noqa: E402

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

R_EARTH = 6378137.0  # WGS84 semi-major axis (m) — matches GWGeoReference.gd


class LatestTelemetry:
    """Thread-safe box holding the most recent telemetry packet."""

    def __init__(self):
        self._lock = threading.Lock()
        self._value = None

    def set(self, value: dict) -> None:
        with self._lock:
            self._value = value

    def get(self) -> dict | None:
        with self._lock:
            return self._value


def udp_listener(sock: socket.socket, box: LatestTelemetry, stop: threading.Event) -> None:
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(65536)
        except socket.timeout:
            continue
        except OSError:
            return
        try:
            box.set(json.loads(data.decode("utf-8")))
        except (ValueError, UnicodeDecodeError):
            continue


def ned_to_geodetic(pos_ned, home_lat: float, home_lon: float, home_alt: float):
    """Flat-tangent NED-from-home -> (lat, lon, alt) deg/deg/m. Mirrors
    GWGeoReference.gd's `ned_to_geodetic()` exactly, for consistency."""
    n, e, d = pos_ned
    lat = home_lat + math.degrees(n / R_EARTH)
    lon = home_lon + math.degrees(e / (R_EARTH * math.cos(math.radians(home_lat))))
    alt = home_alt - d
    return lat, lon, alt


def quat_to_euler_ned(w: float, x: float, y: float, z: float):
    """Body(FRD)->NED quaternion -> (heading, pitch, roll) degrees, aerospace
    ZYX convention (matches ArduPilot/PX4). Heading wraps to [0, 360).

    Verified against real Godot output (GWCoordConvert.attitude_to_dcm ->
    get_rotation_quaternion, decoded back and compared to the original
    roll/pitch/yaw across several attitudes, exact to float precision) — not
    hand-derived-and-hoped.
    """
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw) % 360.0, math.degrees(pitch), math.degrees(roll)


def quat_to_ned_columns(w: float, x: float, y: float, z: float):
    """Body(FRD)->NED quaternion -> its DCM's (fwd, right, down) columns, each
    a 3-tuple in NED. Verified against real Godot output the same way as
    `quat_to_euler_ned` above."""
    fwd = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y))
    right = (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x))
    down = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    return fwd, right, down


def godot_body_vec_to_ned(v_godot, quat_body_to_ned) -> tuple:
    """Any vector expressed in the vehicle's local Godot render-frame axes
    (+X body-right, +Y body-up, -Z body-forward — e.g. `mount_pos`, or a
    `mount_basis` column) -> the same vector in NED, via the platform's
    attitude quaternion."""
    gx, gy, gz = v_godot
    # Godot render-frame -> FRD: forward = -gz, right = gx, down = -gy.
    v_fwd, v_right, v_down = -gz, gx, -gy
    fwd, right, down = quat_to_ned_columns(*quat_body_to_ned)
    return tuple(
        fwd[i] * v_fwd + right[i] * v_right + down[i] * v_down for i in range(3)
    )


def mount_offset_ned(mount_pos, quat_body_to_ned) -> tuple:
    """The camera's static mount offset (`mount_pos`, GWCamera's own
    `transform.origin`) rotated into a NED offset via the platform's
    attitude, so the sensor's true position (not just the platform CG) can
    be used."""
    return godot_body_vec_to_ned(mount_pos, quat_body_to_ned)


def _rodrigues(axis, theta):
    """Rotation matrix (as 3 column-tuples) for `theta` rad about a unit `axis`."""
    ax, ay, az = axis
    c, s = math.cos(theta), math.sin(theta)

    def col(v):
        vx, vy, vz = v
        cx, cy, cz = ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx
        dot = ax * vx + ay * vy + az * vz
        return (
            vx * c + cx * s + ax * dot * (1 - c),
            vy * c + cy * s + ay * dot * (1 - c),
            vz * c + cz * s + az * dot * (1 - c),
        )

    return [col((1, 0, 0)), col((0, 1, 0)), col((0, 0, 1))]


def _mat_mul(a, b):
    def mv(m, v):
        return (
            m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
            m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
        )

    return [mv(a, b[0]), mv(a, b[1]), mv(a, b[2])]


def basis_from_ypr(az_deg: float, el_deg: float, roll_deg: float):
    """Port of Camera.gd's `_gimbal_basis()`: Basis(UP,-yaw) * Basis(RIGHT,pitch)
    * Basis(BACK,-roll). Returns the flattened 9-float [bx, by, bz] columns
    (same layout GWCamera sends as `mount_basis`)."""
    m = _mat_mul(_mat_mul(_rodrigues((0, 1, 0), -math.radians(az_deg)),
                           _rodrigues((1, 0, 0), math.radians(el_deg))),
                 _rodrigues((0, 0, 1), -math.radians(roll_deg)))
    return [*m[0], *m[1], *m[2]]


def ypr_from_mount_basis(basis9) -> tuple:
    """Inverse of `basis_from_ypr`: decode (az_deg, el_deg, roll_deg) relative
    to the platform (body) from GWCamera's `mount_basis` — the camera's FULL
    orientation relative to the aircraft (static mount rotation combined with
    any live servo-gimbal rotation, exactly as GWCamera computes it). az/roll
    positive = clockwise/right, el positive = up.

    Verified against real Godot Basis output (static-mount-only, gimbal-only,
    and the two composed together — the composed case matters because a
    tilted mount is NOT simply "add the gimbal's own angles"): round-trips to
    within float precision, see gen_mount_basis_truth.gd if you want to
    regenerate that ground truth.
    """
    bz = basis9[6:9]
    forward = (-bz[0], -bz[1], -bz[2])  # local -Z is forward
    az = math.degrees(math.atan2(forward[0], -forward[2])) % 360.0
    el = math.degrees(math.asin(max(-1.0, min(1.0, forward[1]))))

    # Recover roll as the signed angle (about `forward`) from the roll=0
    # reference "up" at this az/el to the actual "up" (basis9[3:6]).
    by = basis9[3:6]
    by_ref = basis_from_ypr(az, el, 0.0)[3:6]

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    roll = math.degrees(math.atan2(dot(cross(by_ref, by), forward), dot(by_ref, by))) % 360.0
    return az, el, roll


def hfov_from_vfov(vfov_deg: float, width: int, height: int) -> float:
    aspect = width / max(height, 1)
    return math.degrees(2 * math.atan(math.tan(math.radians(vfov_deg) / 2) * aspect))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _normalize(a):
    n = math.sqrt(_dot(a, a))
    return _scale(a, 1.0 / n) if n > 1e-12 else a


def camera_axes_from_mount_basis(mount_basis, quat_body_to_ned):
    """Vehicle-camera schema: (forward, right, up) unit vectors in NED, from
    `mount_basis` (camera-in-body, Godot axes: bx=right, by=up, bz=-forward)
    rotated into NED by the platform's attitude."""
    bx, by, bz = mount_basis[0:3], mount_basis[3:6], mount_basis[6:9]
    forward_godot = (-bz[0], -bz[1], -bz[2])
    fwd = _normalize(godot_body_vec_to_ned(forward_godot, quat_body_to_ned))
    right = _normalize(godot_body_vec_to_ned(bx, quat_body_to_ned))
    up = _normalize(godot_body_vec_to_ned(by, quat_body_to_ned))
    return fwd, right, up


def camera_axes_from_heading_tilt(heading_deg: float, tilt_deg: float):
    """GWGroundPTZ schema: (forward, right, up) unit vectors in NED from an
    absolute compass heading + elevation (no roll axis on a PTZ head, so
    `right` stays horizontal — level horizon)."""
    h = math.radians(heading_deg)
    e = math.radians(tilt_deg)
    fwd = (math.cos(e) * math.cos(h), math.cos(e) * math.sin(h), -math.sin(e))
    right = (-math.sin(h), math.cos(h), 0.0)
    up = _cross(right, fwd)
    return fwd, right, up


def ray_plane_intersect_ned(origin_ned, direction_ned, ground_d: float):
    """Intersect a ray (NED origin + unit direction) with the flat horizontal
    plane at NED-down = `ground_d`. Returns the intersection point (N,E,D), or
    None if the ray doesn't point down enough to ever reach it (looking at or
    above the horizon) or the plane is behind the origin."""
    if direction_ned[2] <= 1e-6:
        return None
    t = (ground_d - origin_ned[2]) / direction_ned[2]
    if t <= 0.0:
        return None
    return _add(origin_ned, _scale(direction_ned, t))


def ground_footprint_ned(cam_pos_ned, fwd, right, up, hfov_deg: float, vfov_deg: float, ground_d: float):
    """Simple flat-ground-plane footprint: intersect the boresight and the 4
    frame-corner rays (pinhole/rectilinear unprojection of the FOV) with a
    horizontal plane at NED-down = `ground_d`. This is NOT a real terrain
    raycast (Godot has the actual terrain; this external process doesn't) —
    it's the flat-earth approximation, good enough absent real terrain data,
    and clearly labeled as such wherever it's used.

    Returns (center_ned, [corner1..corner4]_ned) — corners ordered clockwise
    from upper-left (upper-left, upper-right, lower-right, lower-left), the
    conventional MISB corner-point order — or None if the boresight itself
    doesn't hit the plane (e.g. camera pointed at/above the horizon)."""
    center = ray_plane_intersect_ned(cam_pos_ned, fwd, ground_d)
    if center is None:
        return None
    half_h = math.tan(math.radians(hfov_deg) / 2.0)
    half_v = math.tan(math.radians(vfov_deg) / 2.0)
    corners = []
    for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1)):  # UL, UR, LR, LL
        d = _normalize(_add(_add(fwd, _scale(right, half_h * sx)), _scale(up, half_v * sy)))
        hit = ray_plane_intersect_ned(cam_pos_ned, d, ground_d)
        if hit is None:
            return None  # a corner ray missing the plane makes the footprint ill-defined
        corners.append(hit)
    return center, corners


def _footprint_fields(cam_pos_ned, fwd, right, up, hfov_deg, vfov_deg, ground_d, home_lat, home_lon, home_alt) -> dict:
    """Frame-center + 4-corner MISB fields from a flat-ground-plane footprint,
    or {} if the boresight doesn't hit the plane (e.g. camera above horizon)."""
    fp = ground_footprint_ned(cam_pos_ned, fwd, right, up, hfov_deg, vfov_deg, ground_d)
    if fp is None:
        return {}
    center, corners = fp
    clat, clon, calt = ned_to_geodetic(center, home_lat, home_lon, home_alt)
    fields = {"frame_center_lat": clat, "frame_center_lon": clon, "frame_center_elevation": calt}
    for i, corner in enumerate(corners, start=1):
        lat_c, lon_c, _ = ned_to_geodetic(corner, home_lat, home_lon, home_alt)
        fields[f"corner_lat_{i}"] = lat_c
        fields[f"corner_lon_{i}"] = lon_c
    return fields


def telemetry_to_klv_fields(msg: dict, args) -> dict | None:
    """Map GWCamera's (vehicle) or GWGroundPTZ's metadata JSON onto MISB
    ST 0601 fields. Returns None if `msg` matches neither schema."""
    if "pos_ned" in msg and "quat_body_to_ned" in msg:
        # --- vehicle GWCamera (Camera.gd:_send_metadata) ---
        quat = msg["quat_body_to_ned"]
        heading, pitch, roll = quat_to_euler_ned(*quat)

        # Sensor position = platform CG + the mount's own offset (its static
        # translation, e.g. "1m ahead of CG"), rotated into NED by the
        # platform's attitude — NOT just the CG, or a mount offset gets
        # silently dropped. `mount_pos` is absent on older Godot builds
        # (pre- this fix); treat that as "camera at the CG" for compatibility.
        pos_ned = msg["pos_ned"]
        mount_pos = msg.get("mount_pos", [0.0, 0.0, 0.0])
        offset_ned = mount_offset_ned(mount_pos, quat)
        sensor_ned = [pos_ned[i] + offset_ned[i] for i in range(3)]
        lat, lon, alt = ned_to_geodetic(sensor_ned, args.home_lat, args.home_lon, args.home_alt)

        # Sensor pointing relative to the platform: `mount_basis` already
        # combines the static mount rotation with any live servo-gimbal
        # rotation (Camera.gd always sends it, unconditionally) — decoding
        # THAT, rather than the separate/conditional `gimbal_deg`, is what
        # actually accounts for a fixed-angle mount (most cameras: no gimbal
        # at all, just tilted down) as well as a gimbal when there is one.
        mount_basis = msg.get("mount_basis")
        if mount_basis is not None:
            rel_az, rel_el, rel_roll = ypr_from_mount_basis(mount_basis)
        else:
            rel_az, rel_el, rel_roll = 0.0, 0.0, 0.0

        vfov = msg.get("fov_deg", 50.0)
        width, height = msg.get("width", args.width), msg.get("height", args.height)
        hfov = hfov_from_vfov(vfov, width, height)

        fields = {
            "timestamp": int(time.time() * 1e6),  # wall-clock; sim_time isn't wall-clock-comparable
            "platform_heading": heading,
            "platform_pitch": pitch,
            "platform_roll": roll,
            "sensor_lat": lat,
            "sensor_lon": lon,
            "sensor_true_alt": alt,
            "sensor_hfov": hfov,
            "sensor_vfov": vfov,
            "sensor_rel_az": rel_az % 360.0,
            "sensor_rel_el": rel_el,
            "sensor_rel_roll": rel_roll % 360.0,
            "uas_ls_version": 13,
        }
        if mount_basis is not None and not args.no_footprint:
            fwd, right, up = camera_axes_from_mount_basis(mount_basis, quat)
            ground_d = args.home_alt - args.ground_alt  # flat-ground elevation, in the same NED-D frame
            fields.update(_footprint_fields(sensor_ned, fwd, right, up, hfov, vfov, ground_d,
                    args.home_lat, args.home_lon, args.home_alt))
        return fields

    if "pan_deg" in msg and "lat" in msg:
        # --- GWGroundPTZ (GroundPTZ.gd:_send_metadata) — fixed mount, so the
        # platform IS the sensor's colocated base; pan/tilt ARE its relative
        # az/el and its own heading is the pan=0 reference. ---
        platform_heading = (msg["heading_deg"] - msg["pan_deg"]) % 360.0
        hfov = msg.get("hfov_deg", 50.0)
        vfov = msg.get("vfov_deg", 50.0)

        fields = {
            "timestamp": int(msg.get("unix_time", time.time()) * 1e6),
            "platform_heading": platform_heading,
            "platform_pitch": 0.0,
            "platform_roll": 0.0,
            "sensor_lat": msg["lat"],
            "sensor_lon": msg["lon"],
            "sensor_true_alt": msg["alt"],
            "sensor_hfov": hfov,
            "sensor_vfov": vfov,
            "sensor_rel_az": msg["pan_deg"] % 360.0,
            "sensor_rel_el": msg["tilt_deg"],
            "sensor_rel_roll": 0.0,
            "uas_ls_version": 13,
        }
        if not args.no_footprint:
            # A fixed mount has no NED home of its own to work in — treat the
            # camera's own position as a per-frame local origin instead (its
            # lat/lon/alt double as that local frame's "home").
            fwd, right, up = camera_axes_from_heading_tilt(msg["heading_deg"], msg["tilt_deg"])
            ground_d = msg["alt"] - args.ground_alt
            fields.update(_footprint_fields((0.0, 0.0, 0.0), fwd, right, up, hfov, vfov, ground_d,
                    msg["lat"], msg["lon"], msg["alt"]))
        return fields

    return None


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def build_pipeline(args) -> Gst.Pipeline:
    fps = Fraction(args.fps).limit_denominator(1000)
    keyint = max(1, round(args.fps))
    # The whole point of `mpegtsmux` here is that it interleaves H.264 and KLV
    # into ONE MPEG-TS program with correct PES/PID framing (see the module
    # docstring for why that's hard to get right any other way) — so pushing
    # to RTSP reuses that unchanged and just wraps the resulting TS bytestream
    # in RTP/MP2T (payload type 33), rather than trying to describe KLV as its
    # own RTSP media (rtspclientsink has no KLV payloader to do that with).
    # MediaMTX (and most RTSP servers) receive this as one MPEG-TS track and
    # re-serve it to any client exactly as they would ffmpeg's `-f rtsp`.
    if args.rtsp_url:
        sink = (
            f"sink.sink_0 rtspclientsink name=sink location={args.rtsp_url} "
            f"protocols=tcp sink_0::payloader=rtpmp2tpay"
        )
    else:
        sink = f"udpsink host={args.out_host} port={args.out_port} sync=false async=false"
    desc = (
        f"appsrc name=vsrc is-live=true format=time block=true "
        f"caps=video/x-raw,format=RGBA,width={args.width},height={args.height},"
        f"framerate={fps.numerator}/{fps.denominator} "
        f"! videoconvert ! video/x-raw,format=I420 "
        f"! x264enc tune=zerolatency speed-preset=ultrafast "
        f"bitrate={args.bitrate} key-int-max={keyint} "
        # -1 repeats SPS/PPS before every IDR frame (not just once/sec on a
        # timer) — required for any consumer joining mid-stream (an RTSP
        # server ingesting the push, a client connecting to the UDP output
        # after start) to actually be able to decode from its first IDR.
        f"! h264parse config-interval=-1 ! mux. "
        f"appsrc name=ksrc is-live=true format=time block=true caps=meta/x-klv,parsed=true "
        f"! queue ! mux. "
        f"mpegtsmux name=mux alignment=7 latency={args.mux_latency_ms * 1_000_000} ! {sink}"
    )
    return Gst.parse_launch(desc)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--godot-host", default="127.0.0.1", help="Host Godot is running on.")
    ap.add_argument("--video-port", type=int, required=True, help="GWCamera/GWGroundPTZ raw_tcp_port.")
    ap.add_argument("--metadata-port", type=int, required=True, help="Its metadata_port (UDP).")
    ap.add_argument("--width", type=int, required=True, help="Must match the camera's `resolution.x`.")
    ap.add_argument("--height", type=int, required=True, help="Must match the camera's `resolution.y`.")
    ap.add_argument("--fps", type=float, default=30.0, help="Must match the camera's `fps`.")
    ap.add_argument("--bitrate", type=int, default=4000, help="H.264 bitrate, kbps.")
    ap.add_argument("--mux-latency-ms", type=int, default=200,
            help="How long mpegtsmux waits for the (slower) video-encode branch before "
            "committing to its first PAT/PMT. Without this, the near-zero-latency KLV appsrc "
            "reaches the muxer before x264enc's first encoded frame does, so the very first "
            "table only declares the KLV track — and readers that read the PMT once at start "
            "(MediaMTX's UDP source among them) latch onto that and reject all subsequent video "
            "as an 'undeclared track', even though a corrected PMT arrives ~100ms later. "
            "Verified: 0 (disabled) reliably reproduces that; 200 reliably fixes it. Raise it if "
            "your video branch is slower to start (bigger resolution/bitrate, slower machine).")
    ap.add_argument("--out-host", default="127.0.0.1", help="Where to udpsink the muxed MPEG-TS (ignored if --rtsp-url is set).")
    ap.add_argument("--out-port", type=int, default=5700)
    ap.add_argument("--rtsp-url", default=None,
            help="Push the muxed MPEG-TS (H.264+KLV, unchanged) to an RTSP server instead of "
            "udpsink — e.g. rtsp://127.0.0.1:8554/groundptz_klv, same MediaMTX GWCamera's own "
            "RTSP protocol already targets, just its own path. Overrides --out-host/--out-port.")
    ap.add_argument("--home-lat", type=float, default=0.0,
            help="Vehicle-camera schema only: match your SITL HOME_LOCATION / GWGeoReference.home_lat.")
    ap.add_argument("--home-lon", type=float, default=0.0)
    ap.add_argument("--home-alt", type=float, default=0.0)
    ap.add_argument("--ground-alt", type=float, default=0.0,
            help="True altitude (m) of a flat ground plane, for the frame-center + corner-point "
            "tags: a ray/plane intersection, NOT a real terrain raycast (Godot has the actual "
            "terrain; this external process doesn't). Vehicle schema: match your home_alt if the "
            "ground near it is roughly flat; GroundPTZ schema: an absolute true altitude.")
    ap.add_argument("--no-footprint", action="store_true",
            help="Omit the frame-center + 4 corner-point tags (23-25, 82-89). With every other "
            "tag this module emits, the packet is ~87 bytes (BER short-form length); adding all "
            "9 footprint tags pushes it to ~135 bytes, which flips the KLV Local Set's outer "
            "length field to BER long-form (multi-byte). That's valid KLV, but some simpler/naive "
            "parsers only handle the single-byte case — if your downstream tool errors loading "
            "tags, try this flag first to check whether that's the cause.")
    ap.add_argument("--duration", type=float, default=None, help="Stop after this many seconds (mainly for testing).")
    args = ap.parse_args()

    Gst.init(None)

    print(f"gw_klv_muxer: connecting to tcp://{args.godot_host}:{args.video_port} for video...", file=sys.stderr)
    video_sock = socket.create_connection((args.godot_host, args.video_port), timeout=10)

    meta_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    meta_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    meta_sock.bind(("0.0.0.0", args.metadata_port))
    telemetry = LatestTelemetry()
    stop = threading.Event()
    threading.Thread(target=udp_listener, args=(meta_sock, telemetry, stop), daemon=True).start()

    pipeline = build_pipeline(args)
    vsrc = pipeline.get_by_name("vsrc")
    ksrc = pipeline.get_by_name("ksrc")

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_bus(_bus, msg):
        if msg.type == Gst.MessageType.EOS:
            print("gw_klv_muxer: EOS", file=sys.stderr)
            loop.quit()
        elif msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"gw_klv_muxer: GStreamer error: {err} ({dbg})", file=sys.stderr)
            loop.quit()

    bus.connect("message", on_bus)
    pipeline.set_state(Gst.State.PLAYING)
    target = args.rtsp_url if args.rtsp_url else f"udp://{args.out_host}:{args.out_port}"
    print(f"gw_klv_muxer: publishing STANAG4609 MPEG-TS (H.264+KLV) -> {target}", file=sys.stderr)

    frame_bytes = args.width * args.height * 4  # RGBA8
    frame_interval_ns = int(Gst.SECOND / args.fps)

    def video_reader() -> None:
        frame_idx = 0
        deadline = time.monotonic() + args.duration if args.duration else None
        while not stop.is_set():
            if deadline and time.monotonic() > deadline:
                break
            raw = recv_exact(video_sock, frame_bytes)
            if raw is None:
                print("gw_klv_muxer: video source disconnected.", file=sys.stderr)
                break
            pts = frame_idx * frame_interval_ns
            frame_idx += 1

            vbuf = Gst.Buffer.new_wrapped(raw)
            vbuf.pts = pts
            vbuf.duration = frame_interval_ns
            vsrc.emit("push-buffer", vbuf)

            msg = telemetry.get()
            fields = telemetry_to_klv_fields(msg, args) if msg is not None else None
            if fields is None:
                fields = {"timestamp": int(time.time() * 1e6), "uas_ls_version": 13}
            kbuf = Gst.Buffer.new_wrapped(klv.pack_local_set(fields))
            kbuf.pts = pts
            kbuf.duration = frame_interval_ns
            ksrc.emit("push-buffer", kbuf)

        stop.set()
        vsrc.emit("end-of-stream")
        ksrc.emit("end-of-stream")
        GLib.idle_add(loop.quit)

    threading.Thread(target=video_reader, daemon=True).start()

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        pipeline.set_state(Gst.State.NULL)
        video_sock.close()
        meta_sock.close()


if __name__ == "__main__":
    main()
