#!/usr/bin/env python3
"""LOCAL DEBUGGING FIXTURE ONLY -- NOT the compliant way to use real Cesium
ion / Google 3D Tiles data. See addons/godotwings/world/Tiles3DStreamer.gd
for that (fetches live every session, holds content in memory only, never
touches disk).

This tool downloads real Cesium 3D Tiles content (e.g. Google Photorealistic
3D Tiles via Cesium ion) for a bounded area around --lat/--lon and caches it
to disk (tile content + a manifest of precomputed local Transform3Ds) so
Tiles3DLoader.gd can load it with zero network access at runtime. That
"prefetch once, cache to disk, run offline" architecture is exactly what
Cesium ion's and Google's actual Terms of Service prohibit -- pulled live
from the current terms, not assumed: see the module-level notes below.

So: use this only to debug the traversal/AOI-pruning/transform math itself
against a small area, ideally with --tileset-url pointed at a public,
UNAUTHENTICATED sample (e.g. Cesium's own 3d-tiles-samples repo -- no ion
account, no terms issue) rather than a real ion token. If you do point it at
a real ion asset, treat that as a momentary local check, not something to
leave running or rely on -- GWTiles3DStreamer is the actual sanctioned path
for real use.

    pip install requests numpy
    python3 gw_3dtiles_prefetch.py --asset-id 2275207 --ion-token YOUR_TOKEN \
        --lat 60.221088825593135 --lon 25.018208331290666 --radius-km 2 \
        --detail-m 50

--asset-id 2275207 is Cesium ion's Google Photorealistic 3D Tiles asset (the
same one examples/Globe.tscn already references). Any other 3D-Tiles-1.x ion
asset ID works too. Requires a Cesium ion account + access token
(cesium.com; free tier has a usage quota) — this tool can't provision one.

Without --ion-token / --asset-id, pass --tileset-url to walk any public,
unauthenticated tileset.json directly.

CACHING IS SESSION-SCOPED, AND THAT'S NOT ENOUGH ON ITS OWN.
Cesium ion ToS S2.2.2 (cesium.com/legal/terms-of-service/): "You may not
copy, store, or redistribute any portion of Cesium Data Output in, or for
use in, AN OFFLINE ENVIRONMENT." The only caching exception is generic
client/proxy caching "that caches other internet traffic too" -- ordinary
HTTP caching during LIVE use, not this. Google Maps Content terms
(cesium.com/legal/terms-for-google/): "You will not... download Google Maps
tiles OR STREET VIEW TILES FOR STORAGE OR REHOSTING." The measures below
reduce how far this can drift from "momentary local debugging" -- they do
NOT make the underlying prefetch-and-store architecture compliant for real
use. Unlike gw_terrain_import.py's baked imagery (open data, one example
committed to the repo), this tool:
  - writes only under .cache/3dtiles/ (gitignored — see .gitignore; never
    commit tile content or a manifest, and there is no committed example),
  - stamps every manifest with `fetched_at` and re-fetches fresh every run;
    `--purge-stale-hours HOURS` deletes every cache directory older than
    that (run it on its own, periodically -- e.g. at the start of a dev
    session -- rather than trusting a manifest to get cleaned up on its
    own), and Tiles3DLoader.gd's `max_cache_age_hours` (default 24h)
    refuses to even LOAD a stale manifest at runtime,
  - captures and surfaces attribution: printed to the console here and
    written to the manifest's `content_attributions`, exposed at runtime as
    Tiles3DLoader.gd's `attributions` -- Cesium ion's and the content
    provider's terms REQUIRE displaying this wherever the content is shown
    to anyone besides you, not just recording it.
This tool cannot enforce a data provider's terms on your behalf — it only
gives you the mechanisms (short-lived gitignored cache, enforced expiry,
surfaced attribution) to stay within them. Read Cesium ion's and, if using
Google Photorealistic 3D Tiles, Google Maps Platform's current terms
yourself before deploying this beyond
local dev use.

NOT imported by the Godot addon — standalone companion tool, same role as
gw_terrain_import.py.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import shutil
import struct
import sys
from pathlib import Path
from urllib.parse import urljoin, urlparse, parse_qs

import numpy as np
import requests

R_EQ = 6378137.0                    # WGS84 semi-major axis (m)
FLATTENING = 1.0 / 298.257223563
E2 = FLATTENING * (2.0 - FLATTENING)  # eccentricity squared

ION_ENDPOINT = "https://api.cesium.com/v1/assets/{asset_id}/endpoint"


# --- geodetic math (verified independently: round-trips to ~1e-9 deg / ---
# --- sub-mm across equator/pole/antimeridian/Helsinki test points) -------

def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    n = R_EQ / math.sqrt(1 - E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1 - E2) + alt_m) * sin_lat
    return np.array([x, y, z])


def ecef_to_geodetic(p: np.ndarray) -> tuple[float, float, float]:
    """Bowring's method (closed-form)."""
    x, y, z = p
    lon = math.atan2(y, x)
    dist_xy = math.hypot(x, y)
    b = R_EQ * (1 - FLATTENING)
    ep2 = (R_EQ * R_EQ - b * b) / (b * b)
    theta = math.atan2(z * R_EQ, dist_xy * b)
    lat = math.atan2(z + ep2 * b * math.sin(theta) ** 3, dist_xy - E2 * R_EQ * math.cos(theta) ** 3)
    sin_lat = math.sin(lat)
    n = R_EQ / math.sqrt(1 - E2 * sin_lat * sin_lat)
    alt = dist_xy / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def enu_basis(lat_deg: float, lon_deg: float) -> np.ndarray:
    """3x3 matrix whose COLUMNS are the East, North, Up unit vectors (in
    ECEF) at (lat, lon). Verified: orthonormal + right-handed (E x N = Up)
    at equator/pole/Helsinki/Sydney, and a point built purely from this
    basis decodes back through it exactly."""
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    east = np.array([-so, co, 0.0])
    north = np.array([-sl * co, -sl * so, cl])
    up = np.array([cl * co, cl * so, sl])
    return np.column_stack([east, north, up])


# ENU (East, North, Up) -> Godot EUS (East, Up, South) axes: Godot's East =
# ENU's East, Godot's Up = ENU's Up, Godot's -Z(=South, per GWCoordConvert)
# = -ENU's North. Applied to a column-vector's components (a permute+negate
# of rows), or equivalently pre-multiplying a rotation matrix whose columns
# are already expressed in the ENU basis.
_ENU_TO_EUS = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
])


def bbox_from_center(lat: float, lon: float, radius_km: float) -> list[float]:
    """Same flat-tangent approximation as gw_terrain_import.py's own
    bbox_from_center — used here only to decide which tiles to keep, not
    for any position math (that's all rigorous ECEF/ENU above)."""
    r_m = radius_km * 1000.0
    dlat = math.degrees(r_m / R_EQ)
    dlon = math.degrees(r_m / (R_EQ * math.cos(math.radians(lat))))
    return [lon - dlon, lat - dlat, lon + dlon, lat + dlat]


def bbox_overlaps(a: list[float], b: list[float]) -> bool:
    a_min_lon, a_min_lat, a_max_lon, a_max_lat = a
    b_min_lon, b_min_lat, b_max_lon, b_max_lat = b
    return a_min_lon <= b_max_lon and a_max_lon >= b_min_lon and a_min_lat <= b_max_lat and a_max_lat >= b_min_lat


def parse_gltf_transform(m: list[float]) -> np.ndarray:
    """3D Tiles `transform`: 16 numbers, COLUMN-major (glTF/3D-Tiles
    convention). Returns the 4x4 matrix such that world = M @ local
    (homogeneous, column vectors)."""
    return np.array(m).reshape(4, 4, order="F")


IDENTITY_4X4 = np.eye(4)


def aoi_ecef_sphere(lat: float, lon: float, alt: float, radius_km: float) -> tuple[np.ndarray, float]:
    """AOI as a bounding sphere in ECEF -- used to test `box`/`sphere`
    boundingVolumes directly in Cartesian space (see box_intersects_sphere's
    docstring for why a geodetic-envelope-of-corners approach is wrong for
    these)."""
    return geodetic_to_ecef(lat, lon, alt), radius_km * 1000.0


def box_intersects_sphere(box: list[float], world_transform: np.ndarray,
        sphere_center: np.ndarray, sphere_radius: float) -> bool:
    """3D Tiles `boundingVolume.box`: [centerX,Y,Z, halfX_x,y,z, halfY_x,y,z,
    halfZ_x,y,z] (12 numbers) -- an oriented box in the tile's own local
    frame, pushed through world_transform into ECEF. Tests overlap against
    an ECEF sphere via closest-point-on-OBB, entirely in linear/Cartesian
    space.

    NOT done via a geodetic envelope of the box's 8 corners: verified live
    against the real Google Photorealistic 3D Tiles root tile, whose box is
    ~7645km half-extents centered on Earth's center (i.e. it encloses the
    whole planet) -- its 8 corners project to a tight, WRONG lat/lon
    envelope ([-35,-135] to [35,135]) because the box's FACES (not corners)
    pass nearest the poles and the antimeridian; corner-sampling a box
    through a nonlinear (geodetic) projection simply doesn't bound it. A
    Cartesian test has no such failure mode."""
    center_local = np.array([box[0], box[1], box[2], 1.0])
    center_world = (world_transform @ center_local)[:3]
    r = world_transform[:3, :3]
    axes_world = [r @ np.array(box[3:6]), r @ np.array(box[6:9]), r @ np.array(box[9:12])]

    d = sphere_center - center_world
    closest = center_world.copy()
    for axis in axes_world:
        half_extent = float(np.linalg.norm(axis))
        if half_extent < 1e-9:
            continue  # degenerate (flat) axis
        axis_dir = axis / half_extent
        dist_along = float(np.clip(np.dot(d, axis_dir), -half_extent, half_extent))
        closest = closest + dist_along * axis_dir
    return float(np.linalg.norm(sphere_center - closest)) <= sphere_radius


def sphere_bv_intersects_sphere(bv_sphere: list[float], world_transform: np.ndarray,
        sphere_center: np.ndarray, sphere_radius: float) -> bool:
    """3D Tiles `boundingVolume.sphere`: [centerX,Y,Z, radius] in the tile's
    local frame. Same Cartesian-space reasoning as box_intersects_sphere."""
    cx, cy, cz, radius = bv_sphere
    center_world = (world_transform @ np.array([cx, cy, cz, 1.0]))[:3]
    # Scale the local radius by the transform's scale (assumed uniform across
    # all three axes -- true for every real transform seen this session).
    scale = float(np.linalg.norm(world_transform[:3, 0])) or 1.0
    return float(np.linalg.norm(sphere_center - center_world)) <= sphere_radius + radius * scale


def bounding_volume_overlaps_aoi(bv: dict, world_transform: np.ndarray,
        aoi_bbox: list[float], aoi_sphere: tuple[np.ndarray, float]) -> bool:
    """`region` is native geodetic (a fixed EPSG:4979-like frame per the 3D
    Tiles spec, independent of the tile's own local transform) -- an exact
    comparison against the AOI's own geodetic bbox, no projection involved.
    `box`/`sphere` are tested in Cartesian ECEF space against aoi_sphere (see
    box_intersects_sphere). An unrecognized volume type is NOT pruned
    (conservative: over-fetch rather than silently drop real content)."""
    if "region" in bv:
        west, south, east, north = (math.degrees(v) for v in bv["region"][:4])
        return bbox_overlaps([west, south, east, north], aoi_bbox)
    if "box" in bv:
        return box_intersects_sphere(bv["box"], world_transform, *aoi_sphere)
    if "sphere" in bv:
        return sphere_bv_intersects_sphere(bv["sphere"], world_transform, *aoi_sphere)
    return True


def local_transform_to_godot(world_transform: np.ndarray, origin_lat: float, origin_lon: float,
        origin_alt: float) -> dict:
    """A tile's ECEF-space transform -> a Godot Transform3D (position +
    3x3 basis) relative to (origin_lat, origin_lon, origin_alt), in Godot's
    EUS axes (matches GWCoordConvert: x=East, y=Up, z=South)."""
    origin_ecef = geodetic_to_ecef(origin_lat, origin_lon, origin_alt)
    enu = enu_basis(origin_lat, origin_lon)  # columns E, N, U

    tile_pos_ecef = world_transform[:3, 3]
    offset_ecef = tile_pos_ecef - origin_ecef
    enu_offset = enu.T @ offset_ecef  # (east, north, up) meters
    godot_pos = [float(enu_offset[0]), float(enu_offset[2]), float(-enu_offset[1])]  # (E, Up, -N)

    tile_rot_ecef = world_transform[:3, :3]
    rot_in_enu = enu.T @ tile_rot_ecef  # tile's local axes, expressed in the ENU basis
    rot_in_godot = _ENU_TO_EUS @ rot_in_enu
    return {"position": godot_pos, "basis": rot_in_godot.tolist()}


# --- 3D Tiles content: plain glTF/GLB, or legacy .b3dm (strip its header) --

def unwrap_b3dm(data: bytes) -> bytes:
    """b3dm = [28-byte header][feature table JSON+bin][batch table
    JSON+bin][embedded glb]. Header: magic(4)='b3dm', version(4),
    byteLength(4), featureTableJSONByteLength(4), featureTableBinaryByteLength(4),
    batchTableJSONByteLength(4), batchTableBinaryByteLength(4)."""
    magic = data[0:4]
    if magic != b"b3dm":
        return data  # already a plain glb/gltf
    (ft_json_len, ft_bin_len, bt_json_len, bt_bin_len) = struct.unpack("<IIII", data[12:28])
    glb_start = 28 + ft_json_len + ft_bin_len + bt_json_len + bt_bin_len
    return data[glb_start:]


def fetch_bytes(url: str, session: requests.Session) -> bytes:
    resp = session.get(url, timeout=30)
    resp.raise_for_status()
    return resp.content


def fetch_json(url: str, session: requests.Session) -> dict:
    resp = session.get(url, timeout=30)
    resp.raise_for_status()
    return resp.json()


def strip_query(url: str) -> str:
    """Ion tile/session tokens ride as a query string on every URL; strip it
    before deriving a cache filename so we don't leak tokens into filenames."""
    return url.split("?", 1)[0]


class Walker:
    """Recursively walks one tileset.json, pruning subtrees outside the AOI
    and stopping each branch once geometricError <= detail_m, downloading
    that tile's content. Handles: tile.transform composition, 3D Tiles 1.1
    `contents` (plural) as well as legacy singular `content`, external
    tileset references (a content .uri ending in .json), and `refine`
    inheritance (tracked but not currently used to skip ADD-parent content --
    v1 keeps every content-bearing tile it stops on, which over-fetches
    slightly for REPLACE trees but never under-fetches or mis-positions)."""

    def __init__(self, session: requests.Session, aoi_bbox: list[float], aoi_sphere: tuple[np.ndarray, float],
            detail_m: float, out_dir: Path, origin_lat: float, origin_lon: float, origin_alt: float,
            auth_query: str | None):
        self.session = session
        self.aoi_bbox = aoi_bbox
        self.aoi_sphere = aoi_sphere
        self.detail_m = detail_m
        self.out_dir = out_dir
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt
        # Query params to ensure are present on every request (tileset.json
        # AND content fetches alike) -- e.g. "access_token=..." for a normal
        # ion-hosted asset, or "key=..." for Google Photorealistic 3D Tiles
        # (see resolve_ion_asset: Google's proxy requires its own API key on
        # every request, not a bearer token we control the name of --
        # verified live: a content URI's own embedded `session=` query alone
        # 403s; re-appending the tileset's `key` fixes that). Google *also*
        # mints a fresh `session` token per tileset.json fetch, embedded in
        # some (not all) of that response's own content URIs -- it must be
        # captured and reused for every subsequent request tied to that
        # fetch (verified live: a content/nested-tileset URI lacking its own
        # `session=` 400s with just `key=`; reusing the session captured
        # from a SIBLING uri in the same response succeeds). _ensure_params
        # holds both kinds uniformly, keyed by param name.
        self._ensure_params: list[str] = [auth_query] if auth_query else []
        self.tiles: list[dict] = []
        self.visited_tilesets: set[str] = set()
        self.stats = {"visited": 0, "pruned": 0, "downloaded": 0, "skipped_no_content": 0}

    def _set_ensure_param(self, param: str) -> None:
        name = param.split("=", 1)[0]
        self._ensure_params = [p for p in self._ensure_params if p.split("=", 1)[0] != name]
        self._ensure_params.append(param)

    def _auth_url(self, url: str) -> str:
        for param in self._ensure_params:
            name = param.split("=", 1)[0]
            if f"{name}=" not in url:
                sep = "&" if "?" in url else "?"
                url = f"{url}{sep}{param}"
        return url

    @staticmethod
    def _find_session_token(tile: dict) -> str | None:
        contents = tile.get("contents")
        if contents is None and "content" in tile:
            contents = [tile["content"]]
        for c in contents or []:
            q = parse_qs(urlparse((c or {}).get("uri", "")).query)
            if "session" in q:
                return q["session"][0]
        for child in tile.get("children", []):
            found = Walker._find_session_token(child)
            if found:
                return found
        return None

    def walk_tileset(self, tileset_url: str, base_transform: np.ndarray) -> None:
        if tileset_url in self.visited_tilesets:
            return  # avoid infinite loops on a malformed/self-referential tileset
        self.visited_tilesets.add(tileset_url)
        doc = fetch_json(self._auth_url(tileset_url), self.session)
        root = doc.get("root")
        if root is None:
            return
        session_token = self._find_session_token(root)
        if session_token:
            self._set_ensure_param(f"session={session_token}")
        self._walk_tile(root, base_transform, tileset_url)

    def _walk_tile(self, tile: dict, parent_transform: np.ndarray, base_url: str) -> None:
        self.stats["visited"] += 1
        transform = parent_transform
        if "transform" in tile:
            transform = parent_transform @ parse_gltf_transform(tile["transform"])

        bv = tile.get("boundingVolume", {})
        if not bounding_volume_overlaps_aoi(bv, transform, self.aoi_bbox, self.aoi_sphere):
            self.stats["pruned"] += 1
            return  # outside the AOI -- prune this whole subtree

        geometric_error = tile.get("geometricError", 0.0)
        children = tile.get("children", [])

        contents = tile.get("contents")
        if contents is None and "content" in tile:
            contents = [tile["content"]]
        contents = contents or []

        # A tile with no content of its own (e.g. a pure LOD/grouping node,
        # like this sample's root) has nothing to stop on regardless of its
        # geometricError -- must recurse into children or there's nothing to
        # render at all. Only a content-bearing tile can be "detailed enough"
        # to stop on.
        if contents and (geometric_error <= self.detail_m or not children):
            for content in contents:
                self._handle_content(content, transform, base_url)
        elif children:
            for child in children:
                self._walk_tile(child, transform, base_url)
        else:
            self.stats["skipped_no_content"] += 1

    def _handle_content(self, content: dict, transform: np.ndarray, base_url: str) -> None:
        uri = content.get("uri") or content.get("url")
        if not uri:
            return
        content_url = urljoin(base_url, uri)
        if strip_query(content_url).endswith(".json"):
            # External tileset reference -- recurse into it with the
            # transform accumulated so far as its new base. walk_tileset()
            # applies auth itself.
            self.walk_tileset(content_url, transform)
            return

        try:
            data = fetch_bytes(self._auth_url(content_url), self.session)
        except requests.RequestException as e:
            print(f"  ! failed to fetch {strip_query(content_url)}: {e}", file=sys.stderr)
            return
        data = unwrap_b3dm(data)
        if data[0:4] not in (b"glTF", b"b3dm"):
            # Not a glb/b3dm we can use (e.g. .pnts point cloud, .i3dm
            # instanced model) -- skip; v1 targets mesh content only.
            print(f"  - skipping non-glb content: {strip_query(content_url)}", file=sys.stderr)
            return

        filename = f"tile_{len(self.tiles):05d}.glb"
        (self.out_dir / filename).write_bytes(data)
        xform = local_transform_to_godot(transform, self.origin_lat, self.origin_lon, self.origin_alt)
        self.tiles.append({"file": filename, "source_url": strip_query(content_url), **xform})
        self.stats["downloaded"] += 1
        print(f"  + {filename}  <-  {strip_query(content_url)}")


def resolve_ion_asset(asset_id: int, ion_token: str, session: requests.Session) -> tuple[str, str | None, list[str]]:
    """Returns (tileset_root_url, auth_query, attribution_html) --
    auth_query is a ready-made "param=value" string to append to every
    subsequent request (tileset.json AND content fetches alike), or None if
    the tileset needs no further auth. attribution_html is the list of
    attribution snippets ion's own endpoint response supplies for this asset
    -- Cesium ion's and Google's terms REQUIRE these be displayed wherever
    the content itself is shown to anyone besides you, not just recorded
    for the record; see the manifest's `content_attributions` field and
    GWTiles3DLoader's `attributions` property.

    Two response shapes exist, verified live against a real ion account:
      - a normal ion-hosted asset: {"url": ..., "accessToken": ...} -- the
        token rides as `access_token=` on every request.
      - an externally-proxied asset (e.g. Google Photorealistic 3D Tiles,
        `externalType: "3DTILES"`): {"options": {"url": "...?key=..."}} --
        no separate accessToken; the SAME `key=` query param from this URL
        must be re-appended to every subsequent request (tileset.json and
        content), including ones whose own URI already carries an unrelated
        `session=` param -- a content URI's `session=` alone 403s without it.
    """
    url = ION_ENDPOINT.format(asset_id=asset_id) + f"?access_token={ion_token}"
    endpoint = fetch_json(url, session)
    tileset_url = endpoint.get("url") or endpoint.get("options", {}).get("url")
    if not tileset_url:
        raise RuntimeError(f"ion endpoint response has no tileset url (keys: {sorted(endpoint.keys())})")
    attribution_html = [a["html"] for a in endpoint.get("attributions", []) if "html" in a]
    tile_token = endpoint.get("accessToken")
    if tile_token:
        return tileset_url, f"access_token={tile_token}", attribution_html
    if "?" in tileset_url:
        return tileset_url, urlparse(tileset_url).query, attribution_html
    return tileset_url, None, attribution_html


def purge_stale_caches(max_age_hours: float) -> None:
    """Delete every .cache/3dtiles/*/ subdirectory whose manifest.json
    `fetched_at` is older than max_age_hours (or that has no readable
    manifest at all -- a leftover from an interrupted run is exactly the
    kind of thing that shouldn't linger either). Session-scoped caching only
    works if something actually enforces the "session" part; this is that
    enforcement, meant to be run periodically (e.g. at the start of a dev
    session) rather than trusted to happen on its own."""
    root = Path(".cache/3dtiles")
    if not root.is_dir():
        print("No .cache/3dtiles/ directory -- nothing to purge.")
        return
    now = datetime.datetime.now(datetime.timezone.utc)
    removed = 0
    for entry in sorted(root.iterdir()):
        if not entry.is_dir():
            continue
        manifest_path = entry / "manifest.json"
        stale = True
        age_desc = "no manifest.json"
        if manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text())
                fetched_at = datetime.datetime.fromisoformat(manifest["fetched_at"])
                age_hours = (now - fetched_at).total_seconds() / 3600.0
                stale = age_hours > max_age_hours
                age_desc = f"{age_hours:.1f}h old"
            except (KeyError, ValueError, json.JSONDecodeError) as e:
                age_desc = f"unreadable manifest ({e})"
        if stale:
            shutil.rmtree(entry)
            print(f"  removed {entry}  ({age_desc})")
            removed += 1
        else:
            print(f"  kept    {entry}  ({age_desc}, within {max_age_hours}h)")
    print(f"Purged {removed} stale cache director{'y' if removed == 1 else 'ies'}.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, help="AOI center latitude (deg)")
    ap.add_argument("--lon", type=float, help="AOI center longitude (deg)")
    ap.add_argument("--alt", type=float, default=0.0, help="AOI center altitude, meters above WGS84 ellipsoid (default 0)")
    ap.add_argument("--radius-km", type=float, default=2.0, help="prefetch radius around lat/lon (default 2km)")
    ap.add_argument("--detail-m", type=float, default=50.0,
        help="stop recursing once a tile's geometricError drops to this many meters or below (default 50)")
    ap.add_argument("--asset-id", type=int, help="Cesium ion asset id (e.g. 2275207 = Google Photorealistic 3D Tiles)")
    ap.add_argument("--ion-token", help="Cesium ion access token (or set CESIUM_ION_TOKEN env var)")
    ap.add_argument("--tileset-url", help="fetch this tileset.json directly instead of resolving a Cesium ion asset "
        "(for public/unauthenticated tilesets, e.g. Cesium's 3d-tiles-samples repo)")
    ap.add_argument("--out", default=None, help="cache output dir (default: .cache/3dtiles/<asset or host>_<lat>_<lon>/)")
    ap.add_argument("--purge-stale-hours", type=float, default=None, metavar="HOURS",
        help="don't fetch anything -- delete every .cache/3dtiles/ subdirectory whose manifest.json is older than "
        "HOURS (or unreadable), then exit. Run this periodically; caching is session-scoped on purpose (see the "
        "module docstring), and nothing else enforces that on its own.")
    args = ap.parse_args()

    if args.purge_stale_hours is not None:
        purge_stale_caches(args.purge_stale_hours)
        return

    ion_token = args.ion_token or os.environ.get("CESIUM_ION_TOKEN")

    if args.lat is None or args.lon is None:
        ap.error("--lat and --lon are required (unless using --purge-stale-hours)")
    if not args.tileset_url and not (args.asset_id and ion_token):
        ap.error("pass --tileset-url, or both --asset-id and --ion-token (or CESIUM_ION_TOKEN)")

    session = requests.Session()
    auth_query = None
    attribution_html: list[str] = []
    if args.tileset_url:
        tileset_url = args.tileset_url
        label = urlparse(tileset_url).netloc.replace(".", "-")
    else:
        print(f"Resolving Cesium ion asset {args.asset_id}...")
        tileset_url, auth_query, attribution_html = resolve_ion_asset(args.asset_id, ion_token, session)
        label = f"asset{args.asset_id}"
        if attribution_html:
            print("\nATTRIBUTION REQUIRED -- Cesium ion's and this content provider's terms require displaying")
            print("this wherever the content itself is shown to anyone besides you (not just recorded for the")
            print("record). It's also saved in the manifest's `content_attributions` field.")
            for html in attribution_html:
                print(f"  {html}")
            print()

    # Google Photorealistic 3D Tiles bakes each tile's real-world anchor
    # directly into its glTF content's own root node matrix (the tileset.json
    # transform chain stays identity throughout -- there's no `transform`
    # field anywhere), and that matrix's translation uses a DIFFERENT ECEF
    # axis convention than the standard one everything else here uses:
    # verified live by brute-force axis-permutation search against a real
    # downloaded tile's known location -- Google's (X,Y,Z) = standard ECEF's
    # (X, Z, -Y), i.e. Y points at the north pole instead of Z. Godot loads
    # that matrix raw (it has no idea it's "ECEF" at all), so
    # Tiles3DLoader.gd must apply this fixed correction to the WRAPPER's
    # rotation -- not the translation, which always comes from the tileset
    # transform chain (standard ECEF, spec-guaranteed) -- whenever content
    # came from this host. Recorded once per fetch, not per tile, since it's
    # a fixed property of the provider, not of any individual tile.
    content_axis_correction = "google_yup_ecef" if urlparse(tileset_url).netloc == "tile.googleapis.com" else None

    out_dir = Path(args.out) if args.out else Path(".cache/3dtiles") / f"{label}_{args.lat:.5f}_{args.lon:.5f}"
    out_dir.mkdir(parents=True, exist_ok=True)

    aoi_bbox = bbox_from_center(args.lat, args.lon, args.radius_km)
    aoi_sphere = aoi_ecef_sphere(args.lat, args.lon, args.alt, args.radius_km)
    print(f"AOI: center=({args.lat}, {args.lon}) radius={args.radius_km}km bbox={aoi_bbox}")
    print(f"Tileset: {tileset_url}")
    print(f"Cache dir: {out_dir}  (gitignored -- session-scoped, do not commit)")

    walker = Walker(session, aoi_bbox, aoi_sphere, args.detail_m, out_dir, args.lat, args.lon, args.alt, auth_query)
    walker.walk_tileset(tileset_url, IDENTITY_4X4)  # walk_tileset applies auth itself

    manifest = {
        "fetched_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "source_tileset": strip_query(tileset_url),
        "asset_id": args.asset_id,
        "origin_lat": args.lat,
        "origin_lon": args.lon,
        "origin_alt": args.alt,
        "radius_km": args.radius_km,
        "detail_m": args.detail_m,
        "content_axis_correction": content_axis_correction,
        "content_attributions": attribution_html,
        "tiles": walker.tiles,
    }
    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))

    s = walker.stats
    print(f"\nDone: {s['downloaded']} tiles downloaded, {s['pruned']} subtrees pruned (outside AOI), "
        f"{s['skipped_no_content']} tiles skipped (no content at this depth), {s['visited']} tiles visited total.")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
