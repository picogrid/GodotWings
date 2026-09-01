#!/usr/bin/env python3
"""Bake a real-world terrain tile (satellite imagery + elevation) into files
GWImportedTerrain.gd can load, from entirely open, no-registration-required
data: Sentinel-2 L2A true-color imagery and Copernicus DEM GLO-30 elevation,
both pulled from public AWS Open Data buckets via Element84's STAC API.

This is an offline BAKE step, not a runtime dependency — Godot never touches
the network for this. Output is a 16-bit heightmap PNG, an RGB texture PNG,
and a small JSON sidecar with the geodetic metadata needed to align it: the
mesh is built in a local azimuthal-equidistant (AEQD) projection centered
EXACTLY on --lat/--lon, which is the geospatially rigorous version of the same
flat-tangent-plane approximation GWCoordConvert/GWGeoReference already use
elsewhere in this project (see their docstrings) — so a vehicle spawning at
NED (0,0), which is where GWVehicleBody spawns by default, lands exactly at
the tile's center: your AOI's center IS the takeoff point, with no extra glue.

    python3 -m venv .venv && source .venv/bin/activate
    pip install rasterio numpy pillow requests
    python3 gw_terrain_import.py --lat 60.221088825593135 --lon 25.018208331290666 \
        --size-km 4 --out terrain_helsinki

Prints a suggested HOME_LOCATION line at the end — paste it into your
docker-compose.yml (or .env) so ArduPilot's GPS origin matches the imagery.

NOT imported by the Godot addon — standalone companion tool, same role as
gw_klv_muxer.py.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rasterio
import requests
from PIL import Image
from rasterio.merge import merge as rio_merge
from rasterio.transform import from_origin
from rasterio.warp import Resampling, reproject

STAC_URL = "https://earth-search.aws.element84.com/v1/search"
DEM_BUCKET = "https://copernicus-dem-30m.s3.amazonaws.com"
R_EARTH = 6378137.0  # WGS84 semi-major axis (m) — matches GWGeoReference.gd


def bbox_from_center(lat: float, lon: float, size_km: float) -> list[float]:
    """Center + size (km) -> [min_lon, min_lat, max_lon, max_lat]. Flat-tangent
    approximation (fine at this scale) — same formula as GWGeoReference.gd."""
    half_m = size_km * 1000.0 / 2.0
    dlat = math.degrees(half_m / R_EARTH)
    dlon = math.degrees(half_m / (R_EARTH * math.cos(math.radians(lat))))
    return [lon - dlon, lat - dlat, lon + dlon, lat + dlat]


def aeqd_crs(lat: float, lon: float) -> str:
    """PROJ4 for an azimuthal-equidistant projection centered at (lat, lon) —
    distances FROM the center are exact, which is exactly the local-flat-plane
    behavior this project's own NED tangent-plane approximation wants, just
    done with a real, GDAL-recognized projection instead of a hand-rolled one."""
    return f"+proj=aeqd +lat_0={lat} +lon_0={lon} +datum=WGS84 +units=m +no_defs"


def stac_search_best_scene(bbox: list[float], cloud_lt: float, datetime_range: str) -> dict:
    resp = requests.post(
        STAC_URL,
        json={
            "collections": ["sentinel-2-l2a"],
            "bbox": bbox,
            "datetime": datetime_range,
            "query": {"eo:cloud_cover": {"lt": cloud_lt}},
            "sortby": [{"field": "properties.eo:cloud_cover", "direction": "asc"}],
            "limit": 1,
        },
        timeout=30,
    )
    resp.raise_for_status()
    features = resp.json()["features"]
    if not features:
        raise SystemExit(
            f"No Sentinel-2 L2A scene found over {bbox} with cloud cover < {cloud_lt}% "
            f"in {datetime_range}. Try --cloud-cover-lt higher or --datetime a wider range."
        )
    return features[0]


def dem_tile_ids_for_bbox(bbox: list[float]) -> list[str]:
    """Copernicus DEM GLO-30 is tiled in 1x1 degree cells named by their SW
    corner, e.g. Copernicus_DSM_COG_10_N60_00_E025_00_DEM. Returns every tile
    id whose cell overlaps `bbox` (usually 1, up to 4 near a degree boundary)."""
    min_lon, min_lat, max_lon, max_lat = bbox
    tiles = set()
    for lat_i in range(math.floor(min_lat), math.floor(max_lat) + 1):
        for lon_i in range(math.floor(min_lon), math.floor(max_lon) + 1):
            ns, ew = ("N" if lat_i >= 0 else "S"), ("E" if lon_i >= 0 else "W")
            tiles.add(f"Copernicus_DSM_COG_10_{ns}{abs(lat_i):02d}_00_{ew}{abs(lon_i):03d}_00_DEM")
    return sorted(tiles)


def fetch_dem_mosaic(bbox: list[float]):
    """Open every Copernicus DEM tile touching `bbox` and merge them into one
    in-memory dataset (rasterio MemoryFile), cropped to `bbox`. Returns
    (array, transform, crs)."""
    tile_ids = dem_tile_ids_for_bbox(bbox)
    datasets = []
    opened = []
    for tid in tile_ids:
        url = f"{DEM_BUCKET}/{tid}/{tid}.tif"
        try:
            ds = rasterio.open(url)
            opened.append(ds)
            datasets.append(ds)
        except rasterio.errors.RasterioIOError:
            print(f"gw_terrain_import: warning: DEM tile {tid} not found (ocean/no data?), skipping", file=sys.stderr)
    if not datasets:
        raise SystemExit(f"No Copernicus DEM tiles found for bbox {bbox}.")
    mosaic, out_transform = rio_merge(datasets, bounds=bbox)
    crs = datasets[0].crs
    for ds in opened:
        ds.close()
    return mosaic[0], out_transform, crs, tile_ids


def reproject_band(src_array, src_transform, src_crs, dst_crs, dst_transform,
        dst_width, dst_height, resampling, src_count=1, dtype=None) -> np.ndarray:
    dtype = dtype or src_array.dtype
    if src_array.ndim == 2:
        src_array = src_array[np.newaxis, ...]
        src_count = 1
    dst = np.zeros((src_count, dst_height, dst_width), dtype=dtype)
    reproject(
        source=src_array,
        destination=dst,
        src_transform=src_transform,
        src_crs=src_crs,
        dst_transform=dst_transform,
        dst_crs=dst_crs,
        resampling=resampling,
    )
    return dst


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, required=True, help="AOI center latitude — becomes the SITL takeoff point.")
    ap.add_argument("--lon", type=float, required=True, help="AOI center longitude.")
    ap.add_argument("--size-km", type=float, default=4.0, help="Square AOI edge length, km.")
    ap.add_argument("--out", required=True, help="Output basename — writes <out>_heightmap.png, "
            "<out>_texture.png, <out>.json.")
    ap.add_argument("--dem-resolution-m", type=float, default=30.0,
            help="Heightmap grid spacing, meters/pixel. 30 matches Copernicus DEM's native resolution.")
    ap.add_argument("--texture-resolution-m", type=float, default=None,
            help="Texture grid spacing, meters/pixel. Default: the source imagery's native 10m, "
            "capped so the texture doesn't exceed --max-texture-px per side.")
    ap.add_argument("--max-texture-px", type=int, default=4096)
    ap.add_argument("--cloud-cover-lt", type=float, default=20.0, help="Max acceptable cloud cover, percent.")
    ap.add_argument("--datetime", default="2023-01-01T00:00:00Z/2026-12-31T00:00:00Z",
            help="STAC datetime range to search, RFC3339/interval.")
    args = ap.parse_args()

    bbox = bbox_from_center(args.lat, args.lon, args.size_km)
    print(f"gw_terrain_import: AOI bbox = {bbox}", file=sys.stderr)

    print("gw_terrain_import: searching Sentinel-2 L2A (Element84 STAC)...", file=sys.stderr)
    scene = stac_search_best_scene(bbox, args.cloud_cover_lt, args.datetime)
    cloud = scene["properties"].get("eo:cloud_cover")
    print(f"gw_terrain_import: using {scene['id']} ({scene['properties']['datetime']}, "
            f"cloud cover {cloud:.2f}%)", file=sys.stderr)
    visual_href = scene["assets"]["visual"]["href"]

    print("gw_terrain_import: fetching Copernicus DEM GLO-30...", file=sys.stderr)
    dem_arr, dem_transform, dem_crs, dem_tiles = fetch_dem_mosaic(bbox)
    print(f"gw_terrain_import: DEM tiles: {', '.join(dem_tiles)}", file=sys.stderr)

    dst_crs = aeqd_crs(args.lat, args.lon)
    half_m = args.size_km * 1000.0 / 2.0

    # --- heightmap: reproject DEM onto the local AEQD grid ---
    dem_px = max(2, round((args.size_km * 1000.0) / args.dem_resolution_m))
    dst_transform_dem = from_origin(-half_m, half_m, args.dem_resolution_m, args.dem_resolution_m)
    height_arr = reproject_band(dem_arr, dem_transform, dem_crs, dst_crs, dst_transform_dem,
            dem_px, dem_px, Resampling.bilinear, dtype=np.float32)[0]

    valid = np.isfinite(height_arr) & (height_arr > -1000)  # DEM nodata is a large negative sentinel
    if not valid.any():
        raise SystemExit("Reprojected heightmap has no valid data — AOI may be outside DEM coverage.")
    elev_min, elev_max = float(height_arr[valid].min()), float(height_arr[valid].max())
    if elev_max - elev_min < 1e-3:
        elev_max = elev_min + 1.0  # avoid a degenerate 0-range normalization on dead-flat terrain
    # Center elevation, read off the reprojected grid's middle cell -- this is
    # what "spawn sits at real ground level" pins to.
    center_row, center_col = dem_px // 2, dem_px // 2
    elev_center = float(height_arr[center_row, center_col]) if valid[center_row, center_col] else \
            float(np.nanmean(height_arr[valid]))

    height_norm = np.where(valid, (height_arr - elev_min) / (elev_max - elev_min), 0.0)
    # Godot's Image has no true 16-bit-per-channel format — Image.load() on a
    # 16-bit grayscale PNG silently truncates to FORMAT_L8 (256 levels),
    # verified directly rather than assumed. So the height is packed across
    # the R (high byte) and G (low byte) channels of an ordinary RGB8 PNG
    # instead, which Godot loads at full, exact 8-bit-per-channel precision:
    # height16 = round(r*255)*256 + round(g*255). B is unused (0).
    height16 = np.clip(np.round(height_norm * 65535.0), 0, 65535).astype(np.uint16)
    hi = (height16 >> 8).astype(np.uint8)
    lo = (height16 & 0xFF).astype(np.uint8)
    packed = np.stack([hi, lo, np.zeros_like(hi)], axis=-1)
    Image.fromarray(packed, mode="RGB").save(f"{args.out}_heightmap.png")

    # --- texture: reproject the Sentinel-2 true-color visual onto the same grid ---
    tex_res_m = args.texture_resolution_m
    if tex_res_m is None:
        tex_res_m = max(10.0, (args.size_km * 1000.0) / args.max_texture_px)
    tex_px = min(args.max_texture_px, max(2, round((args.size_km * 1000.0) / tex_res_m)))
    dst_transform_tex = from_origin(-half_m, half_m, args.size_km * 1000.0 / tex_px, args.size_km * 1000.0 / tex_px)

    with rasterio.open(visual_href) as vsrc:
        src_rgb = vsrc.read()  # (3, H, W) uint8
        src_transform, src_crs = vsrc.transform, vsrc.crs
    rgb = reproject_band(src_rgb, src_transform, src_crs, dst_crs, dst_transform_tex,
            tex_px, tex_px, Resampling.bilinear, src_count=3, dtype=np.uint8)
    Image.fromarray(np.moveaxis(rgb, 0, -1), mode="RGB").save(f"{args.out}_texture.png")

    meta = {
        "center_lat": args.lat,
        "center_lon": args.lon,
        "size_m": args.size_km * 1000.0,
        "dem_resolution_m": args.dem_resolution_m,
        "heightmap_px": dem_px,
        "texture_px": tex_px,
        "elevation_min_m": elev_min,
        "elevation_max_m": elev_max,
        "elevation_center_m": elev_center,
        "source_sentinel2_id": scene["id"],
        "source_sentinel2_datetime": scene["properties"]["datetime"],
        "source_sentinel2_cloud_cover_pct": cloud,
        "source_dem_tiles": dem_tiles,
        "projection": f"AEQD centered at ({args.lat}, {args.lon})",
    }
    with open(f"{args.out}.json", "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\ngw_terrain_import: wrote {args.out}_heightmap.png ({dem_px}x{dem_px}), "
            f"{args.out}_texture.png ({tex_px}x{tex_px}), {args.out}.json", file=sys.stderr)
    print(f"gw_terrain_import: elevation range {elev_min:.1f}..{elev_max:.1f} m, "
            f"center {elev_center:.1f} m", file=sys.stderr)
    print(f"\nDrop a GWImportedTerrain node with:", file=sys.stderr)
    print(f"    heightmap_path = \"{args.out}_heightmap.png\"", file=sys.stderr)
    print(f"    texture_path = \"{args.out}_texture.png\"", file=sys.stderr)
    print(f"    metadata_path = \"{args.out}.json\"", file=sys.stderr)
    print(f"\nAnd set HOME_LOCATION so ArduPilot's GPS origin matches the imagery "
            f"(this is what makes the takeoff point = image center):", file=sys.stderr)
    print(f"    HOME_LOCATION={args.lat},{args.lon},{elev_center:.1f},0", file=sys.stderr)


if __name__ == "__main__":
    main()
