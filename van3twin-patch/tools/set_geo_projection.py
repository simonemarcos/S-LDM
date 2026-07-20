#!/usr/bin/env python3
"""
set_geo_projection.py - Set a real geo-projection in a SUMO .net.xml file.

This script fixes the "dummy" x/y → lat/lon conversion in SUMO networks
that have projParameter="!" (no projection). It sets up a Transverse Mercator
(TM) projection centered on user-provided coordinates.

HOW IT WORKS:
=============
SUMO's .net.xml contains a <location> element with these key attributes:
  - projParameter: a PROJ.4 string defining the map projection
  - netOffset: "dx,dy" offset applied as: network_coords = projected_coords + netOffset
  - origBoundary: "lon_min,lat_min,lon_max,lat_max" geographic bounding box
  - convBoundary: "x_min,y_min,x_max,y_max" network coordinate bounding box

When projParameter="!", SUMO performs an identity conversion (lon=x, lat=y).
This script replaces it with a proper TM projection.

THE MATH:
=========
1. TM projection: +proj=tmerc +lon_0=central_lon +lat_0=central_lat
   - Projecting (central_lon, central_lat) gives (0, 0) in meters
   - X axis = easting (meters east from central meridian)
   - Y axis = northing (meters north from latitude origin)

2. netOffset computation:
   - We want the center of the x/y network to correspond to (central_lon, central_lat)
   - Since proj(central_lon, central_lat) = (0, 0), and
     network_center = projected_center + netOffset, then:
   - netOffset = (x_center, y_center) of the network

3. origBoundary computation:
   - For each corner of convBoundary, compute:
     projected = network_corner - netOffset
     (lon, lat) = inverse_proj(projected)
   - This gives the geographic bounding box

USAGE:
======
  python3 set_geo_projection.py --net-file <path.net.xml> \\
      --central-lat <lat> --central-lon <lon>

The script prints the derived bounding box (lat_min, lat_max, lon_min, lon_max)
which you can use for the inverse TM conversion in downstream processing.
"""

import argparse
import re
import sys

from pyproj import Transformer


def parse_location(xml_content: str):
    """Extract the <location ...> element and parse its attributes."""
    match = re.search(r'<location\s+([^/]*)/>', xml_content)
    if not match:
        raise ValueError("No <location .../> element found in the .net.xml file.")

    attrs_str = match.group(1)

    def get_attr(name):
        m = re.search(rf'{name}\s*=\s*"([^"]*)"', attrs_str)
        return m.group(1) if m else None

    conv_boundary_str = get_attr("convBoundary")
    if not conv_boundary_str:
        raise ValueError("convBoundary attribute not found in <location> element.")

    parts = [float(x) for x in conv_boundary_str.split(",")]
    conv_boundary = {
        "x_min": parts[0], "y_min": parts[1],
        "x_max": parts[2], "y_max": parts[3],
    }

    return match.group(0), conv_boundary


def compute_projection(central_lat: float, central_lon: float, conv_boundary: dict):
    """
    Compute TM projection parameters, netOffset, and origBoundary.

    Returns:
        proj_string: PROJ.4 string for the TM projection
        net_offset: (offset_x, offset_y)
        orig_boundary: (lon_min, lat_min, lon_max, lat_max)
    """
    # --- 1. Build the PROJ.4 string ---
    proj_string = (
        f"+proj=tmerc +lon_0={central_lon} +lat_0={central_lat} "
        f"+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
    )

    # --- 2. Compute netOffset ---
    # Network center (from convBoundary)
    x_center = (conv_boundary["x_min"] + conv_boundary["x_max"]) / 2.0
    y_center = (conv_boundary["y_min"] + conv_boundary["y_max"]) / 2.0

    # TM projection of (central_lon, central_lat) = (0, 0) by definition.
    # SUMO convention: network = projected + netOffset
    # We want: network_center = proj(central_lon, central_lat) + netOffset
    #   (x_center, y_center) = (0, 0) + (offset_x, offset_y)
    net_offset = (x_center, y_center)

    # --- 3. Compute origBoundary ---
    # projected = network - netOffset
    # Then inverse-project to get (lon, lat)
    proj_to_lonlat = Transformer.from_proj(proj_string, "EPSG:4326", always_xy=True)

    # Corner 1: (x_min, y_min) in network
    proj_x1 = conv_boundary["x_min"] - net_offset[0]
    proj_y1 = conv_boundary["y_min"] - net_offset[1]
    lon1, lat1 = proj_to_lonlat.transform(proj_x1, proj_y1)

    # Corner 2: (x_max, y_max) in network
    proj_x2 = conv_boundary["x_max"] - net_offset[0]
    proj_y2 = conv_boundary["y_max"] - net_offset[1]
    lon2, lat2 = proj_to_lonlat.transform(proj_x2, proj_y2)

    orig_boundary = (
        min(lon1, lon2), min(lat1, lat2),
        max(lon1, lon2), max(lat1, lat2),
    )

    return proj_string, net_offset, orig_boundary


def update_net_xml(filepath: str, central_lat: float, central_lon: float):
    """Update the <location> element in the .net.xml file."""
    with open(filepath, "r") as f:
        content = f.read()

    old_location, conv_boundary = parse_location(content)
    proj_string, net_offset, orig_boundary = compute_projection(
        central_lat, central_lon, conv_boundary
    )

    # Build the new <location> element
    conv_boundary_str = (
        f"{conv_boundary['x_min']:.2f},{conv_boundary['y_min']:.2f},"
        f"{conv_boundary['x_max']:.2f},{conv_boundary['y_max']:.2f}"
    )
    new_location = (
        f'<location netOffset="{net_offset[0]:.2f},{net_offset[1]:.2f}" '
        f'convBoundary="{conv_boundary_str}" '
        f'origBoundary="{orig_boundary[0]:.6f},{orig_boundary[1]:.6f},'
        f'{orig_boundary[2]:.6f},{orig_boundary[3]:.6f}" '
        f'projParameter="{proj_string}"/>'
    )

    # Replace in file
    new_content = content.replace(old_location, new_location)

    with open(filepath, "w") as f:
        f.write(new_content)

    # --- Print summary ---
    print("=" * 70)
    print("GEO-PROJECTION CONFIGURED SUCCESSFULLY")
    print("=" * 70)
    print()
    print(f"  Central point:  lat={central_lat}, lon={central_lon}")
    print(f"  Projection:     Transverse Mercator (TM)")
    print(f"  PROJ.4 string:  {proj_string}")
    print()
    print(f"  netOffset:      ({net_offset[0]:.2f}, {net_offset[1]:.2f})")
    print(f"  convBoundary:   {conv_boundary_str}  (x/y, unchanged)")
    print(f"  origBoundary:   {orig_boundary[0]:.6f},{orig_boundary[1]:.6f},"
          f"{orig_boundary[2]:.6f},{orig_boundary[3]:.6f}")
    print()
    print("  Derived geographic bounding box:")
    print(f"    lon_min = {orig_boundary[0]:.6f}")
    print(f"    lat_min = {orig_boundary[1]:.6f}")
    print(f"    lon_max = {orig_boundary[2]:.6f}")
    print(f"    lat_max = {orig_boundary[3]:.6f}")
    print(f"    central_lon = {central_lon}")
    print(f"    central_lat = {central_lat}")
    print()
    print("  Use these values for inverse TM conversion (lat/lon → x/y):")
    print(f"    lon_0 = {central_lon}  (central meridian for TM)")
    print(f"    lat_0 = {central_lat}  (latitude of origin for TM)")
    print()

    # Verify: project central point and check it maps to network center
    lonlat_to_proj = Transformer.from_proj("EPSG:4326", proj_string, always_xy=True)
    px, py = lonlat_to_proj.transform(central_lon, central_lat)
    net_x = px + net_offset[0]
    net_y = py + net_offset[1]
    x_center = (conv_boundary["x_min"] + conv_boundary["x_max"]) / 2.0
    y_center = (conv_boundary["y_min"] + conv_boundary["y_max"]) / 2.0

    print("  Verification:")
    print(f"    proj(central_lon, central_lat) = ({px:.4f}, {py:.4f})  [should be ~(0, 0)]")
    print(f"    network center = ({net_x:.2f}, {net_y:.2f})  [should be ({x_center:.2f}, {y_center:.2f})]")
    print()
    print(f"  File updated: {filepath}")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description="Set a real TM geo-projection in a SUMO .net.xml file."
    )
    parser.add_argument(
        "--net-file", required=True,
        help="Path to the SUMO .net.xml file to modify."
    )
    parser.add_argument(
        "--central-lat", type=float, required=True,
        help="Latitude of the geographic center of the map."
    )
    parser.add_argument(
        "--central-lon", type=float, required=True,
        help="Longitude of the geographic center of the map."
    )
    args = parser.parse_args()

    update_net_xml(args.net_file, args.central_lat, args.central_lon)


if __name__ == "__main__":
    main()
