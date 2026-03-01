#!/usr/bin/env python3
"""Export simplified STL meshes from the HP Robots Otto Starter STEP file.

Usage:
    python3 scripts/export_meshes.py [/path/to/step/file]

Default STEP path: ~/Downloads/HPRobots_Ottostarter(1).step

Outputs to: ros2_ws/src/otto_description/meshes/
Units are converted from mm (STEP) to meters (ROS convention).
"""

import sys
import os
import cadquery as cq

# Paths
DEFAULT_STEP = os.path.expanduser("~/Downloads/HPRobots_Ottostarter(1).step")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
MESH_DIR = os.path.join(PROJECT_ROOT, "ros2_ws", "src", "otto_description", "meshes")

# Scale factor: mm to meters
MM_TO_M = 0.001


def identify_parts(solids):
    """Identify parts by their bounding box dimensions (mm).

    Based on inspection of the STEP file:
      Solid 1  (72x72x24mm):    Bottom plate
      Solid 9  (72x72x35.5mm):  Middle section
      Solid 4  (72x72x19mm):    Top/lights
      Solid 6  (62x11.4x32mm):  Face/ultrasonic mount
      Solid 0  (29x22x14mm):    PCB (not exported as mesh)
      Solid 2  (5.5x49x49mm):   Left wheel (o-ring tire)
      Solid 10 (5.5x49x49mm):   Right wheel (o-ring tire)
      Solid 5  (8x42x42mm):     Right servo bracket
      Solid 7  (8x42x42mm):     Left servo bracket
      Solid 3  (12x12x12mm):    Ball caster
      Solid 8  (15x22x9.5mm):   Caster housing
    """
    parts = {
        "body_bottom": [],
        "body_middle": [],
        "body_top": [],
        "face": [],
        "wheel_left": [],
        "wheel_right": [],
        "bracket_left": [],
        "bracket_right": [],
        "caster_ball": [],
        "caster_housing": [],
        "pcb": [],
    }

    for i, solid in enumerate(solids):
        bb = solid.BoundingBox()
        w, d, h = bb.xlen, bb.ylen, bb.zlen

        # Classify by size heuristics
        if 70 < w < 75 and 70 < d < 75:
            if h < 25:
                parts["body_bottom"].append(solid)
            elif h > 30:
                parts["body_middle"].append(solid)
            else:
                parts["body_top"].append(solid)
        elif 60 < w < 65 and d < 15:
            parts["face"].append(solid)
        elif w < 7 and 45 < d < 55 and 45 < h < 55:
            # O-ring tire — distinguish left vs right by x position
            cx = (bb.xmin + bb.xmax) / 2
            if cx < 0:
                parts["wheel_left"].append(solid)
            else:
                parts["wheel_right"].append(solid)
        elif 6 < w < 10 and 40 < d < 45 and 40 < h < 45:
            # Servo bracket — distinguish by x position
            cx = (bb.xmin + bb.xmax) / 2
            if cx < 0:
                parts["bracket_left"].append(solid)
            else:
                parts["bracket_right"].append(solid)
        elif 10 < w < 14 and 10 < d < 14 and 10 < h < 14:
            parts["caster_ball"].append(solid)
        elif 13 < w < 17 and 20 < d < 24:
            parts["caster_housing"].append(solid)
        elif 25 < w < 32 and 20 < d < 25:
            parts["pcb"].append(solid)
        else:
            print(f"  Unclassified solid {i}: {w:.1f} x {d:.1f} x {h:.1f} mm")

    return parts


def export_mesh(solids, name, mesh_dir, tolerance=0.5):
    """Export a list of CadQuery solids as a single STL file in meters."""
    if not solids:
        print(f"  Skipping {name}: no solids found")
        return

    # Combine multiple solids into one compound if needed
    if len(solids) == 1:
        shape = solids[0]
    else:
        shape = solids[0]
        for s in solids[1:]:
            shape = shape.fuse(s)

    filepath = os.path.join(mesh_dir, f"{name}.stl")

    # Export with tessellation tolerance (lower = more triangles)
    cq.exporters.export(
        cq.Workplane().add(shape),
        filepath,
        exportType="STL",
        tolerance=tolerance,
        angularTolerance=0.2,
    )

    # Read back and scale to meters
    with open(filepath, "rb") as f:
        data = f.read()

    # STL binary format: 80-byte header, 4-byte triangle count, then triangles
    # Each triangle: 12 floats (normal + 3 vertices) + 2 byte attribute
    # We need to scale all vertex coordinates by MM_TO_M
    import struct

    if len(data) < 84:
        print(f"  {name}: STL too small, skipping scale")
        return

    header = data[:80]
    num_triangles = struct.unpack_from("<I", data, 80)[0]
    print(f"  {name}: {num_triangles} triangles")

    offset = 84
    scaled = bytearray(data[:84])
    for _ in range(num_triangles):
        # Normal vector (3 floats) — keep as-is (direction doesn't scale)
        nx, ny, nz = struct.unpack_from("<fff", data, offset)
        scaled.extend(struct.pack("<fff", nx, ny, nz))
        offset += 12

        # 3 vertices — scale each by MM_TO_M
        for _ in range(3):
            x, y, z = struct.unpack_from("<fff", data, offset)
            scaled.extend(struct.pack("<fff", x * MM_TO_M, y * MM_TO_M, z * MM_TO_M))
            offset += 12

        # Attribute byte count
        attr = struct.unpack_from("<H", data, offset)[0]
        scaled.extend(struct.pack("<H", attr))
        offset += 2

    with open(filepath, "wb") as f:
        f.write(bytes(scaled))

    file_size = os.path.getsize(filepath)
    print(f"  {name}: saved ({file_size / 1024:.1f} KB)")


def main():
    step_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STEP

    if not os.path.exists(step_path):
        print(f"STEP file not found: {step_path}")
        sys.exit(1)

    os.makedirs(MESH_DIR, exist_ok=True)

    print(f"Loading STEP file: {step_path}")
    result = cq.importers.importStep(step_path)
    solids = list(result.solids())
    print(f"Found {len(solids)} solids")

    print("\nIdentifying parts...")
    parts = identify_parts(solids)

    print("\nExporting meshes (mm → meters)...")

    # Body: combine bottom + middle + top into single mesh
    body_solids = parts["body_bottom"] + parts["body_middle"] + parts["body_top"]
    export_mesh(body_solids, "body", MESH_DIR, tolerance=0.5)

    # Face (ultrasonic mount)
    export_mesh(parts["face"], "face", MESH_DIR, tolerance=0.3)

    # Wheels (export one, reuse for both sides in URDF with mirroring)
    wheel_solids = parts["wheel_left"] or parts["wheel_right"]
    export_mesh(wheel_solids, "wheel", MESH_DIR, tolerance=0.2)

    # Caster
    caster_solids = parts["caster_ball"] + parts["caster_housing"]
    export_mesh(caster_solids, "caster", MESH_DIR, tolerance=0.3)

    # Brackets (export one, reuse)
    bracket_solids = parts["bracket_left"] or parts["bracket_right"]
    export_mesh(bracket_solids, "bracket", MESH_DIR, tolerance=0.3)

    print(f"\nDone! Meshes written to: {MESH_DIR}")


if __name__ == "__main__":
    main()
