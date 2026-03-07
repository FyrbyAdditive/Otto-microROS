#!/usr/bin/env python3
"""Export STL meshes from the ZGX Otto STEP file.

Usage:
    python3 scripts/export_meshes.py [/path/to/step/file]

Default STEP path: hardware/ZGX Otto.step  (relative to project root)

Outputs to: ros2_ws/src/otto_description/meshes/
Units are converted from mm (STEP) to meters (ROS convention).

STEP coordinate system (ZGX Otto.step):
    X  = transverse (left/right): +X = right side of robot
    Y  = longitudinal: −Y = front of robot (face)
    Z  = vertical: +Z = up
    Wheel axle centre is at STEP Z = −10 mm (cz of all wheel solids)

ROS coordinate system (REP-103):
    X = forward, Y = left, Z = up
    base_link origin = at axle height

Coordinate transform from STEP centroid to ROS URDF origin:
    ros_x = −step_cy * 0.001          (STEP −Y is forward = ROS +X)
    ros_y = −step_cx * 0.001          (STEP −X is left    = ROS +Y)
    ros_z = (step_cz − (−10)) * 0.001 (STEP Z shifted so axle = 0)

Solid classification (determined from STEP inspection):
    body (body.stl):
        Solid 0:  Logo          14×14×1.6    cz= 2.2  (surface feature on Middle)
        Solid 10: Top           72×72×19     cz=38.5
        Solid 11: top cap disc  60×60×2      cz=47
        Solid 12: top disc      21.6×21.6×2  cz=47
        Solid 17: Middle        72×72×35.5   cz=14.25
        Solid 18: Bottom        72×72×24     cz=−12

    face (face.stl):
        Solid  8: Lines sheet   60×60×0.2    cz=45.7
        Solid  9: Lines sheet   60×60×0.2    cz=45.9
        Solid 22: Face plate    66×11×36     cy=−37.5

    wheel_right (wheel_right.stl):
        Solids 2,3,4: cx=+41.65 (3 concentric rings, 8mm thick, 40–49mm diam)

    wheel_left (wheel_left.stl):
        Solids 5,6,7: cx=−41.65

    caster (caster.stl):
        Solid 20: ball          12×12×12     cz=−29
        Solid 21: housing       15×22×9.5    cz=−28.75

    battery_cover (battery_cover.stl):
        Solid 1: panel          50×55×16.6   cy=+6.5 (rear of robot)

    skipped:
        Solid 13–16: ultrasonic transducer pins (tiny, decorative)
        Solid 19:    PCB module (internal component)
"""

import sys
import os
import struct
import cadquery as cq

# Paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_STEP = os.path.join(PROJECT_ROOT, "hardware", "ZGX Otto.step")
MESH_DIR = os.path.join(PROJECT_ROOT, "ros2_ws", "src", "otto_description", "meshes")

MM_TO_M = 0.001
STEP_AXLE_Z = -10.0  # STEP Z coordinate of wheel axle centre (mm)
STEP_AXLE_Y =  23.0  # STEP Y coordinate of wheel axle centre (mm)
                      # = wheel centroid STEP-Y; defines base_link X origin in STEP space


def classify_solids(solids):
    """Classify solids by bounding box into named mesh groups.

    Classification is deterministic based on inspection of ZGX Otto.step.
    See module docstring for full solid list with measured dimensions.
    """
    groups = {
        "body": [],
        "face": [],
        "wheel_right": [],
        "wheel_left": [],
        "caster": [],
        "battery_cover": [],
    }

    for i, solid in enumerate(solids):
        bb = solid.BoundingBox()
        w, d, h = bb.xlen, bb.ylen, bb.zlen
        cx = (bb.xmin + bb.xmax) / 2
        cy = (bb.ymin + bb.ymax) / 2
        cz = (bb.zmin + bb.zmax) / 2

        # Body: large square plates (Top, Middle, Bottom) + small top features + Logo
        if (65 < w < 80 and 65 < d < 80) or (w < 65 and d < 65 and cz > 0 and h < 5 and w > 10):
            groups["body"].append(solid)

        # Face plate (front, cy < −30mm) + thin Lines sheets (h < 0.5mm, wide)
        elif cy < -30 or (h < 0.5 and w > 50 and d > 50):
            groups["face"].append(solid)

        # Wheels: 8mm thick disc stacks (cx ≠ 0, d and h 38–51mm)
        elif w <= 9 and min(d, h) > 38 and max(d, h) < 52:
            if cx > 0:
                groups["wheel_right"].append(solid)
            else:
                groups["wheel_left"].append(solid)

        # Caster: near bottom (cz < −20mm), small
        elif cz < -20 and w < 20 and max(d, h) < 25:
            groups["caster"].append(solid)

        # Battery cover: medium panel at rear (cy > 0), not body size
        elif cy > 0 and 40 < w < 60 and 40 < d < 65 and h < 25:
            groups["battery_cover"].append(solid)

        # Skip everything else (PCB, ultrasonic pins)
        else:
            print(f"  [skip] Solid {i}: {w:.1f}×{d:.1f}×{h:.1f} mm  cx={cx:.1f} cy={cy:.1f} cz={cz:.1f}")

    return groups


def transform_stl(filepath, jx, jy, jz):
    """Transform binary STL from STEP mm-space to ROS meter-space, centered at joint origin.

    Applies the STEP→ROS axis remap and translates vertices so the mesh centroid
    is at the link frame origin.  Joint origin (jx, jy, jz) is in STEP mm.

    Axis mapping:  STEP(x,y,z) - joint  →  ROS(-y, -x, z) * MM_TO_M
    Normals:       same rotation, negated to preserve outward-pointing direction
                   (the STEP→ROS rotation has det = −1, so winding flips).
    """
    with open(filepath, "rb") as f:
        data = f.read()

    if len(data) < 84:
        print(f"  WARNING: {filepath} too small to transform")
        return

    num_triangles = struct.unpack_from("<I", data, 80)[0]
    offset = 84
    out = bytearray(data[:84])

    for _ in range(num_triangles):
        nx, ny, nz = struct.unpack_from("<fff", data, offset)
        # Normal: rotate (STEP→ROS) then negate to restore outward direction
        out.extend(struct.pack("<fff", ny, nx, -nz))
        offset += 12
        for _ in range(3):
            vx, vy, vz = struct.unpack_from("<fff", data, offset)
            rx = -(vy - jy) * MM_TO_M   # ROS +X = STEP −Y
            ry = -(vx - jx) * MM_TO_M   # ROS +Y = STEP −X
            rz =  (vz - jz) * MM_TO_M   # ROS +Z = STEP +Z (relative to joint)
            out.extend(struct.pack("<fff", rx, ry, rz))
            offset += 12
        out.extend(struct.pack("<H", struct.unpack_from("<H", data, offset)[0]))
        offset += 2

    with open(filepath, "wb") as f:
        f.write(bytes(out))


def get_combined_bbox(solids):
    """Return the combined bounding box of a list of solids."""
    bb = solids[0].BoundingBox()
    for s in solids[1:]:
        b = s.BoundingBox()
        bb.xmin = min(bb.xmin, b.xmin)
        bb.xmax = max(bb.xmax, b.xmax)
        bb.ymin = min(bb.ymin, b.ymin)
        bb.ymax = max(bb.ymax, b.ymax)
        bb.zmin = min(bb.zmin, b.zmin)
        bb.zmax = max(bb.zmax, b.zmax)
    return bb


def export_mesh(solids, name, mesh_dir, tolerance=0.5):
    """Export a list of CadQuery solids as a single STL in ROS meter-space.

    The mesh is centered at the solid group's bounding-box centroid.
    The printed URDF joint origin places that centroid correctly in base_link.
    """
    if not solids:
        print(f"  [skip] {name}: no matching solids")
        return

    shape = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)

    filepath = os.path.join(mesh_dir, f"{name}.stl")
    cq.exporters.export(
        cq.Workplane().add(shape),
        filepath,
        exportType="STL",
        tolerance=tolerance,
        angularTolerance=0.2,
    )

    bb = get_combined_bbox(solids)
    jx = (bb.xmin + bb.xmax) / 2
    jy = (bb.ymin + bb.ymax) / 2
    jz = (bb.zmin + bb.zmax) / 2
    transform_stl(filepath, jx, jy, jz)

    # URDF joint origin in ROS base_link frame (base_link = STEP axle centre)
    ros_x = -(jy - STEP_AXLE_Y) * MM_TO_M
    ros_y = -jx * MM_TO_M
    ros_z = (jz - STEP_AXLE_Z) * MM_TO_M

    size_kb = os.path.getsize(filepath) / 1024
    print(f"  {name}: {size_kb:.1f} KB  →  joint xyz=\"{ros_x:.4f} {ros_y:.4f} {ros_z:.4f}\"")


def main():
    step_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STEP

    if not os.path.exists(step_path):
        print(f"STEP file not found: {step_path}")
        sys.exit(1)

    os.makedirs(MESH_DIR, exist_ok=True)

    print(f"Loading: {step_path}")
    result = cq.importers.importStep(step_path)
    solids = list(result.solids().vals())
    print(f"Found {len(solids)} solids\n")

    print("Classifying...")
    groups = classify_solids(solids)
    print()

    print("Exporting meshes (mm → meters)...")
    export_mesh(groups["body"],          "body",          MESH_DIR, tolerance=0.5)
    export_mesh(groups["face"],          "face",          MESH_DIR, tolerance=0.3)
    export_mesh(groups["wheel_left"],    "wheel_left",    MESH_DIR, tolerance=0.2)
    export_mesh(groups["wheel_right"],   "wheel_right",   MESH_DIR, tolerance=0.2)
    export_mesh(groups["caster"],        "caster",        MESH_DIR, tolerance=0.3)
    export_mesh(groups["battery_cover"], "battery_cover", MESH_DIR, tolerance=0.3)

    print(f"\nDone! Meshes written to: {MESH_DIR}")


if __name__ == "__main__":
    main()
