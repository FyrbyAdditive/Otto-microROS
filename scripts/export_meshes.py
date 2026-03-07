#!/usr/bin/env python3
"""Export STL meshes from the ZGX Otto STEP file.

Usage:
    python3 scripts/export_meshes.py [/path/to/step/file]

Default STEP path: hardware/ZGX Otto.step  (relative to project root)
Output:           ros2_ws/src/otto_description/meshes/

STEP coordinate system (Shapr3D export, ZGX Otto.step):
    +X = right of robot,  -X = left
    -Y = front of robot,  +Y = rear
    +Z = up
    Model built around origin; lowest point of wheels = ground plane.

ROS REP-103:
    X = forward, Y = left, Z = up
    base_footprint at ground (Z = 0)
    base_link     at wheel axle height

Vertex transform applied to every STL (STEP mm → ROS m):
    ros_x = -(step_y - AXLE_Y) * MM_TO_M
    ros_y = -step_x             * MM_TO_M
    ros_z = (step_z  - AXLE_Z) * MM_TO_M

AXLE_Y and AXLE_Z are computed automatically from the wheel geometry.

Each exported STL is centred at the part bounding-box centroid (= joint
origin in URDF).  Wheels are centred at the axle midpoint so continuous
joints rotate them correctly.

Part groups → STL files (classification by STEP solid bounding box):
    body.stl          : large shell solids (≥50 mm in 2 axes)
    wheel_left.stl    : disc solids with cx < 0
    wheel_right.stl   : disc solids with cx > 0
    caster.stl        : small spherical/cylindrical solids near bottom
    led_ring.stl      : ring-shaped solid near top centre
    ultrasonic.stl    : rectangular solid near front top
    line_sensor_left  : small PCB solid near bottom-left
    line_sensor_right : small PCB solid near bottom-right
    (LED individual positions are computed from their bounding boxes
     and printed for copy-paste into the URDF LED frames.)
"""

import sys
import os
import struct

import cadquery as cq

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_STEP = os.path.join(PROJECT_ROOT, "hardware", "ZGX Otto.step")
MESH_DIR     = os.path.join(PROJECT_ROOT, "ros2_ws", "src",
                             "otto_description", "meshes")
MM_TO_M = 0.001


# ── STL coordinate transform ───────────────────────────────────────────

def transform_stl(filepath, jx, jy, jz, axle_y, axle_z):
    """Transform a binary STL from STEP mm-space to ROS m-space.

    Centres the mesh at the joint origin (jx, jy, jz) in STEP space,
    then applies the STEP→ROS axis remap.  After transformation, the STL
    origin corresponds to the link frame origin in the URDF so the visual
    origin can be left at (0 0 0).

    Axis mapping (det = -1, improper rotation):
        ros_x = -(vy - jy) * MM_TO_M
        ros_y = -(vx - jx) * MM_TO_M
        ros_z =  (vz - jz) * MM_TO_M

    Because det = -1 this transform mirrors the geometry, reversing the
    winding order of every triangle.  Ogre3D (used by RViz2) determines
    front/back faces from vertex winding, not the stored STL normal, so
    reversed winding causes every face to be backface-culled (transparent
    from outside).  Fix: swap vertices 1 and 2 to restore CCW winding.
    Stored normal updated consistently: n' = M * n = (-ny, -nx, nz).
    """
    with open(filepath, "rb") as f:
        data = f.read()
    if len(data) < 84:
        return

    n_tri  = struct.unpack_from("<I", data, 80)[0]
    offset = 84
    out    = bytearray(data[:84])

    for _ in range(n_tri):
        nx, ny, nz = struct.unpack_from("<fff", data, offset)
        # M * n for the det=-1 mapping; winding swap below restores outward sense
        out.extend(struct.pack("<fff", -ny, -nx, nz))
        offset += 12
        # Read all three vertices first so we can swap winding
        verts = []
        for _ in range(3):
            vx, vy, vz = struct.unpack_from("<fff", data, offset)
            verts.append((
                -(vy - jy) * MM_TO_M,
                -(vx - jx) * MM_TO_M,
                 (vz - jz) * MM_TO_M))
            offset += 12
        # Write v0, v2, v1 — swaps the last two to correct the winding reversal
        out.extend(struct.pack("<fff", *verts[0]))
        out.extend(struct.pack("<fff", *verts[2]))
        out.extend(struct.pack("<fff", *verts[1]))
        out.extend(struct.pack("<H", struct.unpack_from("<H", data, offset)[0]))
        offset += 2

    with open(filepath, "wb") as f:
        f.write(bytes(out))


# ── Bounding-box helpers ───────────────────────────────────────────────

def bbox(solid):
    bb = solid.BoundingBox()
    return bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax

def centroid(solid):
    bb = solid.BoundingBox()
    return ((bb.xmin+bb.xmax)/2,
            (bb.ymin+bb.ymax)/2,
            (bb.zmin+bb.zmax)/2)

def group_bbox(solids):
    x0=y0=z0= 1e18; x1=y1=z1=-1e18
    for s in solids:
        a,b,c,d,e,f = bbox(s)
        x0=min(x0,a); y0=min(y0,b); z0=min(z0,c)
        x1=max(x1,d); y1=max(y1,e); z1=max(z1,f)
    return x0,y0,z0,x1,y1,z1


# ── Export one group ───────────────────────────────────────────────────

def export_group(solids, name, mesh_dir, axle_y, axle_z,
                 tolerance=0.5, angular=0.2, jx=None, jy=None, jz=None):
    if not solids:
        print(f"  [skip] {name}: no solids")
        return None

    shape = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
    filepath = os.path.join(mesh_dir, f"{name}.stl")
    cq.exporters.export(
        cq.Workplane().add(shape),
        filepath, exportType="STL",
        tolerance=tolerance, angularTolerance=angular)

    if jx is None:
        x0,y0,z0,x1,y1,z1 = group_bbox(solids)
        jx,jy,jz = (x0+x1)/2,(y0+y1)/2,(z0+z1)/2

    transform_stl(filepath, jx, jy, jz, axle_y, axle_z)

    ros_x = -(jy - axle_y) * MM_TO_M
    ros_y = -jx             * MM_TO_M
    ros_z = (jz - axle_z)  * MM_TO_M
    kb    = os.path.getsize(filepath) / 1024
    print(f'  {name}: {kb:.0f} KB  →  joint xyz="{ros_x:.4f} {ros_y:.4f} {ros_z:.4f}"')
    return jx, jy, jz


# ── Classify solids ────────────────────────────────────────────────────

def classify(solids, ground_z):
    """Bin solids into named groups based on geometry.

    Returns dict of name → [solid, ...]
    """
    groups = {
        "body": [], "wheel_left": [], "wheel_right": [],
        "caster": [], "led_ring": [], "ultrasonic": [],
        "line_sensor_left": [], "line_sensor_right": [],
        "led_individuals": [],
        "skipped": [],
    }

    for s in solids:
        bb = s.BoundingBox()
        w, d, h = bb.xlen, bb.ylen, bb.zlen
        cx = (bb.xmin+bb.xmax)/2
        cy = (bb.ymin+bb.ymax)/2
        cz = (bb.zmin+bb.zmax)/2
        zmin_rel = bb.zmin - ground_z  # Z above ground

        # ── Large body shell parts ─────────────────────────────────────
        # Round shell: large footprint (w>50, d>50)
        # Front face panel: wide+tall but thin, centred, at front of robot
        if (w > 50 and d > 50) or \
           (w > 55 and h > 25 and d < 20 and abs(cx) < 5 and cy < -30):
            groups["body"].append(s)

        # ── Wheels: thin discs, large diameter, laterally offset ──────
        elif w < 12 and min(d,h) > 35:
            if cx > 0:
                groups["wheel_right"].append(s)
            else:
                groups["wheel_left"].append(s)

        # ── LED Ring: medium ring shape, near top centre ───────────────
        elif 30 < w < 80 and 30 < d < 80 and h < 15 and zmin_rel > 60:
            groups["led_ring"].append(s)

        # ── Individual ring LEDs: tiny, near top centre ────────────────
        elif w < 10 and d < 10 and zmin_rel > 55:
            groups["led_individuals"].append(s)

        # ── Ultrasonic sensor: small box, near front, on centreline ──────
        # Housing is ~10x5x6mm; cx≈0 excludes servo motors at cx=±13
        elif cy < -20 and abs(cx) < 5 and 5 < w < 15 and 3 < d < 10 and 3 < h < 10:
            groups["ultrasonic"].append(s)

        # ── Caster: small near-spherical solid at ground level ─────────
        # Robot has a front caster (cy ≈ -20 in STEP), not rear
        elif zmin_rel < 3 and max(w,d,h) < 30 and abs(cx) < 5:
            groups["caster"].append(s)

        # ── Line sensors: thin PCBs (h≈1.6mm), laterally offset ────────
        elif h < 3 and max(w,d) > 10 and abs(cx) > 5 and cy < 0 and zmin_rel < 30:
            if cx > 0:
                groups["line_sensor_right"].append(s)
            else:
                groups["line_sensor_left"].append(s)

        else:
            groups["skipped"].append(s)

    return groups


# ── Main ───────────────────────────────────────────────────────────────

def main():
    step_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STEP
    if not os.path.exists(step_path):
        print(f"STEP file not found: {step_path}")
        sys.exit(1)

    os.makedirs(MESH_DIR, exist_ok=True)

    print(f"Loading: {step_path}")
    print("This may take several minutes…", flush=True)
    import time; t0 = time.time()
    result = cq.importers.importStep(step_path)
    print(f"Loaded in {time.time()-t0:.0f}s", flush=True)

    solids = list(result.solids().vals())
    print(f"Found {len(solids)} solids\n")

    # ── Print all solids for inspection ───────────────────────────────
    print("All solids:")
    for i, s in enumerate(solids):
        bb = s.BoundingBox()
        cx=(bb.xmin+bb.xmax)/2; cy=(bb.ymin+bb.ymax)/2; cz=(bb.zmin+bb.zmax)/2
        print(f"  S{i:3d}: {bb.xlen:6.1f}x{bb.ylen:6.1f}x{bb.zlen:6.1f}  "
              f"cx={cx:7.2f} cy={cy:7.2f} cz={cz:7.2f}  zmin={bb.zmin:.2f}")
    print()

    # ── Ground plane and wheel axle from overall minimum Z ────────────
    all_zmin = min(s.BoundingBox().zmin for s in solids)
    all_zmax = max(s.BoundingBox().zmax for s in solids)
    ground_z = all_zmin
    print(f"Ground Z (lowest point) = {ground_z:.2f} mm")

    # ── Classify ───────────────────────────────────────────────────────
    groups = classify(solids, ground_z)

    # Derive wheel axle constants from wheel geometry
    all_wheels = groups["wheel_left"] + groups["wheel_right"]
    if all_wheels:
        x0,y0,wz0,x1,y1,wz1 = group_bbox(all_wheels)
        axle_z = (wz0 + wz1) / 2        # wheel centroid Z = axle height
        axle_y = (y0  + y1)  / 2        # wheel centroid Y
        wheel_r = (wz1 - wz0) / 2
        half_base = (abs(x0) + abs(x1)) / 2   # half wheel-to-wheel distance
    else:
        print("WARNING: no wheel solids found — using zero axle constants")
        axle_z = axle_y = 0.0
        wheel_r = 0.0245
        half_base = 0.0417

    print(f"AXLE_Z   = {axle_z:.2f} mm  (wheel centroid Z)")
    print(f"AXLE_Y   = {axle_y:.2f} mm  (wheel centroid Y ≈ 0)")
    print(f"GROUND_Z = {ground_z:.2f} mm")
    print(f"wheel_radius = {wheel_r*MM_TO_M:.4f} m")
    print(f"wheel_base   = {2*half_base*MM_TO_M:.4f} m")
    print()

    # ── Report classification ──────────────────────────────────────────
    for gname, gsolids in groups.items():
        if gname == "skipped":
            print(f"  [skip] {len(gsolids)} unclassified solids")
            for s in gsolids:
                bb = s.BoundingBox()
                cx=(bb.xmin+bb.xmax)/2; cy=(bb.ymin+bb.ymax)/2; cz=(bb.zmin+bb.zmax)/2
                print(f"    {bb.xlen:.1f}x{bb.ylen:.1f}x{bb.zlen:.1f} "
                      f"cx={cx:.1f} cy={cy:.1f} cz={cz:.1f}")
        else:
            print(f"  {gname}: {len(gsolids)} solid(s)")
    print()

    # ── Export meshes ──────────────────────────────────────────────────
    print("Exporting meshes…")

    export_group(groups["body"],              "body",              MESH_DIR, axle_y, axle_z,
                 tolerance=0.05, angular=0.02)

    # Wheels: centre at axle midpoint for correct rotation
    if groups["wheel_left"]:
        x0,_,_,x1,_,_ = group_bbox(groups["wheel_left"])
        export_group(groups["wheel_left"],    "wheel_left",        MESH_DIR, axle_y, axle_z,
                     tolerance=0.05, angular=0.02,
                     jx=(x0+x1)/2, jy=axle_y, jz=axle_z)
    if groups["wheel_right"]:
        x0,_,_,x1,_,_ = group_bbox(groups["wheel_right"])
        export_group(groups["wheel_right"],   "wheel_right",       MESH_DIR, axle_y, axle_z,
                     tolerance=0.05, angular=0.02,
                     jx=(x0+x1)/2, jy=axle_y, jz=axle_z)

    export_group(groups["caster"],            "caster",            MESH_DIR, axle_y, axle_z,
                 tolerance=0.05, angular=0.02)
    export_group(groups["led_ring"],          "led_ring",          MESH_DIR, axle_y, axle_z,
                 tolerance=0.05, angular=0.02)
    export_group(groups["ultrasonic"],        "ultrasonic",        MESH_DIR, axle_y, axle_z,
                 tolerance=0.05, angular=0.02)
    export_group(groups["line_sensor_left"],  "line_sensor_left",  MESH_DIR, axle_y, axle_z,
                 tolerance=0.05, angular=0.02)
    export_group(groups["line_sensor_right"], "line_sensor_right", MESH_DIR, axle_y, axle_z,
                 tolerance=0.05, angular=0.02)

    # ── Individual LED positions ───────────────────────────────────────
    leds = groups["led_individuals"]
    if leds:
        print(f"\nLED ring frame positions ({len(leds)} LEDs):")
        import math
        led_data = []
        for s in leds:
            jx,jy,jz = centroid(s)
            rx = -(jy - axle_y) * MM_TO_M
            ry = -jx * MM_TO_M
            rz = (jz - axle_z) * MM_TO_M
            led_data.append((rx, ry, rz))
        led_data.sort(key=lambda t: math.atan2(t[1], t[0]))
        for i, (rx, ry, rz) in enumerate(led_data):
            print(f'  led_{i}: xyz="{rx:.4f} {ry:.4f} {rz:.4f}"')

    print(f"\nDone! Meshes in: {MESH_DIR}")
    print("\n=== Copy into URDF ===")
    print(f"  wheel_radius = {wheel_r*MM_TO_M:.4f}")
    print(f"  wheel_base   = {2*half_base*MM_TO_M:.4f}")


if __name__ == "__main__":
    main()
