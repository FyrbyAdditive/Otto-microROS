#!/usr/bin/env python3
"""Export STL meshes from the ZGX Otto STEP file using OCP CAF name-based grouping.

Usage:
    conda run -n cadquery python3 scripts/export_meshes.py [/path/to/step/file]

Default STEP path: hardware/ZGX Otto.step  (relative to project root)
Output:           ros2_ws/src/otto_description/meshes/

STEP coordinate system (Shapr3D export):
    +X = right of robot,  -X = left
    -Y = front of robot,  +Y = rear
    +Z = up
    Model built around origin; lowest point of wheels = ground plane.

ROS REP-103:
    X = forward, Y = left, Z = up
    base_footprint at ground (Z = 0)
    base_link     at wheel axle height

Vertex transform applied to every STL (STEP mm → ROS m, det=+1 proper rotation):
    ros_x = -(step_y - AXLE_Y) * MM_TO_M
    ros_y = +(step_x - jx)     * MM_TO_M   ← note: + not −
    ros_z = +(step_z - AXLE_Z) * MM_TO_M

Note: STEP +X is the robot's left side (as seen from the front by a viewer).
With ros_y = +step_x, STEP +X (robot left) → ROS +Y (left). det=+1, no mirror.

AXLE_Y and AXLE_Z are computed automatically from the 'Wheels' named group.

Named STEP groups → STL files:
    'Middle' main cylinder (max side > 60mm)  → body_shell.stl   (dark_grey)
    'Middle' clips (max side ≤ 60mm)          → body_clips.stl   (dark_grey)
    'Top' main parts (zlen ≥ 0.5mm)           → body_top.stl     (white)
    'Top' logo lines (zlen < 0.5mm)           → body_top_lines.stl (black)
    unnamed COMPOUND square ~72x72mm          → body_bottom.stl  (black)
    unnamed COMPOUND elongated ~66x12mm       → body_face.stl    (dark_grey)
    'Wheels' cx<0 dia>50mm                    → wheel_left_tire.stl  (rubber_black)
    'Wheels' cx<0 dia≤50mm                    → wheel_left_rim.stl   (dark_grey)
    'Wheels' cx>0 dia>50mm                    → wheel_right_tire.stl (rubber_black)
    'Wheels' cx>0 dia≤50mm                    → wheel_right_rim.stl  (dark_grey)
    'Ballcaster'                              → caster.stl       (otto_dark)
    'LED Ring' large solid (PCB)             → led_ring.stl     (pcb_green)
    'Ultrasonic Distance Sensor'              → ultrasonic.stl   (sensor_green)
    'Line Sensor Left'                            → line_sensor_left.stl  (pcb_green)
    'Line Sensor Right'                           → line_sensor_right.stl (pcb_green)

All body sub-meshes share the body_shell centroid as their reference origin so
they align correctly when placed at the same body_link joint with xyz="0 0 0".
Wheel sub-meshes (tire+rim per side) share the axle midpoint as reference.

With the det=+1 transform (ros_y=+step_x), STEP 'Left'/'Right' names match the
physical robot sides — no filename swap is needed.
"""

import sys
import os
import struct
import time
import math

import cadquery as cq

# OCP CAF imports for name extraction
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_STEP = os.path.join(PROJECT_ROOT, "hardware", "ZGX Otto.step")
MESH_DIR     = os.path.join(PROJECT_ROOT, "ros2_ws", "src",
                             "otto_description", "meshes")
MM_TO_M = 0.001

# Tessellation quality — fine enough for visible fillets, not so fine as to be slow
TOL = 0.05   # mm
ANG = 0.02   # rad


# ── OCP CAF loading ────────────────────────────────────────────────────

def _get_label_name(label):
    n = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), n):
        return n.Get().ToExtString()
    return ""


def _label_to_solids(label):
    """Return list of cq.Solid for all sub-solids of a CAF label's shape."""
    try:
        shape = XCAFDoc_ShapeTool.GetShape_s(label)
    except Exception:
        return []
    if shape.IsNull():
        return []
    result = []
    exp = TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        try:
            result.append(cq.Solid(TopoDS.Solid_s(exp.Current())))
        except Exception:
            pass
        exp.Next()
    return result


def load_named_solids(step_path):
    """Load STEP with OCP CAF.

    Returns:
        name_to_solids : dict {name: [cq.Solid, ...]}
        unnamed_solids : [cq.Solid, ...]  — solids from labels with no name
    """
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XDE"))
    app.NewDocument(TCollection_ExtendedString("XDE"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(step_path)
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    all_labels = TDF_LabelSequence()
    st.GetShapes(all_labels)

    name_to_solids: dict = {}
    unnamed_solids: list = []

    for i in range(1, all_labels.Length() + 1):
        label = all_labels.Value(i)
        name  = _get_label_name(label)
        if name.startswith('=>'):
            continue  # skip reference labels
        solids = _label_to_solids(label)
        if not solids:
            continue
        if name:
            name_to_solids.setdefault(name, []).extend(solids)
        else:
            unnamed_solids.extend(solids)

    # Deduplicate unnamed_solids against named ones (by centroid, 0.1mm tolerance)
    named_centroids = set()
    for solids in name_to_solids.values():
        for s in solids:
            bb = s.BoundingBox()
            cx = round((bb.xmin + bb.xmax) / 2, 1)
            cy = round((bb.ymin + bb.ymax) / 2, 1)
            cz = round((bb.zmin + bb.zmax) / 2, 1)
            named_centroids.add((cx, cy, cz))

    filtered_unnamed = []
    for s in unnamed_solids:
        bb = s.BoundingBox()
        cx = round((bb.xmin + bb.xmax) / 2, 1)
        cy = round((bb.ymin + bb.ymax) / 2, 1)
        cz = round((bb.zmin + bb.zmax) / 2, 1)
        if (cx, cy, cz) not in named_centroids:
            filtered_unnamed.append(s)
    unnamed_solids = filtered_unnamed

    return name_to_solids, unnamed_solids


# ── Bounding-box helpers ───────────────────────────────────────────────

def _bb(solid):
    return solid.BoundingBox()

def group_bb(solids):
    x0 = y0 = z0 =  1e18
    x1 = y1 = z1 = -1e18
    for s in solids:
        bb = _bb(s)
        x0 = min(x0, bb.xmin); y0 = min(y0, bb.ymin); z0 = min(z0, bb.zmin)
        x1 = max(x1, bb.xmax); y1 = max(y1, bb.ymax); z1 = max(z1, bb.zmax)
    return x0, y0, z0, x1, y1, z1

def group_centroid(solids):
    x0, y0, z0, x1, y1, z1 = group_bb(solids)
    return (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2


# ── STL coordinate transform ───────────────────────────────────────────

def transform_stl(filepath, jx, jy, jz):
    """Transform binary STL from STEP mm-space to ROS m-space in-place.

    Centres the mesh at the reference point (jx, jy, jz) in STEP space,
    then applies the STEP→ROS axis remap.

    Axis mapping (det = +1, proper rotation — no mirror):
        ros_x = -(step_y - jy) * MM_TO_M
        ros_y = +(step_x - jx) * MM_TO_M
        ros_z =  (step_z - jz) * MM_TO_M

    det=+1 preserves triangle winding, so no vertex swap needed.
    Stored normal updated: n' = M·n = (-ny, nx, nz).
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
        # det=+1 transform: ros_x=-step_y, ros_y=+step_x, ros_z=+step_z
        # Normal transform n' = M·n: (-ny, nx, nz)
        out.extend(struct.pack("<fff", -ny, nx, nz))
        offset += 12

        for _ in range(3):
            vx, vy, vz = struct.unpack_from("<fff", data, offset)
            out.extend(struct.pack("<fff",
                -(vy - jy) * MM_TO_M,   # ros_x = -(step_y - jy)
                 (vx - jx) * MM_TO_M,   # ros_y = +(step_x - jx)  ← det=+1
                 (vz - jz) * MM_TO_M))  # ros_z =  (step_z - jz)
            offset += 12

        # det=+1 preserves winding — write v0, v1, v2 unchanged
        out.extend(struct.pack("<H", struct.unpack_from("<H", data, offset)[0]))
        offset += 2

    with open(filepath, "wb") as f:
        f.write(bytes(out))


# ── Export one STL group ───────────────────────────────────────────────

def export_stl(solids, name, jx, jy, jz, axle_y, axle_z, label=""):
    """Tessellate solids → <name>.stl, transform to ROS space.

    jx, jy, jz  — reference origin in STEP space (mm); after transform,
                   this point lands at (0,0,0) in the STL file.
    axle_y, axle_z — wheel axle constants for computing ROS joint xyz.
    label       — optional extra info printed on the summary line.
    """
    if not solids:
        print(f"  [skip] {name}: no solids")
        return

    filepath = os.path.join(MESH_DIR, f"{name}.stl")
    shape = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
    cq.exporters.export(
        cq.Workplane().add(shape), filepath,
        exportType="STL", tolerance=TOL, angularTolerance=ANG)

    transform_stl(filepath, jx, jy, jz)

    ros_x = -(jy - axle_y) * MM_TO_M
    ros_y = +jx             * MM_TO_M   # det=+1: ros_y = +step_x
    ros_z =  (jz - axle_z) * MM_TO_M
    kb    = os.path.getsize(filepath) / 1024
    note  = f"  [{label}]" if label else ""
    print(f'  {name}: {kb:.0f} KB  →  joint xyz="{ros_x:.4f} {ros_y:.4f} {ros_z:.4f}"{note}')


# ── Main ───────────────────────────────────────────────────────────────

def main():
    step_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STEP
    if not os.path.exists(step_path):
        print(f"STEP file not found: {step_path}")
        sys.exit(1)

    os.makedirs(MESH_DIR, exist_ok=True)

    t0 = time.time()
    print(f"Loading STEP (OCP CAF): {step_path}", flush=True)
    name_to_solids, unnamed_solids = load_named_solids(step_path)
    print(f"  Loaded in {time.time()-t0:.1f}s — "
          f"{sum(len(v) for v in name_to_solids.values())} named + "
          f"{len(unnamed_solids)} unnamed solids", flush=True)

    # ── Print all groups for inspection ───────────────────────────────
    print("\nNamed groups:")
    for name, solids in sorted(name_to_solids.items()):
        print(f"  '{name}': {len(solids)} solid(s)")
    if unnamed_solids:
        print(f"  (unnamed): {len(unnamed_solids)} solid(s)")
        for s in unnamed_solids:
            bb = _bb(s)
            cx = (bb.xmin + bb.xmax) / 2
            cy = (bb.ymin + bb.ymax) / 2
            cz = (bb.zmin + bb.zmax) / 2
            print(f"    {bb.xlen:.1f}x{bb.ylen:.1f}x{bb.zlen:.1f}  "
                  f"cx={cx:.1f} cy={cy:.1f} cz={cz:.1f}")

    # ── Wheel axle constants from 'Wheels' named group ────────────────
    wheel_all = name_to_solids.get("Wheels", [])
    if wheel_all:
        x0, y0, wz0, x1, y1, wz1 = group_bb(wheel_all)
        axle_z    = (wz0 + wz1) / 2
        axle_y    = (y0  + y1)  / 2
        wheel_r   = (wz1 - wz0) / 2
        # half_base: average of |xmin| and |xmax| (left and right wheel centroids)
        left_cx  = sum((bb := _bb(s), (bb.xmin + bb.xmax) / 2)[1]
                       for s in wheel_all if (bb := _bb(s)).xmax < 0) / max(
                       1, sum(1 for s in wheel_all if _bb(s).xmax < 0))
        right_cx = sum((bb := _bb(s), (bb.xmin + bb.xmax) / 2)[1]
                       for s in wheel_all if (bb := _bb(s)).xmin > 0) / max(
                       1, sum(1 for s in wheel_all if _bb(s).xmin > 0))
        half_base = (abs(left_cx) + abs(right_cx)) / 2
    else:
        print("WARNING: no 'Wheels' group found!")
        axle_z = axle_y = 0.0
        wheel_r = 0.0245
        half_base = 0.0457

    # Ground Z
    all_solids_flat = [s for v in name_to_solids.values() for s in v] + unnamed_solids
    ground_z = min(_bb(s).zmin for s in all_solids_flat)

    print(f"\nAXLE_Z={axle_z:.2f}mm  AXLE_Y={axle_y:.2f}mm  GROUND_Z={ground_z:.2f}mm")
    print(f"wheel_radius={wheel_r * MM_TO_M:.4f}m  wheel_base={2 * half_base * MM_TO_M:.4f}m")
    print()

    # ══════════════════════════════════════════════════════════════════
    # EXPORT
    # ══════════════════════════════════════════════════════════════════
    print("Exporting meshes…\n")

    # ── WHEELS ────────────────────────────────────────────────────────
    # Three bodies per wheel:
    #   inner hub  ≈ 40.6 mm diameter (dia ≤ 50)  → rim
    #   outer rim  ≈ 45.0 mm diameter (dia ≤ 50)  → rim
    #   tyre       ≈ 53.0 mm diameter (dia > 50)  → tire
    # STEP cx < 0 = robot left; cx > 0 = robot right.

    left_tire  = []
    left_rim   = []
    right_tire = []
    right_rim  = []

    for s in wheel_all:
        bb  = _bb(s)
        cx  = (bb.xmin + bb.xmax) / 2
        # Diameter = max cross-section perpendicular to the axle (Y or Z, not X)
        dia = max(bb.ylen, bb.zlen)
        # Outer tire ≈ 49 mm, rim/hub ≤ 45 mm — threshold at 46 mm
        is_tire = dia > 46
        # With det=+1 transform: STEP cx > 0 → ros_y = +cx > 0 = ROS left
        if cx > 0:
            (left_tire  if is_tire else left_rim).append(s)
        else:
            (right_tire if is_tire else right_rim).append(s)

    print(f"  Wheels: L tire={len(left_tire)} rim={len(left_rim)}  "
          f"R tire={len(right_tire)} rim={len(right_rim)}")

    for side, tire, rim in (("left", left_tire, left_rim),
                             ("right", right_tire, right_rim)):
        all_side = tire + rim
        if not all_side:
            continue
        # Shared reference = axle midpoint for this side (correct rotation centre)
        side_x0 = min(_bb(s).xmin for s in all_side)
        side_x1 = max(_bb(s).xmax for s in all_side)
        side_cx = (side_x0 + side_x1) / 2
        export_stl(tire, f"wheel_{side}_tire", side_cx, axle_y, axle_z, axle_y, axle_z)
        export_stl(rim,  f"wheel_{side}_rim",  side_cx, axle_y, axle_z, axle_y, axle_z)

    # ── BODY SHELL: 'Middle' ──────────────────────────────────────────
    # Main cylinder: both X and Y span > 60 mm
    # Rear battery clips: smaller

    middle = name_to_solids.get("Middle", [])
    body_shell = [s for s in middle
                  if max(_bb(s).xlen, _bb(s).ylen) > 60]
    body_clips  = [s for s in middle
                   if max(_bb(s).xlen, _bb(s).ylen) <= 60]

    body_ref = None   # (jx, jy, jz) shared by all body sub-meshes
    if body_shell:
        jx, jy, jz = group_centroid(body_shell)
        body_ref = (jx, jy, jz)
        export_stl(body_shell, "body_shell", jx, jy, jz, axle_y, axle_z,
                   label="body_link joint")
        if body_clips:
            export_stl(body_clips, "body_clips", jx, jy, jz, axle_y, axle_z,
                       label="sub-visual of body_link")
    else:
        print("  [skip] body_shell: 'Middle' group not found or has no large solid")

    # ── BODY TOP: 'Top' ───────────────────────────────────────────────
    # Logo lines: zlen < 0.5 mm (ultra-thin 0.2 mm surfaces)
    # Main dome sections: zlen ≥ 0.5 mm

    top_all   = name_to_solids.get("Top", [])
    top_main  = [s for s in top_all if _bb(s).zlen >= 0.5]
    top_logos = [s for s in top_all if _bb(s).zlen <  0.5]

    if body_ref:
        jx, jy, jz = body_ref
        export_stl(top_main,  "body_top",        jx, jy, jz, axle_y, axle_z,
                   label="sub-visual of body_link")
        export_stl(top_logos, "body_top_lines",  jx, jy, jz, axle_y, axle_z,
                   label="sub-visual of body_link")
    else:
        print("  [skip] body_top / body_top_lines: body_ref not established")

    # ── BODY BOTTOM + FACE: search all unique named solids ───────────
    # These appear in a 'COMPOUND' label (shared name with other COMPOUND labels)
    # so we search geometrically across all unique named solids.
    # bottom shell: ≈ 72×72×24 mm  →  both X and Y span > 60 mm
    # front face:   ≈ 66×11×36 mm  →  one span > 55 mm, other < 20 mm

    # Build deduplicated set of all named solids (centroid key, 0.1 mm res)
    _seen_ck = set()
    _all_unique_named: list = []
    for _name, _solids in name_to_solids.items():
        for _s in _solids:
            _bb2 = _bb(_s)
            _ck = (round((_bb2.xmin + _bb2.xmax) / 2, 1),
                   round((_bb2.ymin + _bb2.ymax) / 2, 1),
                   round((_bb2.zmin + _bb2.zmax) / 2, 1))
            if _ck not in _seen_ck:
                _seen_ck.add(_ck)
                _all_unique_named.append(_s)

    # Exclude solids already claimed by Middle or Top (body_shell, clips, top_main, top_logos)
    _claimed_ck = set()
    for _s in (body_shell + body_clips + top_main + top_logos):
        _bb2 = _bb(_s)
        _claimed_ck.add((round((_bb2.xmin + _bb2.xmax) / 2, 1),
                         round((_bb2.ymin + _bb2.ymax) / 2, 1),
                         round((_bb2.zmin + _bb2.zmax) / 2, 1)))

    _unclaimed = [_s for _s in _all_unique_named
                  if (round((_bb(_s).xmin + _bb(_s).xmax) / 2, 1),
                      round((_bb(_s).ymin + _bb(_s).ymax) / 2, 1),
                      round((_bb(_s).zmin + _bb(_s).zmax) / 2, 1)) not in _claimed_ck]

    body_bottom = [s for s in _unclaimed
                   if _bb(s).xlen > 60 and _bb(s).ylen > 60]
    body_face   = [s for s in _unclaimed
                   if max(_bb(s).xlen, _bb(s).ylen) > 55
                   and min(_bb(s).xlen, _bb(s).ylen) < 20]

    print(f"  body_bottom: {len(body_bottom)} solid(s), body_face: {len(body_face)} solid(s)")

    if body_ref:
        jx, jy, jz = body_ref
        export_stl(body_bottom, "body_bottom", jx, jy, jz, axle_y, axle_z,
                   label="sub-visual of body_link")
        export_stl(body_face,   "body_face",   jx, jy, jz, axle_y, axle_z,
                   label="sub-visual of body_link")
    else:
        print("  [skip] body_bottom / body_face: body_ref not established")

    # ── CASTER: 'Ballcaster' ─────────────────────────────────────────

    caster = name_to_solids.get("Ballcaster", [])
    if caster:
        jx, jy, jz = group_centroid(caster)
        export_stl(caster, "caster", jx, jy, jz, axle_y, axle_z)

    # ── LED RING: 'LED Ring' ─────────────────────────────────────────
    # PCB: large flat solid (max side > 20 mm) → led_ring.stl
    # LED pads: small solids → centroid positions only (no STL)

    led_all  = name_to_solids.get("LED Ring", [])
    led_pcb  = [s for s in led_all if max(_bb(s).xlen, _bb(s).ylen) > 20]
    led_pads = [s for s in led_all if max(_bb(s).xlen, _bb(s).ylen) <= 20]

    if led_pcb:
        jx, jy, jz = group_centroid(led_pcb)
        export_stl(led_pcb, "led_ring", jx, jy, jz, axle_y, axle_z)

    if led_pads:
        print(f"\nLED pad positions ({len(led_pads)} LEDs):")
        led_data = []
        for s in led_pads:
            lx, ly, lz = group_centroid([s])
            rx = -(ly - axle_y) * MM_TO_M
            ry = -lx             * MM_TO_M
            rz =  (lz - axle_z) * MM_TO_M
            led_data.append((rx, ry, rz))
        led_data.sort(key=lambda t: math.atan2(t[1], t[0]))
        for i, (rx, ry, rz) in enumerate(led_data):
            print(f'  led_{i}: xyz="{rx:.4f} {ry:.4f} {rz:.4f}"')
        print()

    # ── ULTRASONIC SENSOR: 'Ultrasonic Distance Sensor' ──────────────

    ultrasonic = name_to_solids.get("Ultrasonic Distance Sensor", [])
    if ultrasonic:
        jx, jy, jz = group_centroid(ultrasonic)
        export_stl(ultrasonic, "ultrasonic", jx, jy, jz, axle_y, axle_z)

    # ── LINE SENSORS ─────────────────────────────────────────────────
    # With det=+1 transform (ros_y = +step_x):
    #   STEP 'Line Sensor Left'  has cx > 0 → ros_y = +cx > 0 = ROS left  → line_sensor_left
    #   STEP 'Line Sensor Right' has cx < 0 → ros_y = +cx < 0 = ROS right → line_sensor_right
    # Names match physical sides — no swap needed.

    lsl = name_to_solids.get("Line Sensor Left",  [])   # STEP 'Left'  → robot left
    lsr = name_to_solids.get("Line Sensor Right", [])   # STEP 'Right' → robot right

    if lsl:
        jx, jy, jz = group_centroid(lsl)
        export_stl(lsl, "line_sensor_left",  jx, jy, jz, axle_y, axle_z)
    if lsr:
        jx, jy, jz = group_centroid(lsr)
        export_stl(lsr, "line_sensor_right", jx, jy, jz, axle_y, axle_z)

    # ── Summary ───────────────────────────────────────────────────────
    elapsed = time.time() - t0
    print(f"\nDone in {elapsed:.0f}s.  Meshes in: {MESH_DIR}")
    print("\n=== Copy into URDF ===")
    print(f'  wheel_radius="{wheel_r * MM_TO_M:.4f}"')
    print(f'  wheel_base="{2 * half_base * MM_TO_M:.4f}"')
    print("  (body_link joint xyz = body_shell line above, marked [body_link joint])")
    print("  (all other body_* STLs share the same body_link joint origin)")


if __name__ == "__main__":
    main()
