# Hardware — CAD Source Files

## STEP File

The robot design source is `hardware/ZGX Otto.step` (Shapr3D export, 2026-03-02).

This file is **not committed to git** (large binary). If you need it, obtain it from the project maintainer or Shapr3D workspace.

## Re-exporting Meshes

The STL meshes in `ros2_ws/src/otto_description/meshes/` are generated from the STEP file. To regenerate them after a design update:

```bash
# Install CadQuery (one-time, using conda)
conda create -n cadquery -c cadquery -c conda-forge cadquery python=3.11 -y
conda run -n cadquery pip install "ezdxf==0.18.1"

# Export meshes
conda run -n cadquery python3 scripts/export_meshes.py
```

The script prints the URDF origin for any new parts (e.g. `battery_cover`) so you can update `otto_starter.urdf.xacro` if positions change.

## Part Breakdown

| STEP part(s) | Output mesh | Notes |
|---|---|---|
| Top + Middle + Bottom + Logo | `body.stl` | Logo is a surface feature on Middle |
| Face plate + Lines Upper/Lower | `face.stl` | Ultrasonic sensor mount |
| Wheels (left side) | `wheel_left.stl` | 3 concentric disc rings |
| Wheels (right side) | `wheel_right.stl` | 3 concentric disc rings |
| Ballcaster + housing | `caster.stl` | Front ball caster |
| Battery Cover | `battery_cover.stl` | Rear panel |

Skipped (not exported): PCB module, ultrasonic transducer pins (decorative only).

## STEP Coordinate System

For reference when updating URDF origins after re-export:

```
STEP axes → ROS axes (base_link at axle height):
  STEP −Y = ROS +X  (forward)
  STEP −X = ROS +Y  (left)
  STEP  Z → ROS +Z  (up, offset: STEP axle Z = −10 mm)

Formula:
  ros_x = −step_cy × 0.001
  ros_y = −step_cx × 0.001
  ros_z = (step_cz − (−10)) × 0.001
```
