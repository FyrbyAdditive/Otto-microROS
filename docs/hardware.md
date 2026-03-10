# Hardware — CAD Source Files

Design source and mesh export workflow for the Otto Starter Kit.

---

## STEP File

The robot design source is [`hardware/ZGX Otto.step`](../hardware/ZGX%20Otto.step) (Shapr3D export, 2026-03-02).

## Re-exporting Meshes

The STL meshes in [`ros2_ws/src/otto_description/meshes/`](../ros2_ws/src/otto_description/meshes/) are generated from the STEP file. To regenerate after a design update:

```bash
# Install CadQuery (one-time, using conda)
conda create -n cadquery -c cadquery -c conda-forge cadquery python=3.11 -y
conda run -n cadquery pip install "ezdxf==0.18.1"

# Export meshes
conda run -n cadquery python3 scripts/export_meshes.py
```

The script prints URDF origins for any new parts so you can update `otto_starter.urdf.xacro` if positions change.

---

## Part Breakdown

The export script groups named STEP parts into individual meshes:

### Body

| STEP group | Output mesh | Colour |
|------------|-------------|--------|
| Middle (main cylinder) | `body_shell.stl` | Dark grey |
| Middle (clips) | `body_clips.stl` | Dark grey |
| Top (main) | `body_top.stl` | White |
| Top (logo lines) | `body_top_lines.stl` | Black |
| Top (honeycomb pattern) | `body_top_pattern.stl` | Dark grey |
| Unnamed (square plate) | `body_bottom.stl` | Black |
| Unnamed (face plate) | `body_face.stl` | Dark grey |

### Wheels

| STEP group | Output mesh | Colour |
|------------|-------------|--------|
| Wheels (left, large) | `wheel_left_tire.stl` | Black rubber |
| Wheels (left, small) | `wheel_left_rim.stl` | Dark grey |
| Wheels (left, hub) | `wheel_left_hub.stl` | White |
| Wheels (right, large) | `wheel_right_tire.stl` | Black rubber |
| Wheels (right, small) | `wheel_right_rim.stl` | Dark grey |
| Wheels (right, hub) | `wheel_right_hub.stl` | White |

### Caster

| STEP group | Output mesh | Colour |
|------------|-------------|--------|
| Ballcaster (housing) | `caster.stl` | Dark grey |
| Ballcaster (bracket) | `caster_bracket.stl` | Dark grey |
| Ballcaster (ball) | `caster_ball.stl` | Silver |

### Sensors and LEDs

| STEP group | Output mesh | Colour |
|------------|-------------|--------|
| LED Ring (PCB) | `led_ring.stl` | Blue |
| LED Ring (pads) | `led_pads.stl` | White |
| Ultrasonic (full) | `ultrasonic.stl` | Mixed |
| Ultrasonic (board) | `ultrasonic_board.stl` | Blue |
| Ultrasonic (plug) | `ultrasonic_plug.stl` | White |
| Ultrasonic (cans) | `ultrasonic_cans.stl` | Silver |
| Ultrasonic (pins) | `ultrasonic_metal.stl` | Silver |
| Line Sensor Left (full) | `line_sensor_left.stl` | PCB green |
| Line Sensor Left (board) | `line_sensor_left_board.stl` | PCB green |
| Line Sensor Left (white) | `line_sensor_left_white.stl` | White |
| Line Sensor Left (black) | `line_sensor_left_black.stl` | Black |
| Line Sensor Right (full) | `line_sensor_right.stl` | PCB green |
| Line Sensor Right (board) | `line_sensor_right_board.stl` | PCB green |
| Line Sensor Right (white) | `line_sensor_right_white.stl` | White |
| Line Sensor Right (black) | `line_sensor_right_black.stl` | Black |

---

## STEP Coordinate System

For reference when updating URDF origins after re-export:

```
STEP axes → ROS axes (base_link at axle height):

  STEP −Y  →  ROS +X  (forward)
  STEP +X  →  ROS +Y  (left)
  STEP +Z  →  ROS +Z  (up, offset so axle = 0)

Formula (mm → m):
  ros_x = -(step_y - AXLE_Y) × 0.001
  ros_y = +(step_x)          × 0.001
  ros_z = +(step_z - AXLE_Z) × 0.001
```

AXLE_Y and AXLE_Z are computed automatically from the Wheels group by the export script.

---

<p align="center">
  <br>
  <img src="images/fame-logo.letterbox.png" alt="FAME logo" width="200">
</p>
