# Robot Structure and Kinematics

> **IMPORTANT:** This document represents the robot state AFTER preprocessing and applying the `link23_connected_stand` keyframe. **Do not use the raw `轮腿v1.xml` file to judge global positions or robot posture, as its default CAD pose is invalid.**

## Bodies and Kinematic Tree

### Body: `world`
- **Parent**: `world`
- **Mass**: 0.0000 kg
- **Global Pos (standing)**: [0.0000, 0.0000, 0.0000]
- **Global Quat (w,x,y,z)**: [1.0000, 0.0000, 0.0000, 0.0000]
- **Geoms**:
  - `geom_0` (plane) - Size parameters: [0.0000, 0.0000, 1.0000]

### Body: `base_link`
- **Parent**: `world`
- **Mass**: 0.0000 kg
- **Global Pos (standing)**: [0.0000, 0.0000, 0.1500]
- **Global Quat (w,x,y,z)**: [1.0000, 0.0000, 0.0000, 0.0000]
- **Joints**:
  - `root` (free) - Axis: [0.0000, 0.0000, 1.0000] - Limits: unlimited

### Body: `base_link`
- **Parent**: `base_link`
- **Mass**: 2.2000 kg
- **Global Pos (standing)**: [0.0000, 0.0000, 0.2198]
- **Global Quat (w,x,y,z)**: [-0.0923, 0.7011, 0.7011, 0.0923]
- **Geoms**:
  - `base_link_geom_part_001` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.1070, 0.0991, 0.2002]
    - Center offset: [-0.0187, 0.0051, 0.0013]
  - `base_link_geom_part_002` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0064, 0.0349, 0.0350]
    - Center offset: [-0.0008, -0.0038, -0.0018]
  - `base_link_geom_part_003` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0306, 0.0316]
    - Center offset: [0.0001, 0.0013, -0.0002]
  - `base_link_geom_part_004` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0050, 0.0030, 0.0030]
    - Center offset: [0.0004, -0.0000, 0.0000]
  - `base_link_geom_part_005` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0058, 0.0165, 0.0030]
    - Center offset: [-0.0001, -0.0030, 0.0000]
  - `base_link_geom_part_006` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0058, 0.0098, 0.0147]
    - Center offset: [-0.0004, -0.0000, -0.0000]
  - `base_link_geom_part_007` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0057, 0.0031, 0.0264]
    - Center offset: [-0.0005, 0.0000, 0.0029]
  - `base_link_geom_part_008` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0057, 0.0030, 0.0030]
    - Center offset: [0.0001, -0.0000, 0.0000]
  - `base_link_geom_part_009` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.1644, 0.1975, 0.1990]
    - Center offset: [-0.0019, -0.0043, 0.0042]
  - `base_link_geom_part_010` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0058, 0.0263, 0.0299]
    - Center offset: [0.0002, -0.0018, 0.0024]
  - `base_link_geom_part_011` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0678, 0.1307, 0.1388]
    - Center offset: [0.0091, -0.0215, 0.0009]

### Body: `robot_link1_left`
- **Parent**: `base_link`
- **Mass**: 0.1200 kg
- **Global Pos (standing)**: [0.0749, 0.0007, 0.6459]
- **Global Quat (w,x,y,z)**: [-0.0445, -0.7057, 0.6701, 0.2256]
- **Joints**:
  - `robot_base_link_r1` (hinge) - Axis: [-1.0000, 0.0000, -0.0000] - Limits: [0.4000, 2.0000]
- **Geoms**:
  - `robot_link1_left_geom` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0200, 0.0350, 0.1000]
    - Center offset: [0.0000, 0.0000, -0.0099]

### Body: `robot_link2_left`
- **Parent**: `robot_link1_left`
- **Mass**: 0.1200 kg
- **Global Pos (standing)**: [0.0171, 0.1514, 0.0838]
- **Global Quat (w,x,y,z)**: [0.0756, -0.7030, 0.6987, 0.1089]
- **Joints**:
  - `robot_link1_left_r8` (hinge) - Axis: [-1.0000, 0.0000, 0.0000] - Limits: unlimited
- **Geoms**:
  - `robot_link2_left_geom` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0214, 0.0458, 0.1934]
    - Center offset: [0.0007, -0.0000, -0.0171]

### Body: `robot_wheel_left`
- **Parent**: `robot_link2_left`
- **Mass**: 0.2500 kg
- **Global Pos (standing)**: [0.0000, 0.1973, 0.0360]
- **Global Quat (w,x,y,z)**: [0.0884, -0.6718, -0.0960, 0.7291]
- **Joints**:
  - `robot_link2_left_r3` (hinge) - Axis: [0.0000, -1.0000, 0.0000] - Limits: unlimited
- **Geoms**:
  - `robot_wheel_left_geom` (cylinder) - Size parameters: [0.0360, 0.0222, 0.0000]
  - `robot_wheel_left_collision` (sphere) - Size parameters: [0.0360, 0.0000, 0.0000]

### Body: `robot_link3_left`
- **Parent**: `base_link`
- **Mass**: 0.0300 kg
- **Global Pos (standing)**: [0.0595, 0.0961, 0.2476]
- **Global Quat (w,x,y,z)**: [0.6846, 0.1771, 0.0061, -0.7071]
- **Joints**:
  - `robot_base_link_r2` (hinge) - Axis: [-1.0000, -0.0000, -0.0000] - Limits: unlimited
- **Geoms**:
  - `robot_link3_left_geom` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0288, 0.0850]
    - Center offset: [0.0000, 0.0006, 0.0000]

### Body: `robot_link3_right`
- **Parent**: `base_link`
- **Mass**: 0.0300 kg
- **Global Pos (standing)**: [0.0143, -0.1374, 0.3463]
- **Global Quat (w,x,y,z)**: [0.0950, 0.7007, 0.6522, 0.2731]
- **Joints**:
  - `robot_base_link_r4` (hinge) - Axis: [1.0000, 0.0000, 0.0000] - Limits: unlimited
- **Geoms**:
  - `robot_link3_right_geom` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0288, 0.0850]
    - Center offset: [0.0000, -0.0006, 0.0000]

### Body: `robot_link1_right`
- **Parent**: `base_link`
- **Mass**: 0.1200 kg
- **Global Pos (standing)**: [0.0749, -0.2091, 0.5714]
- **Global Quat (w,x,y,z)**: [-0.6622, -0.2479, -0.0680, -0.7038]
- **Joints**:
  - `robot_base_link_r9` (hinge) - Axis: [1.0000, 0.0000, -0.0000] - Limits: [0.4000, 2.0000]
- **Geoms**:
  - `robot_link1_right_geom` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0200, 0.0350, 0.1000]
    - Center offset: [0.0000, -0.0000, -0.0099]

### Body: `robot_link2_right`
- **Parent**: `robot_link1_right`
- **Mass**: 0.1200 kg
- **Global Pos (standing)**: [0.2723, -0.0478, -0.0308]
- **Global Quat (w,x,y,z)**: [-0.6996, 0.1027, 0.2803, -0.6492]
- **Joints**:
  - `robot_link1_right_r6` (hinge) - Axis: [1.0000, -0.0000, -0.0000] - Limits: unlimited
- **Geoms**:
  - `robot_link2_right_geom` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0214, 0.0458, 0.1934]
    - Center offset: [0.0007, 0.0000, -0.0171]

### Body: `robot_wheel_right`
- **Parent**: `robot_link2_right`
- **Mass**: 0.2500 kg
- **Global Pos (standing)**: [-0.0162, -0.0983, 0.0342]
- **Global Quat (w,x,y,z)**: [-0.6464, -0.0851, -0.7517, -0.0990]
- **Joints**:
  - `robot_link2_right_r7` (hinge) - Axis: [0.0000, 1.0000, 0.0000] - Limits: unlimited
- **Geoms**:
  - `robot_wheel_right_geom` (cylinder) - Size parameters: [0.0360, 0.0222, 0.0000]
  - `robot_wheel_right_collision` (sphere) - Size parameters: [0.0360, 0.0000, 0.0000]

### Body: `command_slider_body`
- **Parent**: `world`
- **Mass**: 0.0000 kg
- **Global Pos (standing)**: [0.0000, 0.0000, -10.0000]
- **Global Quat (w,x,y,z)**: [1.0000, 0.0000, 0.0000, 0.0000]
- **Joints**:
  - `cmd_linear_x` (hinge) - Axis: [1.0000, 0.0000, 0.0000] - Limits: unlimited
  - `cmd_angular_z` (hinge) - Axis: [0.0000, 1.0000, 0.0000] - Limits: unlimited
