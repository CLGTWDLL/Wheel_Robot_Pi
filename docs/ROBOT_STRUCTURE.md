# Robot Structure and Kinematics

> **IMPORTANT:** This document represents the robot state after `sim/robot/robot.urdf` is converted and postprocessed into MJCF, then reset to the `stand` keyframe. Use this processed model for simulation structure checks, not the raw URDF alone.

## Bodies and Kinematic Tree

### Body: `world`
- **Parent**: `world`
- **Mass**: 0.0000 kg
- **Global Pos (standing)**: [0.0000, 0.0000, 0.0000]
- **Global Quat (w,x,y,z)**: [1.0000, 0.0000, 0.0000, 0.0000]
- **Geoms**:
  - `floor` (plane) - Size parameters: [0.0000, 0.0000, 1.0000]

### Body: `base_link`
- **Parent**: `world`
- **Mass**: 0.9400 kg
- **Global Pos (standing)**: [0.0000, 0.0000, 0.1750]
- **Global Quat (w,x,y,z)**: [1.0000, 0.0000, 0.0000, 0.0000]
- **Joints**:
  - `root` (free) - Axis: [0.0000, 0.0000, 1.0000] - Limits: unlimited
- **Geoms**:
  - `base_link_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.1162, 0.1843, 0.2063]
    - Center offset: [0.0038, 0.0023, 0.0004]
  - `base_link_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.1162, 0.1843, 0.2063]
    - Center offset: [0.0038, 0.0023, 0.0004]
  - `base_link_collision_proxy` (box) - Size parameters: [0.1000, 0.0500, 0.0300]

### Body: `link1_right`
- **Parent**: `base_link`
- **Mass**: 0.0400 kg
- **Global Pos (standing)**: [-0.0940, 0.0115, 0.1804]
- **Global Quat (w,x,y,z)**: [0.6584, -0.2553, -0.2588, -0.6590]
- **Joints**:
  - `base_link_旋转-1` (hinge) - Axis: [0.0026, -1.0000, 0.0043] - Limits: [0.2618, 1.6581]
- **Geoms**:
  - `link1_right_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0200, 0.0350, 0.1000]
    - Center offset: [0.0000, 0.0000, 0.0099]
  - `link1_right_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0200, 0.0350, 0.1000]
    - Center offset: [0.0000, 0.0000, 0.0099]
  - `link1_right_collision_proxy` (capsule) - Size parameters: [0.0080, 0.0367, 0.0000]

### Body: `link2_right`
- **Parent**: `link1_right`
- **Mass**: 0.0800 kg
- **Global Pos (standing)**: [-0.1090, 0.0433, 0.1152]
- **Global Quat (w,x,y,z)**: [0.7011, 0.0846, 0.0818, -0.7033]
- **Joints**:
  - `link1_right_旋转-5` (hinge) - Axis: [-0.0026, 1.0000, -0.0043] - Limits: unlimited
- **Geoms**:
  - `link2_right_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0214, 0.0458, 0.1934]
    - Center offset: [0.0007, -0.0000, 0.0171]
  - `link2_right_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0214, 0.0458, 0.1934]
    - Center offset: [0.0007, -0.0000, 0.0171]
  - `link2_right_collision_proxy` (capsule) - Size parameters: [0.0080, 0.0497, 0.0000]

### Body: `wheel_right`
- **Parent**: `link2_right`
- **Mass**: 0.5000 kg
- **Global Pos (standing)**: [-0.1090, -0.0152, 0.0352]
- **Global Quat (w,x,y,z)**: [0.0846, -0.7011, 0.7033, 0.0818]
- **Joints**:
  - `link2_right_旋转-12` (hinge) - Axis: [-0.0026, -1.0000, 0.0043] - Limits: unlimited
- **Geoms**:
  - `wheel_right_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0445, 0.0680, 0.0680]
    - Center offset: [0.0034, -0.0000, -0.0000]
  - `wheel_right_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0445, 0.0680, 0.0680]
    - Center offset: [0.0034, -0.0000, -0.0000]
  - `wheel_right_collision_proxy` (sphere) - Size parameters: [0.0360, 0.0000, 0.0000]

### Body: `link1_left`
- **Parent**: `base_link`
- **Mass**: 0.0400 kg
- **Global Pos (standing)**: [0.0940, 0.0115, 0.1804]
- **Global Quat (w,x,y,z)**: [0.6584, -0.2553, -0.2588, -0.6590]
- **Joints**:
  - `base_link_旋转-2` (hinge) - Axis: [0.0026, -1.0000, 0.0043] - Limits: [0.2618, 1.6581]
- **Geoms**:
  - `link1_left_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0200, 0.0350, 0.1000]
    - Center offset: [0.0000, 0.0000, -0.0099]
  - `link1_left_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0200, 0.0350, 0.1000]
    - Center offset: [0.0000, 0.0000, -0.0099]
  - `link1_left_collision_proxy` (capsule) - Size parameters: [0.0080, 0.0367, 0.0000]

### Body: `link2_left`
- **Parent**: `link1_left`
- **Mass**: 0.0800 kg
- **Global Pos (standing)**: [0.1090, 0.0433, 0.1152]
- **Global Quat (w,x,y,z)**: [0.7011, 0.0846, 0.0818, -0.7033]
- **Joints**:
  - `link1_left_旋转-6` (hinge) - Axis: [-0.0026, 1.0000, -0.0043] - Limits: unlimited
- **Geoms**:
  - `link2_left_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0214, 0.0458, 0.1934]
    - Center offset: [0.0007, -0.0000, -0.0171]
  - `link2_left_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0214, 0.0458, 0.1934]
    - Center offset: [0.0007, -0.0000, -0.0171]
  - `link2_left_collision_proxy` (capsule) - Size parameters: [0.0080, 0.0497, 0.0000]

### Body: `wheel_left`
- **Parent**: `link2_left`
- **Mass**: 0.5000 kg
- **Global Pos (standing)**: [0.1090, -0.0152, 0.0352]
- **Global Quat (w,x,y,z)**: [0.0846, -0.7011, 0.7033, 0.0818]
- **Joints**:
  - `link2_left_旋转-13` (hinge) - Axis: [-0.0026, -1.0000, 0.0043] - Limits: unlimited
- **Geoms**:
  - `wheel_left_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0445, 0.0680, 0.0680]
    - Center offset: [0.0034, 0.0000, 0.0000]
  - `wheel_left_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0445, 0.0680, 0.0680]
    - Center offset: [0.0034, 0.0000, 0.0000]
  - `wheel_left_collision_proxy` (sphere) - Size parameters: [0.0360, 0.0000, 0.0000]

### Body: `link3_right`
- **Parent**: `base_link`
- **Mass**: 0.0100 kg
- **Global Pos (standing)**: [-0.0940, 0.0559, 0.2183]
- **Global Quat (w,x,y,z)**: [-0.2447, -0.6624, 0.6631, -0.2482]
- **Joints**:
  - `base_link_旋转-3` (hinge) - Axis: [0.0026, 1.0000, -0.0043] - Limits: unlimited
- **Geoms**:
  - `link3_right_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0288, 0.0850]
    - Center offset: [0.0000, 0.0006, -0.0000]
  - `link3_right_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0288, 0.0850]
    - Center offset: [0.0000, 0.0006, -0.0000]
  - `link3_right_collision_proxy` (capsule) - Size parameters: [0.0060, 0.0224, 0.0000]

### Body: `link3_left`
- **Parent**: `base_link`
- **Mass**: 0.0100 kg
- **Global Pos (standing)**: [0.0940, 0.0559, 0.2183]
- **Global Quat (w,x,y,z)**: [0.6624, -0.2447, -0.2482, -0.6631]
- **Joints**:
  - `base_link_旋转-4` (hinge) - Axis: [0.0026, -1.0000, 0.0043] - Limits: unlimited
- **Geoms**:
  - `link3_left_visual` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0288, 0.0850]
    - Center offset: [0.0000, 0.0006, 0.0000]
  - `link3_left_collision` (Mesh) - **Local Bounding Box Size (X, Y, Z)**: [0.0060, 0.0288, 0.0850]
    - Center offset: [0.0000, 0.0006, 0.0000]
  - `link3_left_collision_proxy` (capsule) - Size parameters: [0.0060, 0.0224, 0.0000]

### Body: `command_slider_body`
- **Parent**: `world`
- **Mass**: 0.0000 kg
- **Global Pos (standing)**: [0.0000, 0.0000, -10.0000]
- **Global Quat (w,x,y,z)**: [1.0000, 0.0000, 0.0000, 0.0000]
- **Joints**:
  - `cmd_linear_x` (hinge) - Axis: [1.0000, 0.0000, 0.0000] - Limits: unlimited
  - `cmd_angular_z` (hinge) - Axis: [0.0000, 1.0000, 0.0000] - Limits: unlimited
  - `cmd_height` (hinge) - Axis: [0.0000, 0.0000, 1.0000] - Limits: unlimited
  - `cmd_jump` (hinge) - Axis: [0.7071, 0.7071, 0.0000] - Limits: unlimited
