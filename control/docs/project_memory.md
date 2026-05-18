# Project Memory

## 当前代码状态（2026-05-18）

本文件记录的是早期 XML 建模阶段的坐标经验。当前仿真入口是 `sim/robot/robot.urdf`，运行时由 `sim/model_xml.py` 转换并处理成 MJCF；不要用下文的旧 XML `base_link` 位姿判断当前仿真结构。

## URDF → MuJoCo MJCF 坐标系映射

**关键结论**: MuJoCo编译URDF时，body frame = URDF visual frame（不是URDF link frame）。

原因：URDF中visual/collision有origin offset（xyz + rpy），MuJoCo编译后geom pos="0 0 0"，
说明visual origin被bake到了body frame中。

**转换公式**：将URDF link frame中的点转换为MuJoCo body frame中的坐标：
```
p_mj_body = visual_rot^T @ (p_urdf_link - visual_xyz)
```

### connect site坐标计算

URDF中marker通过fixed joint挂在link2/link3上。
marker的fixed joint origin xyz给出了marker在URDF link frame中的位置。
通过上述公式转换为MuJoCo body local坐标作为site pos。

site_a和site_b沿marker的Z轴（圆柱轴向）偏移±0.005m，
用两个connect点模拟铰链约束。

## base_link偏移

本节为历史 XML 记录，当前受控 MJCF 以 `sim/model_xml.py` 生成后的 `stand` keyframe 为准。

- URDF: root_link(0,0,0) → base_link via rpy="pi 0 0" xyz="0 0 0"
- MuJoCo XML: base_link pos="0 0 0.31" euler="pi 0 0"
- 差异：MuJoCo额外添加了z=0.31的高度偏移
