# Lessons Learned

## URDF闭环连杆 → MuJoCo connect约束: marker坐标转换

### 背景

轮腿机器人使用连杆机构（五杆/四杆），URDF不支持闭环。
Fusion 360中断开link2/link3的连接，用marker（薄圆柱体）标记断开处的连接点。
在MuJoCo XML中用 `<equality><connect>` 闭合连杆。

### 核心陷阱: body frame不等于URDF link frame

**当MuJoCo编译URDF（或手动构建MJCF对应URDF结构）时，
若XML中geom的pos/euler全为零，说明URDF的visual origin已被bake到body frame中。**

此时：
- MuJoCo body frame = URDF visual frame
- MuJoCo body frame ≠ URDF link frame

直接使用URDF中fixed joint的origin xyz作为site坐标**是错误的**，
会导致site偏差0.3m量级。

### 正确的坐标转换流程

1. 从URDF中提取marker的fixed joint origin xyz → `marker_xyz`（URDF link frame下坐标）
2. 从URDF中提取父link的visual origin → `visual_xyz`, `visual_rpy`
3. 计算visual旋转矩阵 → `visual_rot = Rotation.from_euler("xyz", visual_rpy)`
4. 转换:

```python
from scipy.spatial.transform import Rotation

# URDF visual origin
visual_xyz = np.array([...])  # 从 <visual><origin xyz="...">
visual_rot = Rotation.from_euler("xyz", visual_rpy).as_matrix()

# marker在URDF link frame中的位置
marker_xyz = np.array([...])  # 从 fixed joint <origin xyz="...">

# 转换为MuJoCo body local坐标 (= site pos)
site_center = visual_rot.T @ (marker_xyz - visual_xyz)

# 如果需要两个site模拟铰链(pin joint)，
# 沿marker的圆柱轴向(Z)偏移±0.005m:
marker_rot = Rotation.from_euler("xyz", marker_rpy).as_matrix()
marker_z_local = visual_rot.T @ marker_rot[:, 2]

site_a = site_center + 0.005 * marker_z_local
site_b = site_center - 0.005 * marker_z_local
```

### 验证方法

1. 更新XML后渲染，将site设为可见红球（`rgba="1 0 0 1" size="0.005"`）
2. 近距离观察红球是否在连杆交汇处
3. qpos=0时配对site有间隙是正常的（断开的链），connect约束会在仿真中拉合
4. 完成验证后将site改回透明（`rgba="0 0 0 0"`）

### 双site模拟铰链的原理

单个connect约束 = 球铰（3个位移自由度被约束）。
两个connect点沿轴向偏移 → 额外约束绕连线的旋转自由度 → 近似铰链(pin joint)。

### connect约束参数建议

```xml
<connect site1="..." site2="..." solref="0.001 1" solimp="0.99 0.999 0.001 0.5 2" />
```

- `solref="0.001 1"`: 高刚度
- `solimp="0.99 0.999 0.001 0.5 2"`: 严格的阻抗参数
- 配合 `<option timestep="0.001"/>` 使用以提高数值稳定性

### 常见错误

| 错误 | 后果 |
|------|------|
| 直接用URDF fixed joint xyz作site pos | 偏差~0.3m，红球飘到机器人外面 |
| 混淆URDF世界坐标和MuJoCo世界坐标 | base_link有z=0.31偏移和X轴翻转 |
| 用weld代替connect | 过约束，与已有hinge冲突，数值不稳定 |
| keyframe qpos全零 | 不一定对应正确的站立/闭合构型 |

### 平衡态定义

静态XML中的连杆构型是未闭合的近似初值。MuJoCo的equality/connect约束在仿真开始后将site拉合。

约束求解器的固有残差约为1mm（与solimp/solref参数无关），这是软约束的正常行为。

可重复的平衡态通过数值求解获得：从qpos=0出发，加阻尼仿真至收敛，提取连杆关节角作为keyframe。
轮子关节（robot_link2_left_r3, robot_link2_right_r7）无阻尼自由旋转，keyframe中置零。

当前base_link固定于世界坐标系（无free joint），后续做LQR平衡控制需要添加free joint。
