# 调试经验与已知问题

## 当前代码状态（2026-05-18）

本文件包含多轮历史调试记录。阅读旧章节前先以当前代码为准：

- 仿真入口是 `sim/robot/robot.urdf`，运行时由 `sim/model_xml.py` 生成并处理成 MJCF。
- 平衡 LQR 当前是 5D：`[pitch, pitch_rate, roll, roll_rate, wheel_vel]`，不含 `wheel_pos`。
- `pitch_trim` 已删除，当前由 `equilibrium_pitch_from_geometry()` 按当前几何计算自然平衡 pitch。
- JumpPhase 当前为 `STAND / CROUCH / EXTEND / FLIGHT / LAND / FALLEN`，没有 `RECOVER`。
- 当前 VMC 是 `cmd_height -> LUT -> active motor angle target -> joint-space PID + gravity feedforward`，不使用 `qfrc_bias`。
- 当前物理 actuator：轮子 `[-1.0, 1.0]`，腿部 MJCF `[-12.5, 12.5]`；`STAND` / `CROUCH` / `LAND` 阶段腿部软限制为 `[-3.5, 3.5]`。
- 当前 `cmd_height` 运行时范围为 `[0.118, 0.142]`。
- `STAND_THEN_DRIVE_PARAMS` 已按当前 5D LQR 参数替换，旧“不可用”记录只说明历史问题。

## 已解决: 机器人腿部塌陷

**原问题**: 仿真中机器人腿部在 0.5s 内对称折叠，机体从初始高度 0.06m 垂直下沉穿过地面。

**根因**: 三个问题叠加：
1. `ctrlrange=[-0.5, 0.5]` 远低于实际电机能力（DM4310 额定 3.5Nm/峰值 12.5Nm，DMH6215 额定 1Nm/峰值 2Nm）
2. VMC 纯 PD 无重力前馈——在目标高度时 height_error=0 导致零力矩输出
3. **base 和 link body 之间存在大量自碰撞**（collision_base_main 与 collision_link2_left/right 穿透 13mm），产生 26 个接触点，干扰正常力传递

**解决方案**:
1. 根据电机手册将 ctrlrange 分离：轮子 [-1.0, 1.0]（DMH6215 额定 1Nm），腿 [-3.5, 3.5]（DM4310 额定 3.5Nm）
2. 历史版本曾添加 `qfrc_bias` 重力/科氏力补偿；当前 VMC 不再使用该路径
3. **contype/conaffinity 位掩码分组**：base/link 碰撞代理 contype=2, conaffinity=1，轮子 contype=1, conaffinity=3。同组 contype=2 不互碰
4. 200 轮自动优化 LQR/VMC 参数：kp=47, kd=11.3, Q=[1118,210,9.4,40.6], R=[0.86,4.57]

**结果**: 10s 站稳测试通过（默认 fall_height=0.05），高度 0.061±0.004m，pitch < 0.001 rad，零饱和。接触点从 26 降到 4。

## 模型相关

### 机器人沉入地面

**问题**: 仿真中机器人穿透地面，视觉上"沉入"地板。

**原因**: mesh geom 同时作为视觉和碰撞几何，高面数 STL 的碰撞检测不稳定。

**解决方案**: `model_xml.py` 将所有 mesh geom 标记为 visual-only（contype=0, conaffinity=0），单独添加 box/cylinder 碰撞代理。轮子使用已有的 cylinder geom 启用碰撞。

### STL 包围盒碰撞代理

**方法**: 对每个 body 的 STL mesh 计算包围盒中心和半尺寸，在 body 局部坐标系下添加 box geom 作为碰撞代理。碰撞代理设为 group=3、rgba 半透明，不影响视觉渲染但提供稳定碰撞。

**参考**: `_COLLISION_PROXY_DEFS` 字典在 `model_xml.py` 中定义。

### base_link.stl 超大 (108MB)

**原因**: CAD 导出的原始 mesh 面数极高。

**处理**: `mujoco_mesh_preprocess.py` 对超大 STL 进行分片处理。测试中将 STL 大小限制标记为 xfail。

### 轮子简化为 cylinder

**原因**: 轮子 STL mesh 作为碰撞几何不稳定。

**方法**: 从 STL 顶点计算半径和厚度，用 cylinder primitive 替代。轮子 geom 启用 friction="1.0 0.02 0.001"。

## 控制器相关

### 纯 VMC 跳跃失败

**现象**: `test_vmc_jump_rollout_reaches_small_jump_height` 标记为 xfail。纯 VMC 跳跃时 `max_jump_height=0.0`，机器人直接摔倒。

**原因**: VMC 只控制腿部力矩做高度 PD，没有 pitch 平衡。跳跃的腿伸展力矩导致机体 pitch 迅速偏移，随即摔倒。

**结论**: 跳跃必须 LQR+VMC 组合控制。纯 VMC 不具备独立跳跃能力。

### ctrlrange 限制前进能力

**现象**: `test_lqr_vmc_drive_forward_10s_at_least_1m` 标记为 xfail。10s 前进距离仅约 0.26m，远不到 SPEC 目标 1.0m。

**原因**: 统一 `ctrlrange=[-0.5, 0.5]` 对轮子力矩限制过大。LQR 增益主要分配给 pitch 平衡，剩余给速度跟踪的力矩余量极小。

**解决方向**: 需要根据实际电机能力为轮子和腿部设置不同 ctrlrange，或改用速度执行器模型。

### 站稳时高度缓慢下沉

**现象**: 10s 站稳测试中 `min_base_height=-0.0536`（从 0.06m 初始高度下沉约 0.11m）。

**原因**: VMC 高度 PD 的 `kp_height=1.0` 和 `ctrlrange=[-0.5, 0.5]` 不足以抵抗重力引起的缓慢下沉。

**当前处理**: 测试使用 `fall_height=-0.1` 放宽阈值，区分"pitch 发散摔倒"和"高度缓慢下沉"。

**结论**: 这不是 LQR 平衡问题（pitch < 0.001 rad），而是 VMC 高度保持能力不足。需要增大 kp_height 或 ctrlrange。

## 线性化相关

### 数值线性化在闭链约束附近可能不稳定

**已知风险**: `linearize_transition` 使用有限差分（epsilon=1e-6），五连杆闭链通过 equality/connect 约束实现。MuJoCo 的约束求解器在约束面附近可能导致数值雅可比不连续。

**当前状态**: 测试通过（增益矩阵 finite、闭环特征值在单位圆内），但未在大范围状态空间验证。

### LQR P 矩阵与 balance_tangent_state 符号不一致（已修复）

**原问题**: `balance_tangent_state` 的 `pitch_rate` 直接取 `+qvel[wy]`，但由于 `base_link` 绕X轴翻转180°（euler="3.14159 0 0"，keyframe quat=[0,1,0,0]），pitch = -wy_tangent。P 矩阵正确使用 `-1.0` 符号，但 `balance_tangent_state` 未取负。

**影响**: LQR 增益 K 基于 P 矩阵（符号正确）计算，但在线控制输入符号相反的 pitch_rate。K 的 pitch_rate 阻尼项变成正反馈，削弱稳定裕度。系统靠 pitch 位置反馈勉强稳定。

**修复**: `balance_tangent_state` 中 pitch_rate 改为 `-state.pitch_rate`。

**教训**: 当 base_link 有非零初始旋转时，freejoint qvel 的角速度分量符号与角度的变化方向关系取决于初始旋转。P 矩阵和在线状态提取必须使用一致的符号约定。用短时仿真（5-10ms）数值验证符号一致性。

### 轮子前进方向符号（WHEEL_FORWARD_SIGNS）

**问题**: LQR 的 P 矩阵和 balance_tangent_state 对轮子角速度的取值未考虑"正角速度=前进还是后退"。

**验证方法**: 对两轮同时施加 ±0.1 力矩，跑10步（5ms），观察 vx 方向。两轮正力矩都使 vx < 0（后退），前进时两轮 wheel_vel 都 < 0。

**修复**: 引入 `WHEEL_FORWARD_SIGNS`（两轮均为 -1.0），在 P 矩阵、S 矩阵和 `balance_tangent_state` 中统一使用。

**注意**: 旧的优化参数（Q=[1118,210,9.4,40.6], R=[0.86,4.57]）基于错误符号约定优化，站稳测试仍通过但速度跟踪失效，需要重新优化。

## 工作流相关

### sim 和 control 隔离

**规则**: `sim/` 代码不 import `control/` 模块，不发送 CAN 帧，不调用实机 CLI。

**原因**: 防止仿真代码意外触发真实电机。这是安全关键边界。

### 测试分层

1. 无硬件单元测试: `uv run pytest`（纯 Python，不依赖串口或电机）
2. 本机接口测试: `list-ports`、`sniff` 对不存在串口的错误处理
3. 实机冒烟测试: 按严格顺序执行（sniff → enable/disable → pair → group）

## Viewer 相关

### viewer 循环仿真速度过慢导致"慢慢倒下"假象

**现象**: viewer 中机器人看起来"慢慢前后倾倒"，但 headless rollout 验证 5s 内完全稳定（pitch < 0.001 rad）。

**根因**: `model.opt.timestep = 0.0005s`（2000Hz），viewer 循环每帧只调一次 `mj_step`。在 60fps 显示下，仿真以 0.0005 × 60 = 0.03x 实时运行（慢 33 倍）。初始瞬态（~0.5s 仿真时间）在 viewer 中需要 ~17s 墙钟时间才能完成，用户误以为机器人在倒。

**修复**: viewer 循环改为每帧步进多步，使用 `time.perf_counter()` 做墙钟-仿真时间同步。每帧计算 `target_sim_time = sim_start + wall_elapsed`，循环步进直到 `data.time >= target_sim_time`。

**教训**: MuJoCo `launch_passive` 的 viewer 循环必须做实时同步。每帧步数 = round(1 / (timestep × display_fps))，否则小 timestep 模型会以极慢速度运行。

### Optuna SQLite 并发限制

**问题**: 使用 `Optuna` 时配置了 SQLite 作为存储，并在同时发起的多进程（Pool.starmap）中同时读取 enqueue_trial，导致偶发的 `ValueError: Cannot tell a COMPLETE trial.`。

**原因**: 当 `enqueue_trial` 存在时，如果有多个 workers 在极其相近的时间（例如0ms延迟）调用 `study.optimize`，它们可能都会取出同一个 enqueue trial 进行评估，导致第二个完成的 trial 在汇报结果时因为状态已被修改为 COMPLETE 而报错。

**教训**: 如果使用 SQLite 和多进程进行 Optuna 优化，避免对 warm start（enqueue_trial）直接使用大并发（或者通过 `try-except` 忽略此特定的并发错误）。

### 两轮运动不对称（"爬行"运动，部分修复）

**现象**: 机器人前进时左轮转速是右轮的 6-7 倍，导致持续偏航（10秒偏转50°），视觉上表现为"画圆弧"而非直线行驶。

**已修复**:
- `ROOT_QPOS` z 高度 `0.06078` 导致右轮悬空 0.006mm → 已修复为 `0.06067`
- **keyframe r9 角度修正**（2026-05-04）: 
  - 发现 r1（左腿电机）和 r9（右腿电机）的 URDF joint rpy 不同（r1 多一个绕 X 翻转 π），导致相同角度值对应不同物理姿态
  - 旧 keyframe: r1=+0.035, r9=-0.111 → 两轮 geom 高度差 0.43mm，body xpos z 差 39.5mm
  - 通过扫描 r9 角度，找到 r9=-0.2108 使两轮 geom_xpos z 差从 0.43mm 降至 0.018mm
  - 对称力矩测试: 两轮速度比从不稳定（0.14 ~ 26）收敛到 1.09:1
  - **副作用**: 五连杆几何改变，旧的 LQR/VMC 参数不再适用，需重新优化

**关键教训（body xpos vs geom_xpos）**:
- 轮子 cylinder geom 在 body 局部坐标系中有大偏移（~420mm），`body.xpos` 不代表轮心世界位置
- 必须用 `data.geom_xpos` 获取轮子 cylinder 的实际世界位置
- `ROOT_QPOS` z 基于 `geom_xpos` 校准，不能用 `body.xpos` 计算

### 控制器逻辑错误导致的后退和单轮旋转

**现象**: 机器人在 drive 场景下目标速度设为正，但只有一个轮子在转，另一个不转；或者机器人不前进。

**根因1 (角速度索引错误)**: `combined.py` 的 yaw 阻尼计算错误地使用了 `base_angular_velocity[0]` (Roll rate) 而不是 `[2]` (Yaw rate)。修复为 `[2]`。

**根因2 (WHEEL_FORWARD_SIGNS 的正确值)**: 实测验证——对两轮施加 +1.0 力矩会使 vx < 0 (后退)，施加 -1.0 力矩使 vx > 0 (前进)。因此 `WHEEL_FORWARD_SIGNS = -1.0` 是**正确的**。

**根因3 (launch_mujoco 未使用驱动参数)**: `create_controlled_controller` 对所有场景（含 drive）都使用 `STAND_PARAMS`，没有在 drive 场景切换为 `STAND_THEN_DRIVE_PARAMS`。已修复。

**根因4 (LQR 参数阻止前倾)**: 优化器产出的参数 `q_pitch_rate=1307` 极高，LQR 拼命阻止 pitch 变化，导致即使设了 `pitch_lean_gain=0.3`，实际倾斜角也只有 0.001 rad。自平衡轮式机器人**必须前倾才能前进**。手调参数 `q_pitch_rate=30` 后机器人可以正常加速。

**yaw PI 控制 (新增)**: 五连杆不对称导致持续 ~0.25 rad/s 的偏航偏置。纯 P 阻尼 (`yaw_damping`) 无法消除稳态偏置。新增 `yaw_ki` 积分偏航角速率，逐步消除偏置。当前最佳参数: `yaw_damping=1.5, yaw_ki=0.3`，10s 前进 11m。

**教训**: 实测力矩方向比理论推导可靠。始终用短时仿真 (200步) 验证 `+1 力矩 → vx 方向` 再定 WHEEL_FORWARD_SIGNS。

**控制器层面补偿尝试（均失败）**:
1. **独立轮速同步 PI**: 能同步两轮速度，但差速力矩直接导致偏航（50°/10s），效果适得其反
2. **yaw 角度 PD 闭环**: 偏航修正力矩与前进力矩争抢有限的力矩预算（总共约 0.1-0.2 Nm），导致一个轮子反转，另一个超速
3. **yaw 修正限幅**: 限制修正幅度后力矩不足以抵消五连杆耦合产生的持续偏航力矩

**结论**: 问题的根源在 XML 五连杆的 body 定义。每个 body 的局部坐标系原点和方向在 CAD 导出时没有统一为镜像对称：
- link1: 左右在 base_link 下的 pos 的 y/z 差 50.8mm / 42.6mm
- link3: 左右 y 差 103.8mm
- connect site 在各自 body 局部坐标下的位置也不对称（link2 的 y 差 39.2mm）
- 即使关节角度完全不同（左=-0.009 vs 右=+0.420），connect 残差也无法降到 0

**根本修复路径**: 需要从 CAD 重新导出 URDF/XML，确保左右 part 的局部坐标系原点放在各自关节轴心，并且方向满足 x 镜像对称。当前 XML 不适合做这种修正。

**教训**:
- MuJoCo 碰撞检测对 0.01mm 高度差敏感，keyframe 必须保证所有轮都有 negative clearance
- 差速力矩修正偏航与前进力矩存在根本矛盾，力矩预算小的系统中控制器补偿空间极有限
### 五连杆驱动不对称（已彻底修复）

**现象**: 前期的各种偏航、偏转、不对称问题，其实并不是机械几何带来的根本问题，而是仿真控制器应用力矩的位置和方向完全错误。

**根因**:
1. **多驱动了被动关节（作弊行为）**: 机器人的五连杆腿部在硬件上只有一个主动电机（位于 `base_link` 和 `link1` 之间的关节，即 `r1` 和 `r9`），而 `link3` 的关节（`r2` 和 `r4`）是纯被动关节。之前的 `MODEL_SEMANTICS` 错误地将 4 个关节全标记为了主动电机（`leg_motor_joints`），导致 VMC 把用来站立的力矩强加到了无动力的被动关节上。这不仅引发了巨大的内部破坏性力矩，还相当于在物理引擎里“作弊”，让机器人获得了不符合物理现实的稳定力。
2. **CAD 坐标系导致的左右电机受力方向不一致**: 左腿 `r1` 和右腿 `r9` 关节在全球绝对坐标系下的旋转轴向量都是 `[-1, 0, 0]`。但是由于 CAD 导出时坐标系的原因（`r1` 多了一个 `Rx(-π)` 的翻转），它们的局部几何朝向是相反的。这意味着，如果给左右两腿施加相同符号的力矩，左腿会伸直，而右腿会弯曲！

**修复**:
1. 修改 `sim/model_semantics.py`：将 `r2` 和 `r4` 移出 `leg_motor_joints` 归入 `passive_joints`，去除加在被动关节上的作弊假力，完全依靠真实的 4 个电机（2个轮子+2个腿）支撑身体。
2. 修改 `sim/controllers/vmc.py`：`LEG_EFFORT_SIGNS` 仅保留 `r1` 和 `r9`，并将 `r1` 设为 `-1.0`，`r9` 设为 `1.0`。现在 VMC 终于可以对称、同步地抬起机器人的双腿了。

**教训**:
处理闭链五连杆机构时，必须准确理清**哪个是主动电机，哪个是被动轴**。同时，不要想当然地认为“旋转轴相同就该用相同的力矩符号”，必须结合连杆的局部坐标系翻转（如 URDF 的 rpy 差异）来推导真实的物理受力。只有底层的物理模型配置正确，控制器的增益调节才有意义。

## 性能与内存优化

### 自动优化器导致 30GB 巨量内存占用与崩溃

**现象**: 在 M5 笔记本上使用多进程 (workers=4 或更多) 跑 Optuna 优化时，系统内存（含 App Memory 和 File Cache）急剧飙升至 30GB，触发严重的系统 Swap 和卡顿。

**根因**:
1. **频繁生成受控模型并导致文件系统缓存爆炸**: 在 `optimize.py` 中，如果未指定固定的 `cache_dir`，每次进入 `_objective` 时，`run_rollout` 内的 `prepare_controlled_mujoco_xml` 都会调用 `tempfile.mkdtemp`。这导致了在单次优化的数千个 trials 中，于 `/tmp/` 目录下生成了上千个 `mujoco_controlled_*` 和 `mesh_preprocess` 文件夹，包含大量的拆分后 STL mesh。macOS 会主动将这些高频写入的临时文件全部压入内存（File Cache），直接耗尽物理内存。
2. **多进程中的 MuJoCo 实例未主动释放**: 在长生命周期的 worker 进程中，`mujoco.MjModel` 和 `mujoco.MjData` 对象依附于每次生成的 `rollout` 结果上。由于 Python GC 在多进程密集的 C-extension 循环中容易出现延迟回收，内存进一步膨胀。

**修复**:
1. **强制复用全局缓存并实现幂等检查**: 在 `optimize.py` 中强制默认 `cache_dir="tmp/mujoco_cache"`。同时修改 `sim/model_xml.py`，使 `prepare_controlled_mujoco_xml` 利用生成目标文件的 `mtime` 判断是否跳过处理（如果已经存在且比源文件新，则直接返回），避免多进程重复处理和 IO。另外确保内部如 `_add_collision_proxies` 也具有幂等性（添加前检查 `geom` 是否存在）。
2. **强制 GC 回收 MjData 内存**: 
   - `_objective` 末尾使用 `del rollout` 和 `gc.collect()` 强制回收孤立的仿真上下文。
   - `_optimize_worker` 中设置 `study.optimize(..., gc_after_trial=True)`。

**教训**:
当在多进程循环内高频调用具有“构建期”缓存功能（如加载大型 STL mesh 生成 XML）和 C++ 对象实例化的函数时，一定要提供固定的**全局预热缓存路径**并设置**文件层面的幂等性拦截**。绝不能依赖 `/tmp` 目录被动消耗系统的缓存资源。

### qfrc_bias 重力补偿在 2 电机闭链五连杆中完全失效

**现象**: 使用 `qfrc_bias` 做关节级重力补偿后，机器人从 0.06m 飞到 0.38m 再摔落，无法稳定站立。所有 200 轮优化全部 FAIL (fell)。

**根因**: `qfrc_bias` 是整个连杆树（含 freejoint 和闭链约束）的广义力分量。对于闭链五连杆，单个主动关节的 `qfrc_bias` 包含了通过 `equality/connect` 约束耦合过来的多体力矩成分。直接将其作为执行器力矩补偿会产生远超重力的力。在之前 4 个腿电机的版本中碰巧工作（4 个关节过度约束），但 2 个电机时暴露出本质错误。

**修复**: VMC 改为纯 PD 控制 `control = LEG_EFFORT_SIGNS[name] * pd_effort`，完全去除 `qfrc_bias`。纯 PD 可以在 10s 内稳定站立（pitch=0.019）。

**教训**: `qfrc_bias` 做关节级补偿仅适用于开链或全主动关节的机器人。对于闭链五连杆等欠驱动系统，不能直接使用某个 DOF 的 `qfrc_bias` 作为该 DOF 执行器的补偿力矩。

### Rollout 启动瞬态需要免疫期

**现象**: 机器人从 keyframe 释放后有约 0.5s 的瞬态下降（z 从 0.06 降到 -0.025），之后在 2s 内完全恢复并稳定。但 `fall_height_drop=0.02` 的跌倒检测在前 0.15s 就触发了误判。

**修复**: `rollout.py` 中 fall detection 和 `min_base_height` 跟踪添加 1s 启动免疫期。免疫期内不检查跌倒，不更新 min_base_height。

**教训**: 五连杆机器人从 keyframe 释放到控制器收敛有显著的瞬态过程（机器人先下沉再弹回），跌倒检测必须在瞬态结束后才能生效。

### Optuna enqueue_trial 多进程并发竞态

**现象**: 多个 worker 同时取到 enqueue_trial 进行评估，第二个完成的报 `ValueError: Cannot tell a COMPLETE trial`，导致整个优化崩溃。

**修复**: `_optimize_worker` 中用 try-except 捕获此特定 ValueError，重新计算剩余 trial 数量继续优化。

### 启动撞地：WHEEL_FORWARD_SIGNS 和 ROOT_QPOS 联合导致

**现象**: 机器人一启动就翻倒撞地。LQR 轮子力矩在 ±3.0 Nm 之间逐步 bang-bang 振荡（频率 = 仿真频率/2），50ms 内将机体弹射到 z=0.09m，丢失接触后翻倒。

**根因 1 (WHEEL_FORWARD_SIGNS=+1 应为 -1)**:
- `model_semantics.py` 中 `WHEEL_FORWARD_SIGNS` 初始化为 `+1.0`
- 实测验证: `+1.0 力矩 → vx < 0（后退）`，因此前进方向符号应为 `-1.0`
- `project_memory.md` 和 `lessons_learned.md` 早已记录应为 -1.0，但代码从初始提交就一直是 +1.0
- 影响: P/S 矩阵中轮速方向反转、速度 PI 前馈方向反转、yaw 修正方向反转
- pitch 平衡碰巧工作（P 和 S 的方向错误在乘积中抵消），但速度跟踪和 yaw 修正全部失效

**根因 2 (ROOT_QPOS z=0.06067 导致右轮悬空)**:
- 左轮 clearance=-0.12mm（接地），右轮 clearance=+0.31mm（悬空 0.31mm）
- 启动后只有左轮接地 → 机体失去右侧支撑 → 侧倾
- 修正为 z=0.06030 后两轮都接地（左 -0.49mm，右 -0.06mm，4 个接触点）

**修复**: 
1. `WHEEL_FORWARD_SIGNS` 改为 `-1.0`
2. `ROOT_QPOS` z 从 0.06067 改为 0.06030
3. 测试断言更新为匹配 2 电机配置

**结果**: 5s stand 完全稳定（pitch=0.00001 rad），轮子力矩从 ±3.0 Nm 振荡降到 ±0.02 Nm 平稳控制。

**教训**: 
1. 文档记录的修复必须立即落实到代码。"文档写了但代码没改"是一种隐蔽的技术债务。
2. ROOT_QPOS 校准必须验证**所有轮子**的 clearance，不能只基于一侧。用 `geom_xpos` 而非 `body.xpos` 获取轮心世界位置。
3. WHEEL_FORWARD_SIGNS 错误在 pitch 平衡中碰巧被 P/S 矩阵乘积抵消，导致 stand 场景看似工作，掩盖了方向错误。始终用独立的力矩方向验证脚本确认。

### drive 场景转圈：yaw 差速方向与轮子布局不匹配

**现象**: drive 模式下机器人原地逆时针画圈, 5s 内 yaw 偏移 109°。

**根因**: 两轮物理布局是**前后布局**(X 轴对称), 不是传统的左右布局(Y 轴对称)。
- left_r3 geom 在 world [+0.12, -0.34], right_r7 geom 在 world [-0.12, -0.32]
- 两轮都在 Y<0 侧, X 方向上一前一后
- 差速产生 yaw 的方向与传统左右布局假设相反

**代码问题**: `combined.py` line 158 的差速分配 `(-1 if i==0 else +1)` 基于传统左右布局, 导致 yaw 修正变成正反馈。

**修复**: 翻转差速符号为 `(+1 if i==0 else -1)`, 使修正力矩方向与实际布局匹配。

### collision_base_main 碰撞代理阻碍前进

**现象**: drive 模式下轮子在转但机体不动 (forward_wheel_vel=+0.25, vx=-0.01)。

**根因**: `collision_base_main` (base_link 碰撞代理盒子) 的底面在站立时触及地面 (接触点 z≈0), 分走了轮子的法向力, 同时碰撞盒子的摩擦力阻碍了水平方向的移动。

**附带问题**: 如果只禁止 base_link 与地面碰撞 (conaffinity=2), 它仍然会与相邻 link2 碰撞代理互穿, 产生 8 个寄生碰撞点, 将机体推向地面 (z 从 0.06 降到 -0.06)。

**修复**: 保留 base_link 碰撞代理用于跌倒保护, 但设置 `friction="0 0 0"` 消除水平摩擦力。
直接移除碰撞代理会改变物理特性导致不稳定。

**教训**: 
1. 碰撞代理的 size 必须确保在所有工作姿态下都不会意外触及地面。
2. 修改碰撞配置后必须用实际 launcher 参数验证, 不能只用脚本验证。
3. 移除碰撞体是高风险操作, 优先用 friction=0 或 conaffinity 调整。

### STAND_THEN_DRIVE_PARAMS 在修正 WHEEL_FORWARD_SIGNS 后不可用（历史）

**现象**: drive 场景启动后机器人弹飞 (z 飙升到 0.33), yaw 瞬间偏移 180°, 力矩全部打满 ±3.0。

**根因**: 该参数集由 Optuna 在错误的 WHEEL_FORWARD_SIGNS=+1 下优化得到。kp_height=11.2 在旧碰撞配置下碰巧工作, 但在修正后的物理模型下完全不足。

**临时修复**: drive 场景改用 STAND_PARAMS, 后续需要用 Optuna 重新优化。

**当前状态**: `sim/controllers/default_params.py` 中的 `STAND_THEN_DRIVE_PARAMS` 已按当前 5D LQR 参数替换，旧 Optuna 参数不再是运行时默认值。

## 物理模型重构与控制算法参数修正

### 平衡构型（鸟腿）的质心与姿态匹配（历史）

**原问题**: 在修复了模型直立姿态（0.47m）后，LQR和参数优化程序仍旧频繁报错 `excessive_pitch` (pitch > 1.5)，且在仿真开始时车体弹射。
**排查过程**:
1. **穿模弹射**：通过提取初始姿态高度（Z=0.47）发现此时车轮在地平线以下 0.0341m，引发初始穿透弹射。因此必须将 `ROOT_QPOS` 的高度设定为 `0.5041m`。
2. **重心后置倾角**：机器人腿部质量占比高（2.3kg/3.3kg），采用+0.6/-0.6的鸟腿构型后，整体 CoM 会向后偏移（Y=+0.22）。必须向后倾斜约 15度（Pitch = +0.26 弧度）才能使总 CoM 重新与轮轴对齐。
**修复**: 
- 设置初始四元数包含 0.26 倾角：`0.991562 0.129634 0 0`
- LQR 默认目标平衡点 `pitch_trim` 设为 `0.26`
- 优化器搜索空间 `pitch_trim` 缩小至 `[0.2, 0.32]`

**当前状态**: `pitch_trim` 参数已删除，当前自然平衡 pitch 由 `equilibrium_pitch_from_geometry()` 计算。

### LQR 控制器增益符号：前馈控制符号反向

**原问题**: LQR 在目标速度跟踪中发生正反馈摔倒。
**原因**: 当 `target_velocity` 要求加速前进（-Y 方向，因为轮子前进等于负Y）时，`v_err = target - current > 0`。原始 `pitch_lean` 的相加逻辑导致 `target_pitch` 增大，这在右手法则下意味着向后仰。向后仰会导致重心后移，车轮必须向后加速去接，结果背道而驰！此外，两轮电机转向轴均朝向同一侧，而 `WHEEL_FORWARD_SIGNS` 遗留有不对称符号，也导致了差速 Yawing。
**修复**: 
- `WHEEL_FORWARD_SIGNS` 对于两个电机统一设为 `-1.0`。
- 将 `pitch_lean` 的合并方式改为相减（`pitch_trim - pitch_lean`），确保加速时降低目标 pitch (前倾)。
- 在 `yaw_correction` 时，由于两轮基础符号已被统一，原先在两轮上的修正符号从 `(+1, -1)` 修正为 `(-1, +1)`，避免 yaw 的正反馈加速。

**当前状态**: 当前代码使用 `equilibrium_pitch_from_geometry() + pitch_lean` 更新目标 pitch，`pitch_trim` 不再存在。

### LEG_EFFORT_SIGNS 两电机互相抵消（2026-05-05）

**现象**: VMC 的腿电机力矩从第一步就打满饱和（±3.5 Nm），但机器人高度几乎不变（0.2354 vs 自然高度 0.2354），VMC 完全无法控制高度。

**根因**: `LEG_EFFORT_SIGNS = {r1: +1, r9: -1}` 导致两个电机施加方向**相反**的力矩，互相抵消。r1 和 r9 的关节轴在世界坐标系方向相反（`[+1,0,0]` vs `[-1,0,0]`），因此要同向旋转（伸展腿部），需要**相同符号**的力矩。

**验证**: 
- `r1=+3.5, r9=+3.5` → z=0.31（最大伸展）
- `r1=-3.5, r9=-3.5` → z=0.05（最大收缩）
- `r1=+3.5, r9=-3.5`（旧值）→ z=0.236（≈自然高度，无效！）

**修复**: `LEG_EFFORT_SIGNS = {r1: +1, r9: +1}`

### WHEEL_FORWARD_SIGNS 导致 LQR pitch 可控性极低

**现象**: 两轮虽然能让机器人前进（ctrl都取负时 vx > 0），但 LQR B 矩阵的 pitch 行两列符号相反（+4.09e-3 vs -1.45e-3），合并后 pitch 增益只有 2.64e-3。

**根因**: 两轮关节轴在世界系方向相反（r3=[+1,0,0], r7=[-1,0,0]）。前进时虽然 ctrl 符号相同，但物理上一个让 pitch rate 增加、另一个让 pitch rate 减少。WHEEL_FORWARD_SIGNS 应该编码**物理力矩方向**与**虚拟控制方向**的映射，而不是简单的"前进时 qvel 的符号"。

**修复**: `WHEEL_FORWARD_SIGNS = {r3: -1, r7: +1}`。这样 S 矩阵让虚拟控制通过 r3 给负力矩、通过 r7 给正力矩，物理上两轮同向滚动，pitch 增益从 2.64e-3 翻倍到 5.54e-3。

### VMC ramp 起点与实际初始高度不匹配

**现象**: 机器人启动后 1.5 秒内腿电机一直饱和抖动。

**根因**: VMC 中硬编码 `init_h = 0.06`（旧模型高度），但实际初始高度已改为 0.24~0.5m。在 t=0 时 VMC 的 target=0.06 vs actual=0.5，产生 -133 Nm 的误差信号，被 clip 后持续输出 -3.5 Nm 满力矩。

**修复**: 删除 ramp 逻辑，直接使用 `nominal_height` 作为目标。

### 物理可达高度范围

**发现**: 当前五连杆 + 2 电机构型（r1/r9 同向出力）：
- 最大伸展高度: z = 0.31m
- 自然静止高度: z = 0.24m
- 最大收缩高度: z = 0.05m

之前设定的 `nominal_height = 0.47m` 和 `0.5041m` 完全不可达，导致 VMC 永远饱和。

### 2026-05-13 (B): VMC 高度 metric 改为世界 z 距离

**变更**: `_leg_height` 由“base 到 wheel 在 base 本体 z 方向的投影”改为“base `xipos.z` 到 wheel `xipos.z` 的世界 z 距离”。解析 Jacobian 同步改为 `mj_jac` 的世界 z 行差值。

**验证**: stand keyframe 下 wheel body `xipos.z` 与 wheel collision proxy `geom_xpos.z` 一致，误差约 `1e-8m`，因此当前实现使用 wheel body `xipos`。

### 2026-05-13 (P3): LQR 扩展为 6D 并纳入 base roll（历史）

**变更**: LQR 状态从 `[pitch, pitch_rate, wheel_pos, wheel_vel]` 扩为 `[pitch, pitch_rate, roll, roll_rate, wheel_pos, wheel_vel]`。LQR 输出改为两个虚拟量: `forward_wheel` 和 `roll_diff_leg`。Yaw PI 仍只作用于轮子差速，VMC 仍只作用于腿部同向高度控制。

**关键实测**: 当前模型左右轮中心主要沿 base X 分离，差值约 `0.249m`；沿 base Y 分离接近 0。因此 roll 子系统的 `track_width` 必须取左右轮中心在 base X 方向的距离。若按 base Y 投影，roll 杠杆臂会接近 0，LQR roll 增益会被放大到不可用。

**符号**: 短时仿真验证显示“左腿 +1Nm、右腿 -1Nm”会让 `roll` 朝负方向变化。因此虚拟 `roll_diff_leg > 0` 映射为左腿负、右腿正，并用测试固定。

**结果**: `verify_cmd_height_tracking.py` 中 `max_abs_roll` 约 `0.0002rad`，roll 已进入 LQR 控制。

**当前状态**: `CombinedController` 主路径已改为 5D LQR：`[pitch, pitch_rate, roll, roll_rate, wheel_vel]`。旧 6D 路径仍用于历史测试/对照，不是当前控制器主路径。

## 2026-05-05: 轮腿机器人对称性与控制问题排查
1. **坐标系镜像的轴向量（Axial Vector）处理**：
   - 现象：在进行结构左右镜像时，直接使用 `[-ax, ay, az]` 反转关节旋转轴导致运动学严重扭曲，导致模型在 MuJoCo 中启动时解体（散架）。
   - 原因：关节旋转轴属于轴向量（Axial Vector/Pseudovector），在全局空间经历反射变换 $M_x$ 时，其变换法则为 $-M_x w_{world}$。换算到镜像后的局部坐标系中，正确的轴向量镜像公式应当是 `[ax, -ay, -az]` 而不是单纯翻转 x 轴。
2. **偏航修正的正反馈问题**：
   - 现象：机器人在启动时出现极高的持续偏航角速度（Yaw Drift），甚至原地转圈。
   - 原因：控制逻辑中 `yaw_correction` 应用到轮子力矩时，符号与物理旋转方向不匹配，导致正误差产生负反馈修正，但在经过 `_wheel_control_selection` 映射后，实际上加剧了偏航旋转，形成了正反馈。
   - 解决：反转 `yaw_correction` 的符号。
