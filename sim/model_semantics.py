from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ModelSemantics:
    body_roles: dict[str, str]
    joint_roles: dict[str, str]
    wheel_joints: tuple[str, ...]
    leg_motor_joints: tuple[str, ...]
    passive_joints: tuple[str, ...]
    connect_sites: tuple[str, ...]


# 轮子前进方向符号: 关节正力矩/正角速度对应前进时为 +1, 否则为 -1
# 实测验证: 轮子关节 r3/r7 施加 +1.0 力矩 → vx < 0 (后退), 轮速为正
# 施加 -1.0 力矩 → vx > 0 (前进), 轮速为负
# 因此两轮前进方向符号都取 -1.0
WHEEL_FORWARD_SIGNS: dict[str, float] = {
    "robot_link2_left_r3": -1.0,
    "robot_link2_right_r7": -1.0,
}


MODEL_SEMANTICS = ModelSemantics(
    body_roles={
        "base_link": "base",
        "robot_link1_left": "left_leg_link1",
        "robot_link2_left": "left_leg_link2",
        "robot_link3_left": "left_leg_link3",
        "robot_wheel_left": "left_wheel",
        "robot_link1_right": "right_leg_link1",
        "robot_link2_right": "right_leg_link2",
        "robot_link3_right": "right_leg_link3",
        "robot_wheel_right": "right_wheel",
    },
    joint_roles={
        "robot_base_link_r1": "left_leg_motor",
        "robot_base_link_r2": "left_leg_passive",
        "robot_link1_left_r8": "left_leg_passive",
        "robot_link2_left_r3": "left_wheel_drive",
        "robot_base_link_r4": "right_leg_passive",
        "robot_base_link_r9": "right_leg_motor",
        "robot_link1_right_r6": "right_leg_passive",
        "robot_link2_right_r7": "right_wheel_drive",
    },
    wheel_joints=("robot_link2_left_r3", "robot_link2_right_r7"),
    leg_motor_joints=(
        "robot_base_link_r1",
        "robot_base_link_r9",
    ),
    passive_joints=("robot_base_link_r2", "robot_base_link_r4", "robot_link1_left_r8", "robot_link1_right_r6"),
    connect_sites=(
        "robot_link2_left_connect_site_a",
        "robot_link2_left_connect_site_b",
        "robot_link3_left_connect_site_a",
        "robot_link3_left_connect_site_b",
        "robot_link2_right_connect_site_a",
        "robot_link2_right_connect_site_b",
        "robot_link3_right_connect_site_a",
        "robot_link3_right_connect_site_b",
    ),
)
