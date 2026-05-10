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


# 轮子前进方向符号: 虚拟正前进力矩映射到物理执行器的符号。
# 当前朝向下, 两轮同时 +1.0 力矩会产生 +X 位移。
WHEEL_RADIUS = 0.036

WHEEL_FORWARD_SIGNS: dict[str, float] = {
    "link2_left_旋转-13": 1.0,
    "link2_right_旋转-12": 1.0,
}


MODEL_SEMANTICS = ModelSemantics(
    body_roles={
        "base_link": "base",
        "link1_left": "left_leg_link1",
        "link2_left": "left_leg_link2",
        "link3_left": "left_leg_link3",
        "wheel_left": "left_wheel",
        "link1_right": "right_leg_link1",
        "link2_right": "right_leg_link2",
        "link3_right": "right_leg_link3",
        "wheel_right": "right_wheel",
    },
    joint_roles={
        "base_link_旋转-2": "left_leg_motor",
        "link1_left_旋转-6": "left_leg_passive",
        "base_link_旋转-4": "left_leg_passive",
        "link2_left_旋转-13": "left_wheel_drive",
        "base_link_旋转-1": "right_leg_motor",
        "link1_right_旋转-5": "right_leg_passive",
        "base_link_旋转-3": "right_leg_passive",
        "link2_right_旋转-12": "right_wheel_drive",
    },
    wheel_joints=("link2_left_旋转-13", "link2_right_旋转-12"),
    leg_motor_joints=(
        "base_link_旋转-2",
        "base_link_旋转-1",
    ),
    passive_joints=("base_link_旋转-4", "base_link_旋转-3", "link1_left_旋转-6", "link1_right_旋转-5"),
    connect_sites=(
        "link2_left_connect_site_a",
        "link2_left_connect_site_b",
        "link3_left_connect_site_a",
        "link3_left_connect_site_b",
        "link2_right_connect_site_a",
        "link2_right_connect_site_b",
        "link3_right_connect_site_a",
        "link3_right_connect_site_b",
    ),
)
