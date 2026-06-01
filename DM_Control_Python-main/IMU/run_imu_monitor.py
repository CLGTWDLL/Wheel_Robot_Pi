from __future__ import annotations

# argparse 负责解析 IMU 单独监视程序的命令行参数。
import argparse

# csv 用于把 IMU 平衡信号记录到文件。
import csv

# sys 用于把项目根目录加入导入路径。
import sys

# time 用于生成日志文件名、刷新节拍和运行时长。
import time

# dataclass 用于定义轻量控制器调试接口。
from dataclasses import dataclass

# Path 用于稳定生成 CSV 日志路径。
from pathlib import Path

# 旧的独立 IMU Rich 面板已经停用，统一改用 RobotDashboard。
# from rich.console import Group
# from rich.live import Live
# from rich.panel import Panel
# from rich.table import Table
# from rich.text import Text

# ROOT 指向项目根目录，方便直接运行本文件。
ROOT = Path(__file__).resolve().parents[1]

# 直接运行 IMU/run_imu_monitor.py 时，把根目录加入 sys.path。
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 单独监视 IMU 时也使用统一机器人面板，只是电机表会显示 waiting。
from robot_state import RobotDashboard, RobotState

# 同时兼容“直接运行脚本”和“作为包导入”两种方式。
try:
    from balance_signal import BalanceObservation, BalanceSignalProcessor
    from imu_can import DMIMUSerialReader, IMUState
except ImportError:
    from .balance_signal import BalanceObservation, BalanceSignalProcessor
    from .imu_can import DMIMUSerialReader, IMUState


@dataclass
class ControllerDebugInterface:
    """给控制器调试使用的最近一次 IMU 观测缓存。"""

    # latest 保存最近一次有效观测。
    latest: BalanceObservation | None = None

    def update(self, observation: BalanceObservation | None) -> None:
        """收到有效观测时更新缓存。"""
        if observation is not None:
            self.latest = observation

    def get_observation_vector(self) -> tuple[float, float, float, float] | None:
        """返回控制器需要的四维观测向量。"""
        if self.latest is None:
            return None
        return self.latest.as_controller_vector()


class CSVRecorder:
    """CSV 记录器，用上下文管理器保证文件关闭。"""

    def __init__(self, path: Path | None):
        """保存日志路径；path 为 None 时关闭 CSV 记录。"""
        self.path = path
        self.file = None
        self.writer = None

    def __enter__(self) -> "CSVRecorder":
        """打开 CSV 文件并写入表头。"""
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.file = self.path.open("w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            self.writer.writerow(
                [
                    "timestamp",
                    "pitch_deg",
                    "roll_deg",
                    "pitch_rad",
                    "roll_rad",
                    "pitch_rate_rad_s",
                    "roll_rate_rad_s",
                ]
            )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出时关闭 CSV 文件。"""
        if self.file is not None:
            self.file.close()
        return False

    def write(self, observation: BalanceObservation | None) -> None:
        """把一帧有效 IMU 平衡观测写入 CSV。"""
        if observation is None or self.writer is None:
            return
        self.writer.writerow(
            [
                f"{observation.timestamp:.6f}",
                f"{observation.pitch_deg:.6f}",
                f"{observation.roll_deg:.6f}",
                f"{observation.pitch_rad:.9f}",
                f"{observation.roll_rad:.9f}",
                f"{observation.pitch_rate_rad_s:.9f}",
                f"{observation.roll_rate_rad_s:.9f}",
            ]
        )


def main() -> None:
    """运行 IMU 单独监视程序。"""
    args = parse_args()

    # 根据参数决定是否开启 CSV 记录。
    csv_path = None if args.no_csv else Path(args.csv)

    # 处理器负责把 IMU 原始状态变成 pitch/roll 观测。
    processor = BalanceSignalProcessor()

    # 调试接口保留最近一次控制器观测向量。
    debug_interface = ControllerDebugInterface()

    # 统一机器人状态对象；此脚本只更新 IMU 部分。
    robot_state = RobotState()

    # 同时打开 IMU 串口和 CSV 记录器。
    with DMIMUSerialReader(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        validate_crc=args.validate_crc,
    ) as reader, CSVRecorder(csv_path) as recorder:
        # 可选：配置 IMU 只输出平衡控制需要的数据。
        if args.configure:
            reader.configure_usb_balance_stream(save=args.save_config)

        # last_observation 仅用于保留最近一次有效观测。
        last_observation = None

        # sample_count 用于 Rich 面板显示有效样本数。
        sample_count = 0

        # next_refresh 控制面板刷新频率。
        next_refresh = 0.0

        # start_time 用于支持 --duration 限时运行。
        start_time = time.time()

        # 使用统一 RobotDashboard 显示 IMU 状态。
        with RobotDashboard(
            robot_state, refresh_per_second=args.refresh_hz
        ) as dashboard:
            while args.duration <= 0 or time.time() - start_time < args.duration:
                # 读取串口里已经到达的所有 IMU 状态帧。
                states = reader.read_states()
                for state in states:
                    # 只在 pitch/roll/角速度齐全时生成观测。
                    observation = processor.update(state)
                    if observation is None:
                        continue

                    last_observation = observation
                    robot_state.update_imu_from_observation(observation)
                    debug_interface.update(observation)
                    recorder.write(observation)
                    sample_count += 1

                # 按 refresh_hz 刷新 Rich 面板。
                now = time.time()
                if now >= next_refresh:
                    dashboard.update(
                        imu_samples=sample_count,
                        csv_path=csv_path,
                    )
                    next_refresh = now + 1.0 / args.refresh_hz

                # 小睡 1ms，避免空转占满 CPU。
                time.sleep(0.001)


def parse_args() -> argparse.Namespace:
    """解析 IMU 单独监视程序的命令行参数。"""
    default_csv = Path(__file__).resolve().parent / "logs" / (
        "imu_" + time.strftime("%Y%m%d_%H%M%S") + ".csv"
    )

    # 默认仍然使用 COM8 和 921600。
    parser = argparse.ArgumentParser(description="Monitor DM-IMU-L1 balance signals.")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baudrate", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=0.02)
    parser.add_argument("--refresh-hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--csv", default=str(default_csv))
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument("--validate-crc", action="store_true")
    parser.add_argument("--configure", action="store_true")
    parser.add_argument("--save-config", action="store_true")
    return parser.parse_args()


# Legacy independent IMU Rich dashboard is no longer used.


if __name__ == "__main__":
    main()
