"""Xbox Series 手柄读取（macOS Apple DEXT 驱动 / hidapi 路径）。

设计要点：
- macOS Sequoia 自带的 `XboxSeriesXGamepad` DEXT 驱动通过 USB HID 暴露
  **左摇杆 + LT + RT + 按键**。右摇杆数据走 Apple GameController.framework，
  在这条 HID 路径上拿不到——所以本模块用**单摇杆驾驶**：左摇杆 Y 控线速度，
  左摇杆 X 控 yaw，LT/RT 控高度。
- DEXT 驱动只在控制器状态变化时发 HID 报文；idle 时 read() 返空。本类用
  `read(N, timeout_ms=0)` 排干当前可用报文，并缓存最后一次解析结果。
- `close()` 显式释放 HID 句柄，避免进程异常退出后驱动卡住下一次 open。
- Bluetooth 路径使用 GameController.framework（pyobjc）读取摇杆和扳机。

19 字节报文布局（report id 0x20）：
    b5-b6   LT      uint16 LE, 0..1023
    b7-b8   RT      uint16 LE, 0..1023
    b13-b14 left X  int16 LE,  -32768..32767  右为正
    b15-b16 left Y  int16 LE                  HID 约定向下为正，本类翻成向上为正
"""

from __future__ import annotations

import atexit
import logging
import struct
import time
from dataclasses import dataclass
from typing import Any, Optional, Protocol, Tuple

import numpy as np

logger = logging.getLogger(__name__)

XBOX_VENDOR = 0x045E
XBOX_PRODUCT_IDS = (0x0B12, 0x0B13, 0x0B20)
STICK_FULL_SCALE = 32768.0
TRIGGER_FULL_SCALE = 1023.0
REPORT_ID_INPUT = 0x20
READ_BUF_SIZE = 64  # 实测 read(19, ...) 在 macOS hidapi 上偶发返空，over-allocate 更稳


def _apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _get_attr(obj: Any, name: str) -> Any:
    if obj is None:
        return None
    attr = getattr(obj, name, None)
    if callable(attr):
        try:
            return attr()
        except TypeError:
            return attr
    return attr


def _axis_value(axis: Any) -> float:
    if axis is None:
        return float('nan')
    value = _get_attr(axis, "value")
    if value is None:
        return float('nan')
    try:
        return float(value)
    except (TypeError, ValueError):
        return float('nan')


@dataclass(frozen=True)
class XboxState:
    """归一化手柄状态。`left_x` ∈ [-1, 1]（右为正），`left_y` ∈ [-1, 1]（上为正），
    `lt` / `rt` ∈ [0, 1]。"""

    left_x: float
    left_y: float
    lt: float
    rt: float


def _parse_report(data: bytes) -> Optional[XboxState]:
    """解析 input report id 0x20。其他 rid 返 None。"""
    if len(data) < 17 or data[0] != REPORT_ID_INPUT:
        return None
    lt_raw = struct.unpack_from("<H", data, 5)[0]
    rt_raw = struct.unpack_from("<H", data, 7)[0]
    lx_raw = struct.unpack_from("<h", data, 13)[0]
    ly_raw = struct.unpack_from("<h", data, 15)[0]
    return XboxState(
        left_x=lx_raw / STICK_FULL_SCALE,
        left_y=-ly_raw / STICK_FULL_SCALE,
        lt=min(1.0, lt_raw / TRIGGER_FULL_SCALE),
        rt=min(1.0, rt_raw / TRIGGER_FULL_SCALE),
    )


class GamepadDevice(Protocol):
    @property
    def name(self) -> str: ...

    def poll(self) -> Optional[XboxState]: ...

    def close(self) -> None: ...


class XboxGamepad:
    def __init__(self, dev: Any, name: str, deadzone: float = 0.10) -> None:
        self._dev = dev
        self._name = name
        self._deadzone = deadzone
        self._last_state: Optional[XboxState] = None
        self._closed = False
        # 进程退出兜底释放 HID 句柄，避免 stuck process 把驱动卡住
        atexit.register(self.close)

    @classmethod
    def try_open(cls, deadzone: float = 0.10) -> Optional["XboxGamepad"]:
        try:
            import hid
        except ImportError:
            logger.info("hidapi not installed, gamepad disabled")
            return None

        info = None
        for pid in XBOX_PRODUCT_IDS:
            try:
                devs = hid.enumerate(XBOX_VENDOR, pid)
            except Exception as exc:
                logger.warning("hid.enumerate failed: %s", exc)
                return None
            if devs:
                info = devs[0]
                break
        if info is None:
            logger.info("No Xbox controller found via hidapi")
            return None

        try:
            dev = hid.device()
            dev.open_path(info["path"])
            set_nonblocking = getattr(dev, "set_nonblocking", None)
            if callable(set_nonblocking):
                set_nonblocking(True)
        except Exception as exc:
            logger.warning("Failed to open Xbox HID device: %s", exc)
            return None

        name = info.get("product_string") or "Xbox Controller"
        logger.info("Xbox controller opened: vid=0x%04x pid=0x%04x name=%s",
                    info["vendor_id"], info["product_id"], name)
        return cls(dev, name=name, deadzone=deadzone)

    @property
    def name(self) -> str:
        return self._name

    def poll(self) -> Optional[XboxState]:
        """排干当前可用报文，返回最新状态（无新数据则返缓存）。"""
        if self._closed:
            return None
        try:
            latest: Optional[XboxState] = None
            while True:
                data = self._dev.read(READ_BUF_SIZE, timeout_ms=0)
                if not data:
                    break
                parsed = _parse_report(bytes(data))
                if parsed is not None:
                    latest = parsed
            if latest is not None:
                self._last_state = XboxState(
                    left_x=_apply_deadzone(latest.left_x, self._deadzone),
                    left_y=_apply_deadzone(latest.left_y, self._deadzone),
                    lt=latest.lt,
                    rt=latest.rt,
                )
            if self._last_state is None:
                return XboxState(left_x=float('nan'), left_y=float('nan'), lt=float('nan'), rt=float('nan'))
            return self._last_state
        except Exception as exc:
            logger.warning("HID read failed: %s", exc)
            return XboxState(left_x=float('nan'), left_y=float('nan'), lt=float('nan'), rt=float('nan'))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._dev.close()
        except Exception:
            pass


class GCBluetoothGamepad:
    def __init__(self, controller: Any, name: str, deadzone: float = 0.10) -> None:
        self._controller = controller
        self._name = name
        self._deadzone = deadzone
        self._last_state: Optional[XboxState] = None
        self._closed = False
        atexit.register(self.close)

    @classmethod
    def try_open(cls, deadzone: float = 0.10, fast: bool = False) -> Optional["GCBluetoothGamepad"]:
        try:
            import GameController
        except ImportError:
            if not fast:
                logger.info("pyobjc GameController not installed, bluetooth gamepad disabled")
            return None

        controller_class = getattr(GameController, "GCController", None)
        if controller_class is None:
            if not fast:
                logger.info("GameController.framework unavailable")
            return None

        controllers = list(controller_class.controllers() or [])
        if not controllers and not fast:
            try:
                controller_class.startWirelessControllerDiscoveryWithCompletionHandler_(None)
            except Exception:
                pass
            try:
                from Foundation import NSRunLoop, NSDate
                loop = NSRunLoop.currentRunLoop()
                for _ in range(40):
                    date = NSDate.dateWithTimeIntervalSinceNow_(0.05)
                    loop.runUntilDate_(date)
                    controllers = list(controller_class.controllers() or [])
                    if controllers:
                        break
            except ImportError:
                for _ in range(40):
                    time.sleep(0.05)
                    controllers = list(controller_class.controllers() or [])
                    if controllers:
                        break
            try:
                controller_class.stopWirelessControllerDiscovery()
            except Exception:
                pass

        if not controllers:
            if not fast:
                logger.info("No Bluetooth controller found via GameController")
            return None

        controller = controllers[0]
        name = _get_attr(controller, "vendorName") or "Bluetooth Gamepad"
        logger.info("Bluetooth controller opened: name=%s", name)
        return cls(controller, name=str(name), deadzone=deadzone)

    @property
    def name(self) -> str:
        return self._name

    def poll(self) -> Optional[XboxState]:
        if self._closed:
            return None

        pad = _get_attr(self._controller, "extendedGamepad")
        if pad is None:
            pad = _get_attr(self._controller, "gamepad")
        if pad is None:
            pad = _get_attr(self._controller, "microGamepad")
        if pad is None:
            return XboxState(left_x=float('nan'), left_y=float('nan'), lt=float('nan'), rt=float('nan'))

        left = _get_attr(pad, "leftThumbstick") or _get_attr(pad, "dpad")
        lx = _axis_value(_get_attr(left, "xAxis"))
        ly = _axis_value(_get_attr(left, "yAxis"))
        lt = _axis_value(_get_attr(pad, "leftTrigger"))
        rt = _axis_value(_get_attr(pad, "rightTrigger"))

        import math
        if math.isnan(lx) or math.isnan(ly) or math.isnan(lt) or math.isnan(rt):
            state = XboxState(left_x=lx, left_y=ly, lt=lt, rt=rt)
        else:
            state = XboxState(
                left_x=_apply_deadzone(_clamp(lx, -1.0, 1.0), self._deadzone),
                left_y=_apply_deadzone(_clamp(ly, -1.0, 1.0), self._deadzone),
                lt=_clamp(lt, 0.0, 1.0),
                rt=_clamp(rt, 0.0, 1.0),
            )
        self._last_state = state
        return state

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True


class AutoGamepad:
    """自动管理并无缝切换/升级手柄连接。
    
    在 macOS 上，startup 时 Cocoa run loop 还没启动，GCController 无法发现蓝牙手柄。
    本类会先 fallback 到 hidapi (XboxGamepad) 打开手柄（此时能 open 但 read 读不出数据）；
    随后在仿真循环的 poll() 中，自动并在后台以 0 阻塞方式周期性检测 native GameController
    连接，一旦发现，便无缝升级到 GCBluetoothGamepad，使蓝牙手柄能正常工作。
    """
    def __init__(self, deadzone: float = 0.10) -> None:
        self.deadzone = deadzone
        self.device: Optional[GamepadDevice] = None
        self._last_check = 0.0
        self._try_init()

    def _try_init(self) -> None:
        import sys
        if sys.platform == "darwin":
            # 优先进行 non-blocking 的快速原生检测，避免 2s 阻塞
            self.device = GCBluetoothGamepad.try_open(deadzone=self.deadzone, fast=True)
            if self.device is not None:
                return
            self.device = XboxGamepad.try_open(deadzone=self.deadzone)
        else:
            self.device = XboxGamepad.try_open(deadzone=self.deadzone)
            if self.device is None:
                self.device = GCBluetoothGamepad.try_open(deadzone=self.deadzone, fast=True)

    @property
    def name(self) -> str:
        return self.device.name if self.device is not None else "No Gamepad"

    def poll(self) -> Optional[XboxState]:
        if self.device is None:
            self._try_init()
            if self.device is None:
                return XboxState(left_x=float('nan'), left_y=float('nan'), lt=float('nan'), rt=float('nan'))

        state = self.device.poll()

        # macOS 特殊逻辑：如果是 hidapi 打开的手柄（对于蓝牙手柄它能 open 但 poll 返回空/NaNs），
        # 我们在运行中周期性检测并尝试升级到原生 GameController 路径
        import sys
        if sys.platform == "darwin" and isinstance(self.device, XboxGamepad):
            now = time.time()
            if now - self._last_check > 2.0:
                self._last_check = now
                native = GCBluetoothGamepad.try_open(deadzone=self.deadzone, fast=True)
                if native is not None:
                    logger.info("Automatically upgrading gamepad to native GameController: %s", native.name)
                    self.device.close()
                    self.device = native
                    state = self.device.poll()

        return state

    def close(self) -> None:
        if self.device is not None:
            self.device.close()
            self.device = None


def open_gamepad(deadzone: float = 0.10) -> Optional[GamepadDevice]:
    auto = AutoGamepad(deadzone=deadzone)
    if auto.device is None:
        return None
    return auto


@dataclass(frozen=True)
class GamepadCommandMapper:
    """把 XboxState 映射到 cmd_linear_x / cmd_angular_z / cmd_height。

    单摇杆驾驶（DEXT 路径不暴露右摇杆）：
      cmd_linear_x  ← left_y * linear_max
      cmd_angular_z ← -left_x * angular_max   （左摇杆向右扳 → yaw 顺时针 → 负值）
      cmd_height    ← mid + (rt - lt) * (height_high - height_low) / 2
    """

    linear_range: Tuple[float, float]
    angular_range: Tuple[float, float]
    height_range: Tuple[float, float]

    def map(self, state: XboxState) -> Tuple[float, float, float]:
        linear_max = max(abs(self.linear_range[0]), abs(self.linear_range[1]))
        angular_max = max(abs(self.angular_range[0]), abs(self.angular_range[1]))
        h_low, h_high = self.height_range
        h_mid = 0.5 * (h_low + h_high)
        h_half = 0.5 * (h_high - h_low)

        linear_x = float(np.clip(state.left_y * linear_max, *self.linear_range))
        angular_z = float(np.clip(-state.left_x * angular_max, *self.angular_range))
        height = float(np.clip(h_mid + (state.rt - state.lt) * h_half, h_low, h_high))
        return linear_x, angular_z, height
