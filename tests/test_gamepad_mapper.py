"""Pure mapping/parsing tests for sim.gamepad (hidapi/DEXT backend)."""

from __future__ import annotations

import sys
import struct
import types

import pytest

from sim.gamepad import (
    GamepadCommandMapper,
    XboxGamepad,
    XboxState,
    _apply_deadzone,
    _parse_report,
)


LINEAR_RANGE = (-1.0, 1.0)
ANGULAR_RANGE = (-3.0, 3.0)
HEIGHT_RANGE = (0.07844, 0.142)


@pytest.fixture
def mapper() -> GamepadCommandMapper:
    return GamepadCommandMapper(
        linear_range=LINEAR_RANGE,
        angular_range=ANGULAR_RANGE,
        height_range=HEIGHT_RANGE,
    )


def _make_report(*, lt: int = 0, rt: int = 0, lx: int = 0, ly: int = 0) -> bytes:
    buf = bytearray(19)
    buf[0] = 0x20
    struct.pack_into("<H", buf, 5, lt)
    struct.pack_into("<H", buf, 7, rt)
    struct.pack_into("<h", buf, 13, lx)
    struct.pack_into("<h", buf, 15, ly)
    return bytes(buf)


def test_zero_state_yields_height_midpoint(mapper: GamepadCommandMapper) -> None:
    state = XboxState(left_x=0.0, left_y=0.0, lt=0.0, rt=0.0)
    lin, ang, h = mapper.map(state)
    assert lin == 0.0
    assert ang == 0.0
    assert h == pytest.approx(0.5 * (HEIGHT_RANGE[0] + HEIGHT_RANGE[1]))


def test_left_y_drives_linear(mapper: GamepadCommandMapper) -> None:
    lin, _, _ = mapper.map(XboxState(left_x=0.0, left_y=1.0, lt=0.0, rt=0.0))
    assert lin == pytest.approx(LINEAR_RANGE[1])
    lin, _, _ = mapper.map(XboxState(left_x=0.0, left_y=-1.0, lt=0.0, rt=0.0))
    assert lin == pytest.approx(LINEAR_RANGE[0])


def test_left_x_drives_yaw_with_sign_flip(mapper: GamepadCommandMapper) -> None:
    _, ang, _ = mapper.map(XboxState(left_x=1.0, left_y=0.0, lt=0.0, rt=0.0))
    assert ang == pytest.approx(-ANGULAR_RANGE[1])
    _, ang, _ = mapper.map(XboxState(left_x=-1.0, left_y=0.0, lt=0.0, rt=0.0))
    assert ang == pytest.approx(-ANGULAR_RANGE[0])


def test_rt_raises_height(mapper: GamepadCommandMapper) -> None:
    _, _, h = mapper.map(XboxState(left_x=0.0, left_y=0.0, lt=0.0, rt=1.0))
    assert h == pytest.approx(HEIGHT_RANGE[1])


def test_lt_lowers_height(mapper: GamepadCommandMapper) -> None:
    _, _, h = mapper.map(XboxState(left_x=0.0, left_y=0.0, lt=1.0, rt=0.0))
    assert h == pytest.approx(HEIGHT_RANGE[0])


def test_height_clamps_inside_ctrlrange(mapper: GamepadCommandMapper) -> None:
    _, _, h = mapper.map(XboxState(left_x=0.0, left_y=0.0, lt=0.0, rt=2.0))
    assert HEIGHT_RANGE[0] <= h <= HEIGHT_RANGE[1]


def test_deadzone_zeroes_small_inputs() -> None:
    assert _apply_deadzone(0.05, 0.10) == 0.0
    assert _apply_deadzone(-0.09, 0.10) == 0.0


def test_deadzone_rescales_outside_band() -> None:
    assert _apply_deadzone(1.0, 0.10) == pytest.approx(1.0)
    assert _apply_deadzone(-1.0, 0.10) == pytest.approx(-1.0)


def test_parse_report_rejects_wrong_id() -> None:
    buf = bytearray(19)
    buf[0] = 0x07
    assert _parse_report(bytes(buf)) is None


def test_parse_report_decodes_sticks_and_triggers() -> None:
    report = _make_report(lt=512, rt=1023, lx=16384, ly=16384)
    state = _parse_report(report)
    assert state is not None
    assert state.left_x == pytest.approx(0.5)
    assert state.left_y == pytest.approx(-0.5)  # ly=+16384 → 向下 → 翻成 -0.5
    assert state.lt == pytest.approx(512 / 1023.0)
    assert state.rt == pytest.approx(1.0)


def test_parse_report_handles_extreme_values() -> None:
    report = _make_report(lx=-32768, ly=-32768)
    state = _parse_report(report)
    assert state is not None
    assert state.left_x == pytest.approx(-1.0)
    assert state.left_y == pytest.approx(1.0)


def test_hid_backend_sets_nonblocking(monkeypatch: pytest.MonkeyPatch) -> None:
    class _FakeDevice:
        def __init__(self) -> None:
            self.nonblocking: bool | None = None

        def open_path(self, _path: bytes) -> None:
            pass

        def set_nonblocking(self, value: bool) -> None:
            self.nonblocking = value

        def close(self) -> None:
            pass

    opened = _FakeDevice()
    fake_hid = types.SimpleNamespace(
        enumerate=lambda vendor, product: [
            {
                "path": b"fake",
                "vendor_id": vendor,
                "product_id": product,
                "product_string": "Fake Xbox",
            }
        ],
        device=lambda: opened,
    )
    monkeypatch.setitem(sys.modules, "hid", fake_hid)

    gamepad = XboxGamepad.try_open()

    assert gamepad is not None
    assert opened.nonblocking is True
    gamepad.close()
