"""End-to-end smoke test for sim.gamepad (hidapi backend).

Move left stick / squeeze LT / squeeze RT for 8 seconds. Each state change
prints a line showing parsed XboxState and mapped cmd_* values.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.gamepad import GamepadCommandMapper, open_gamepad


def main() -> None:
    g = open_gamepad()
    if g is None:
        print("No gamepad opened. Check USB or Bluetooth connection.")
        return
    print(f"Opened: {g.name}")
    mapper = GamepadCommandMapper(
        linear_range=(-1.0, 1.0),
        angular_range=(-3.0, 3.0),
        height_range=(0.07844, 0.142),
    )
    print("Move left stick / press triggers for 8 seconds. Ctrl+C to stop.")
    print("-" * 70)

    t0 = time.perf_counter()
    last = None
    try:
        while time.perf_counter() - t0 < 8.0:
            state = g.poll()
            if state is not None:
                cur = (round(state.left_x, 2), round(state.left_y, 2),
                       round(state.lt, 2), round(state.rt, 2))
                if cur != last:
                    last = cur
                    lin, ang, h = mapper.map(state)
                    print(
                        f"  lx={state.left_x:+.3f} ly={state.left_y:+.3f} "
                        f"lt={state.lt:.3f} rt={state.rt:.3f}  ->  "
                        f"lin={lin:+.3f} ang={ang:+.3f} h={h:.4f}",
                        flush=True,
                    )
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        g.close()
    print("-" * 70)
    print("Done.")


if __name__ == "__main__":
    main()
