# Todo: 达妙四电机 USB-CAN 控制器

- [x] Save approved plan to `tasks/plan.md` and this checklist to `tasks/todo.md`.
- [x] Add `damiao_can` package skeleton and static motor registry.
- [x] Implement pure protocol frame packing and feedback decoding.
- [x] Add protocol tests for command frames, MIT packing, velocity packing, and feedback decoding.
- [ ] Implement dry-run CLI status and command previews.
- [ ] Add dry-run CLI tests and safety-gate tests.
- [ ] Add `python-can` and `python-can-damiao` dependencies.
- [ ] Implement CAN bus adapter for macOS `damiao` serial channel.
- [ ] Implement feedback receive by Master_ID matching.
- [ ] Wire real enable/disable/clear-error commands.
- [ ] Wire single-motor MIT and velocity commands with nonzero approval gate for bring-up only.
- [ ] Implement synchronized four-motor command execution and feedback aggregation.
- [ ] Implement guarded smoke-test with unconditional disable cleanup.
- [ ] Verify with `uv run pytest tests/test_damiao_protocol.py tests/test_damiao_cli.py`.
- [ ] Verify with `uv run pytest`.
- [ ] Perform manual hardware validation only after explicit approval.
