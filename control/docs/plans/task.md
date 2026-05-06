| Step | Status | Notes |
| --- | --- | --- |
| Inspect AGENTS and robot pose source | completed | Root pose issue isolated to `robot_description/robot.xml` `base_link` world euler |
| Preserve current upright preview pose without extra URDF root | completed | Absorbed former `world_to_base_link` rotation into `base_link` origins and `base_link_*` joint origins so joint semantics stay unchanged |
| Sync Mujoco pose and remove explicit ground geom | completed | Set XML `base_link` root `euler` to `pi 0 0`; ground plane geom retained for visual reference |
| Verify URDF/XML parse after edit | completed | URDF no longer contains `world`/`world_to_base_link`; XML parse ok, plane geom count is 0 |
| Restore missing `link1_right` visual mesh | completed | Fixed stray `·` in `link1_right_visual` mesh filename so visual and collision both resolve to `meshes/link1_right.stl` |
| Inspect same-side marker/connect mismatch | completed | `轮腿v1/轮腿v1.xml` uses `equality/connect` directly on `robot_link2_*` and `robot_link3_*`; marker links are fixed visual children and are not the connection source |
| Connect same-side link2/link3 markers in MuJoCo | completed | Added marker connect sites, switched equality connects to site pairs, added `marker_closed_home` keyframe, and made `launch_mujoco.py` load it |
| Replace marker runtime closure with real link closure | completed | Removed marker runtime bodies/assets from `轮腿v1/轮腿v1.xml`, moved connect sites onto `robot_link2_*` and `robot_link3_*`, and updated the home keyframe to a real closed-link pose |
| Upgrade link2-link3 closure from point contact to shared axis | completed | Replaced each side's single `connect` with two marker-axis-aligned `connect` pairs, updated the home pose, and stiffened equality parameters to keep post-step axis drift below 0.13 mm |
| Make reset/pause pose use the connected home geometry | completed | Updated `link23_connected_home` keyframe to numerically solved constraint-settled equilibrium; static XML remains open, constraints close the loop during simulation |
