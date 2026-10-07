# Pick-and-place diagnosis

## Historical regression evidence

The last historical revision whose headless checks completed both the Predefined Task and Autonomous Pick was `04aa92a` (`Upload robotics pick-and-place simulator`). Its table was 0.34 x 0.20 m, centered at (0.71, -0.04) m. The predefined task used the P1-P4 coordinates already present at that revision; the default book was at (0.68, -0.04) m. The autonomous sequence did not require a declared destination.

| Commit | Table and start pose | Task 5 waypoints | Planner gating |
|---|---|---|---|
| `04aa92a` | Compact 0.34 x 0.20 m at (0.71, -0.04); book (0.68, -0.04) | P1-P4 set the paper task and were reachable in headless execution | Autonomous used its built-in drop position; no declaration gate |
| `15aea6e` | Changed to 0.60 x 0.40 m at (0.60, 0.00); moved book to (0.60, 0.00) | P1-P4 were not moved; P3 stayed at (0.68, -0.04, 0.89) | Added pedestal/table path collision diagnostics and the planner's fixed drop position; still no declaration gate |
| `e90a2bf` | No geometry or start-pose change | No waypoint change | Documentation-only reachable-sector adjustment |
| `ec22670` | No geometry or start-pose change | No waypoint change | Added the complete-planner sweep and structured failures; fixed drop-position planning remained |
| `88f43b8` | No geometry or start-pose change | No waypoint change | UI/store synchronization only; it did not introduce the current manual declaration gate |

Historical `npm test` was run at `04aa92a` and `15aea6e`; both suites passed at their respective revisions. A separate headless task run showed both sequences DONE at `04aa92a`, while `15aea6e` ended the predefined sequence in ERROR.

`15aea6e` (`Measure pickable table zone and verify grasp geometry`) changed the table to 0.60 x 0.40 m centered at (0.60, 0.00), moved the default book to (0.60, 0.00), and added planner reachability/collision diagnostics. It left the predefined P3 waypoint at (0.68, -0.04, 0.89), 0.116726 m from the new book center; the historical headless run ended ERROR before grasp. Later `ec22670` added the complete planner sweep and `88f43b8` is the current base revision. The source tree supplied to this task already contained manual-destination gating, so the regression is the changed table/default pose plus task waypoint mismatch, compounded by destination gating.

At the requested Reset pose (0.366, -0.114), the pre-fix planner reported PRE-GRASP `link-vs-base collision`, upper arm, 14.8984 mm penetration, sample Z 896.335 mm. The pre-fix default book at (0.600, 0.000) was 0.116726 m from P3. Historical test runs showed `04aa92a` completed both paths and `15aea6e` failed the predefined path.

For HEAD-era Autonomous checks with a test destination, the default (0.600, 0.000) case reached PRE-PLACE and failed IK convergence (`Destination out of reach at this yaw`). The Reset coordinate failed earlier at PRE-GRASP with the structured upper-arm/base collision above. The Predefined P3-to-book center distance at the failing revision was 0.116726 m.

The current table selector offers Compact (verified), 0.34 x 0.20 m at (0.71, -0.04), and Large (experimental), 0.60 x 0.40 m at (0.60, 0.00). Compact is the default. Reset/default pickup position is computed by searching with the actual yaw-zero pick planner. Autonomous book center height is table top plus 12.5 mm. Predefined Task restores its own known-good object pose and runs without `dropTarget` on either preset. No DH dimensions, joint limits, tool length, table mesh, or collision threshold was changed.

## Destination interaction and plan gating

The first-load source audit confirmed the cyan PLACE marker only rendered when `dropTarget` was non-null, so no marker could be dragged before a destination was declared. The typed fields were in the Task panel and could require scrolling. Book drag disabled OrbitControls; PLACE drag did the same after the marker existed. The table surface and overlay were separate Three.js meshes, and the old flow had no click-to-set interaction.

The panel now places destination X/Y and yaw controls near its top. Click-to-set arms a crosshair mode, raycasts against the table plane, shows a translucent green/red ghost footprint, disables orbit controls, and supports Escape cancellation. Typed X/Y are retained as entered. The destination marker is draggable after declaration and the yaw slider updates the same Zustand destination. Without a destination, Plan remains blocked with `Set a destination (click the table or type X/Y)` while pickup reachability remains available. Destination edits clear the plan and return the state to IDLE.

The ray-to-table mapping is covered by a node test. Browser interaction itself was not verified visually in this environment.

## Rotated grasp diagnosis

Before the planner change, the supplied seven-yaw sweep at 0, 30, 60, 90, 120, 150, and 180 degrees reported 797, 658, 642, 613, 583, 583, and 677 successful cells. The yaw-zero cells that failed at other yaws were classified as follows:

| Yaw | Outside table bounds | Orientation limit | IK non-convergence | Joint limit |
|---:|---:|---:|---:|---:|
| 30 | 139 | 0 | 0 | 0 |
| 60 | 139 | 8 | 8 | 0 |
| 90 | 58 | 0 | 149 | 0 |
| 120 | 139 | 0 | 125 | 0 |
| 150 | 139 | 0 | 40 | 85 |
| 180 | 0 | 0 | 10 | 168 |

Thus many differences were table-footprint exclusions, while the remaining failures were principally numerical IK convergence and joint limits. The prior sweep had 533 cells in the all-yaw intersection on its 1 cm grid (0.0533 m2). Its yaw-zero/yaw-180 mismatch was 178 yaw-zero-only cells and 58 yaw-180-only cells, consistent with a single-seed wrist representation failing to exploit the symmetric grasp.

The planner now tries narrow-axis, flipped narrow wrist, and then wide-axis candidates; the wide-axis candidate is permitted by the existing 0.130 m opening: (0.130 - 0.012 - 0.100)/2 = 0.009 m clearance per side, above the requested 8 mm. It derives spherical-wrist seeds for both elbow and wrist-flip branches and retains numerical refinement. A 1,000-configuration FK verification measured worst position error below 1e-9 mm, tool-Z error 0.000002 degrees, yaw error below 1e-9 degrees, and zero failures. Representative post-change cells at yaw 0 and 180 now use identical joint solutions; additional yaw samples also succeeded with flipped-wrist candidates.

A complete post-change seven-yaw grid sweep has not been run, so post-change cell counts and the new all-yaw intersection area are unverified. The request to report a numerical before/after sweep is therefore not complete. The old sweep identifies IK convergence and joint limits as the rotated-grasp causes, but the new full-grid acceptance criterion remains open.

## Current checks and remaining work

- Pick reachability overlay: implemented for current pickup yaw, cached by robot/table/object configuration, debounced, computed in small animation-frame batches on the main thread, and available with no destination. It is not a worker. There is no separate place overlay or separate pick/place visibility toggle yet.
- Snapping: book and marker snapping to a valid cell is implemented; `Allow unreachable placement (testing)` disables it. Typed destinations are not snapped.
- Via-point diagnostics table: not implemented. The panel shows the chosen grasp candidate and close width, but not per-waypoint joint margin, manipulability, or collision penetration metrics. Consequently the requested debug visualization/tables are incomplete.
- Held-book collision: an oriented box check against the table and pedestal is now called during lift, transport, and lower. It is an approximate box/cylinder check; full link/object self-collision and a randomized clearance suite remain unverified.
- Predefined Task and one Compact default Autonomous Pick are covered by headless tests. Both predefined presets and typed/table ray mapping are also tested. A 20-run check repeats the same known-good Compact pair; it is not a randomized-pair test. The requested 50 randomized declared-pose completion suite and the complete post-change seven-yaw sweep have not been verified.
- The app could not be run in a browser here. Marker dragging, crosshair/ghost interactions, visible overlays, and the animated placement remain visually unverified.

## Geometry guardrails

The arm uses DH dimensions d1=0.70, l2=0.43, l3=0.37, d4=d5=0, d6=0.10 m. Tool length remains 0.075 m. Joint limits remain J1 [-170,170], J2 [-225,45], J3 [-216,90], J4 [-110,170], J5 [-100,100], J6 [-266,266] degrees. The book is 0.100 x 0.080 x 0.025 m. Collision thresholds and table mesh remain unchanged.
