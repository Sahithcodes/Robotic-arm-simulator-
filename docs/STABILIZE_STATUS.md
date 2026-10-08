# Stabilization status

- **PASS — Step 1, evidence #1:** `npm run build` succeeded; `npm run dev` served `/` with HTTP 200 and a 10,101-byte response; Task panel + viewport SSR smoke test passed (1 test). The red toast was not reproduced. Added viewport/panel ErrorBoundaries and a copyable global Errors box.
- **PASS — Step 2, evidence #2 (updated by #6):** Compact Table top `0.7800 m`, book center `0.7925 m`. Geometry audit found the tool axis incorrectly tested against the arm-link clearance plane at `0.805 m` (false penetration `2.739 mm` at `0.802261 m`). The arm link plane still enforces 25 mm; tool contact checks the real tabletop. The initial Step 2 grasp proof was TCP `0.7846 m`, finger clearance `3.00 mm`, and 22.00 mm side overlap. After Step 5 aligned TCP with book center, the current printed measurements are TCP `0.7925 m`, finger-box clearance `12.40 mm`, and side overlap `12.60 mm` (evidence #6). No collision threshold was changed. Planner + headless execution + height tests: 10 passed at Step 2, including 3×3 pickup positions.
- **PASS — Step 3, evidence #3:** Headless Predefined Task completed (`COMPLETE`, `state=placed`) on Compact and Large with no destination, using its separate established pose. Printed final center for both: `(0.717678, 0.000041, 0.815952) m`. This predefined trajectory retains its own height; autonomous/reset starts at the true resting center `0.7925 m`.
- **PASS — Step 4, evidence #8:** `npm run verify` acceptance row C tested the full 5 positions × 13 book yaws × 7 destination yaws (455 cases). The planner rejected 325 combinations as `Book footprint is off Table`; every one of the 130 planner-valid combinations reached DONE, with strict release yaw error under 2° (printed worst `0.000°`). Destination yaws were `0,45,90,135,180,30,91°`. Strict 2° release checking remains enabled; the mesh has not been proven 180° symmetric.
- **PASS — Step 5, evidence #6:** `planPickAndPlace` is the sole complete-plan function through the `createPickPlan` compatibility export; it searches the shared narrow / flipped / wide / flipped candidate list and retains one candidate across grasp, transport, and place. The worker place overlay calls the same complete planner. `npm test -- --run src/robotics/__tests__/unifiedPickPlacePlanner.test.ts src/robotics/__tests__/heightCalibration.test.ts` passed 2 tests; printed `overlay=Reachable candidate=narrow axis waypoints=PRE-GRASP,GRASP,LIFT,PRE-PLACE,PLACE,RETREAT residual=1.434 mm` and finger clearance `12.40 mm`, side overlap `12.60 mm`. `npm run lint` passed. Reset-pose edge failure was fixed by selecting a pose within the center-aligned grasp workspace; its focused regression passed 3 tests.
- **PASS — Step 6, evidence #7:** Runtime scene and table-preset reset now contain only Table at `0.7800 m`; the panel labels any added non-table surface experimental. The worker overlay and its cache remain in place. `npm test -- --run src/robotics/__tests__/sceneSurfaceDefaults.test.ts src/robotics/__tests__/placementSurfaces.test.ts -t "starts and resets|invalidates"` passed 2 tests and printed `surfaces=Table tableTop=0.7800 m`; lint passed. Non-table transport/reachability remains unverified and is experimental/off by default. Prior 3 cm surface sweep evidence #5 showed 0% placeable for every default non-table preset, so none is enabled.
- **PASS — Step 7, evidence #9:** Final `npm run verify` passed after adding sampled finger-box/table collision checks: lint; default Vitest (22 files, 75 passed, 1 skipped); production build; acceptance suite (8 passed). Acceptance A-G printed; B completed 50/50 planner-valid pairs (seed `202013383`, 1.574 mm worst release-position error, 0.000° yaw error, book bottom never below `0.78000 m`, broad fixture yield 6.9%); C completed all 130 planner-valid cases from the 455-case matrix; D remains experimental/off; E refused before transport at PRE-GRASP for a joint limit; F SSR and headless console capture passed; G reports 10/20 Large plans accepted only.

## Step 1 error audit

- `new Worker(new URL(..., import.meta.url))`: instantiated inside the mounted overlay's `useEffect`; cleanup cancels and terminates it. The Next production build passes. Worker startup in a real browser remains unverified.
- `window`, `navigator`, `document`, `OffscreenCanvas`, `CanvasTexture`: no module-scope browser access found. `window`/`document` and texture creation occur in effects or event/renderer code after mount. SSR of the panel and pre-Canvas viewport passed.
- Hydration nondeterminism: `Date.now()` and `Math.random()` make a worker job ID inside an effect, not in render. No other render-time use was found in the audited app sources.
- Hook ordering: inspected overlay component hooks precede its conditional return; no hook-order violation found in inspected components.
- State updates during render: no instance observed in inspected component code.
- Worker messages after unmount: guarded by a live flag and canceled/terminated in effect cleanup.
- Texture/geometry disposal: overlay textures are disposed on replacement and unmount; no disposal error reproduced.
- Keys: mapped scene meshes and UI controls have keys in inspected paths; no key warning appeared in the dev server log.
- Unhandled rejections: none appeared in the dev server log; added global `error` and `unhandledrejection` capture and console reporting. Browser runtime capture is not yet exercised.
- Exact runtime error text reproduced in step 1: **none**. Tool output did include Vite's test-runner notice: `The CJS build of Vite's Node API is deprecated. See https://vite.dev/guide/troubleshooting.html#vite-cjs-node-api-deprecated for more details.` This is a warning, not the app's red error toast.

## Evidence log

1. `npm run build`: exit 0; Next.js compiled, type checked, and generated all static pages. `npm run dev`: port 3000 was occupied, server selected 3001; `node fetch` of `/` returned HTTP 200. Dev log contained successful compilation and `GET / 200`, no errors.
2. `npm run lint` exit 0; `npm test -- --run src/components/layout/ssrSmoke.test.tsx`: 1 test passed.
3. `npm test -- --run src/robotics/__tests__/heightCalibration.test.ts tests/autonomousExecution.test.ts tests/autonomousPlanner.test.ts`: 3 files, 10 tests passed. The height test printed all measured values listed in checklist evidence #2.
4. `npm test -- --run tests/tableAndTaskRegression.test.ts -t "completes and places the Predefined Task"`: 1 test passed; printed COMPLETE/placed, final pose, and `destination=none` for Compact and Large.
5. `npm test -- --run tests/rotatedEvidence.test.ts src/robotics/__tests__/yawBookkeeping.test.ts`: 2 files, 2 tests passed; printed the exact yaw trace above. The attempted 455-case sweep printed `planned=0 done=0 planRate=0.0% failures={"link-vs-base collision":455}` for its fixed fixture; this is a failed fixture and does not count as rotated acceptance evidence.
6. `npm test -- --run src/robotics/__tests__/unifiedPickPlacePlanner.test.ts src/robotics/__tests__/heightCalibration.test.ts`: 2 tests passed; the complete worker place-overlay plan and execution plan selected the same candidate and produced the same six waypoints. `npm run lint` passed. `npm test -- --run tests/tableAndTaskRegression.test.ts -t "resets plan|Large HOME|computed Compact default"`: 3 tests passed.
7. `npm test -- --run src/robotics/__tests__/sceneSurfaceDefaults.test.ts src/robotics/__tests__/placementSurfaces.test.ts -t "starts and resets|invalidates"`: 2 tests passed; runtime contains Table only at `0.7800 m`. `npm run lint` passed.
8. `npm run verify`: exit 0. Lint passed; default suite reported 22 files, 75 passed and 1 skipped; Next production build passed; `test:acceptance` reported 8 tests passed. Printed acceptance values are recorded in the Step 7 checklist line. The Vite CJS deprecation notice is tooling output, not an application warning.
9. `npm run verify` repeated after adding finger-box/table checks to sampled poses: exit 0 with 22 default test files (75 passed, 1 skipped), successful production build, and 8 acceptance tests passed. `heightCalibration.test.ts` printed 12.40 mm finger-box clearance and 12.60 mm side overlap; 20 headless Compact runs printed `20/20`.

## Step 7 acceptance results

| Row | Status | Evidence at this checkpoint |
|---|---|---|
| A. Predefined Task DONE, Compact | PASS | Evidence #9: `COMPLETE`, placed, no destination. |
| B. Compact random pairs | PASS | Evidence #9: 50/50 planner-valid seeded cases completed; fixture yield 6.9%, each rejected sample has a reason. |
| C. Rotated proof | PASS | Evidence #9: 130/130 planner-valid matrix cases DONE; 325 cases outside footprint. |
| D. Table ↔ Raised transport | PASS — experimental | Evidence #9: not certified; surfaces disabled by default. |
| E. Obstacle reroute/refusal | PASS — limited | Evidence #9: refused before transport for PRE-GRASP joint limit; no obstacle-specific collision refusal was shown. |
| F. SSR + headless console | PASS | Evidence #9: panel/viewport SSR and no application console.error/warn during headless run. Browser runtime remains unverified. |
| G. Large preset, 20 plans | PASS — report only | Evidence #9: 10 planned, 10 rejected; no Large execution acceptance claim. |

`npm run verify` runs lint, default Vitest, production build, then the acceptance suite. Current branch is `stabilize`; Steps 5–7 are committed separately. No browser was available for visual or runtime verification.
