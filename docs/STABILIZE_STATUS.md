# Stabilization status

- **PASS — Step 1, evidence #1:** `npm run build` succeeded; `npm run dev` served `/` with HTTP 200 and a 10,101-byte response; Task panel + viewport SSR smoke test passed (1 test). The red toast was not reproduced. Added viewport/panel ErrorBoundaries and a copyable global Errors box.
- **PASS — Step 2, evidence #2:** Compact Table top `0.7800 m`, book center `0.7925 m`. Geometry audit found the tool axis incorrectly tested against the arm-link clearance plane at `0.805 m` (false penetration `2.739 mm` at `0.802261 m`). The arm link plane still enforces 25 mm; tool contact now checks the real tabletop. Printed grasp proof: TCP `0.7846 m`, finger-box clearance `3.00 mm`, book-side overlap `22.00 mm`. Planner + headless execution + height tests: 10 passed, including 3×3 pickup positions.
- **PASS — Step 3, evidence #3:** Headless Predefined Task completed (`COMPLETE`, `state=placed`) on Compact and Large with no destination, using its separate established pose. Printed final center for both: `(0.717678, 0.000041, 0.815952) m`. This predefined trajectory retains its own height; autonomous/reset starts at the true resting center `0.7925 m`.
- **FAIL — Step 4, evidence #4:** The 105° → 45° trace passes: grasp tool yaw 15°, J6 −18.8°, saved relative yaw 90°, place tool yaw −45°, release yaw 45°, error 0.00°, phase DONE. Existing zero/180 equivalent-wrist test also passes. The requested 5 × 13 × 7 matrix is not verified: the attempted fixed-pair fixture yielded 0/455 plans and 455 `link-vs-base collision` results, so it did not establish the requested valid-pair coverage. Strict 2° release checking remains enabled. Book mesh yaw symmetry has not been audited.
- **NOT STARTED — Step 5, evidence #0:** Unified candidate-coupled pick and place planning.
- **NOT STARTED — Step 6, evidence #0:** Table-only defaults and surface preset reachability.
- **NOT STARTED — Step 7, evidence #0:** Acceptance suite and `npm run verify`.

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
