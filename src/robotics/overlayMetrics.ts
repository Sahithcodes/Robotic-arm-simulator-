export type OverlayMetrics = { gridCells: number; firstPaintMs: number; fullGridMs: number; workerComputeMs: number; stepM: number; animationFrames:number; sceneRenders:number; longestMainBlockMs:number; surfaceReachability:Record<string,{pickablePct:number|null;placeablePct:number|null}> };
let current: OverlayMetrics = { gridCells: 0, firstPaintMs: 0, fullGridMs: 0, workerComputeMs: 0, stepM: 0, animationFrames:0, sceneRenders:0, longestMainBlockMs:0,surfaceReachability:{} };
const listeners = new Set<(metrics: OverlayMetrics) => void>();
export function getOverlayMetrics() { return current; }
export function setOverlayMetrics(next: Partial<OverlayMetrics>) { current = { ...current, ...next }; listeners.forEach((listener) => listener(current)); }
export function subscribeOverlayMetrics(listener: (metrics: OverlayMetrics) => void) { listeners.add(listener); return () => { listeners.delete(listener); }; }
export function recordAnimationFrame() { current.animationFrames++; }
export function recordSceneRender() { current.sceneRenders++; }
export function resetRenderCounters() { current.animationFrames=0; current.sceneRenders=0; current.longestMainBlockMs=0; }
export function recordMainThreadBlock(ms:number) { current.longestMainBlockMs=Math.max(current.longestMainBlockMs,ms); }
