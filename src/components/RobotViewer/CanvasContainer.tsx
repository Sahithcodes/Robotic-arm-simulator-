'use client';

import React, { useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { CameraView, Scene } from './Scene';
import { useSimulationStore } from '../../store/simulationStore';
import { useShallow } from 'zustand/react/shallow';
import { getOverlayMetrics, OverlayMetrics, resetRenderCounters, subscribeOverlayMetrics } from '../../robotics/overlayMetrics';

const cameraViews: { id: CameraView; label: string }[] = [
  { id: 'perspective', label: '3D' },
  { id: 'top', label: 'TOP' },
  { id: 'front', label: 'FRONT' },
  { id: 'right', label: 'RIGHT' },
  { id: 'fitRobot', label: 'FIT ROBOT' },
  { id: 'fitTask', label: 'FIT TASK' },
];

export const CanvasContainer: React.FC = () => {
  const [mounted, setMounted] = useState(false);
  const [cameraView, setCameraView] = useState<CameraView>('perspective');
  const [cameraRevision, setCameraRevision] = useState(0);
  const cancelDragRef = React.useRef<() => void>(() => {});
  const { tablePreset, debugMode, showReachabilityOverlay, setShowReachabilityOverlay, showPerformanceDebug, setShowPerformanceDebug } = useSimulationStore(useShallow((s) => ({tablePreset:s.tablePreset,debugMode:s.debugMode,showReachabilityOverlay:s.showReachabilityOverlay,setShowReachabilityOverlay:s.setShowReachabilityOverlay,showPerformanceDebug:s.showPerformanceDebug,setShowPerformanceDebug:s.setShowPerformanceDebug})));

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return (
      <div className="w-full h-full flex flex-col items-center justify-center text-slate-500 space-y-2 viewport-shell">
        <span className="text-xs font-mono">Initializing 3D WebGL Kinematic Renderer...</span>
      </div>
    );
  }

  return (
    <div className="relative w-full h-full overflow-hidden viewport-shell">
      <div className="absolute top-2 left-2 z-10 viewport-title">
        3D KINEMATIC CHAIN VIEWPORT
      </div>

      <div className="absolute top-2 right-2 z-10 camera-hint hidden sm:flex">
        <span>Rotate: Left Click + Drag</span>
        <span>Pan: Right Click + Drag</span>
        <span>Zoom: Scroll</span>
      </div>

      <div className="absolute top-8 left-2 z-10 camera-control-strip">
        {cameraViews.map((view) => (
          <button
            key={view.id}
            type="button"
            className={`camera-button ${cameraView === view.id ? 'active' : ''}`}
            onClick={() => {
              setCameraView(view.id);
              setCameraRevision((revision) => revision + 1);
            }}
          >
            {view.label}
          </button>
        ))}
      </div>

      {debugMode && <div className="absolute top-16 left-2 z-10 flex gap-3 rounded bg-slate-900/80 px-2 py-1 text-[10px] font-mono text-slate-200">
        <label><input type="checkbox" checked={showReachabilityOverlay} onChange={(event)=>setShowReachabilityOverlay(event.target.checked)} /> PICK OVERLAY</label>
        <label><input type="checkbox" checked={showPerformanceDebug} onChange={(event)=>setShowPerformanceDebug(event.target.checked)} /> PERF</label>
      </div>}
      {debugMode && showPerformanceDebug && <PerformanceReadout />}

      <Canvas
        camera={{ position: [1.48, -2.0, 1.84], fov: 40, up: [0, 0, 1] }}
        shadows={tablePreset === 'compact'}
        dpr={[1, 1.5]}
        gl={{ antialias: true }}
        frameloop="always"
        className="w-full h-full"
        onPointerMissed={() => cancelDragRef.current()}
      >
        <Scene cameraView={cameraView} cameraRevision={cameraRevision} cancelDragRef={cancelDragRef} />
      </Canvas>

      <div className="absolute bottom-2 left-2 z-10 axis-readout">
        X RED&nbsp;&nbsp;Y GREEN&nbsp;&nbsp;Z BLUE
      </div>
    </div>
  );
};

const PerformanceReadout: React.FC = () => {
  const [fps,setFps]=useState(0),[metrics,setMetrics]=useState<OverlayMetrics>(getOverlayMetrics);
  useEffect(()=>{resetRenderCounters();const unsubscribe=subscribeOverlayMetrics((next)=>setMetrics(next));let frame=0,count=0,start=performance.now(),alive=true;const tick=(now:number)=>{if(!alive)return;count++;if(now-start>=500){setFps(Math.round(count*1000/(now-start)));count=0;start=now;setMetrics(getOverlayMetrics());}frame=requestAnimationFrame(tick);};frame=requestAnimationFrame(tick);return()=>{alive=false;unsubscribe();cancelAnimationFrame(frame);};},[]);
  const rendersPer100=metrics.animationFrames?Math.round(metrics.sceneRenders*100/metrics.animationFrames):0;
  return <div className="absolute bottom-8 right-2 z-10 rounded bg-slate-950/85 px-2 py-1 text-[10px] font-mono text-emerald-300 pointer-events-none">FPS {fps} · Scene renders/100 frames {rendersPer100} · overlay {metrics.gridCells} cells · first {metrics.firstPaintMs.toFixed(0)} ms · full {metrics.fullGridMs<0?'working':`${metrics.fullGridMs.toFixed(0)} ms`} · longest main block {metrics.longestMainBlockMs.toFixed(2)} ms · worker {metrics.workerComputeMs.toFixed(0)} ms</div>;
};
