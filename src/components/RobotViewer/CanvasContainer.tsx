'use client';

import React, { useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { CameraView, Scene } from './Scene';

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

      <Canvas
        camera={{ position: [1.48, -2.0, 1.84], fov: 40, up: [0, 0, 1] }}
        shadows
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
