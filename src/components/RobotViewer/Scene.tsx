'use client';

import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { Html, OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useSimulationStore } from '../../store/simulationStore';
import { BOOK, GRIPPER, TABLE, TASK5_WAYPOINTS } from '../../robotics/task5';
import { RobotKinematicChain, BookMesh } from './RobotKinematicChain';
import { clampObjectToTable, createPickPlan, evaluatePickabilityCell, PickabilityCell } from '../../robotics/autonomousPlanner';
import { rayToTableXY } from '../../robotics/tableInteraction';

export type CameraView = 'perspective' | 'top' | 'front' | 'right' | 'fitRobot' | 'fitTask';

const cameraPresets: Record<CameraView, { position: [number, number, number]; target: [number, number, number] }> = {
  perspective: { position: [1.48, -2.0, 1.84], target: [0.46, -0.04, 0.76] },
  top: { position: [0.8, -0.08, 2.95], target: [0.60, 0, 0.78] },
  front: { position: [0.8, -2.35, 1.35], target: [0.60, 0, 0.76] },
  right: { position: [3.0, -0.08, 1.35], target: [0.60, 0, 0.76] },
  fitRobot: { position: [1.04, -1.08, 1.15], target: [0.18, -0.02, 0.68] },
  fitTask: { position: [2.05, -2.72, 2.02], target: [0.63, -0.08, 0.76] },
};

export const Scene: React.FC<{ cameraView: CameraView; cameraRevision: number; cancelDragRef: React.MutableRefObject<() => void> }> = ({ cameraView, cameraRevision, cancelDragRef }) => {
  const controlsRef = useRef<React.ElementRef<typeof OrbitControls>>(null);
  const dragPointerRef = useRef<number | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [isHovered, setIsHovered] = useState(false);
  const [ghostPosition, setGhostPosition] = useState<{x:number;y:number}|null>(null);
  const [ghostValid, setGhostValid] = useState(false);
  const { camera, gl } = useThree();
  const {
    fkResult,
    isGripperOpen,
    gripperCloseWidth,
    showRobotDebug,
    showJointLabels,
    showGraspRegion,
    showPath,
    tickTask,
    setBookPosition,
    taskMode,
    autonomousPlan,
    simulatedObject,
    setObjectDragging,
    setObjectSelected,
    checkObjectReachability,
    isTaskPlaying,
    dropTarget,
    setDropTarget,
    destinationPickArmed,
    setDestinationPickArmed,
  } = useSimulationStore();

  const finishObjectDrag = useCallback((pointerId?: number) => {
    if (dragPointerRef.current === null || (pointerId !== undefined && dragPointerRef.current !== pointerId)) return;
    dragPointerRef.current = null;
    if (controlsRef.current) controlsRef.current.enabled = true;
    setObjectDragging(false);
    setIsDragging(false);
    gl.domElement.style.cursor = isHovered ? 'grab' : '';
    checkObjectReachability();
  }, [gl, isHovered, setObjectDragging, checkObjectReachability]);

  useEffect(() => {
    cancelDragRef.current = finishObjectDrag;
    const onWindowBlur = () => finishObjectDrag();
    window.addEventListener('blur', onWindowBlur);
    return () => {
      window.removeEventListener('blur', onWindowBlur);
      if (controlsRef.current) controlsRef.current.enabled = true;
      cancelDragRef.current = () => {};
    };
  }, [cancelDragRef, finishObjectDrag]);

  useEffect(() => {
    if (destinationPickArmed) {
      if (controlsRef.current) controlsRef.current.enabled = false;
      gl.domElement.style.cursor = 'crosshair';
      const onKeyDown = (event: KeyboardEvent) => { if (event.key === 'Escape') setDestinationPickArmed(false); };
      window.addEventListener('keydown', onKeyDown);
      return () => { window.removeEventListener('keydown', onKeyDown); if (controlsRef.current) controlsRef.current.enabled = true; gl.domElement.style.cursor = ''; };
    }
    setGhostPosition(null);
  }, [destinationPickArmed, gl, setDestinationPickArmed]);

  useEffect(() => {
    if (!destinationPickArmed || !ghostPosition) return;
    const timer = window.setTimeout(() => {
      const state = useSimulationStore.getState();
      const target = { ...ghostPosition, yaw: state.dropTarget?.yaw ?? 0 };
      const plan = createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, { ...target, yaw: target.yaw * Math.PI / 180 }, state.simulatedObject.rotation.yaw);
      setGhostValid(plan.reachable);
    }, 100);
    return () => window.clearTimeout(timer);
  }, [destinationPickArmed, ghostPosition]);

  useFrame((_, delta) => tickTask(Math.min(delta, 0.05)));

  useEffect(() => {
    const preset = cameraPresets[cameraView];
    let position = preset.position;
    let target = preset.target;
    if (cameraView === 'fitTask') {
      const minX = Math.min(0, TABLE.center.x - TABLE.width / 2, simulatedObject.position.x);
      const maxX = Math.max(TABLE.center.x + TABLE.width / 2, simulatedObject.position.x);
      const minY = Math.min(0, TABLE.center.y - TABLE.depth / 2, simulatedObject.position.y);
      const maxY = Math.max(TABLE.center.y + TABLE.depth / 2, simulatedObject.position.y);
      target = [(minX + maxX) / 2, (minY + maxY) / 2, TABLE.height * 0.92];
      const scale = Math.max(TABLE.width, TABLE.depth) / 0.9;
      position = [target[0] + 1.05 * scale, target[1] - 1.95 * scale, target[2] + 1.2 * scale];
    }
    camera.position.set(...position);
    camera.up.set(0, 0, 1);
    camera.lookAt(...target);
    if (controlsRef.current) {
      controlsRef.current.target.set(...target);
      controlsRef.current.update();
    }
  }, [camera, cameraView, cameraRevision, simulatedObject.position]);

  const pathGeometry = useMemo(
    () =>
      new THREE.BufferGeometry().setFromPoints(
        (taskMode === 'autonomous' && autonomousPlan?.reachable
          ? [fkResult.endEffectorPose, ...autonomousPlan.waypoints]
          : TASK5_WAYPOINTS).map(
          (waypoint) =>
            new THREE.Vector3(waypoint.position.x, waypoint.position.y, waypoint.position.z)
        )
      ),
    [taskMode, autonomousPlan, fkResult]
  );

  return (
    <>
      <ambientLight intensity={0.7} />
      <directionalLight position={[1.4, -2.1, 3.2]} intensity={1.25} castShadow />
      <pointLight position={[-1.5, 1.2, 2.2]} intensity={0.3} />

      {showRobotDebug && <axesHelper args={[0.32]} />}
      <gridHelper args={[1.55, 18, '#3f4a51', '#252d32']} rotation={[Math.PI / 2, 0, 0]} />

      <TableMesh />
      <ReachabilityEnvelope />
      {destinationPickArmed && <mesh position={[TABLE.center.x, TABLE.center.y, TABLE.height + 0.0005]} onPointerMove={(event) => { event.stopPropagation(); const point = rayToTableXY({ origin: event.ray.origin, direction: event.ray.direction }, TABLE.height); if (point) setGhostPosition(point); }} onPointerDown={(event) => { event.stopPropagation(); const point = rayToTableXY({ origin: event.ray.origin, direction: event.ray.direction }, TABLE.height); if (point) { const state = useSimulationStore.getState(); setDropTarget({ ...point, yaw: state.dropTarget?.yaw ?? 0 }, true); setDestinationPickArmed(false); } }}>
        <planeGeometry args={[TABLE.width, TABLE.depth]} /><meshBasicMaterial transparent opacity={0} depthWrite={false} />
      </mesh>}
      {destinationPickArmed && ghostPosition && <group position={[ghostPosition.x, ghostPosition.y, TABLE.height + BOOK.size.z / 2 + 0.003]} rotation={[0,0,(dropTarget?.yaw ?? 0)*Math.PI/180]} raycast={() => null}>
        <mesh raycast={() => null}><boxGeometry args={[BOOK.size.x, BOOK.size.y, 0.004]} /><meshBasicMaterial color={ghostValid ? '#49d877' : '#ed5f62'} wireframe /></mesh>
        <mesh raycast={() => null} position={[0,0,0.003]}><planeGeometry args={[BOOK.size.x, BOOK.size.y]} /><meshBasicMaterial color={ghostValid ? '#49d877' : '#ed5f62'} transparent opacity={0.16} side={THREE.DoubleSide} /></mesh>
      </group>}
      {dropTarget && <group position={[dropTarget.x, dropTarget.y, TABLE.height + BOOK.size.z / 2 + 0.002]} rotation={[0, 0, dropTarget.yaw * Math.PI / 180]} onPointerDown={destinationPickArmed ? undefined : (event) => { if (isTaskPlaying) return; event.stopPropagation(); event.nativeEvent.preventDefault(); event.nativeEvent.stopImmediatePropagation(); dragPointerRef.current = event.pointerId; controlsRef.current && (controlsRef.current.enabled = false); (event.target as unknown as { setPointerCapture(pointerId: number): void }).setPointerCapture(event.pointerId); }} onPointerMove={destinationPickArmed ? undefined : (event) => { if (dragPointerRef.current !== event.pointerId) return; event.stopPropagation(); const point = rayToTableXY({ origin: event.ray.origin, direction: event.ray.direction }, TABLE.height); if(point) setDropTarget({...dropTarget,x:point.x,y:point.y}); }} onPointerUp={destinationPickArmed ? undefined : (event) => { if(dragPointerRef.current !== event.pointerId) return; dragPointerRef.current=null; if(controlsRef.current) controlsRef.current.enabled=true; setDropTarget(useSimulationStore.getState().dropTarget,true); (event.target as unknown as { releasePointerCapture(pointerId:number):void }).releasePointerCapture(event.pointerId); }}>
        <mesh raycast={destinationPickArmed ? () => null : undefined}><boxGeometry args={[BOOK.size.x, BOOK.size.y, 0.004]} /><meshBasicMaterial color="#42d9e8" wireframe /></mesh>
        <mesh raycast={destinationPickArmed ? () => null : undefined} position={[0,0,0.006]}><planeGeometry args={[BOOK.size.x, BOOK.size.y]} /><meshBasicMaterial color="#42d9e8" transparent opacity={0.12} side={THREE.DoubleSide} /></mesh>
        <Html position={[0, 0, 0.02]} center distanceFactor={1.5} style={{ color: '#42d9e8', fontSize: '11px', fontWeight: 700, pointerEvents: 'none' }}>PLACE</Html>
      </group>}
      <group
          raycast={destinationPickArmed ? () => null : undefined}
          position={[simulatedObject.position.x, simulatedObject.position.y, simulatedObject.position.z]}
          rotation={[simulatedObject.rotation.roll, simulatedObject.rotation.pitch, simulatedObject.rotation.yaw]}
          onPointerDown={(event) => {
            if (destinationPickArmed) return;
            if (isTaskPlaying) { event.stopPropagation(); return; }
            if (event.button !== 0) {
              event.stopPropagation();
              event.nativeEvent.stopImmediatePropagation();
              return;
            }
            event.stopPropagation();
            event.nativeEvent.preventDefault();
            event.nativeEvent.stopImmediatePropagation();
            dragPointerRef.current = event.pointerId;
            controlsRef.current && (controlsRef.current.enabled = false);
            setObjectDragging(true);
            setObjectSelected(true);
            setIsDragging(true);
            gl.domElement.style.cursor = 'grabbing';
            (event.target as unknown as { setPointerCapture(pointerId: number): void }).setPointerCapture(event.pointerId);
          }}
          onPointerMove={(event) => {
            if (dragPointerRef.current !== event.pointerId) return;
            event.stopPropagation();
            event.nativeEvent.preventDefault();
            event.nativeEvent.stopImmediatePropagation();
            const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -TABLE.height);
            const point = event.ray.intersectPlane(plane, new THREE.Vector3());
            if (point) setBookPosition(clampObjectToTable(point, simulatedObject.rotation.yaw));
          }}
          onPointerUp={(event) => {
            event.stopPropagation();
            event.nativeEvent.stopImmediatePropagation();
            (event.target as unknown as { releasePointerCapture(pointerId: number): void }).releasePointerCapture(event.pointerId);
            finishObjectDrag(event.pointerId);
          }}
          onPointerCancel={(event) => finishObjectDrag(event.pointerId)}
          onPointerMissed={() => setObjectSelected(false)}
          onPointerLeave={(event) => {
            if (!event.buttons) finishObjectDrag(event.pointerId);
          }}
          onPointerOver={() => {
            if (destinationPickArmed) return;
            setIsHovered(true);
            if (!isDragging) gl.domElement.style.cursor = 'grab';
          }}
          onPointerOut={() => {
            if (destinationPickArmed) return;
            setIsHovered(false);
            if (!isDragging) setObjectSelected(false);
            if (!isDragging) gl.domElement.style.cursor = '';
          }}
      >
          <BookMesh highlighted={isHovered || isDragging || simulatedObject.isSelected || simulatedObject.isAttached} />
      </group>
      <mesh raycast={destinationPickArmed ? () => null : undefined} position={[simulatedObject.position.x, simulatedObject.position.y, TABLE.height + 0.004]}>
        <ringGeometry args={[0.064, 0.071, 32]} />
        <meshBasicMaterial
          color={simulatedObject.reachability.reachable ? '#6bc982' : simulatedObject.reachability.reason.includes('Position changed') ? '#d8b05c' : '#d36a60'}
          side={THREE.DoubleSide}
        />
      </mesh>

      {taskMode === 'autonomous' && autonomousPlan?.reachable && (
        <mesh position={[autonomousPlan.waypoints[1].position.x, autonomousPlan.waypoints[1].position.y, autonomousPlan.waypoints[1].position.z]}>
          <sphereGeometry args={[0.012, 16, 12]} />
          <meshStandardMaterial color="#f0ca62" emissive="#675018" />
        </mesh>
      )}

      {showPath && (
        <>
          <line>
            <primitive object={pathGeometry} attach="geometry" />
            <lineBasicMaterial color="#d8b05c" />
          </line>
          {(taskMode === 'autonomous' && autonomousPlan?.reachable ? autonomousPlan.waypoints : TASK5_WAYPOINTS).map((waypoint, index) => (
            <mesh key={waypoint.id} position={[waypoint.position.x, waypoint.position.y, waypoint.position.z]}>
              <sphereGeometry args={[index === 2 ? 0.018 : 0.014, 16, 12]} />
              <meshStandardMaterial color={index === 2 ? '#e86f51' : '#d8b05c'} />
            </mesh>
          ))}
        </>
      )}

      {showGraspRegion && (
        <mesh
          position={[
            TASK5_WAYPOINTS[2].position.x,
            TASK5_WAYPOINTS[2].position.y,
            TASK5_WAYPOINTS[2].position.z + GRIPPER.fingerLength,
          ]}
        >
          <boxGeometry args={[GRIPPER.closedWidth + 0.035, BOOK.size.y + 0.035, BOOK.size.z + 0.03]} />
          <meshStandardMaterial color="#7dd3fc" transparent opacity={0.22} wireframe />
        </mesh>
      )}

      <RobotKinematicChain
        fkResult={fkResult}
        isGripperOpen={isGripperOpen}
        closeWidth={gripperCloseWidth}
        showRobotDebug={showRobotDebug}
        showJointLabels={showJointLabels}
      />

      <OrbitControls
        ref={controlsRef}
        makeDefault
        target={[0.46, -0.04, 0.76]}
        minDistance={0.32}
        maxDistance={2.8}
        enableDamping
        dampingFactor={0.06}
      />
    </>
  );
};

const TableMesh: React.FC = () => {
  const visualWidth = TABLE.width;
  const visualDepth = TABLE.depth;
  const topZ = TABLE.height;
  const topCenterZ = topZ - TABLE.topThickness / 2;
  const legHeight = topZ - TABLE.topThickness;
  const legZ = legHeight / 2;
  const legInsetX = visualWidth / 2 - 0.035;
  const legInsetY = visualDepth / 2 - 0.035;

  return (
    <group position={[TABLE.center.x, TABLE.center.y, 0]}>
      <mesh position={[0, 0, topCenterZ]} castShadow receiveShadow>
        <boxGeometry args={[visualWidth, visualDepth, TABLE.topThickness]} />
        <meshStandardMaterial color="#3b4246" roughness={0.68} metalness={0.08} />
      </mesh>
      {[
        [-legInsetX, -legInsetY],
        [legInsetX, -legInsetY],
        [-legInsetX, legInsetY],
        [legInsetX, legInsetY],
      ].map(([x, y]) => (
        <mesh key={`${x}-${y}`} position={[x, y, legZ]} castShadow receiveShadow>
          <boxGeometry args={[0.026, 0.026, legHeight]} />
          <meshStandardMaterial color="#30383d" roughness={0.58} metalness={0.22} />
        </mesh>
      ))}
    </group>
  );
};

const ReachabilityEnvelope: React.FC = () => {
  const { dhTable, jointAngles, simulatedObject, isTaskPlaying, dropTarget, tablePreset } = useSimulationStore();
  const [cells, setCells] = useState<PickabilityCell[]>([]);
  const cache = useRef(new Map<string, PickabilityCell[]>());
  useEffect(() => {
    if (isTaskPlaying) return;
    const yaw = simulatedObject.rotation.yaw;
    const key = `${tablePreset}:${TABLE.width}:${TABLE.depth}:${TABLE.height}:${TABLE.topThickness}:${TABLE.center.x}:${TABLE.center.y}:${BOOK.size.x}:${BOOK.size.y}:${BOOK.size.z}:${dhTable.map((link) => `${link.a},${link.alpha},${link.d},${link.thetaMin},${link.thetaMax}`).join(';')}:${yaw.toFixed(4)}:${jointAngles.map((q) => q.toFixed(3)).join(',')}`;
    const cached = cache.current.get(key);
    if (cached) { setCells(cached); return; }
    setCells([]);
    const timer = window.setTimeout(() => {
      const computed: PickabilityCell[] = [];
      const points: { x: number; y: number; z: number }[] = [];
      const step = 0.01;
      const xmin = TABLE.center.x - TABLE.width / 2 + step / 2;
      const xmax = TABLE.center.x + TABLE.width / 2;
      const ymin = TABLE.center.y - TABLE.depth / 2 + step / 2;
      const ymax = TABLE.center.y + TABLE.depth / 2;
      for (let x = xmin; x < xmax; x += step) for (let y = ymin; y < ymax; y += step) points.push({ x, y, z: simulatedObject.position.z });
      let index = 0;
      let frame = 0;
      const process = () => {
        const end = Math.min(index + 2, points.length);
        for (; index < end; index++) computed.push(evaluatePickabilityCell(dhTable, jointAngles, points[index], yaw, dropTarget ? { ...dropTarget, yaw: dropTarget.yaw * Math.PI / 180 } : null));
        if (index < points.length) frame = window.requestAnimationFrame(process);
        else {
          cache.current.set(key, computed);
          if (cache.current.size > 12) cache.current.delete(cache.current.keys().next().value as string);
          setCells(computed);
        }
      };
      frame = window.requestAnimationFrame(process);
      cleanupFrame = () => window.cancelAnimationFrame(frame);
    }, 300);
    let cleanupFrame = () => {};
    return () => { window.clearTimeout(timer); cleanupFrame(); };
  }, [dhTable, dropTarget, isTaskPlaying, jointAngles, simulatedObject.position.z, simulatedObject.rotation.yaw, tablePreset]);

  const geometry = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
    const vertices: number[] = [];
    const indices: number[] = [];
    const step = 0.01;
    for (const cell of cells) {
      if (cell.reachable) {
        const x = cell.position.x - TABLE.center.x - step / 2;
        const y = cell.position.y - TABLE.center.y - step / 2;
        const base = vertices.length / 3;
        vertices.push(x, y, 0, x + step, y, 0, x + step, y + step, 0, x, y + step, 0);
        indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
      }
    }
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    return geometry;
  }, [cells]);

  useEffect(() => () => geometry.dispose(), [geometry]);

  return (
    <mesh position={[TABLE.center.x, TABLE.center.y, TABLE.height + 0.002]} geometry={geometry} raycast={() => null}>
      <meshBasicMaterial color="#64b77b" transparent opacity={0.16} side={THREE.DoubleSide} depthWrite={false} />
    </mesh>
  );
};
