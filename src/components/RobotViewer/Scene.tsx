'use client';

import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useSimulationStore } from '../../store/simulationStore';
import { BOOK, GRIPPER, TABLE, TASK5_WAYPOINTS } from '../../robotics/task5';
import { RobotKinematicChain, BookMesh } from './RobotKinematicChain';
import { clampObjectToTable } from '../../robotics/autonomousPlanner';

export type CameraView = 'perspective' | 'top' | 'front' | 'right' | 'fitRobot' | 'fitTask';

const cameraPresets: Record<CameraView, { position: [number, number, number]; target: [number, number, number] }> = {
  perspective: { position: [1.48, -2.0, 1.84], target: [0.46, -0.04, 0.76] },
  top: { position: [0.8, -0.08, 2.95], target: [0.8, -0.08, 0.78] },
  front: { position: [0.8, -2.35, 1.35], target: [0.8, -0.08, 0.76] },
  right: { position: [3.0, -0.08, 1.35], target: [0.8, -0.08, 0.76] },
  fitRobot: { position: [1.04, -1.08, 1.15], target: [0.18, -0.02, 0.68] },
  fitTask: { position: [2.05, -2.72, 2.02], target: [0.63, -0.08, 0.76] },
};

export const Scene: React.FC<{ cameraView: CameraView; cameraRevision: number; cancelDragRef: React.MutableRefObject<() => void> }> = ({ cameraView, cameraRevision, cancelDragRef }) => {
  const controlsRef = useRef<React.ElementRef<typeof OrbitControls>>(null);
  const dragPointerRef = useRef<number | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [isHovered, setIsHovered] = useState(false);
  const { camera, gl } = useThree();
  const {
    fkResult,
    isGripperOpen,
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
      <mesh position={[simulatedObject.targetPosition.x, simulatedObject.targetPosition.y, simulatedObject.targetPosition.z + 0.012]}>
        <sphereGeometry args={[0.018, 16, 12]} />
        <meshStandardMaterial color={simulatedObject.reachability.reachable ? '#78c98a' : '#cf9861'} emissive={simulatedObject.reachability.reachable ? '#174c28' : '#4a2d14'} />
      </mesh>
      <group
          position={[simulatedObject.position.x, simulatedObject.position.y, simulatedObject.position.z]}
          rotation={[simulatedObject.rotation.roll, simulatedObject.rotation.pitch, simulatedObject.rotation.yaw]}
          onPointerDown={(event) => {
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
            setIsHovered(true);
            if (!isDragging) gl.domElement.style.cursor = 'grab';
          }}
          onPointerOut={() => {
            setIsHovered(false);
            if (!isDragging) setObjectSelected(false);
            if (!isDragging) gl.domElement.style.cursor = '';
          }}
      >
          <BookMesh highlighted={isHovered || isDragging || simulatedObject.isSelected || simulatedObject.isAttached} />
      </group>
      <mesh position={[simulatedObject.position.x, simulatedObject.position.y, TABLE.height + 0.004]}>
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
  const geometry = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
    const vertices: number[] = [];
    const indices: number[] = [];
    const usableX = TABLE.width / 2 - BOOK.size.x / 2 - 0.012;
    const usableY = TABLE.depth / 2 - BOOK.size.y / 2 - 0.012;
    const step = 0.025;
    const pickableHalfDepth = TABLE.depth * 0.29;
    for (let x = -usableX; x < usableX; x += step) {
      for (let y = -usableY; y < usableY; y += step) {
        const centerX = TABLE.center.x + x + step / 2;
        const centerY = TABLE.center.y + y + step / 2;
        if (Math.abs(centerY - TABLE.center.y) > pickableHalfDepth || centerX > 0.78 || Math.hypot(centerX, centerY) < 0.25) continue;
        const base = vertices.length / 3;
        vertices.push(x, y, 0, x + step, y, 0, x + step, y + step, 0, x, y + step, 0);
        indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
      }
    }
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    return geometry;
  }, []);

  useEffect(() => () => geometry.dispose(), [geometry]);

  return (
    <mesh position={[TABLE.center.x, TABLE.center.y, TABLE.height + 0.002]} geometry={geometry}>
      <meshBasicMaterial color="#64b77b" transparent opacity={0.16} side={THREE.DoubleSide} depthWrite={false} />
    </mesh>
  );
};
