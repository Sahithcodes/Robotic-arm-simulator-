'use client';

import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { Html, OrbitControls } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useShallow } from 'zustand/react/shallow';
import { useSimulationStore } from '../../store/simulationStore';
import { BOOK, GRIPPER, TABLE, TASK5_WAYPOINTS } from '../../robotics/task5';
import { RobotKinematicChain, BookMesh } from './RobotKinematicChain';
import { PickabilityGridShape, PickabilityWorkerConfig, pickabilityCacheKey } from '../../robotics/pickabilityGrid';
import { getOverlayMetrics, recordAnimationFrame, recordMainThreadBlock, recordSceneRender, setOverlayMetrics } from '../../robotics/overlayMetrics';
import { PlacementSurface, rayToNearestSurface, surfaceBookZ } from '../../robotics/placementSurfaces';

export type CameraView = 'perspective' | 'top' | 'front' | 'right' | 'fitRobot' | 'fitTask';

const cameraPresets: Record<CameraView, { position: [number, number, number]; target: [number, number, number] }> = {
  perspective: { position: [1.48, -2.0, 1.84], target: [0.46, -0.04, 0.76] },
  top: { position: [0.8, -0.08, 2.95], target: [0.60, 0, 0.78] },
  front: { position: [0.8, -2.35, 1.35], target: [0.60, 0, 0.76] },
  right: { position: [3.0, -0.08, 1.35], target: [0.60, 0, 0.76] },
  fitRobot: { position: [1.04, -1.08, 1.15], target: [0.18, -0.02, 0.68] },
  fitTask: { position: [2.05, -2.72, 2.02], target: [0.63, -0.08, 0.76] },
};

type GridData = { shape: PickabilityGridShape; states: Uint8Array };
const overlayGridCache = new Map<string, Map<number, GridData>>();
const MAX_GRID_CACHE_KEYS = 8;
function makeOverlayTexture({ shape, states }: GridData): THREE.CanvasTexture {
  const canvas = document.createElement('canvas'); canvas.width = shape.columns; canvas.height = shape.rows;
  const context = canvas.getContext('2d');
  if (!context) throw new Error('Canvas 2D context unavailable for reachability overlay');
  const image = context.createImageData(shape.columns, shape.rows);
  for (let ix = 0; ix < shape.columns; ix++) for (let iy = 0; iy < shape.rows; iy++) {
    if (states[ix * shape.rows + iy] !== 2) continue;
    const at = ((shape.rows - 1 - iy) * shape.columns + ix) * 4;
    image.data[at] = 100; image.data[at + 1] = 183; image.data[at + 2] = 123; image.data[at + 3] = 92;
  }
  context.putImageData(image, 0, 0);
  const texture = new THREE.CanvasTexture(canvas); texture.colorSpace = THREE.SRGBColorSpace; texture.needsUpdate = true;
  return texture;
}

export const Scene: React.FC<{ cameraView: CameraView; cameraRevision: number; cancelDragRef: React.MutableRefObject<() => void> }> = ({ cameraView, cameraRevision, cancelDragRef }) => {
  recordSceneRender();
  const controlsRef = useRef<React.ElementRef<typeof OrbitControls>>(null);
  const dragPointerRef = useRef<number | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [isHovered, setIsHovered] = useState(false);
  const [ghostPosition, setGhostPosition] = useState<{x:number;y:number;z:number;surfaceId:string}|null>(null);
  const { camera, gl } = useThree();
  const { fkResult,isGripperOpen,gripperCloseWidth,showRobotDebug,showJointLabels,showGraspRegion,showPath,setBookPosition,taskMode,autonomousPlan,simulatedObject,setObjectDragging,setObjectSelected,checkObjectReachability,isTaskPlaying,dropTarget,setDropTarget,destinationPickArmed,setDestinationPickArmed,tablePreset,placementSurfaces } = useSimulationStore(useShallow((s)=>({fkResult:s.fkResult,isGripperOpen:s.isGripperOpen,gripperCloseWidth:s.gripperCloseWidth,showRobotDebug:s.showRobotDebug,showJointLabels:s.showJointLabels,showGraspRegion:s.showGraspRegion,showPath:s.showPath,setBookPosition:s.setBookPosition,taskMode:s.taskMode,autonomousPlan:s.autonomousPlan,simulatedObject:s.simulatedObject,setObjectDragging:s.setObjectDragging,setObjectSelected:s.setObjectSelected,checkObjectReachability:s.checkObjectReachability,isTaskPlaying:s.isTaskPlaying,dropTarget:s.dropTarget,setDropTarget:s.setDropTarget,destinationPickArmed:s.destinationPickArmed,setDestinationPickArmed:s.setDestinationPickArmed,tablePreset:s.tablePreset,placementSurfaces:s.placementSurfaces})));
  const tickTask = useSimulationStore((s) => s.tickTask);

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

  useFrame((_, delta) => { recordAnimationFrame(); tickTask(Math.min(delta, 0.05)); });

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
  const targetSurface=dropTarget?placementSurfaces.find((surface)=>surface.id===(dropTarget.surfaceId??'table')):undefined;

  return (
    <>
      <ambientLight intensity={0.7} />
      <directionalLight position={[1.4, -2.1, 3.2]} intensity={1.25} castShadow={tablePreset === 'compact'} />
      <pointLight position={[-1.5, 1.2, 2.2]} intensity={0.3} />

      {showRobotDebug && <axesHelper args={[0.32]} />}
      <gridHelper args={[1.55, 18, '#3f4a51', '#252d32']} rotation={[Math.PI / 2, 0, 0]} />

      <TableMesh />
      <AdditionalSurfaceMeshes surfaces={placementSurfaces} />
      {!isTaskPlaying && placementSurfaces.map((surface)=><ReachabilityEnvelope key={surface.id} surface={surface} />)}
      {destinationPickArmed && placementSurfaces.map((surface)=><mesh key={surface.id} position={[surface.center.x,surface.center.y,surface.z+0.0005]} rotation={[0,0,surface.yaw]} onPointerMove={(event) => { event.stopPropagation(); const hit=rayToNearestSurface({origin:event.ray.origin,direction:event.ray.direction},placementSurfaces,simulatedObject.rotation.yaw); if(hit)setGhostPosition({...hit.point,surfaceId:hit.surfaceId}); }} onPointerDown={(event) => { event.stopPropagation(); const hit=rayToNearestSurface({origin:event.ray.origin,direction:event.ray.direction},placementSurfaces,simulatedObject.rotation.yaw); if(hit){const state=useSimulationStore.getState();setDropTarget({...hit.point,yaw:state.dropTarget?.yaw??0,surfaceId:hit.surfaceId},true);setDestinationPickArmed(false);} }}><planeGeometry args={[surface.size.width,surface.size.depth]} /><meshBasicMaterial transparent opacity={0} depthWrite={false} /></mesh>)}
      {destinationPickArmed && ghostPosition && <group position={[ghostPosition.x, ghostPosition.y, ghostPosition.z + 0.003]} rotation={[0,0,(dropTarget?.yaw ?? 0)*Math.PI/180]} raycast={() => null}>
        <mesh raycast={() => null}><boxGeometry args={[BOOK.size.x, BOOK.size.y, 0.004]} /><meshBasicMaterial color="#42d9e8" wireframe /></mesh>
        <mesh raycast={() => null} position={[0,0,0.003]}><planeGeometry args={[BOOK.size.x, BOOK.size.y]} /><meshBasicMaterial color="#42d9e8" transparent opacity={0.12} side={THREE.DoubleSide} /></mesh>
      </group>}
      {dropTarget && <group position={[dropTarget.x, dropTarget.y, (dropTarget.z??(targetSurface?surfaceBookZ(targetSurface):0))+0.002]} rotation={[0, 0, dropTarget.yaw * Math.PI / 180]} onPointerDown={destinationPickArmed ? undefined : (event) => { if (isTaskPlaying) return; event.stopPropagation(); event.nativeEvent.preventDefault(); event.nativeEvent.stopImmediatePropagation(); dragPointerRef.current = event.pointerId; controlsRef.current && (controlsRef.current.enabled = false); (event.target as unknown as { setPointerCapture(pointerId: number): void }).setPointerCapture(event.pointerId); }} onPointerMove={destinationPickArmed ? undefined : (event) => { if (dragPointerRef.current !== event.pointerId) return; event.stopPropagation(); const hit=rayToNearestSurface({origin:event.ray.origin,direction:event.ray.direction},placementSurfaces,dropTarget.yaw*Math.PI/180); if(hit)setDropTarget({...dropTarget,...hit.point,surfaceId:hit.surfaceId},false,true); }} onPointerUp={destinationPickArmed ? undefined : (event) => { if(dragPointerRef.current !== event.pointerId) return; dragPointerRef.current=null; if(controlsRef.current) controlsRef.current.enabled=true; setDropTarget(useSimulationStore.getState().dropTarget,true); (event.target as unknown as { releasePointerCapture(pointerId:number):void }).releasePointerCapture(event.pointerId); }}>
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
            const hit=rayToNearestSurface({origin:event.ray.origin,direction:event.ray.direction},placementSurfaces,simulatedObject.rotation.yaw);
            if(hit)setBookPosition(hit.point,hit.surfaceId);
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

const AdditionalSurfaceMeshes:React.FC<{surfaces:PlacementSurface[]}>=({surfaces})=><>
  {surfaces.filter((surface)=>surface.id!=='table').map((surface)=>{
    const thickness=surface.tray?.floorThickness??.025,legs=surface.support.type==='legs';
    return <group key={surface.id} position={[surface.center.x,surface.center.y,0]} rotation={[0,0,surface.yaw]}>
      {surface.shape==='rectangle'?<mesh position={[0,0,surface.z-thickness/2]} castShadow receiveShadow><boxGeometry args={[surface.size.width,surface.size.depth,thickness]}/><meshStandardMaterial color={surface.color} roughness={.68}/></mesh>:<mesh position={[0,0,surface.z]} rotation={[0,0,0]}><ringGeometry args={[surface.size.innerRadius??.1,surface.size.outerRadius??.2,48,surface.size.startAngle??0,(surface.size.endAngle??Math.PI*2)-(surface.size.startAngle??0)]}/><meshStandardMaterial color={surface.color} side={THREE.DoubleSide}/></mesh>}
      {surface.support.type==='column'&&<mesh position={[0,0,(surface.support.height??surface.z)/2]}><cylinderGeometry args={[surface.support.radius??.035,surface.support.radius??.035,surface.support.height??surface.z,16]}/><meshStandardMaterial color="#41484d"/></mesh>}
      {legs&&[[-1,-1],[1,-1],[-1,1],[1,1]].map(([sx,sy])=><mesh key={`${sx}:${sy}`} position={[sx*(surface.size.width/2-.025),sy*(surface.size.depth/2-.025),(surface.z-thickness)/2]}><boxGeometry args={[surface.support.width??.025,surface.support.depth??.025,surface.support.height??surface.z-thickness]}/><meshStandardMaterial color="#454d51"/></mesh>)}
      {surface.tray&&<>{[[-1,0],[1,0]].map(([sx])=><mesh key={`wall-x-${sx}`} position={[sx*(surface.size.width/2-.004),0,surface.z+surface.tray!.wallHeight/2]}><boxGeometry args={[.008,surface.size.depth,surface.tray!.wallHeight]}/><meshStandardMaterial color={surface.color}/></mesh>)}{[[-1,0],[1,0]].map(([,sy])=><mesh key={`wall-y-${sy}`} position={[0,sy*(surface.size.depth/2-.004),surface.z+surface.tray!.wallHeight/2]}><boxGeometry args={[surface.size.width,.008,surface.tray!.wallHeight]}/><meshStandardMaterial color={surface.color}/></mesh>)}</>}
    </group>;
  })}
</>;

const ReachabilityEnvelope: React.FC<{surface:PlacementSurface}> = ({surface}) => {
  const { dhTable, jointAngles, bookYaw, tablePreset, enabled, simulatedObject, placementSurfaces } = useSimulationStore(useShallow((s) => ({ dhTable:s.dhTable,jointAngles:s.jointAngles,bookYaw:s.simulatedObject.rotation.yaw,tablePreset:s.tablePreset,enabled:s.showReachabilityOverlay,simulatedObject:s.simulatedObject,placementSurfaces:s.placementSurfaces })));
  const [texture, setTexture] = useState<THREE.CanvasTexture | null>(null);
  const textureRef = useRef<THREE.CanvasTexture | null>(null);
  useEffect(() => {
    if (!enabled) { textureRef.current?.dispose();textureRef.current=null;setTexture(null);return; }
    setOverlayMetrics({surfaceReachability:{...getOverlayMetrics().surfaceReachability,[surface.id]:{pickablePct:null,placeablePct:null}}});
    const yawRad = Math.round(bookYaw * 180 / Math.PI) * Math.PI / 180;
    const config: PickabilityWorkerConfig = { preset:tablePreset,dhTable,jointAngles,yawRad,bookZ:surfaceBookZ(surface),surface,surfaces:placementSurfaces };
    const key = pickabilityCacheKey(config), cached = overlayGridCache.get(key), coarseStep=.03, fineStep=.01;
    if (cached?.has(coarseStep)) { const grid=cached.get(coarseStep)!; textureRef.current?.dispose(); const next=makeOverlayTexture(grid); textureRef.current=next; setTexture(next);setOverlayMetrics({gridCells:grid.shape.count,firstPaintMs:0,fullGridMs:cached.has(fineStep)?0:-1,stepM:coarseStep}); }
    const runPlaceOnly=!!cached?.has(fineStep);
    if (cached?.has(fineStep)) { const grid=cached.get(fineStep)!; textureRef.current?.dispose(); const next=makeOverlayTexture(grid); textureRef.current=next; setTexture(next);const valid=grid.states.reduce((n,state)=>n+(state>0?1:0),0),reachable=grid.states.reduce((n,state)=>n+(state===2?1:0),0);setOverlayMetrics({gridCells:grid.shape.count,firstPaintMs:0,fullGridMs:0,stepM:fineStep,surfaceReachability:{...getOverlayMetrics().surfaceReachability,[surface.id]:{pickablePct:valid?100*reachable/valid:0,placeablePct:null}}}); }
    let live=true, jobId=Date.now()+Math.floor(Math.random()*1000), worker:Worker|null=null, started=0;
    const startWorker=(delay:number,steps:number[])=>window.setTimeout(()=>{
      if(!live)return;
      worker=new Worker(new URL('../../robotics/pickability.worker.ts',import.meta.url),{type:'module'});
      worker.onmessage=(event:MessageEvent<any>)=>{const blockStart=performance.now();try{const msg=event.data;if(!live||msg.jobId!==jobId)return;
        if(msg.type==='stageStart'){workerGrid.set(msg.stepM,{shape:msg.shape,states:new Uint8Array(msg.shape.count),reasons:new Uint8Array(msg.shape.count)});}
        if(msg.type==='chunk'){const target=workerGrid.get(msg.stepM);if(target){target.states.set(msg.states,msg.start);target.reasons.set(msg.reasons,msg.start);}}
        if(msg.type==='stageComplete'){const result=workerGrid.get(msg.stepM);if(!result)return;const reachable=result.states.reduce((n,state)=>n+(state===2?1:0),0),valid=result.states.reduce((n,state)=>n+(state>0?1:0),0),percentage=valid?100*reachable/valid:0;const report={...getOverlayMetrics().surfaceReachability,[surface.id]:{...(getOverlayMetrics().surfaceReachability[surface.id]??{pickablePct:null,placeablePct:null}),[msg.mode==='place'?'placeablePct':'pickablePct']:msg.stepM===fineStep?percentage:(getOverlayMetrics().surfaceReachability[surface.id]?.[msg.mode==='place'?'placeablePct':'pickablePct']??null)}};setOverlayMetrics({surfaceReachability:report});if(msg.mode==='place')return;const grid={shape:result.shape,states:result.states};let entry=overlayGridCache.get(key);if(!entry){entry=new Map();overlayGridCache.set(key,entry);}entry.set(msg.stepM,grid);while(overlayGridCache.size>MAX_GRID_CACHE_KEYS)overlayGridCache.delete(overlayGridCache.keys().next().value as string);textureRef.current?.dispose();const next=makeOverlayTexture(grid);textureRef.current=next;setTexture(next);const elapsed=performance.now()-started;const nextTiming=msg.stepM===coarseStep?{firstPaintMs:elapsed,fullGridMs:-1,stepM:msg.stepM}:{firstPaintMs:coarseCached?0:elapsed,fullGridMs:elapsed,stepM:msg.stepM};setOverlayMetrics({gridCells:result.shape.count,firstPaintMs:nextTiming.firstPaintMs,fullGridMs:nextTiming.fullGridMs,workerComputeMs:msg.totalMs,stepM:msg.stepM});}
        if(msg.type==='jobComplete'&&msg.mode==='pick')worker?.postMessage({type:'start',jobId,config:{...config,mode:'place',yawRad:0,sourcePosition:simulatedObject.position,sourceSurfaceId:simulatedObject.surfaceId??'table',surfaces:placementSurfaces},stepsM:[coarseStep,fineStep],chunkSize:8});
      }finally{recordMainThreadBlock(performance.now()-blockStart);}};
      const placementConfig={...config,mode:'place' as const,yawRad:0,sourcePosition:simulatedObject.position,sourceSurfaceId:simulatedObject.surfaceId??'table',surfaces:placementSurfaces};
      worker.postMessage({type:'start',jobId,config:runPlaceOnly?placementConfig:config,stepsM:runPlaceOnly?[coarseStep,fineStep]:steps,chunkSize:8});
    },delay);
    const workerGrid=new Map<number,{shape:PickabilityGridShape;states:Uint8Array;reasons:Uint8Array}>();
    const coarseCached=!!cached?.has(coarseStep); started=performance.now();
    const delay=window.setTimeout(()=>{started=performance.now();},300);
    const startTimer=startWorker(300,cached?.has(coarseStep)?[fineStep]:[coarseStep,fineStep]);
    return ()=>{live=false;window.clearTimeout(delay);window.clearTimeout(startTimer);if(worker){worker.postMessage({type:'cancel',jobId});worker.terminate();}};
  }, [bookYaw, dhTable, enabled, jointAngles, placementSurfaces, simulatedObject, tablePreset, surface]);
  useEffect(()=>()=>textureRef.current?.dispose(),[]);
  if (!enabled || !texture) return null;
  return <mesh position={[surface.center.x,surface.center.y,surface.z+.002]} rotation={[0,0,surface.yaw]} raycast={()=>null}><planeGeometry args={[surface.size.width,surface.size.depth]} /><meshBasicMaterial map={texture} transparent side={THREE.DoubleSide} depthWrite={false} /></mesh>;
};
