'use client';

import React, { useEffect, useState } from 'react';
import { useThrottledSimulationSelector } from '../../store/useThrottledSimulationSelector';
import { GRIPPER, TASK5_WAYPOINTS } from '../../robotics/task5';
import { computeForwardKinematics } from '../../robotics/forwardKinematics';
import { distance } from '../../robotics/task5';
import { getOverlayMetrics, subscribeOverlayMetrics } from '../../robotics/overlayMetrics';

function reachabilityMessage(reason: string): string {
  if (/Destination too close/i.test(reason)) return 'Destination too close to the robot base';
  if (/overlaps the book/i.test(reason)) return 'Destination overlaps the book';
  if (/partly off|Outside table bounds/i.test(reason)) return 'Destination footprint is partly off the table';
  if (/Destination out of reach/i.test(reason)) return 'Destination out of reach at this yaw';
  if (/Set a destination/i.test(reason)) return 'Set a destination (click the table or type X/Y)';
  if (/collision with table/i.test(reason)) return 'Path collides with table';
  if (/keep-out|base|link-vs-base/i.test(reason)) return 'Too close to the robot base';
  if (/outside workspace/i.test(reason)) return 'Too far';
  if (/joint limit|orientation unreachable/i.test(reason)) return 'Joint limit';
  if (/table|bounds/i.test(reason)) return 'Table collision';
  return reason === 'Reachable' ? 'Ready to plan' : 'Plan required';
}

export const TaskExecutionPanel: React.FC = () => {
  const [reachability,setReachability]=useState(getOverlayMetrics().surfaceReachability);
  useEffect(()=>subscribeOverlayMetrics((metrics)=>setReachability(metrics.surfaceReachability)),[]);
  const {
    taskPhase,
    pickStatus,
    simulatorMode,
    setSimulatorMode,
    setBookYawDegrees,
    isTaskPlaying,
    currentWaypointIndex,
    positionError,
    placementError,
    isGripperOpen,
    isHoldingBook,
    showRobotDebug,
    showGraspDebug,
    graspDebugLog,
    showJointLabels,
    showGraspRegion,
    showPath,
    taskWaypoints,
    playTask,
    pauseTask,
    resetTask,
    setShowRobotDebug,
    setShowGraspDebug,
    setShowJointLabels,
    setShowGraspRegion,
    setShowPath,
    simulatedObject,
    fkResult,
    dhTable,
    autonomousPlan,
    planAutonomousPick,
    executeAutonomousPick,
    dropTarget,
    setDropTarget,
    destinationPickArmed,
    setDestinationPickArmed,
    tablePreset,
    setTablePreset,
    placementSurfaces,
    surfaceEditError,
    updatePlacementSurface,
    addPlacementSurface,
    removePlacementSurface,
    allowUnreachablePlacement,
    setAllowUnreachablePlacement,
  } = useThrottledSimulationSelector((s) => ({
    taskPhase:s.taskPhase,pickStatus:s.pickStatus,simulatorMode:s.simulatorMode,setSimulatorMode:s.setSimulatorMode,setBookYawDegrees:s.setBookYawDegrees,isTaskPlaying:s.isTaskPlaying,currentWaypointIndex:s.currentWaypointIndex,positionError:s.positionError,placementError:s.placementError,isGripperOpen:s.isGripperOpen,isHoldingBook:s.isHoldingBook,showRobotDebug:s.showRobotDebug,showGraspDebug:s.showGraspDebug,graspDebugLog:s.graspDebugLog,showJointLabels:s.showJointLabels,showGraspRegion:s.showGraspRegion,showPath:s.showPath,taskWaypoints:s.taskWaypoints,playTask:s.playTask,pauseTask:s.pauseTask,resetTask:s.resetTask,setShowRobotDebug:s.setShowRobotDebug,setShowGraspDebug:s.setShowGraspDebug,setShowJointLabels:s.setShowJointLabels,setShowGraspRegion:s.setShowGraspRegion,setShowPath:s.setShowPath,simulatedObject:s.simulatedObject,fkResult:s.fkResult,dhTable:s.dhTable,autonomousPlan:s.autonomousPlan,planAutonomousPick:s.planAutonomousPick,executeAutonomousPick:s.executeAutonomousPick,dropTarget:s.dropTarget,setDropTarget:s.setDropTarget,destinationPickArmed:s.destinationPickArmed,setDestinationPickArmed:s.setDestinationPickArmed,tablePreset:s.tablePreset,setTablePreset:s.setTablePreset,placementSurfaces:s.placementSurfaces,surfaceEditError:s.surfaceEditError,updatePlacementSurface:s.updatePlacementSurface,addPlacementSurface:s.addPlacementSurface,removePlacementSurface:s.removePlacementSurface,allowUnreachablePlacement:s.allowUnreachablePlacement,setAllowUnreachablePlacement:s.setAllowUnreachablePlacement,
  }));
  const [destX, setDestX] = useState('');
  const [destY, setDestY] = useState('');
  const [destYaw, setDestYaw] = useState('0');
  const [destSurface, setDestSurface] = useState('table');
  const declareDestination = () => {
    if (!destX.trim() || !destY.trim()) return;
    const x = Number(destX), y = Number(destY);
    const yaw = Number(destYaw);
    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(yaw)) setDropTarget({ x, y, yaw, surfaceId: destSurface });
  };
  useEffect(() => {
    if (dropTarget) { setDestX(String(dropTarget.x)); setDestY(String(dropTarget.y)); setDestYaw(String(dropTarget.yaw)); setDestSurface(dropTarget.surfaceId ?? 'table'); }
  }, [dropTarget]);
  const graspWaypoint = autonomousPlan?.waypoints.find((waypoint) => waypoint.id === 'GRASP');
  const plannedFk = graspWaypoint ? computeForwardKinematics(dhTable, graspWaypoint.jointAngles) : null;
  const plannedFkError = plannedFk && graspWaypoint ? distance(plannedFk.endEffectorPose.position, graspWaypoint.position) : null;

  return (
    <section className="control-panel">
      <div className="panel-heading">
        <h2>TASK 5: PICK &amp; PLACE</h2>
        <span className="task-state">{pickStatus}</span>
      </div>
      <label className="readout-label" htmlFor="simulator-mode">MODE</label>
      <select id="simulator-mode" value={simulatorMode} onChange={(event) => setSimulatorMode(event.target.value as 'manual' | 'predefined' | 'autonomous')} className="technical-button">
        <option value="manual">Manual</option>
        <option value="predefined">Predefined Task</option>
        <option value="autonomous">Autonomous Pick</option>
      </select>
      <label className="readout-label" htmlFor="table-preset">TABLE PRESET</label>
      <select id="table-preset" value={tablePreset} disabled={isTaskPlaying} onChange={(event) => setTablePreset(event.target.value as 'compact' | 'large')} className="technical-button">
        <option value="compact">Compact (verified)</option>
        <option value="large">Large (experimental)</option>
      </select>
      <div className="readout-group">
        <div className="readout-label">PLACEMENT SURFACES</div>
        <div className="task-coordinates">Yaw 0° reachability (percent of valid surface area)</div>
        {placementSurfaces.map((surface)=>{const report=reachability[surface.id];const floorUnreachable=surface.id==='floor'&&report?.pickablePct===0;return <div className="task-coordinates" key={`reach-${surface.id}`}>{surface.name}: {report?.pickablePct===null||!report? 'Computing' : `${report.pickablePct.toFixed(1)}% pickable · ${report.placeablePct===null? 'computing placeability' : `${report.placeablePct.toFixed(1)}% placeable`}`}{floorUnreachable?' · Not reachable with this arm':''}</div>;})}
        {placementSurfaces.map((surface) => <div className="task-coordinates" key={surface.id}>
          <b>{surface.name}</b> · {surface.id===simulatedObject.surfaceId?'book':''}
          <div style={{display:'grid',gridTemplateColumns:'repeat(3,1fr)',gap:4}}>
            <label>Z <input aria-label={`${surface.name} height`} type="number" step="0.1" value={surface.z.toFixed(2)} disabled={isTaskPlaying} onChange={(e)=>{const value=Number(e.target.value);if(Number.isFinite(value))updatePlacementSurface(surface.id,{z:value});}} onBlur={(e)=>{const value=Number(e.target.value);if(Number.isFinite(value))updatePlacementSurface(surface.id,{z:Math.round(value*10)/10});}} /></label>
            <label>X <input aria-label={`${surface.name} X`} type="number" step="0.01" value={surface.center.x.toFixed(2)} disabled={isTaskPlaying} onChange={(e)=>updatePlacementSurface(surface.id,{center:{...surface.center,x:Number(e.target.value)}})} /></label>
            <label>Y <input aria-label={`${surface.name} Y`} type="number" step="0.01" value={surface.center.y.toFixed(2)} disabled={isTaskPlaying} onChange={(e)=>updatePlacementSurface(surface.id,{center:{...surface.center,y:Number(e.target.value)}})} /></label>
          </div>
          {surface.id!=='table'&&<button type="button" disabled={isTaskPlaying} onClick={()=>removePlacementSurface(surface.id)}>REMOVE</button>}
        </div>)}
        <div className="task-buttons"><button type="button" disabled={isTaskPlaying} onClick={()=>addPlacementSurface('rectangle')}>ADD RECTANGLE</button><button type="button" disabled={isTaskPlaying} onClick={()=>addPlacementSurface('annular-sector')}>ADD ANNULAR SECTOR</button></div>
        {surfaceEditError&&<div className="error-readout">{surfaceEditError}</div>}
      </div>
      <div className="readout-group">
        <div className="readout-label">DESTINATION</div>
        <label className="path-toggle">Surface <select value={destSurface} onChange={(e)=>setDestSurface(e.target.value)}>{placementSurfaces.map((surface)=><option key={surface.id} value={surface.id}>{surface.name}</option>)}</select></label>
        <label className="path-toggle">X (m) <input aria-label="Destination X" type="number" step="0.01" value={destX} onChange={(event) => setDestX(event.target.value)} /></label>
        <label className="path-toggle">Y (m) <input aria-label="Destination Y" type="number" step="0.01" value={destY} onChange={(event) => setDestY(event.target.value)} /></label>
        <label className="path-toggle">Yaw (deg) <input aria-label="Destination yaw in degrees" type="number" step="1" value={destYaw} onChange={(event) => setDestYaw(event.target.value)} /></label>
        <div className="task-buttons">
          <button type="button" disabled={isTaskPlaying} onClick={declareDestination}>SET DESTINATION</button>
          <button type="button" disabled={isTaskPlaying} onClick={() => setDestinationPickArmed(!destinationPickArmed)}>{destinationPickArmed ? 'CANCEL TABLE PICK' : 'CLICK TABLE TO SET DESTINATION'}</button>
        </div>
        {destinationPickArmed && <div className="paper-note">Click the table to place the ghost footprint. Esc cancels.</div>}
        {dropTarget && <>
          <div className="task-coordinates">Declared: X={dropTarget.x.toFixed(3)} Y={dropTarget.y.toFixed(3)} yaw={dropTarget.yaw.toFixed(1)}°</div>
          <label className="path-toggle">PLACE YAW {dropTarget.yaw.toFixed(0)}° <input aria-label="Destination yaw" type="range" min="0" max="180" step="1" value={dropTarget.yaw} disabled={isTaskPlaying} onChange={(event) => setDropTarget({ ...dropTarget, yaw: Number(event.target.value) })} /></label>
        </>}
        {!dropTarget && <div className="error-readout">Set a destination (click the table or type X/Y)</div>}
        {dropTarget && !simulatedObject.reachability.reachable && <div className="error-readout">{reachabilityMessage(simulatedObject.reachability.reason)}</div>}
        <label className="path-toggle"><input type="checkbox" checked={allowUnreachablePlacement} disabled={isTaskPlaying} onChange={(event) => setAllowUnreachablePlacement(event.target.checked)} /> Allow unreachable placement (testing)</label>
      </div>
      {simulatorMode === 'predefined' && <div className="paper-note">Paper via points &middot; Chang &amp; Park (2003)</div>}

      <div className="readout-group">
        <div className="readout-label">AUTONOMOUS PICK / BOOK</div>
        <div className="readout-label">PICKUP POSE / DESTINATION</div>
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px' }}>
          <div className="task-coordinates">Pickup · {placementSurfaces.find((surface)=>surface.id===(simulatedObject.surfaceId??'table'))?.name??'Unknown surface'}<br />X={simulatedObject.position.x.toFixed(3)} m<br />Y={simulatedObject.position.y.toFixed(3)} m<br />yaw={(simulatedObject.rotation.yaw * 180 / Math.PI).toFixed(1)}°</div>
          <div className="task-coordinates">Destination · {dropTarget ? placementSurfaces.find((surface)=>surface.id===(dropTarget.surfaceId??'table'))?.name??'Unknown surface' : 'Not declared'}<br />{dropTarget ? <>X={dropTarget.x.toFixed(3)} m<br />Y={dropTarget.y.toFixed(3)} m<br />yaw={dropTarget.yaw.toFixed(1)}°</> : 'Not declared'}</div>
        </div>
        <div className="readout-label">OBJECT WORLD</div>
        <div className="task-coordinates">
          X={simulatedObject.position.x.toFixed(3)} Y={simulatedObject.position.y.toFixed(3)} Z={simulatedObject.position.z.toFixed(3)} m
        </div>
        <div className="task-coordinates">Yaw={((simulatedObject.rotation.yaw * 180 / Math.PI) % 360).toFixed(1)}deg</div>
        <label className="path-toggle">BOOK YAW
          <input type="range" min="0" max="180" step="1" value={((simulatedObject.rotation.yaw * 180 / Math.PI) % 180 + 180) % 180} disabled={isTaskPlaying} onChange={(event) => setBookYawDegrees(Number(event.target.value))} />
        </label>
        <div className="task-coordinates">
          Target EE: {graspWaypoint ? `X=${graspWaypoint.position.x.toFixed(3)} Y=${graspWaypoint.position.y.toFixed(3)} Z=${graspWaypoint.position.z.toFixed(3)} m` : 'Not planned'}
        </div>
        {autonomousPlan?.reachable && <div className="task-coordinates">Grip candidate: {autonomousPlan.graspCandidate} · close width {autonomousPlan.gripWidth.toFixed(3)} m</div>}
        {autonomousPlan?.reachable && autonomousPlan.liftHeight !== undefined && <div className="task-coordinates">Transport lift height: {autonomousPlan.liftHeight.toFixed(3)} m</div>}
        <div className="paper-note">Wide-axis opening clearance: (0.130 − 0.012 − 0.100) / 2 = 0.009 m per side (9 mm; minimum 8 mm).</div>
        <div className="task-coordinates">
          Actual EE: X={fkResult.endEffectorPose.position.x.toFixed(3)} Y={fkResult.endEffectorPose.position.y.toFixed(3)} Z={fkResult.endEffectorPose.position.z.toFixed(3)} m
        </div>
        <div className="task-coordinates">
          IK/FK: <b>{graspWaypoint ? 'VALID' : 'NOT PLANNED'}</b>
          {plannedFkError !== null && ` · error ${plannedFkError.toFixed(4)} m`}
        </div>
        <div className="task-coordinates">
          Grasp: {simulatedObject.isAttached ? 'ATTACHED' : isTaskPlaying ? 'APPROACHING' : 'WAITING'}
        </div>
        <div className="error-readout">
          Reachability: <b style={{ color: simulatedObject.reachability.reachable ? '#91bd91' : '#d39a50' }}>
            {simulatedObject.reachability.reachable ? 'Reachable' : autonomousPlan ? 'Unreachable' : 'Not planned'}
          </b>
          <div>{reachabilityMessage(simulatedObject.reachability.reason)}</div>
        </div>
        <div className="task-buttons">
          <button type="button" disabled={isTaskPlaying || !dropTarget || !simulatedObject.reachability.reachable} onClick={planAutonomousPick}>PLAN</button>
          <button type="button" disabled={!autonomousPlan?.reachable || isTaskPlaying} onClick={executeAutonomousPick}>EXECUTE</button>
        </div>
      <div className="task-state">PICK STATE: {pickStatus}</div>
      <div className="paper-note">Drag the book on the tabletop to set a new pickup position.</div>
      <label className="path-toggle">
        <input type="checkbox" checked={showGraspDebug} onChange={(event) => setShowGraspDebug(event.target.checked)} /> SHOW GRASP DEBUG
      </label>
      {showGraspDebug && (
        <div className="readout-group" aria-label="Grasp debug log">
          <div className="readout-label">TCP / BOOK / INNER FINGER GAP (m)</div>
          <div className="task-coordinates">Book {simulatedObject.dimensions.x.toFixed(3)} × {simulatedObject.dimensions.y.toFixed(3)} × {simulatedObject.dimensions.z.toFixed(3)} · max opening {GRIPPER.openWidth.toFixed(3)} · finger thickness {GRIPPER.fingerThickness.toFixed(3)}</div>
          {graspDebugLog.length === 0 && <div className="paper-note">No autonomous stages recorded yet.</div>}
          {graspDebugLog.map((entry, index) => (
            <div className="task-coordinates" key={`${entry.stage}-${index}`}>
              {entry.stage}: TCP ({entry.tcp.x.toFixed(3)}, {entry.tcp.y.toFixed(3)}, {entry.tcp.z.toFixed(3)}) ·
              BOOK ({entry.bookCenter.x.toFixed(3)}, {entry.bookCenter.y.toFixed(3)}, {entry.bookCenter.z.toFixed(3)}) · GAP {entry.fingerGap.toFixed(3)}
            </div>
          ))}
        </div>
      )}
      </div>

      <div className="task-buttons">
        <button type="button" disabled={simulatorMode === 'autonomous' && !isTaskPlaying} onClick={simulatorMode === 'autonomous' ? pauseTask : (isTaskPlaying ? pauseTask : playTask)}>
          <span>{simulatorMode === 'autonomous' ? 'PAUSE' : isTaskPlaying ? 'PAUSE' : 'PLAY TASK'}</span>
        </button>
        <button type="button" onClick={resetTask}>
          <span>RESET</span>
        </button>
      </div>

      <div className="task-coordinates">
        {TASK5_WAYPOINTS.map((waypoint) => (
          <div key={waypoint.id}>
            <b>{waypoint.id}</b>
            X={waypoint.position.x.toFixed(2)} Y={waypoint.position.y.toFixed(2)} Z={waypoint.position.z.toFixed(2)} m
          </div>
        ))}
      </div>

      <div className="task-sequence">
        {TASK5_WAYPOINTS.slice(0, 3).map((waypoint, index) => (
          <span
            key={waypoint.id}
            className={index < currentWaypointIndex ? 'done' : index === currentWaypointIndex ? 'current' : undefined}
          >
            {waypoint.id}
          </span>
        ))}
        <span className={isHoldingBook ? 'current' : undefined}>GRASP</span>
        <span className={currentWaypointIndex === 3 ? 'current' : undefined}>P4</span>
        <span className={taskPhase === 'COMPLETE' ? 'current' : undefined}>RELEASE</span>
      </div>

      <div className="error-readout">
        <div>
          FK position error: <b>{positionError.toFixed(4)} m</b>
        </div>
        <div>
          Placement error: <b>{placementError.toFixed(4)} m</b>
        </div>
        <div>
          Gripper: <b>{isGripperOpen ? 'OPEN' : 'CLOSED'}</b> Object: <b>{isHoldingBook ? 'HELD' : 'TABLE'}</b>
        </div>
        <div>
          P3 IK residual: <b>{taskWaypoints[2].ik.positionError.toFixed(4)} m</b>
        </div>
      </div>

      <label className="path-toggle">
        <input type="checkbox" checked={showPath} onChange={(event) => setShowPath(event.target.checked)} /> SHOW PATH
      </label>
      <label className="path-toggle">
        <input
          type="checkbox"
          checked={showJointLabels}
          onChange={(event) => setShowJointLabels(event.target.checked)}
        />{' '}
        SHOW JOINT LABELS
      </label>
      <label className="path-toggle">
        <input
          type="checkbox"
          checked={showRobotDebug}
          onChange={(event) => setShowRobotDebug(event.target.checked)}
        />{' '}
        SHOW FRAMES
      </label>
      <label className="path-toggle">
        <input
          type="checkbox"
          checked={showGraspRegion}
          onChange={(event) => setShowGraspRegion(event.target.checked)}
        />{' '}
        SHOW GRASP REGION
      </label>
    </section>
  );
};
