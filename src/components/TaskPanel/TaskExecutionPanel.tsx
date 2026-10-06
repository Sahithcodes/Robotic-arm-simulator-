'use client';

import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';
import { GRIPPER, TASK5_WAYPOINTS } from '../../robotics/task5';
import { computeForwardKinematics } from '../../robotics/forwardKinematics';
import { distance } from '../../robotics/task5';

export const TaskExecutionPanel: React.FC = () => {
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
  } = useSimulationStore();
  const graspWaypoint = autonomousPlan?.waypoints.find((waypoint) => waypoint.id === 'GRASP');
  const plannedFk = graspWaypoint ? computeForwardKinematics(dhTable, graspWaypoint.jointAngles) : null;
  const plannedFkError = plannedFk && graspWaypoint ? distance(plannedFk.endEffectorPose.position, graspWaypoint.position) : null;

  return (
    <section className="control-panel">
      <div className="panel-heading">
        <h2>TASK 5: PICK &amp; PLACE</h2>
        <span className="task-state">{taskPhase}</span>
      </div>
      <label className="readout-label" htmlFor="simulator-mode">MODE</label>
      <select id="simulator-mode" value={simulatorMode} onChange={(event) => setSimulatorMode(event.target.value as 'manual' | 'predefined' | 'autonomous')} className="technical-button">
        <option value="manual">Manual</option>
        <option value="predefined">Predefined Task</option>
        <option value="autonomous">Autonomous Pick</option>
      </select>
      <div className="paper-note">Paper via points &middot; Chang &amp; Park (2003)</div>

      <div className="readout-group">
        <div className="readout-label">AUTONOMOUS PICK / BOOK</div>
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
            {simulatedObject.reachability.reachable ? 'REACHABLE' : autonomousPlan ? 'OBJECT OUT OF REACH' : 'NOT PLANNED'}
          </b>
          <div>{simulatedObject.reachability.reason}</div>
        </div>
        <div className="task-buttons">
          <button type="button" disabled={isTaskPlaying} onClick={planAutonomousPick}>PLAN</button>
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
