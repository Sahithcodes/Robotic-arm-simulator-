import { afterEach, describe, expect, it } from 'vitest';
import { useSimulationStore } from '../src/store/simulationStore';
import { TABLE, BOOK, TASK5_WAYPOINTS } from '../src/robotics/task5';
import { TABLE_PRESETS, INITIAL_DH_TABLE } from '../src/robot/robotConfig';
import { findDefaultPickPosition, validatePickApproach } from '../src/robotics/autonomousPlanner';
import { HOME_JOINT_ANGLES } from '../src/robotics/task5';
import { rayToTableXY } from '../src/robotics/tableInteraction';
import { heldBookCollision } from '../src/robotics/heldObjectCollision';
import { createPickPlan } from '../src/robotics/autonomousPlanner';
import { computeForwardKinematics } from '../src/robotics/forwardKinematics';
import { fingerBoxesIntersectTable, getFingerBoxCorners } from '../src/robotics/gripperGeometry';
import { GRIPPER } from '../src/robotics/task5';

afterEach(() => { useSimulationStore.getState().setTablePreset('compact'); useSimulationStore.getState().resetTask(); });

function runToStop(maxFrames = 1000) {
  for (let i=0;i<maxFrames&&useSimulationStore.getState().isTaskPlaying;i++) useSimulationStore.getState().tickTask(.05);
  return useSimulationStore.getState();
}

describe('table presets and task regressions', () => {
  it('keeps pickup reachability available while blocking a plan until destination declaration', () => {
    const store=useSimulationStore.getState();store.setTablePreset('compact');store.resetTask();
    const before=[...store.jointAngles];
    store.checkObjectReachability();
    expect(useSimulationStore.getState().simulatedObject.reachability.reachable).toBe(true);
    useSimulationStore.getState().planAutonomousPick();
    const after=useSimulationStore.getState();
    expect(after.pickStatus).toBe('Set a destination (click the table or type X/Y)');
    expect(after.autonomousPlan).toBeNull();
    expect(after.jointAngles).toEqual(before);
  });

  it('resets plan and state when destination position or yaw changes', () => {
    const store=useSimulationStore.getState();store.setTablePreset('compact');store.resetTask();
    store.setDropTarget({x:.61,y:0,yaw:0});
    store.planAutonomousPick();
    expect(useSimulationStore.getState().autonomousPlan?.reachable,useSimulationStore.getState().autonomousPlan?.reason).toBe(true);
    useSimulationStore.getState().setDropTarget({x:.62,y:0,yaw:0});
    expect(useSimulationStore.getState().autonomousPlan).toBeNull();
    expect(useSimulationStore.getState().taskPhase).toBe('IDLE');
    useSimulationStore.getState().planAutonomousPick();
    useSimulationStore.getState().setDropTarget({x:.62,y:0,yaw:5});
    expect(useSimulationStore.getState().autonomousPlan).toBeNull();
    expect(useSimulationStore.getState().pickStatus).toBe('IDLE');
  });

  it('computes a Compact reset cell accepted by the yaw-zero pickup planner', () => {
    const store=useSimulationStore.getState();
    expect(store.tablePreset).toBe('compact');
    expect(TABLE.width).toBe(TABLE_PRESETS.compact.width);
    const computed=findDefaultPickPosition(store.dhTable,store.jointAngles,0);
    store.resetTask();
    expect(useSimulationStore.getState().simulatedObject.position).toEqual(computed);
    const pick=validatePickApproach(INITIAL_DH_TABLE,computed,HOME_JOINT_ANGLES,Math.PI/2);
    expect(pick.reachable).toBe(true);
  });

  it('reuses the exact measured Large HOME default returned by the full picker search', () => {
    const store=useSimulationStore.getState();store.setTablePreset('large');
    const cached=useSimulationStore.getState().simulatedObject.position;
    const searched=findDefaultPickPosition(useSimulationStore.getState().dhTable,HOME_JOINT_ANGLES,0);
    expect(cached).toEqual(searched);
  });

  it('completes and places the Predefined Task on Compact and Large presets without a destination', () => {
    for(const preset of ['compact','large'] as const){
      const store=useSimulationStore.getState();store.setTablePreset(preset);store.resetTask();
      expect(useSimulationStore.getState().dropTarget).toBeNull();
      store.playTask();
      const state=runToStop();
      console.log(`PREDEFINED RESULT preset=${preset} phase=${state.taskPhase} object=${JSON.stringify(state.simulatedObject.position)} destination=none`);
      expect(state.taskPhase,`preset ${preset}: ${state.taskPhase}`).toBe('COMPLETE');
      expect(state.simulatedObject.state).toBe('placed');
      expect(state.simulatedObject.isAttached).toBe(false);
      expect(TASK5_WAYPOINTS.find(w=>w.id==='P3')!.position.x).toBeCloseTo(.68);
      expect(state.simulatedObject.position.x).toBeCloseTo(TASK5_WAYPOINTS[3].position.x,2);
    }
  },30000);

  it('completes Autonomous Pick from the computed Compact default pose with a valid declared destination', () => {
    const state=useSimulationStore.getState();
    state.setTablePreset('compact');state.resetTask();state.setSimulatorMode('autonomous');
    const start=useSimulationStore.getState().simulatedObject.position;
    state.setDropTarget({x:.61,y:0,yaw:0});
    useSimulationStore.getState().checkObjectReachability();
    const planned=useSimulationStore.getState();
    expect(planned.autonomousPlan?.reachable,planned.autonomousPlan?.reason).toBe(true);
    const placePlan=createPickPlan(planned.dhTable,start,planned.jointAngles,{x:.61,y:0,yaw:0},0);
    const release=placePlan.waypoints.find(w=>w.id==='PLACE')!;
    expect(release.tiltDeg).toBe(placePlan.waypoints.find(w=>w.id==='GRASP')!.tiltDeg);
    expect(fingerBoxesIntersectTable(getFingerBoxCorners(computeForwardKinematics(planned.dhTable,release.jointAngles),GRIPPER.openWidth))).toBe(false);
    planned.executeAutonomousPick();
    const done=runToStop();
    expect(done.pickStatus).toBe('DONE');
    expect(done.simulatedObject.state).toBe('placed');
    expect(done.simulatedObject.isAttached).toBe(false);
    expect(done.simulatedObject.position.x).toBeCloseTo(.61,3);
    expect(done.simulatedObject.position.y).toBeCloseTo(0,3);
    expect(start.x).toBeCloseTo(findDefaultPickPosition(done.dhTable,HOME_JOINT_ANGLES,0).x,3);
  },30000);
});

describe('held book collision checks',()=>{
  it('flags a box through the table and leaves a lifted clear box collision-free',()=>{
    expect(heldBookCollision({x:.71,y:-.04,z:.78},{roll:0,pitch:0,yaw:0})?.kind).toBe('held-book/table');
    expect(heldBookCollision({x:.75,y:-.04,z:1.1},{roll:0,pitch:0,yaw:0})).toBeNull();
  });
});

describe('table pointer mapping', () => {
  it('intersects a pointer ray with the tabletop in world X/Y', () => {
    expect(rayToTableXY({origin:{x:1,y:2,z:2},direction:{x:-.5,y:-1,z:-2}},.8)).toEqual({x:.7,y:1.4});
    expect(rayToTableXY({origin:{x:0,y:0,z:1},direction:{x:1,y:0,z:0}},.8)).toBeNull();
    expect(rayToTableXY({origin:{x:0,y:0,z:.5},direction:{x:0,y:0,z:-1}},.8)).toBeNull();
  });
});
