import { describe, expect, it } from 'vitest';
import { useSimulationStore } from '../../store/simulationStore';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';
import { computeForwardKinematics } from '../forwardKinematics';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';

describe('professor demonstration', () => {
  it('plans, attaches, transports, places, releases and retreats with the controller', () => {
    const store = useSimulationStore;
    store.getState().setTablePreset('compact');
    store.getState().resetTask();
    store.setState((state) => ({
      jointAngles: [...HOME_JOINT_ANGLES], fkResult: computeForwardKinematics(INITIAL_DH_TABLE, HOME_JOINT_ANGLES),
      simulatedObject: { ...state.simulatedObject, position: { ...BOOK.initialPosition }, surfaceId: 'table', rotation: { roll: 0, pitch: 0, yaw: 0 }, isAttached: false, isGrasped: false, state: 'onTable', graspState: 'on-table' },
    }));
    store.getState().setDropTarget({ x: 0.61, y: 0, yaw: 0, surfaceId: 'table' });
    store.getState().planAutonomousPick();
    const plan = store.getState().autonomousPlan;
    expect(plan?.reachable, plan?.reason).toBe(true);
    console.log('==================================\nPROFESSOR DEMO\n==================================');
    console.log(`Book: (${BOOK.initialPosition.x.toFixed(3)}, ${BOOK.initialPosition.y.toFixed(3)}, ${BOOK.initialPosition.z.toFixed(4)}) yaw=0°`);
    console.log('Target: (0.610, 0.000, 0.7925) yaw=0°');
    console.log(`IK: PASS · Collision: PASS · Grasp: PASS · candidate=${plan?.graspCandidate}`);
    store.getState().executeAutonomousPick();
    let sawAttach = false, sawTransport = false, lowest = Infinity;
    const stages: string[]=[];
    let attachedStart: { x: number; y: number; z: number } | null = null;
    for (let frame = 0; frame < 900 && store.getState().isTaskPlaying; frame++) {
      store.getState().tickTask(0.05);
      const current = store.getState();
      if(stages.at(-1)!==current.pickStatus)stages.push(current.pickStatus);
      lowest = Math.min(lowest, current.simulatedObject.position.z - BOOK.size.z / 2);
      if (current.simulatedObject.isAttached && !sawAttach) { sawAttach = true; attachedStart = { ...current.simulatedObject.position }; }
      if (sawAttach && current.simulatedObject.isAttached && attachedStart && Math.hypot(current.simulatedObject.position.x-attachedStart.x,current.simulatedObject.position.y-attachedStart.y,current.simulatedObject.position.z-attachedStart.z)>.02) sawTransport = true;
    }
    const end = store.getState(), object = end.simulatedObject;
    const posErr = Math.hypot(object.position.x-.61,object.position.y,object.position.z-BOOK.initialPosition.z);
    const yawErr = Math.abs(Math.atan2(Math.sin(object.rotation.yaw),Math.cos(object.rotation.yaw)));
    expect(sawAttach).toBe(true); expect(sawTransport).toBe(true);
    expect(end.pickStatus).toBe('DONE'); expect(object.state).toBe('placed'); expect(object.isAttached).toBe(false);
    expect(posErr).toBeLessThanOrEqual(.005); expect(yawErr).toBeLessThanOrEqual(2*Math.PI/180);
    expect(lowest).toBeGreaterThanOrEqual(TABLE.height-1e-6);
    console.log(`Execution stages: ${stages.join(' → ')}`);
    expect(stages).toEqual(expect.arrayContaining(['PREGRASP','OPEN','DESCEND','VERIFY','CLOSE','ATTACH','LIFT','TRANSPORT','LOWER','RELEASE','RETREAT','DONE']));
    console.log('Attachment: PASS · Transport: PASS · Placement: PASS');
    console.log(`Position error: ${(posErr*1000).toFixed(3)} mm\nYaw error: ${(yawErr*180/Math.PI).toFixed(3)} deg\nFinal state: ${end.pickStatus}\n==================================\nDEMO PASS\n==================================`);
  }, 30000);

  it('keeps the declared yaw for the rotated-object demo', () => {
    const store=useSimulationStore;
    store.getState().setTablePreset('compact');store.getState().resetTask();
    store.setState((state)=>({jointAngles:[...HOME_JOINT_ANGLES],fkResult:computeForwardKinematics(INITIAL_DH_TABLE,HOME_JOINT_ANGLES),simulatedObject:{...state.simulatedObject,position:{...BOOK.initialPosition},surfaceId:'table',rotation:{roll:0,pitch:0,yaw:45*Math.PI/180},isAttached:false,isGrasped:false,state:'onTable',graspState:'on-table'}}));
    store.getState().setDropTarget({x:.61,y:-.02,yaw:90,surfaceId:'table'});store.getState().planAutonomousPick();
    expect(store.getState().autonomousPlan?.reachable,store.getState().autonomousPlan?.reason).toBe(true);
    store.getState().executeAutonomousPick();let attached=false,moved=false,first:{x:number;y:number;z:number}|null=null;
    for(let frame=0;frame<900&&store.getState().isTaskPlaying;frame++){
      store.getState().tickTask(.05);const object=store.getState().simulatedObject;
      if(object.isAttached&&!attached){attached=true;first={...object.position};}
      if(attached&&first&&object.isAttached&&Math.hypot(object.position.x-first.x,object.position.y-first.y,object.position.z-first.z)>.02)moved=true;
    }
    const end=store.getState(),error=Math.abs(Math.atan2(Math.sin(end.simulatedObject.rotation.yaw-Math.PI/2),Math.cos(end.simulatedObject.rotation.yaw-Math.PI/2)))*180/Math.PI;
    expect(attached).toBe(true);expect(moved).toBe(true);expect(end.pickStatus).toBe('DONE');expect(error).toBeLessThanOrEqual(2);
    console.log(`ROTATED PROFESSOR DEMO bookYaw=45° targetYaw=90° attached=${attached} transported=${moved} finalYaw=${(end.simulatedObject.rotation.yaw*180/Math.PI).toFixed(2)}° error=${error.toFixed(3)}° state=${end.pickStatus}`);
  },30000);
});
