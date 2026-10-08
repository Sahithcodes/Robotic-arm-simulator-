import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { planPickAndPlace } from '../autonomousPlanner';
import { defaultTableSurface } from '../placementSurfaces';
import { BOOK, HOME_JOINT_ANGLES } from '../task5';
import { findDemoWorkspace } from '../demoWorkspace';
import { useSimulationStore } from '../../store/simulationStore';

describe('verified professor demo workspace', () => {
  it('measures complete-plan-valid target cells from the default book pose', () => {
    const source = { ...BOOK.initialPosition };
    const surface = defaultTableSurface();
    const workspace=findDemoWorkspace(INITIAL_DH_TABLE,source,0,surface,{minX:.57,maxX:.77,minY:-.14,maxY:.06,stepM:.01,safetyMarginM:.02});
    const valid=workspace.validCells;
    const minX=Math.min(...valid.map((p)=>p.x)),maxX=Math.max(...valid.map((p)=>p.x)),minY=Math.min(...valid.map((p)=>p.y)),maxY=Math.max(...valid.map((p)=>p.y));
    console.log(`DEMO WORKSPACE sampledBounds=X[${minX.toFixed(2)},${maxX.toFixed(2)}] Y[${minY.toFixed(2)},${maxY.toFixed(2)}] plannerValidPositions=${valid.length} sampledCellArea=${(valid.length*.0001).toFixed(4)}m2 (${(100*valid.length/441).toFixed(1)}% of sweep); largestRectangle=${workspace.largestRectangleCells} cells bounds=${JSON.stringify(workspace.bounds)}; 20mm inset validCells=${workspace.safeCells.length}`);
    console.log(`DEMO WORKSPACE SWEEP source=(${source.x.toFixed(3)},${source.y.toFixed(3)}) grid=21x21 step=0.010m valid=${valid.length}/441 largestRect=${workspace.largestRectangleCells} raw=${JSON.stringify(workspace.bounds)} safetyMargin=0.020m shrunkPoints=${workspace.safeCells.length}`);
    const anchors=[{x:.61,y:0},{x:.62,y:0},{x:.63,y:0}];
    for(const point of anchors) expect(planPickAndPlace(INITIAL_DH_TABLE,source,HOME_JOINT_ANGLES,{...point,yaw:0,surfaceId:'table'},0,true,[surface],'table').reachable,`target=${JSON.stringify(point)}`).toBe(true);
    console.log(`DEMO TARGET ANCHORS fullPlan=PASS positions=${JSON.stringify(anchors)} count=${anchors.length}; no rectangular area remains after 2 cm inset`);
    expect(valid.length).toBeGreaterThan(0); expect(workspace.safeCells.length).toBe(0);
  }, 120000);

  it('defers full destination planning until the marker is released',()=>{
    const store=useSimulationStore;
    store.getState().setTablePreset('compact');store.getState().resetTask();
    store.getState().setDropTarget({x:.61,y:0,yaw:0,surfaceId:'table'});
    expect(store.getState().simulatedObject.reachability.reachable).toBe(true);
    store.getState().setDropTarget({x:.62,y:.01,yaw:0,surfaceId:'table'},false,true);
    expect(store.getState().autonomousPlan).toBeNull();
    expect(store.getState().simulatedObject.reachability.reason).toBe('Release marker to validate destination');
    store.getState().setDropTarget(store.getState().dropTarget,true);
    expect(store.getState().simulatedObject.reachability.reachable).toBe(true);
    console.log('DEMO TARGET DRAG preview=planner-free release=complete-plan-validated');
  });
});
