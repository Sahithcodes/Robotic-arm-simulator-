import { describe, expect, it } from 'vitest';
import { useSimulationStore } from '../../store/simulationStore';
import { computeForwardKinematics } from '../forwardKinematics';
import { BOOK } from '../task5';

const wrap = (angle:number) => Math.atan2(Math.sin(angle),Math.cos(angle));

describe('book yaw is preserved through grasp and release', () => {
  it('prints and verifies one 105-degree pickup to 45-degree placement', () => {
    const store=useSimulationStore.getState();
    store.setTablePreset('compact');
    store.resetTask();
    for(const surface of useSimulationStore.getState().placementSurfaces)if(surface.id!=='table')useSimulationStore.getState().removePlacementSurface(surface.id);
    useSimulationStore.getState().setBookPosition({x:.604,y:-.04,z:BOOK.initialPosition.z});
    store.setBookYawDegrees(105);
    store.setDropTarget({x:.752,y:-.02,yaw:45});
    useSimulationStore.getState().planAutonomousPick();
    const planned=useSimulationStore.getState();
    const plan=planned.autonomousPlan!;
    expect(plan?.reachable,plan?.reason).toBe(true);
    const grasp=plan.waypoints.find((waypoint)=>waypoint.id==='GRASP')!;
    const place=plan.waypoints.find((waypoint)=>waypoint.id==='PLACE')!;
    const graspFk=computeForwardKinematics(planned.dhTable,grasp.jointAngles);
    const placeFk=computeForwardKinematics(planned.dhTable,place.jointAngles);
    const bookYaw=105*Math.PI/180,declaredYaw=45*Math.PI/180;
    const toolYawAtGrasp=graspFk.endEffectorPose.orientation.yaw;
    const relativeYaw=wrap(bookYaw-toolYawAtGrasp);
    const toolYawAtPlace=placeFk.endEffectorPose.orientation.yaw;
    expect(Math.abs(wrap(toolYawAtPlace-(declaredYaw-relativeYaw)))).toBeLessThan(2*Math.PI/180);
    useSimulationStore.getState().executeAutonomousPick();
    for(let frame=0;frame<900&&useSimulationStore.getState().isTaskPlaying;frame++)useSimulationStore.getState().tickTask(.05);
    const end=useSimulationStore.getState();
    const releasedYaw=end.simulatedObject.rotation.yaw;
    const yawError=Math.abs(wrap(releasedYaw-declaredYaw));
    console.log(`YAW TRACE book=${(bookYaw*180/Math.PI).toFixed(1)}deg candidate=${plan.graspCandidate} tool@grasp=${(toolYawAtGrasp*180/Math.PI).toFixed(1)}deg J6=${(grasp.jointAngles[5]*180/Math.PI).toFixed(1)}deg relative=${(relativeYaw*180/Math.PI).toFixed(1)}deg declared=${(declaredYaw*180/Math.PI).toFixed(1)}deg tool@place=${(toolYawAtPlace*180/Math.PI).toFixed(1)}deg released=${(releasedYaw*180/Math.PI).toFixed(1)}deg error=${(yawError*180/Math.PI).toFixed(2)}deg phase=${end.pickStatus}`);
    expect(end.pickStatus).toBe('DONE');
    expect(yawError).toBeLessThan(2*Math.PI/180);
    expect(end.simulatedObject.position.z).toBeCloseTo(BOOK.initialPosition.z,3);
  },30000);

});
