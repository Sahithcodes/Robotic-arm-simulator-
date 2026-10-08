import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { computeForwardKinematics } from '../forwardKinematics';
import { createPickPlan } from '../autonomousPlanner';
import { getFingerBoxCorners } from '../gripperGeometry';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';
import { defaultTableSurface, surfaceBookZ } from '../placementSurfaces';

describe('resting-height grasp calibration', () => {
  it('plans from the true resting height and keeps both pads clear while overlapping the book sides', () => {
    const surface = defaultTableSurface();
    expect(surface.z).toBe(0.78);
    expect(surfaceBookZ(surface)).toBeCloseTo(0.7925, 8);
    expect(BOOK.initialPosition.z).toBeCloseTo(surfaceBookZ(surface), 8);

    const plan = createPickPlan(INITIAL_DH_TABLE, BOOK.initialPosition, HOME_JOINT_ANGLES, { x: 0.61, y: 0, yaw: 0 });
    expect(plan.reachable, plan.reason).toBe(true);
    const grasp = plan.waypoints.find((waypoint) => waypoint.id === 'GRASP')!;
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, grasp.jointAngles);
    const boxes = getFingerBoxCorners(fk, plan.gripWidth);
    const lowestFingerZ = Math.min(...boxes.flatMap((box) => box.corners.map((corner) => corner.z)));
    const clearanceMm = (lowestFingerZ - surface.z) * 1000;
    const bookTop = surfaceBookZ(surface) + BOOK.size.z / 2;
    const overlapMm = (bookTop - lowestFingerZ) * 1000;
    console.log(`HEIGHT PROOF surface=${surface.z.toFixed(4)} m book-center=${BOOK.initialPosition.z.toFixed(4)} m TCP=${fk.tcpPose.position.z.toFixed(4)} m finger-clearance=${clearanceMm.toFixed(2)} mm side-overlap=${overlapMm.toFixed(2)} mm`);
    expect(clearanceMm).toBeGreaterThanOrEqual(3 - 0.002);
    expect(overlapMm).toBeGreaterThanOrEqual(10);
    expect(Math.abs(TABLE.height - surface.z)).toBeLessThan(1e-9);
  });
});
