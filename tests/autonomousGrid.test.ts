import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY } from '../src/robot/robotConfig';
import { createPickPlan, isValidObjectPosition } from '../src/robotics/autonomousPlanner';
import { computeForwardKinematics } from '../src/robotics/forwardKinematics';
import { BOOK, distance, HOME_JOINT_ANGLES, TABLE } from '../src/robotics/task5';

describe('pickup workspace grid diagnostic', () => {
  it('reports IK, FK, joint-limit, and complete-path outcomes over the table', () => {
    const xs = [0.30, 0.48, 0.65];
    const ys = [0.24, TABLE.center.y, -0.4];
    const positions = [...xs.flatMap((x) => ys.map((y) => ({ x, y }))), { x: TABLE.center.x, y: TABLE.center.y }, { x: 0.54, y: 0 }];
    const outcomes = positions.map(({ x, y }) => {
      const position = { x, y, z: BOOK.initialPosition.z };
      const plan = createPickPlan(INITIAL_DH_TABLE, position, HOME_JOINT_ANGLES);
      const grasp = plan.waypoints.find((waypoint) => waypoint.id === 'GRASP');
      const fk = grasp && computeForwardKinematics(INITIAL_DH_TABLE, grasp.jointAngles);
      const fkError = fk && distance(fk.endEffectorPose.position, grasp.position);
      const limitsOk = grasp?.jointAngles.every((angle, index) => {
        const limit = PUMA_GEOMETRY.jointLimits[index];
        return angle >= limit.min * Math.PI / 180 && angle <= limit.max * Math.PI / 180;
      });
      return { x, y, tableValid: isValidObjectPosition(position), reachable: plan.reachable, reason: plan.reason, jointAngles: grasp?.jointAngles, fkError, limitsOk };
    });
    console.info('Pickup grid diagnostics:', outcomes.map(({ x, y, tableValid, reachable, reason }) => ({ x, y, tableValid, reachable, reason })));
    for (const outcome of outcomes.filter((result) => result.reachable)) {
      expect(outcome.fkError).toBeLessThan(0.008);
      expect(outcome.limitsOk).toBe(true);
    }
    expect(outcomes).toHaveLength(11);
  });
});
