import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../src/robot/robotConfig';
import { clampObjectToTable, createPickPlan, isValidObjectPosition } from '../src/robotics/autonomousPlanner';
import { HOME_JOINT_ANGLES, BOOK, TABLE } from '../src/robotics/task5';
import { computeForwardKinematics } from '../src/robotics/forwardKinematics';

describe('autonomous object planning', () => {
  it('constrains a dragged object to the table and outside the base exclusion zone', () => {
    const position = clampObjectToTable({ x: 3, y: -3, z: -1 });
    expect(isValidObjectPosition(position)).toBe(true);
    expect(position.z).toBeCloseTo(BOOK.initialPosition.z);
  });

  it('rejects positions outside the tabletop', () => {
    expect(isValidObjectPosition({ x: -0.7, y: -0.8, z: BOOK.initialPosition.z })).toBe(false);
  });

  it('allows continuous placements at all four usable table edges and holds object height', () => {
    const positions = [
      { x: -10, y: TABLE.center.y, z: -1 },
      { x: 10, y: TABLE.center.y, z: 2 },
      { x: TABLE.center.x, y: -10, z: 0 },
      { x: TABLE.center.x, y: 10, z: 0 },
    ].map(clampObjectToTable);
    positions.forEach((position) => {
      expect(isValidObjectPosition(position)).toBe(true);
      expect(position.z).toBeCloseTo(BOOK.initialPosition.z);
    });
    expect(new Set(positions.map(({ x, y }) => `${x.toFixed(3)},${y.toFixed(3)}`)).size).toBe(4);
  });

  it('clamps rotated book bounds using its yaw', () => {
    const position = clampObjectToTable({ x: 20, y: -20, z: 0 }, Math.PI / 4);
    expect(isValidObjectPosition(position, Math.PI / 4)).toBe(true);
    expect(isValidObjectPosition({ x: position.x + 0.01, y: position.y, z: position.z }, Math.PI / 4)).toBe(false);
  });

  it('plans pickup from the current object coordinates using the existing DH model', () => {
    const movedBook = { ...BOOK.initialPosition };
    const plan = createPickPlan(INITIAL_DH_TABLE, movedBook, HOME_JOINT_ANGLES, { x: 0.61, y: 0, yaw: 0 });
    expect(plan.reachable, plan.reason).toBe(true);
    expect(plan.waypoints[1].id).toBe('GRASP');
    const graspFk = computeForwardKinematics(INITIAL_DH_TABLE, plan.waypoints[1].jointAngles);
    expect(graspFk.tcpPose.position.x).toBeCloseTo(movedBook.x, 2);
    expect(graspFk.tcpPose.position.y).toBeCloseTo(movedBook.y, 2);
    const graspRotation = graspFk.endEffectorPose.rotationMatrix;
    expect(graspRotation[10]).toBeLessThan(-0.97);
  });

  it('plans a valid far-side tabletop placement with the tilted-grasp fallback', () => {
    const farSide = { ...BOOK.initialPosition };
    expect(isValidObjectPosition(farSide)).toBe(true);
    const plan = createPickPlan(INITIAL_DH_TABLE, farSide, HOME_JOINT_ANGLES, { x: 0.61, y: 0, yaw: 0 });
    expect(plan.reachable, plan.reason).toBe(true);
    expect(plan.waypoints.length).toBeGreaterThan(1);
  });
});




