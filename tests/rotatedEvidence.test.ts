import { afterEach, describe, expect, it } from 'vitest';
import { useSimulationStore } from '../src/store/simulationStore';
import { INITIAL_DH_TABLE } from '../src/robot/robotConfig';
import { BOOK, HOME_JOINT_ANGLES } from '../src/robotics/task5';
import { createPickPlan } from '../src/robotics/autonomousPlanner';

afterEach(() => useSimulationStore.getState().setTablePreset('compact'));

describe('symmetric book grasp candidates', () => {
  it('accepts slight rotations and makes yaw zero and 180 use equivalent wrist solutions', () => {
    useSimulationStore.getState().setTablePreset('large');
    const position = { x: 0.77, y: -0.13, z: BOOK.initialPosition.z };
    const destination = { x: 0.60, y: 0, yaw: 0 };
    const planAt = (degrees: number) => createPickPlan(INITIAL_DH_TABLE, position, HOME_JOINT_ANGLES, destination, degrees * Math.PI / 180);
    const zero = planAt(0), slight = planAt(5), halfTurn = planAt(180);
    expect(zero.reachable, zero.reason).toBe(true);
    expect(slight.reachable, slight.reason).toBe(true);
    expect(halfTurn.reachable, halfTurn.reason).toBe(true);
    expect(zero.graspCandidate).not.toBe('');
    expect(zero.gripWidth).toBeGreaterThan(0);
    halfTurn.waypoints.find((w) => w.id === 'GRASP')!.jointAngles.forEach((angle, index) => {
      expect(angle).toBeCloseTo(zero.waypoints.find((w) => w.id === 'GRASP')!.jointAngles[index], 9);
    });
  });
});
