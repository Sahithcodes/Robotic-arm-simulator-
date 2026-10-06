import { describe, it, expect } from 'vitest';
import { computeForwardKinematics } from '../src/robotics/forwardKinematics';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY, ROBOT_MOUNT_HEIGHT } from '../src/robot/robotConfig';

describe('Forward Kinematics Engine (PUMA 6-DOF)', () => {
  it('computes zero-angle home configuration pose accurately', () => {
    const zeroAngles = [0, 0, 0, 0, 0, 0];
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, zeroAngles);

    const { position } = fk.endEffectorPose;

    expect(position.x).toBeCloseTo(0.80, 4);
    expect(position.y).toBeCloseTo(0.0, 4);
    expect(position.z).toBeCloseTo(ROBOT_MOUNT_HEIGHT + PUMA_GEOMETRY.d1 - PUMA_GEOMETRY.d6, 4);
  });

  it('verifies frame transformations chain consistency', () => {
    const angles = [Math.PI / 6, Math.PI / 4, -Math.PI / 6, 0, Math.PI / 3, 0];
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, angles);

    expect(fk.individualTransforms.length).toBe(6);
    expect(fk.cumulativeTransforms.length).toBe(6);
    expect(fk.jointPositions.length).toBe(7); // Base (0) + 6 frames

    // DH frame 0 sits on the fixed pedestal above the floor.
    expect(fk.jointPositions[0]).toEqual({ x: 0, y: 0, z: ROBOT_MOUNT_HEIGHT });

    // Frame 1 origin includes the pedestal height and the arm's d1 offset.
    expect(fk.jointPositions[1].x).toBeCloseTo(0, 4);
    expect(fk.jointPositions[1].y).toBeCloseTo(0, 4);
    expect(fk.jointPositions[1].z).toBeCloseTo(ROBOT_MOUNT_HEIGHT + PUMA_GEOMETRY.d1, 4);
  });

  it('guarantees no NaN or Infinity in outputs', () => {
    const arbitraryAngles = [1.2, -0.5, 0.8, -1.1, 0.4, 2.1];
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, arbitraryAngles);

    const { position, orientation, rotationMatrix } = fk.endEffectorPose;

    expect(Number.isFinite(position.x)).toBe(true);
    expect(Number.isFinite(position.y)).toBe(true);
    expect(Number.isFinite(position.z)).toBe(true);

    expect(Number.isFinite(orientation.roll)).toBe(true);
    expect(Number.isFinite(orientation.pitch)).toBe(true);
    expect(Number.isFinite(orientation.yaw)).toBe(true);

    rotationMatrix.forEach((val) => {
      expect(Number.isFinite(val)).toBe(true);
      expect(Number.isNaN(val)).toBe(false);
    });
  });
});
