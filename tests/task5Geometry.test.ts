import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY, ROBOT_MOUNT_HEIGHT } from '../src/robot/robotConfig';
import { computeForwardKinematics } from '../src/robotics/forwardKinematics';
import {
  buildTask5JointWaypoints,
  HOME_JOINT_ANGLES,
  TASK5_WAYPOINTS,
  distance,
  sampleTaskTrajectory,
} from '../src/robotics/task5';
import { degToRad } from '../src/robotics/transforms';
import { toThreeMatrix4 } from '../src/components/RobotViewer/RobotKinematicChain';

describe('Task 5 robot geometry transform chain', () => {
  it('keeps the home frame chain in meter units with Z as vertical', () => {
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, HOME_JOINT_ANGLES);

    expect(fk.jointPositions[0]).toEqual({ x: 0, y: 0, z: ROBOT_MOUNT_HEIGHT });
    expect(fk.jointPositions[1].z).toBeCloseTo(PUMA_GEOMETRY.d1 + ROBOT_MOUNT_HEIGHT, 6);
    expect(distance(fk.jointPositions[1], fk.jointPositions[2])).toBeCloseTo(PUMA_GEOMETRY.l2, 6);
    expect(distance(fk.jointPositions[2], fk.jointPositions[3])).toBeCloseTo(PUMA_GEOMETRY.l3, 6);
    expect(distance(fk.jointPositions[5], fk.jointPositions[6])).toBeCloseTo(PUMA_GEOMETRY.d6, 6);
  });

  it.each([
    ['J1', [degToRad(20), 0, 0, 0, 0, 0]],
    ['J2', [0, degToRad(-30), 0, 0, 0, 0]],
    ['J3', [0, 0, degToRad(30), 0, 0, 0]],
    ['J4', [0, 0, 0, degToRad(30), 0, 0]],
    ['J5', [0, 0, 0, 0, degToRad(30), 0]],
    ['J6', [0, 0, 0, 0, 0, degToRad(30)]],
  ])('%s rotation produces a finite end-effector transform', (_, angles) => {
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, angles);
    fk.endEffectorPose.rotationMatrix.forEach((value) => expect(Number.isFinite(value)).toBe(true));
  });

  it('solves P3 IK so FK reaches the paper grasp coordinate', () => {
    const p3 = buildTask5JointWaypoints(INITIAL_DH_TABLE)[2];
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, p3.jointAngles);

    expect(distance(fk.endEffectorPose.position, TASK5_WAYPOINTS[2].position)).toBeLessThan(0.025);
  });

  it('keeps visual gripper origin synchronized with FK T06 translation', () => {
    const p3 = buildTask5JointWaypoints(INITIAL_DH_TABLE)[2];
    const fk = computeForwardKinematics(INITIAL_DH_TABLE, p3.jointAngles);
    const threeMatrix = toThreeMatrix4(fk.endEffectorPose.rotationMatrix);
    const visualOrigin = new THREE.Vector3(0, 0, 0).applyMatrix4(threeMatrix);

    expect(visualOrigin.x).toBeCloseTo(fk.endEffectorPose.position.x, 6);
    expect(visualOrigin.y).toBeCloseTo(fk.endEffectorPose.position.y, 6);
    expect(visualOrigin.z).toBeCloseTo(fk.endEffectorPose.position.z, 6);
  });

  it('samples the Task 5 trajectory through FK-reachable P1-P4 endpoints', () => {
    const waypoints = buildTask5JointWaypoints(INITIAL_DH_TABLE);
    const firstSample = sampleTaskTrajectory(0, waypoints);
    const totalDuration = firstSample.totalDuration;
    const segmentDuration = totalDuration / 4;

    waypoints.forEach((waypoint, index) => {
      const sample = sampleTaskTrajectory(segmentDuration * (index + 1), waypoints);
      const fk = computeForwardKinematics(INITIAL_DH_TABLE, sample.jointAngles);

      expect(distance(fk.endEffectorPose.position, waypoint.position)).toBeLessThan(0.025);
    });
  });
});
