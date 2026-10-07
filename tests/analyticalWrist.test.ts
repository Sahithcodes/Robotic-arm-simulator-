import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY } from '../src/robot/robotConfig';
import { computeForwardKinematics } from '../src/robotics/forwardKinematics';
import { analyticalIKSeeds } from '../src/robotics/autonomousPlanner';
import { solveInverseKinematics } from '../src/robotics/inverseKinematics';
import { HOME_JOINT_ANGLES } from '../src/robotics/task5';
import { degToRad } from '../src/robotics/transforms';

describe('analytic spherical wrist seeds', () => {
  it('refines 1000 random joint configurations back to the same pose', () => {
    let seed = 0x5eed;
    const random = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 0x100000000; };
    let worstPosition = 0, worstToolZ = 0, worstYaw = 0, failures = 0, noSeeds = 0, missingRefinement = 0;
    for (let sample = 0; sample < 1000; sample++) {
      const q = PUMA_GEOMETRY.jointLimits.map((limit) => degToRad(limit.min + 0.02 + random() * (limit.max - limit.min - 0.04)));
      const target = computeForwardKinematics(INITIAL_DH_TABLE, q);
      const z = { x: target.endEffectorPose.rotationMatrix[2], y: target.endEffectorPose.rotationMatrix[6], z: target.endEffectorPose.rotationMatrix[10] };
      const analytical = analyticalIKSeeds(INITIAL_DH_TABLE, target.tcpPose.position, target.endEffectorPose.rotationMatrix, q);
      if (!analytical.length) noSeeds++;
      let best = { position: Infinity, toolZ: Infinity, yaw: Infinity };
      for (const initialAngles of analytical.slice(0, 4)) {
        const ik = solveInverseKinematics(INITIAL_DH_TABLE, { position: target.endEffectorPose.position, toolZAxis: z, yaw: target.endEffectorPose.orientation.yaw }, { initialAngles, positionTolerance: 0.0005, orientationTolerance: degToRad(0.25), maxIterations: 40 });
        const fk = computeForwardKinematics(INITIAL_DH_TABLE, ik.angles);
        const fm = fk.endEffectorPose.rotationMatrix;
        const toolZ = Math.acos(Math.max(-1, Math.min(1, z.x*fm[2]+z.y*fm[6]+z.z*fm[10])));
        let yaw = Math.abs(target.endEffectorPose.orientation.yaw - fk.endEffectorPose.orientation.yaw); yaw = Math.abs(Math.atan2(Math.sin(yaw), Math.cos(yaw)));
        if (ik.residualPos < best.position) best = { position: ik.residualPos, toolZ, yaw };
      }
      if (!Number.isFinite(best.position)) missingRefinement++;
      worstPosition = Math.max(worstPosition, best.position); worstToolZ = Math.max(worstToolZ, best.toolZ); worstYaw = Math.max(worstYaw, best.yaw);
      if (best.position > .001 || best.toolZ > degToRad(.5) || best.yaw > degToRad(.5)) failures++;
    }
    console.log('ANALYTIC_IK_1000', JSON.stringify({ worstPositionMm: worstPosition*1000, worstToolZDeg: worstToolZ*180/Math.PI, worstYawDeg: worstYaw*180/Math.PI, failures, noSeeds, missingRefinement }));
    expect(failures).toBe(0);
  }, 120000);
});
