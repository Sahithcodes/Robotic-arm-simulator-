import { DHParameter, FKResult, Matrix4x4, Vector3D } from '../types/robotics';
import { computeDHMatrix, updateDHParametersWithJointAngles } from './dh';
import { multiply4x4, extractTranslation, matrixToRPY, identity4 } from './transforms';
import { ROBOT_BASE_POSITION, TCP_OFFSET } from '../robot/robotConfig';

/**
 * Computes Forward Kinematics for a serial manipulator defined by DH parameters.
 * 
 * @param dhTable List of DH parameters for links 1..N
 * @param jointAngles Array of current joint angles (radians)
 * @returns FKResult object containing transformation matrices, joint origins, and EE pose
 */
export function computeForwardKinematics(
  dhTable: DHParameter[],
  jointAngles: number[]
): FKResult {
  const updatedDH = updateDHParametersWithJointAngles(dhTable, jointAngles);
  const n = updatedDH.length;

  const individualTransforms: Matrix4x4[] = [];
  const cumulativeTransforms: Matrix4x4[] = [];
  const jointPositions: Vector3D[] = [];

  // The DH arm is mounted above the floor on a fixed pedestal.
  jointPositions.push({ ...ROBOT_BASE_POSITION });

  let T_accum = identity4();
  T_accum[3] = ROBOT_BASE_POSITION.x;
  T_accum[7] = ROBOT_BASE_POSITION.y;
  T_accum[11] = ROBOT_BASE_POSITION.z;

  for (let i = 0; i < n; i++) {
    const Ai = computeDHMatrix(updatedDH[i]);
    individualTransforms.push(Ai);

    T_accum = multiply4x4(T_accum, Ai);
    cumulativeTransforms.push(T_accum);

    // Position of origin of frame i+1 in base frame
    jointPositions.push(extractTranslation(T_accum));
  }

  const T_0_EE = cumulativeTransforms[n - 1] || identity4();
  const position = extractTranslation(T_0_EE);
  const orientation = matrixToRPY(T_0_EE);
  const tcpPosition = {
    x: T_0_EE[3] + T_0_EE[0] * TCP_OFFSET.x + T_0_EE[1] * TCP_OFFSET.y + T_0_EE[2] * TCP_OFFSET.z,
    y: T_0_EE[7] + T_0_EE[4] * TCP_OFFSET.x + T_0_EE[5] * TCP_OFFSET.y + T_0_EE[6] * TCP_OFFSET.z,
    z: T_0_EE[11] + T_0_EE[8] * TCP_OFFSET.x + T_0_EE[9] * TCP_OFFSET.y + T_0_EE[10] * TCP_OFFSET.z,
  };

  return {
    individualTransforms,
    cumulativeTransforms,
    endEffectorPose: {
      position,
      orientation,
      rotationMatrix: T_0_EE,
    },
    tcpPose: { position: tcpPosition, orientation, rotationMatrix: T_0_EE },
    jointPositions,
  };
}
