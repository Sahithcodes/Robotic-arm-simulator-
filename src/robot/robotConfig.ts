import { DHParameter, RobotGeometryConfig } from '../types/robotics';
import { degToRad } from '../robotics/transforms';

/**
 * Geometric parameters from approved reference paper:
 * "Development of a Robotic Arm for Handicapped People: A Task-Oriented Design Approach"
 * Pyung Hun Chang and Hyung-Soon Park, 2003.
 * 
 * Dimensions:
 * d1 = 0.70 m (Base height)
 * l2 = 0.43 m (Upper arm length a2)
 * l3 = 0.37 m (Forearm length a3)
 * d4 = 0.00 m
 * d5 = 0.00 m
 * d6 = 0.10 m (Wrist flange to end-effector center)
 */
export const PUMA_GEOMETRY: RobotGeometryConfig = {
  d1: 0.70,
  l2: 0.43,
  l3: 0.37,
  d4: 0.00,
  d5: 0.00,
  d6: 0.10,
  jointLimits: [
    { min: -170, max: 170 }, // J1 (Waist)
    { min: -225, max: 45 },  // J2 (Shoulder)
    { min: -216, max: 90 },  // J3 (Elbow)
    { min: -110, max: 170 }, // J4 (Wrist Roll)
    { min: -100, max: 100 }, // J5 (Wrist Pitch)
    { min: -266, max: 266 }, // J6 (Wrist Yaw)
  ],
};

// Fixed pedestal clears the 0.78 m tabletop without changing the PUMA link geometry.
export const ROBOT_MOUNT_HEIGHT = 0.28;
export const ROBOT_BASE_POSITION = { x: 0, y: 0, z: ROBOT_MOUNT_HEIGHT };
export const JOINT_AXES: readonly [number, number, number][] = Array.from({ length: 6 }, () => [0, 0, 1] as [number, number, number]);
export const ROBOT_BASE_KEEP_OUT = { center: { x: 0, y: 0 }, radius: 0.20 };
export const ROBOT_PEDESTAL = { columnRadius: 0.095, columnTop: ROBOT_MOUNT_HEIGHT + PUMA_GEOMETRY.d1, collisionTop: ROBOT_MOUNT_HEIGHT + PUMA_GEOMETRY.d1 - 0.08 };
export const TABLE_GEOMETRY = { width: 0.60, depth: 0.40, height: 0.78, topThickness: 0.045, center: { x: 0.60, y: 0, z: 0.78 } };
export const TABLE_BOUNDS = { minX: TABLE_GEOMETRY.center.x - TABLE_GEOMETRY.width / 2, maxX: TABLE_GEOMETRY.center.x + TABLE_GEOMETRY.width / 2, minY: TABLE_GEOMETRY.center.y - TABLE_GEOMETRY.depth / 2, maxY: TABLE_GEOMETRY.center.y + TABLE_GEOMETRY.depth / 2 };
// T06 flange to finger tips. Each rendered finger is exactly this long.
export const TOOL_LENGTH = 0.075;
export const TCP_OFFSET = { x: 0, y: 0, z: TOOL_LENGTH };
export const GRIPPER_WIDTHS = { open: 0.13, closed: 0.092 };

/**
 * Standard Denavit-Hartenberg (DH) Table for 6-DOF PUMA Manipulator.
 * Convention: A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
 */
export const INITIAL_DH_TABLE: DHParameter[] = [
  {
    jointIndex: 1,
    name: 'Joint 1 (Waist)',
    a: 0.0,
    alpha: -Math.PI / 2,
    d: PUMA_GEOMETRY.d1,
    theta: 0.0,
    isVariable: true,
    thetaMin: degToRad(PUMA_GEOMETRY.jointLimits[0].min),
    thetaMax: degToRad(PUMA_GEOMETRY.jointLimits[0].max),
  },
  {
    jointIndex: 2,
    name: 'Joint 2 (Shoulder)',
    a: PUMA_GEOMETRY.l2,
    alpha: 0.0,
    d: 0.0,
    theta: 0.0,
    isVariable: true,
    thetaMin: degToRad(PUMA_GEOMETRY.jointLimits[1].min),
    thetaMax: degToRad(PUMA_GEOMETRY.jointLimits[1].max),
  },
  {
    jointIndex: 3,
    name: 'Joint 3 (Elbow)',
    a: PUMA_GEOMETRY.l3,
    alpha: -Math.PI / 2,
    d: 0.0,
    theta: 0.0,
    isVariable: true,
    thetaMin: degToRad(PUMA_GEOMETRY.jointLimits[2].min),
    thetaMax: degToRad(PUMA_GEOMETRY.jointLimits[2].max),
  },
  {
    jointIndex: 4,
    name: 'Joint 4 (Wrist Roll)',
    a: 0.0,
    alpha: Math.PI / 2,
    d: PUMA_GEOMETRY.d4,
    theta: 0.0,
    isVariable: true,
    thetaMin: degToRad(PUMA_GEOMETRY.jointLimits[3].min),
    thetaMax: degToRad(PUMA_GEOMETRY.jointLimits[3].max),
  },
  {
    jointIndex: 5,
    name: 'Joint 5 (Wrist Pitch)',
    a: 0.0,
    alpha: -Math.PI / 2,
    d: PUMA_GEOMETRY.d5,
    theta: 0.0,
    isVariable: true,
    thetaMin: degToRad(PUMA_GEOMETRY.jointLimits[4].min),
    thetaMax: degToRad(PUMA_GEOMETRY.jointLimits[4].max),
  },
  {
    jointIndex: 6,
    name: 'Joint 6 (Wrist Yaw)',
    a: 0.0,
    alpha: 0.0,
    d: PUMA_GEOMETRY.d6,
    theta: 0.0,
    isVariable: true,
    thetaMin: degToRad(PUMA_GEOMETRY.jointLimits[5].min),
    thetaMax: degToRad(PUMA_GEOMETRY.jointLimits[5].max),
  },
];
