/**
 * Core Robotics Data Types & Interfaces
 * 6-DOF PUMA Manipulator & Task-Oriented Design System
 */

/** 4x4 Homogeneous Transformation Matrix in row-major format */
export type Matrix4x4 = [
  number, number, number, number,
  number, number, number, number,
  number, number, number, number,
  number, number, number, number
];

/** 3D Vector representing Cartesian position (meters) or direction */
export type Vector3D = {
  x: number;
  y: number;
  z: number;
};

export type SimulatedObject = {
  id: string;
  type: 'book';
  position: Vector3D;
  rotation: EulerAngles;
  dimensions: Vector3D;
  graspState: 'on-table' | 'held';
  state: 'onTable' | 'grasped' | 'placed';
  isSelected: boolean;
  isBeingDragged: boolean;
  isGrasped: boolean;
  isAttached: boolean;
  targetPosition: Vector3D;
  reachability: { reachable: boolean; reason: string };
};

/** Orientation angles in Roll-Pitch-Yaw (rad or deg) */
export type EulerAngles = {
  roll: number;  // Rotation about X axis
  pitch: number; // Rotation about Y axis
  yaw: number;   // Rotation about Z axis
};

/** 6D Cartesian Pose (Position + Orientation) */
export type Pose = {
  position: Vector3D;
  orientation: EulerAngles;
  rotationMatrix: Matrix4x4;
};

/** Standard Denavit-Hartenberg (DH) parameters for a single link/joint */
export type DHParameter = {
  jointIndex: number;
  a: number;          // Link length (meters) - offset along X_i axis
  alpha: number;      // Link twist (radians) - angle about X_i axis
  d: number;          // Link offset (meters) - distance along Z_{i-1} axis
  theta: number;      // Joint angle (radians) - angle about Z_{i-1} axis
  isVariable: boolean; // True for revolute joints where theta is dynamic
  thetaMin: number;   // Joint min limit (radians)
  thetaMax: number;   // Joint max limit (radians)
  name: string;
};

/** Joint state metadata */
export type JointState = {
  index: number;
  name: string;
  angle: number;        // radians
  velocity: number;     // rad/s
  acceleration: number; // rad/s^2
  minLimit: number;     // radians
  maxLimit: number;     // radians
};

/** Forward Kinematics calculation output */
export type FKResult = {
  /** Individual link transformation matrices A_i = T_{i-1}^i */
  individualTransforms: Matrix4x4[];
  /** Cumulative transformation matrices T_0^i for frame i in base frame 0 */
  cumulativeTransforms: Matrix4x4[];
  /** End effector cartesian pose in base coordinate frame */
  endEffectorPose: Pose;
  tcpPose: Pose;
  /** Joint origin positions in base frame (for visual kinematic chain line rendering) */
  jointPositions: Vector3D[];
};

/** PUMA Robot Geometry and Configuration specification */
export type RobotGeometryConfig = {
  d1: number; // Base height (m)
  l2: number; // Upper arm length a2 (m)
  l3: number; // Forearm length a3 (m)
  d4: number; // Wrist offset (m)
  d5: number; // Wrist offset (m)
  d6: number; // Flange to End-Effector offset (m)
  jointLimits: { min: number; max: number }[]; // In degrees
};

/** UI Tab Navigation option */
export type AppTab = 
  | 'robot'
  | 'task'
  | 'kinematics'
  | 'trajectory'
  | 'execution'
  | 'dynamics'
  | 'results';
