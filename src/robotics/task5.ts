import { DHParameter, Vector3D } from '../types/robotics';
import { INITIAL_DH_TABLE } from '../robot/robotConfig';
import { computeForwardKinematics } from './forwardKinematics';
import { degToRad } from './transforms';
import { IKResult, solveInverseKinematics } from './inverseKinematics';
import { TABLE_GEOMETRY, TABLE_PRESETS, TOOL_LENGTH, GRIPPER_WIDTHS } from '../robot/robotConfig';

export type TaskPhase =
  | 'IDLE'
  | 'MOVING TO P1'
  | 'P1 REACHED'
  | 'MOVING TO P2'
  | 'P2 REACHED'
  | 'MOVING TO P3'
  | 'P3 REACHED'
  | 'GRASPING'
  | 'HOLDING OBJECT'
  | 'MOVING TO P4'
  | 'P4 REACHED'
  | 'RELEASING'
  | 'VERIFYING PLACEMENT'
  | 'PLANNED'
  | 'UNREACHABLE'
  | 'PLANNING'
  | 'MOVING TO PRE-GRASP'
  | 'MOVING TO GRASP'
  | 'GRASPING'
  | 'ATTACHED'
  | 'LIFTING'
  | 'MOVING TO TARGET'
  | 'LOWERING TO TARGET'
  | 'RELEASING'
  | 'MOVING TO HOME'
  | 'AUTONOMOUS PRE-GRASP'
  | 'AUTONOMOUS GRASP'
  | 'AUTONOMOUS LIFT'
  | 'AUTONOMOUS DROP-APPROACH'
  | 'AUTONOMOUS DROP'
  | 'AUTONOMOUS HOME'
  | 'COMPLETE'
  | 'ERROR';

export type TaskWaypoint = {
  id: 'P1' | 'P2' | 'P3' | 'P4';
  position: Vector3D;
  orientation: { roll: number; pitch: number; yaw: number };
};

export type JointWaypoint = TaskWaypoint & {
  jointAngles: number[];
  ik: IKResult;
};

export const TASK5_WAYPOINTS: TaskWaypoint[] = [
  {
    id: 'P1',
    position: { x: 0.68, y: -0.14, z: 1.03 },
    orientation: { roll: Math.PI, pitch: 0, yaw: Math.PI / 2 },
  },
  {
    id: 'P2',
    position: { x: 0.68, y: -0.04, z: 1.03 },
    orientation: { roll: Math.PI, pitch: 0, yaw: Math.PI / 2 },
  },
  {
    id: 'P3',
    position: { x: 0.68, y: -0.04, z: 0.89 },
    orientation: { roll: Math.PI, pitch: 0, yaw: Math.PI / 2 },
  },
  {
    id: 'P4',
    position: { x: 0.72, y: 0, z: 0.89 },
    orientation: { roll: Math.PI, pitch: 0, yaw: Math.PI / 2 },
  },
];

export const HOME_JOINT_ANGLES = [0, degToRad(-35), degToRad(65), 0, degToRad(-30), 0];
export const POSITION_TOLERANCE = 0.025;
export const PLACEMENT_TOLERANCE = 0.035;
export const ORIENTATION_TOLERANCE = 0.35;

export const TABLE = TABLE_GEOMETRY;

export const PREDEFINED_BOOK_POSITION = { x: 0.68, y: -0.04, z: TABLE_PRESETS.compact.height + 0.025 / 2 };

export const BOOK = {
  size: { x: 0.1, y: 0.08, z: 0.025 },
  initialPosition: { x: TABLE_PRESETS.compact.center.x - 0.03, y: TABLE_PRESETS.compact.center.y, z: TABLE_PRESETS.compact.height + 0.025 / 2 },
};

export const GRIPPER = {
  fingerLength: TOOL_LENGTH,
  fingerThickness: 0.012,
  openWidth: GRIPPER_WIDTHS.open,
  closedWidth: GRIPPER_WIDTHS.closed,
};

const SEGMENT_SECONDS = 2.4;

export function distance(a: Vector3D, b: Vector3D): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function buildTask5JointWaypoints(dhTable: DHParameter[] = INITIAL_DH_TABLE): JointWaypoint[] {
  const seeds = [HOME_JOINT_ANGLES];

  return TASK5_WAYPOINTS.map((waypoint, index) => {
    const previous = index > 0 ? seeds[index] : HOME_JOINT_ANGLES;
    const coarseSeed = findCoarsePositionSeed(dhTable, waypoint.position, previous);
    let ik = solveInverseKinematics(dhTable, waypoint, {
      initialAngles: coarseSeed,
      positionTolerance: 0.004,
      orientationTolerance: 0.08,
    });
    if (ik.positionError > POSITION_TOLERANCE) {
      ik = solveInverseKinematics(dhTable, { position: waypoint.position }, {
        initialAngles: coarseSeed,
        positionTolerance: 0.004,
      });
    }
    seeds[index + 1] = ik.angles;
    return { ...waypoint, jointAngles: ik.angles, ik };
  });
}

function findCoarsePositionSeed(dhTable: DHParameter[], target: Vector3D, fallback: number[]): number[] {
  let best = { error: Number.POSITIVE_INFINITY, angles: fallback };
  for (let j1 = -170; j1 <= 170; j1 += 10) {
    for (let j2 = -225; j2 <= 45; j2 += 10) {
      for (let j3 = -216; j3 <= 90; j3 += 10) {
        const angles = [degToRad(j1), degToRad(j2), degToRad(j3), fallback[3] ?? 0, fallback[4] ?? 0, fallback[5] ?? 0];
        const fk = computeForwardKinematics(dhTable, angles);
        const error = distance(fk.endEffectorPose.position, target);
        if (error < best.error) best = { error, angles };
      }
    }
  }
  return best.angles;
}

export function smoothstep(t: number): number {
  const clamped = Math.max(0, Math.min(1, t));
  return clamped * clamped * (3 - 2 * clamped);
}

export function interpolateAngles(a: number[], b: number[], t: number): number[] {
  const s = smoothstep(t);
  return a.map((value, index) => value + (b[index] - value) * s);
}

export function sampleTaskTrajectory(timeSeconds: number, waypoints: JointWaypoint[]) {
  const sequence = [HOME_JOINT_ANGLES, ...waypoints.map((waypoint) => waypoint.jointAngles)];
  const segmentCount = sequence.length - 1;
  const totalDuration = segmentCount * SEGMENT_SECONDS;
  const clamped = Math.max(0, Math.min(totalDuration, timeSeconds));
  const segmentIndex = Math.min(segmentCount - 1, Math.floor(clamped / SEGMENT_SECONDS));
  const localT = (clamped - segmentIndex * SEGMENT_SECONDS) / SEGMENT_SECONDS;

  return {
    jointAngles: interpolateAngles(sequence[segmentIndex], sequence[segmentIndex + 1], localT),
    segmentIndex,
    localT,
    totalDuration,
  };
}
