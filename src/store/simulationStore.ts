import { create } from 'zustand';
import { AppTab, DHParameter, EulerAngles, FKResult, Vector3D } from '../types/robotics';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY, TCP_OFFSET } from '../robot/robotConfig';
import { computeForwardKinematics } from '../robotics/forwardKinematics';
import { transformPoint } from '../robotics/inverseKinematics';
import { degToRad } from '../robotics/transforms';
import { clampObjectToTable, createPickPlan, DEFAULT_DROP_POSITION, PickPlan } from '../robotics/autonomousPlanner';
import { SimulatedObject } from '../types/robotics';
import {
  BOOK,
  GRIPPER,
  HOME_JOINT_ANGLES,
  JointWaypoint,
  ORIENTATION_TOLERANCE,
  PLACEMENT_TOLERANCE,
  POSITION_TOLERANCE,
  TaskPhase,
  buildTask5JointWaypoints,
  distance,
  sampleTaskTrajectory,
} from '../robotics/task5';

interface SimulationState {
  simulatorMode: 'manual' | 'predefined' | 'autonomous';
  setSimulatorMode: (mode: 'manual' | 'predefined' | 'autonomous') => void;
  // Navigation
  activeTab: AppTab;
  setActiveTab: (tab: AppTab) => void;

  // Joint Angles (radians)
  jointAngles: number[]; // [j1, j2, j3, j4, j5, j6]
  
  // DH Table & Forward Kinematics
  dhTable: DHParameter[];
  fkResult: FKResult;

  taskWaypoints: JointWaypoint[];
  taskPhase: TaskPhase;
  isTaskPlaying: boolean;
  trajectoryTime: number;
  currentWaypointIndex: number;
  positionError: number;
  placementError: number;
  isGripperOpen: boolean;
  isHoldingBook: boolean;
  showRobotDebug: boolean;
  showJointLabels: boolean;
  showGraspRegion: boolean;
  showPath: boolean;
  taskMode: 'predefined' | 'autonomous';
  autonomousPhase: TaskPhase;
  pickStatus: string;
  autonomousWaypointIndex: number;
  autonomousSegmentElapsed: number;
  autonomousStartAngles: number[];
  heldObjectLocalOffset: Vector3D;
  heldRotationOffset: EulerAngles;
  simulatedObject: SimulatedObject;
  autonomousPlan: PickPlan | null;
  setBookPosition: (position: { x: number; y: number; z: number }) => void;
  setBookYawDegrees: (yawDegrees: number) => void;
  setObjectDragging: (dragging: boolean) => void;
  setObjectSelected: (selected: boolean) => void;
  checkObjectReachability: () => void;
  planAutonomousPick: () => void;
  executeAutonomousPick: () => void;

  // State Mutators
  setJointAngle: (index: number, angleRad: number) => void;
  setJointAngleDegrees: (index: number, angleDeg: number) => void;
  setAllJointAngles: (anglesRad: number[]) => void;
  resetToHome: () => void;
  playTask: () => void;
  pauseTask: () => void;
  stopTask: () => void;
  resetTask: () => void;
  tickTask: (deltaSeconds: number) => void;
  setShowRobotDebug: (enabled: boolean) => void;
  setShowJointLabels: (enabled: boolean) => void;
  setShowGraspRegion: (enabled: boolean) => void;
  setShowPath: (enabled: boolean) => void;
}

const defaultAngles = HOME_JOINT_ANGLES;
const initialFK = computeForwardKinematics(INITIAL_DH_TABLE, defaultAngles);
const initialTaskWaypoints = buildTask5JointWaypoints(INITIAL_DH_TABLE);
const initialPickPlan = createPickPlan(INITIAL_DH_TABLE, BOOK.initialPosition, defaultAngles, DEFAULT_DROP_POSITION, 0);
const GRASP_TOLERANCE = 0.035;
const END_EFFECTOR_TOLERANCE = 0.018;
const AUTONOMOUS_SEGMENT_SECONDS = 1.5;

function worldOffsetToLocal(matrix: FKResult['endEffectorPose']['rotationMatrix'], worldOffset: Vector3D): Vector3D {
  return {
    x: matrix[0] * worldOffset.x + matrix[4] * worldOffset.y + matrix[8] * worldOffset.z,
    y: matrix[1] * worldOffset.x + matrix[5] * worldOffset.y + matrix[9] * worldOffset.z,
    z: matrix[2] * worldOffset.x + matrix[6] * worldOffset.y + matrix[10] * worldOffset.z,
  };
}

function addEuler(a: EulerAngles, b: EulerAngles): EulerAngles {
  return { roll: a.roll + b.roll, pitch: a.pitch + b.pitch, yaw: a.yaw + b.yaw };
}

function safelyReleaseObject(object: SimulatedObject): SimulatedObject {
  return { ...object, position: clampObjectToTable(object.position, object.rotation.yaw), graspState: 'on-table', state: 'onTable', isAttached: false, isGrasped: false };
}

export const useSimulationStore = create<SimulationState>((set, get) => ({
  simulatorMode: 'manual',
  setSimulatorMode: (mode) => set({ simulatorMode: mode, taskMode: mode === 'autonomous' ? 'autonomous' : 'predefined' }),
  activeTab: 'robot',
  setActiveTab: (tab) => set({ activeTab: tab }),

  jointAngles: defaultAngles,
  dhTable: INITIAL_DH_TABLE,
  fkResult: initialFK,
  taskWaypoints: initialTaskWaypoints,
  taskPhase: 'IDLE',
  isTaskPlaying: false,
  trajectoryTime: 0,
  currentWaypointIndex: 0,
  positionError: distance(initialFK.endEffectorPose.position, initialTaskWaypoints[0].position),
  placementError: 0,
  isGripperOpen: true,
  isHoldingBook: false,
  showRobotDebug: false,
  showJointLabels: false,
  showGraspRegion: false,
  showPath: true,
  taskMode: 'predefined',
  autonomousPhase: 'IDLE',
  pickStatus: 'IDLE',
  autonomousWaypointIndex: 0,
  autonomousSegmentElapsed: 0,
  autonomousStartAngles: defaultAngles,
  heldObjectLocalOffset: TCP_OFFSET,
  heldRotationOffset: { roll: 0, pitch: 0, yaw: 0 },
  simulatedObject: {
    id: 'book-1', type: 'book', position: BOOK.initialPosition,
    rotation: { roll: 0, pitch: 0, yaw: 0 }, dimensions: BOOK.size,
    graspState: 'on-table', state: 'onTable', isSelected: false, isBeingDragged: false, isGrasped: false, isAttached: false,
    targetPosition: DEFAULT_DROP_POSITION,
    reachability: { reachable: initialPickPlan.reachable, reason: initialPickPlan.reason },
  },
  autonomousPlan: null,

  setBookPosition: (position) => {
    const state = get();
    if (state.isHoldingBook || state.isTaskPlaying) return;
    set({
      simulatedObject: { ...state.simulatedObject, position, graspState: 'on-table', state: 'onTable', isGrasped: false, isAttached: false, reachability: { reachable: false, reason: 'Position changed; plan again' } },
      autonomousPlan: null,
      isTaskPlaying: false,
      taskPhase: state.taskMode === 'autonomous' ? 'IDLE' : state.taskPhase,
    });
  },

  setBookYawDegrees: (yawDegrees) => {
    const state = get();
    if (state.isTaskPlaying) return;
    const rotation = { ...state.simulatedObject.rotation, yaw: degToRad(yawDegrees) };
    const plan = createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, state.simulatedObject.targetPosition, rotation.yaw);
    set({ autonomousPlan: plan, simulatedObject: { ...state.simulatedObject, rotation, reachability: { reachable: plan.reachable, reason: plan.reason } } });
  },

  setObjectDragging: (isBeingDragged) => set((state) => ({
    simulatedObject: { ...state.simulatedObject, isBeingDragged },
  })),

  setObjectSelected: (isSelected) => set((state) => ({
    simulatedObject: { ...state.simulatedObject, isSelected },
  })),

  checkObjectReachability: () => {
    const state = get();
    const plan = createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, state.simulatedObject.targetPosition, state.simulatedObject.rotation.yaw);
    set({
      autonomousPlan: plan,
      simulatedObject: { ...state.simulatedObject, reachability: { reachable: plan.reachable, reason: plan.reason } },
    });
  },

  planAutonomousPick: () => {
    const state = get();
    const plan = createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, state.simulatedObject.targetPosition, state.simulatedObject.rotation.yaw);
    set({
      taskMode: 'autonomous', simulatorMode: 'autonomous', autonomousPlan: plan, isTaskPlaying: false,
      taskPhase: plan.reachable ? 'PLANNED' : 'UNREACHABLE', trajectoryTime: 0,
      autonomousPhase: plan.reachable ? 'PLANNED' : 'UNREACHABLE', pickStatus: plan.reachable ? 'PLANNING: READY' : `FAILED: ${plan.reason}`,
      simulatedObject: { ...state.simulatedObject, isGrasped: false, isAttached: false, graspState: 'on-table', state: 'onTable', reachability: { reachable: plan.reachable, reason: plan.reason } },
    });
  },

  executeAutonomousPick: () => {
    const state = get();
    if (!state.autonomousPlan?.reachable) return;
    set({
      taskMode: 'autonomous', simulatorMode: 'autonomous', isTaskPlaying: true, trajectoryTime: 0,
      taskPhase: 'MOVING TO PRE-GRASP', autonomousPhase: 'MOVING TO PRE-GRASP', pickStatus: 'PREGRASP',
      autonomousWaypointIndex: 0, autonomousSegmentElapsed: 0,
      autonomousStartAngles: [...state.jointAngles], isGripperOpen: true, isHoldingBook: false,
      heldObjectLocalOffset: TCP_OFFSET,
      heldRotationOffset: { roll: 0, pitch: 0, yaw: 0 },
      simulatedObject: { ...state.simulatedObject, isGrasped: false, isAttached: false, graspState: 'on-table', state: 'onTable' },
    });
  },

  setJointAngle: (index, angleRad) => {
    const state = get();
    const limits = PUMA_GEOMETRY.jointLimits[index];
    const minRad = degToRad(limits.min);
    const maxRad = degToRad(limits.max);
    const clamped = Math.max(minRad, Math.min(maxRad, angleRad));

    const newAngles = [...state.jointAngles];
    newAngles[index] = clamped;

    const newFK = computeForwardKinematics(state.dhTable, newAngles);

    set({
      jointAngles: newAngles,
      fkResult: newFK,
      isTaskPlaying: false,
      isHoldingBook: false,
      isGripperOpen: true,
      autonomousPlan: null,
      pickStatus: state.isTaskPlaying ? 'PAUSED: manual joint edit stopped the sequence' : state.pickStatus,
      simulatedObject: { ...(state.simulatedObject.isAttached ? safelyReleaseObject(state.simulatedObject) : state.simulatedObject), reachability: { reachable: false, reason: 'Robot pose changed; plan again' } },
    });
  },

  setJointAngleDegrees: (index, angleDeg) => {
    get().setJointAngle(index, degToRad(angleDeg));
  },

  setAllJointAngles: (anglesRad) => {
    const state = get();
    const sanitized = anglesRad.map((ang, idx) => {
      const limits = PUMA_GEOMETRY.jointLimits[idx];
      const minRad = degToRad(limits.min);
      const maxRad = degToRad(limits.max);
      return Math.max(minRad, Math.min(maxRad, ang));
    });

    const newFK = computeForwardKinematics(state.dhTable, sanitized);

    set({
      jointAngles: sanitized,
      fkResult: newFK,
      isTaskPlaying: false,
      isHoldingBook: false,
      isGripperOpen: true,
      autonomousPlan: null,
      pickStatus: state.isTaskPlaying ? 'PAUSED: manual joint edit stopped the sequence' : state.pickStatus,
      simulatedObject: { ...(state.simulatedObject.isAttached ? safelyReleaseObject(state.simulatedObject) : state.simulatedObject), reachability: { reachable: false, reason: 'Robot pose changed; plan again' } },
    });
  },

  resetToHome: () => {
    const state = get();
    const homeAngles = HOME_JOINT_ANGLES;
    const newFK = computeForwardKinematics(get().dhTable, homeAngles);
    set({
      jointAngles: homeAngles,
      fkResult: newFK,
      isTaskPlaying: false,
      isHoldingBook: false,
      isGripperOpen: true,
      autonomousPlan: null,
      pickStatus: state.isTaskPlaying ? 'PAUSED: manual reset stopped the sequence' : state.pickStatus,
      simulatedObject: { ...(state.simulatedObject.isAttached ? safelyReleaseObject(state.simulatedObject) : state.simulatedObject), reachability: { reachable: false, reason: 'Robot pose changed; plan again' } },
    });
  },

  playTask: () => set({ taskMode: 'predefined', simulatorMode: 'predefined', isTaskPlaying: true, pickStatus: 'IDLE' }),

  pauseTask: () => {
    const state = get();
    const safelyReleased = safelyReleaseObject(state.simulatedObject);
    set({
      isTaskPlaying: false,
      isHoldingBook: false,
      isGripperOpen: true,
      pickStatus: state.simulatedObject.isAttached ? 'PAUSED: object safely released on table' : state.pickStatus,
      autonomousPlan: null,
      simulatedObject: safelyReleased,
    });
  },

  stopTask: () => set({ isTaskPlaying: false, taskPhase: 'IDLE', trajectoryTime: 0 }),

  resetTask: () => {
    const newFK = computeForwardKinematics(get().dhTable, HOME_JOINT_ANGLES);
    const firstTarget = get().taskWaypoints[0].position;
    set({
      jointAngles: HOME_JOINT_ANGLES,
      fkResult: newFK,
      taskPhase: 'IDLE',
      isTaskPlaying: false,
      trajectoryTime: 0,
      currentWaypointIndex: 0,
      positionError: distance(newFK.endEffectorPose.position, firstTarget),
      placementError: 0,
      isGripperOpen: true,
      isHoldingBook: false,
      taskMode: 'predefined',
      simulatorMode: 'predefined',
      autonomousPlan: null,
      autonomousPhase: 'IDLE',
      pickStatus: 'IDLE',
      autonomousWaypointIndex: 0,
      autonomousSegmentElapsed: 0,
      autonomousStartAngles: HOME_JOINT_ANGLES,
      heldObjectLocalOffset: TCP_OFFSET,
      heldRotationOffset: { roll: 0, pitch: 0, yaw: 0 },
      simulatedObject: {
        ...get().simulatedObject,
        position: BOOK.initialPosition,
        graspState: 'on-table',
        state: 'onTable',
        rotation: { roll: 0, pitch: 0, yaw: 0 },
        isGrasped: false,
        isAttached: false,
        reachability: { reachable: initialPickPlan.reachable, reason: initialPickPlan.reason },
      },
    });
  },

  tickTask: (deltaSeconds) => {
    const state = get();
    if (!state.isTaskPlaying || state.taskPhase === 'COMPLETE' || state.taskPhase === 'ERROR') return;

    if (state.taskMode === 'autonomous' && state.autonomousPlan?.reachable) {
      const plan = state.autonomousPlan;
      const index = state.autonomousWaypointIndex;
      const waypoint = plan.waypoints[index];
      if (!waypoint) {
        set({ isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: 'FAILED: Missing retreat waypoint' });
        return;
      }
      if (index === 1 && state.pickStatus === 'OPEN') set({ pickStatus: 'DESCEND' });
      if (index === 5 && state.pickStatus === 'RELEASE') set({ pickStatus: 'RETREAT' });
      if (index === 1 && state.pickStatus === 'VERIFY') {
        const contactError = distance(state.fkResult.tcpPose.position, state.simulatedObject.position);
        if (contactError > GRASP_TOLERANCE) {
          set({ isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: `FAILED: alignment error ${contactError.toFixed(3)} m`, positionError: contactError });
        } else {
          set({ isGripperOpen: false, pickStatus: 'CLOSE' });
        }
        return;
      }
      if (index === 1 && state.pickStatus === 'CLOSE') {
        const fk = state.fkResult;
        const worldOffset = {
          x: state.simulatedObject.position.x - fk.endEffectorPose.position.x,
          y: state.simulatedObject.position.y - fk.endEffectorPose.position.y,
          z: state.simulatedObject.position.z - fk.endEffectorPose.position.z,
        };
        set({
          autonomousWaypointIndex: 2, autonomousSegmentElapsed: 0,
          autonomousStartAngles: [...state.jointAngles], autonomousPhase: 'LIFTING',
          taskPhase: 'ATTACHED', pickStatus: 'ATTACH', isHoldingBook: true,
          heldObjectLocalOffset: worldOffsetToLocal(fk.endEffectorPose.rotationMatrix, worldOffset),
          heldRotationOffset: {
            roll: state.simulatedObject.rotation.roll - fk.endEffectorPose.orientation.roll,
            pitch: state.simulatedObject.rotation.pitch - fk.endEffectorPose.orientation.pitch,
            yaw: state.simulatedObject.rotation.yaw - fk.endEffectorPose.orientation.yaw,
          },
          simulatedObject: { ...state.simulatedObject, isGrasped: true, isAttached: true, graspState: 'held', state: 'grasped' },
        });
        return;
      }
      const elapsed = Math.min(AUTONOMOUS_SEGMENT_SECONDS, state.autonomousSegmentElapsed + deltaSeconds);
      const progress = elapsed / AUTONOMOUS_SEGMENT_SECONDS;
      const smooth = progress * progress * (3 - 2 * progress);
      const angles = state.autonomousStartAngles.map((angle, i) => angle + (waypoint.jointAngles[i] - angle) * smooth);
      const fk = computeForwardKinematics(state.dhTable, angles);
      const eeError = distance(fk.endEffectorPose.position, waypoint.position);
      const reached = eeError <= END_EFFECTOR_TOLERANCE;
      const baseUpdate = {
        jointAngles: angles,
        fkResult: fk,
        trajectoryTime: state.trajectoryTime + deltaSeconds,
        autonomousSegmentElapsed: elapsed,
        currentWaypointIndex: index,
        positionError: eeError,
      };
      if (!reached && elapsed >= AUTONOMOUS_SEGMENT_SECONDS) {
        set({ ...baseUpdate, isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: 'FAILED: waypoint tracking error' });
        return;
      }

      if (reached && index === 1) {
        set({ ...baseUpdate, pickStatus: 'VERIFY' });
        return;
      }

      if (reached && index === 4) {
        const releasePosition = transformPoint(fk.endEffectorPose.rotationMatrix, state.heldObjectLocalOffset);
        const releaseError = distance(releasePosition, state.simulatedObject.targetPosition);
        if (releaseError > PLACEMENT_TOLERANCE) {
          set({ ...baseUpdate, isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: `FAILED: placement error ${releaseError.toFixed(3)} m`, placementError: releaseError });
          return;
        }
        set({
          ...baseUpdate,
          autonomousWaypointIndex: index + 1,
          autonomousSegmentElapsed: 0,
          autonomousStartAngles: [...angles],
          autonomousPhase: 'MOVING TO HOME', pickStatus: 'RELEASE',
          taskPhase: 'RELEASING',
          isGripperOpen: true,
          isHoldingBook: false,
          placementError: releaseError,
          simulatedObject: {
            ...state.simulatedObject, position: releasePosition,
            rotation: addEuler(fk.endEffectorPose.orientation, state.heldRotationOffset),
            graspState: 'on-table', state: 'placed', isGrasped: false, isAttached: false,
          },
        });
        return;
      }

      if (reached && index === plan.waypoints.length - 1) {
        set({ ...baseUpdate, isTaskPlaying: false, taskPhase: 'COMPLETE', autonomousPhase: 'COMPLETE', pickStatus: 'DONE' });
        return;
      }

      if (reached) {
        const nextIndex = index + 1;
        const nextPhase: TaskPhase = nextIndex === 1 ? 'MOVING TO GRASP'
          : nextIndex === 2 ? 'LIFTING'
            : nextIndex === 3 ? 'MOVING TO TARGET'
          : nextIndex === 4 ? 'LOWERING TO TARGET' : 'MOVING TO HOME';
        set({
          ...baseUpdate,
          autonomousWaypointIndex: nextIndex,
          autonomousSegmentElapsed: 0,
          autonomousStartAngles: [...angles],
          autonomousPhase: nextPhase,
          pickStatus: nextIndex === 1 ? 'OPEN' : nextIndex === 2 ? 'LIFT' : nextIndex === 3 ? 'TRANSPORT' : nextIndex === 4 ? 'LOWER' : 'RETREAT',
          taskPhase: nextPhase,
        });
        return;
      }

      const heldPosition = state.isHoldingBook
        ? transformPoint(fk.endEffectorPose.rotationMatrix, state.heldObjectLocalOffset)
        : state.simulatedObject.position;
      set({
        ...baseUpdate,
        isTaskPlaying: true,
        taskPhase: state.autonomousPhase,
        simulatedObject: state.isHoldingBook ? {
          ...state.simulatedObject,
          position: heldPosition,
          rotation: addEuler(fk.endEffectorPose.orientation, state.heldRotationOffset),
        } : state.simulatedObject,
      });
      return;
    }

    const sample = sampleTaskTrajectory(state.trajectoryTime + deltaSeconds, state.taskWaypoints);
    const newFK = computeForwardKinematics(state.dhTable, sample.jointAngles);
    const eePosition = newFK.endEffectorPose.position;
    const targetIndex = Math.max(0, Math.min(3, sample.segmentIndex));
    const target = state.taskWaypoints[targetIndex];
    const targetError = distance(eePosition, target.position);

    let taskPhase: TaskPhase = `MOVING TO ${target.id}` as TaskPhase;
    let currentWaypointIndex = targetIndex;
    let isGripperOpen = state.isGripperOpen;
    let isHoldingBook = state.isHoldingBook;
    let placementError = state.placementError;
    let isTaskPlaying = true;

    if (targetError <= POSITION_TOLERANCE) {
      taskPhase = `${target.id} REACHED` as TaskPhase;
      currentWaypointIndex = targetIndex;
    }

    if (target.id === 'P3' && targetError <= POSITION_TOLERANCE) {
      const bookError = distance(BOOK.initialPosition, { ...target.position, z: BOOK.initialPosition.z });
      const orientationOk = state.taskWaypoints[2].ik.orientationError <= ORIENTATION_TOLERANCE;
      if (bookError <= 0.04 && orientationOk) {
        taskPhase = 'HOLDING OBJECT';
        isGripperOpen = false;
        isHoldingBook = true;
      } else {
        taskPhase = 'ERROR';
        isTaskPlaying = false;
      }
    }

    const atEnd = state.trajectoryTime + deltaSeconds >= sample.totalDuration;
    const p4 = state.taskWaypoints[3];
    const p4Error = distance(eePosition, p4.position);
    if (atEnd && p4Error <= POSITION_TOLERANCE) {
      taskPhase = 'COMPLETE';
      isTaskPlaying = false;
      isGripperOpen = true;
      isHoldingBook = false;
      placementError = distance(newFK.tcpPose.position, { x: p4.position.x, y: p4.position.y, z: BOOK.initialPosition.z });
      if (placementError > PLACEMENT_TOLERANCE) {
        taskPhase = 'ERROR';
      }
    }

    set({
      jointAngles: sample.jointAngles,
      fkResult: newFK,
      taskPhase,
      isTaskPlaying,
      trajectoryTime: state.trajectoryTime + deltaSeconds,
      currentWaypointIndex,
      positionError: targetError,
      placementError,
      isGripperOpen,
      isHoldingBook,
      simulatedObject: isHoldingBook ? { ...state.simulatedObject, position: {
        x: eePosition.x + newFK.endEffectorPose.rotationMatrix[2] * GRIPPER.fingerLength,
        y: eePosition.y + newFK.endEffectorPose.rotationMatrix[6] * GRIPPER.fingerLength,
        z: eePosition.z + newFK.endEffectorPose.rotationMatrix[10] * GRIPPER.fingerLength,
      }, graspState: 'held', state: 'grasped', isAttached: true, isGrasped: true } : state.isHoldingBook && atEnd ? {
        ...state.simulatedObject, position: newFK.tcpPose.position, graspState: 'on-table', state: 'placed', isAttached: false, isGrasped: false,
      } : state.simulatedObject,
    });
  },

  setShowRobotDebug: (enabled) => set({ showRobotDebug: enabled }),
  setShowJointLabels: (enabled) => set({ showJointLabels: enabled }),
  setShowGraspRegion: (enabled) => set({ showGraspRegion: enabled }),
  setShowPath: (enabled) => set({ showPath: enabled }),
}));
