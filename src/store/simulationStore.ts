import { create } from 'zustand';
import { AppTab, DHParameter, EulerAngles, FKResult, Vector3D } from '../types/robotics';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY, setTablePreset as applyTablePreset, TablePreset, TABLE_PRESETS, TCP_OFFSET } from '../robot/robotConfig';
import { computeForwardKinematics } from '../robotics/forwardKinematics';
import { transformPoint } from '../robotics/inverseKinematics';
import { heldBookCollision } from '../robotics/heldObjectCollision';
import { degToRad, rpyToMatrix } from '../robotics/transforms';
import { clampObjectToTable, createPickPlan, DropTarget, findDefaultPickPosition, findNearestPickablePosition, findNearestValidDestination, PickPlan, validatePickApproach } from '../robotics/autonomousPlanner';
import { SimulatedObject } from '../types/robotics';
import { evaluatePickabilityCached } from '../robotics/pickabilityGrid';
import { PlacementSurface, PlacementSurfaceShape, defaultPlacementSurfaces, validateSurfaceSet, surfaceBookZ } from '../robotics/placementSurfaces';
import {
  BOOK,
  GRIPPER,
  HOME_JOINT_ANGLES,
  PREDEFINED_BOOK_POSITION,
  TABLE,
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
  tablePreset: TablePreset;
  placementSurfaces: PlacementSurface[];
  surfaceEditError: string | null;
  updatePlacementSurface: (id: string, patch: Partial<PlacementSurface>) => void;
  addPlacementSurface: (shape?:PlacementSurfaceShape) => void;
  removePlacementSurface: (id: string) => void;
  setTablePreset: (preset: TablePreset) => void;
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
  gripperCloseWidth: number;
  isHoldingBook: boolean;
  showRobotDebug: boolean;
  showGraspDebug: boolean;
  graspDebugLog: { stage: string; tcp: Vector3D; bookCenter: Vector3D; fingerGap: number }[];
  showJointLabels: boolean;
  showGraspRegion: boolean;
  showReachabilityOverlay: boolean;
  showPerformanceDebug: boolean;
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
  dropTarget: DropTarget | null;
  setDropTarget: (target: DropTarget | null, snap?: boolean) => void;
  allowUnreachablePlacement: boolean;
  setAllowUnreachablePlacement: (enabled: boolean) => void;
  destinationPickArmed: boolean;
  setDestinationPickArmed: (armed: boolean) => void;
  setBookPosition: (position: { x: number; y: number; z: number }, surfaceId?: string) => void;
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
  setShowGraspDebug: (enabled: boolean) => void;
  setShowJointLabels: (enabled: boolean) => void;
  setShowGraspRegion: (enabled: boolean) => void;
  setShowReachabilityOverlay: (enabled: boolean) => void;
  setShowPerformanceDebug: (enabled: boolean) => void;
  setShowPath: (enabled: boolean) => void;
}

const defaultAngles = HOME_JOINT_ANGLES;
const initialFK = computeForwardKinematics(INITIAL_DH_TABLE, defaultAngles);
const initialTaskWaypoints = buildTask5JointWaypoints(INITIAL_DH_TABLE);
const initialPickability = validatePickApproach(INITIAL_DH_TABLE, BOOK.initialPosition, defaultAngles, Math.PI / 2);
const GRASP_TOLERANCE = 0.005;
const END_EFFECTOR_TOLERANCE = 0.018;
const AUTONOMOUS_SEGMENT_SECONDS = 1.5;
// These are the exact results of findDefaultPickPosition for the fixed DH table at HOME.
// Reusing those measured results avoids a synchronous planner search during preset changes.
const HOME_DEFAULT_POSES = {
  compact: { x: 0.7950000000000002, y: -0.035000000000000024, z: BOOK.initialPosition.z },
  large: { x: 0.7950000000000004, y: -0.044999999999999984, z: BOOK.initialPosition.z },
};
const dhFingerprint = (dh: DHParameter[]) => dh.map((p) => [p.a,p.alpha,p.d,p.theta,p.thetaMin,p.thetaMax].join(',')).join(';');
const homeDhFingerprint = dhFingerprint(INITIAL_DH_TABLE);
function defaultPickPose(dh: DHParameter[], angles: number[], preset: TablePreset) {
  const atHome = angles.length === HOME_JOINT_ANGLES.length && angles.every((angle,index)=>Math.abs(angle-HOME_JOINT_ANGLES[index])<1e-9);
  if(atHome && dhFingerprint(dh)===homeDhFingerprint&&Math.abs(TABLE.height-TABLE_PRESETS[preset].height)<1e-9) return { ...HOME_DEFAULT_POSES[preset],z:BOOK.initialPosition.z };
  // Non-HOME poses use the same center-aligned TCP search as execution.
  return findDefaultPickPosition(dh,angles,0);
}

function graspDebugEntry(stage: string, fk: FKResult, object: SimulatedObject, gripperOpen: boolean, closeWidth = BOOK.size.y) {
  return {
    stage,
    tcp: { ...fk.tcpPose.position },
    bookCenter: { ...object.position },
    fingerGap: (gripperOpen ? GRIPPER.openWidth : closeWidth + GRIPPER.fingerThickness) - GRIPPER.fingerThickness,
  };
}

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
  setSimulatorMode: (mode) => set((state) => ({ simulatorMode: mode, taskMode: mode === 'autonomous' ? 'autonomous' : 'predefined', simulatedObject: mode === 'predefined' && !state.isTaskPlaying ? { ...state.simulatedObject, position: { ...PREDEFINED_BOOK_POSITION }, rotation: { roll: 0, pitch: 0, yaw: 0 }, graspState: 'on-table', state: 'onTable', isAttached: false, isGrasped: false } : state.simulatedObject })),
  tablePreset: 'compact',
  placementSurfaces: defaultPlacementSurfaces().filter((surface) => surface.id === 'table'),
  surfaceEditError: null,
  updatePlacementSurface: (id, patch) => set((state) => {
    const oldTable=state.placementSurfaces.find((surface)=>surface.id==='table');
    const next=state.placementSurfaces.map((surface)=>{
      if(surface.id===id)return {...surface,...patch,center:patch.center?{...surface.center,...patch.center}:surface.center,size:patch.size?{...surface.size,...patch.size}:surface.size};
      if(id==='table'&&oldTable){const dx=(patch.center?.x??oldTable.center.x)-oldTable.center.x,dy=(patch.center?.y??oldTable.center.y)-oldTable.center.y,z=patch.z??oldTable.z;
        if(surface.id==='tray')return {...surface,center:{x:surface.center.x+dx,y:surface.center.y+dy,z:0},z};
        if(surface.id==='raised-platform'&&patch.z!==undefined&&Math.abs(surface.z-(oldTable.z+.15))<1e-8)return {...surface,z:z+.15};
        if(surface.id==='lower-platform'&&patch.z!==undefined&&Math.abs(surface.z-(oldTable.z-.15))<1e-8)return {...surface,z:z-.15};
      }
      return surface;
    });
    const error=validateSurfaceSet(next);
    if(!error&&id==='table'){const table=next.find((surface)=>surface.id==='table')!;TABLE.center.x=table.center.x;TABLE.center.y=table.center.y;TABLE.width=table.size.width;TABLE.depth=table.size.depth;TABLE.height=table.z;BOOK.initialPosition.z=table.z+BOOK.size.z/2;}
    return error?{surfaceEditError:error}:{placementSurfaces:next,surfaceEditError:null,autonomousPlan:null,simulatedObject:state.simulatedObject.surfaceId==='table'&&!state.isHoldingBook?{...state.simulatedObject,position:{...state.simulatedObject.position,z:next.find((surface)=>surface.id==='table')!.z+BOOK.size.z/2}}:state.simulatedObject};
  }),
  addPlacementSurface: (shape='rectangle') => set((state) => {
    const n=state.placementSurfaces.length;
    const surface:PlacementSurface={id:`custom-${n}`,name:`Surface ${n}`,shape,center:{x:.3,y:.45+n*.25,z:0},yaw:0,z:.78,size:shape==='rectangle'?{width:.24,depth:.18}:{width:.4,depth:.4,innerRadius:.06,outerRadius:.2,startAngle:0,endAngle:Math.PI},support:{type:shape==='rectangle'?'legs':'column',width:.025,depth:.025,radius:.035,height:.735},color:'#718096'};
    const next=[...state.placementSurfaces,surface],error=validateSurfaceSet(next);
    return error?{surfaceEditError:error}:{placementSurfaces:next,surfaceEditError:null,autonomousPlan:null};
  }),
  removePlacementSurface: (id) => set((state) => ({placementSurfaces:state.placementSurfaces.filter((surface)=>surface.id!==id),dropTarget:state.dropTarget?.surfaceId===id?null:state.dropTarget,simulatedObject:state.simulatedObject.surfaceId===id?{...state.simulatedObject,surfaceId:'table',position:{...state.simulatedObject.position,z:surfaceBookZ(state.placementSurfaces.find((surface)=>surface.id==='table')!)} }:state.simulatedObject,surfaceEditError:null,autonomousPlan:null})),
  setTablePreset: (preset) => {
    const state = get();
    if (state.isTaskPlaying) return;
    applyTablePreset(preset);
    const defaultPose = defaultPickPose(state.dhTable, state.jointAngles, preset);
    Object.assign(BOOK.initialPosition, defaultPose);
    const surfaces=[{...defaultPlacementSurfaces()[0],center:{...TABLE.center},z:TABLE.height,size:{width:TABLE.width,depth:TABLE.depth}}];
    set({ tablePreset: preset, placementSurfaces:surfaces, showReachabilityOverlay: preset === 'compact', dropTarget: null, autonomousPlan: null, taskPhase: 'IDLE', autonomousPhase: 'IDLE', pickStatus: 'IDLE', simulatedObject: { ...state.simulatedObject, position: { ...defaultPose },surfaceId:'table', rotation: { roll: 0, pitch: 0, yaw: 0 }, reachability: { reachable: true, reason: 'Reachable' }, state: 'onTable', graspState: 'on-table', isAttached: false, isGrasped: false } });
  },
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
  gripperCloseWidth: BOOK.size.y,
  isHoldingBook: false,
  showRobotDebug: false,
  showGraspDebug: false,
  graspDebugLog: [],
  showJointLabels: false,
  showGraspRegion: false,
  showReachabilityOverlay: true,
  showPerformanceDebug: false,
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
    id: 'book-1', type: 'book', position: BOOK.initialPosition, surfaceId:'table',
    rotation: { roll: 0, pitch: 0, yaw: 0 }, dimensions: BOOK.size,
    graspState: 'on-table', state: 'onTable', isSelected: false, isBeingDragged: false, isGrasped: false, isAttached: false,
    reachability: { reachable: initialPickability.reachable, reason: initialPickability.reason },
  },
  autonomousPlan: null,
  dropTarget: null,
  allowUnreachablePlacement: false,
  setAllowUnreachablePlacement: (enabled) => set({ allowUnreachablePlacement: enabled }),
  destinationPickArmed: false,
  setDestinationPickArmed: (armed) => set({ destinationPickArmed: armed }),

  setDropTarget: (target, snap = false) => {
    const state = get();
    if (state.isTaskPlaying) return;
    const requestedSurface=state.placementSurfaces.find((surface)=>surface.id===(target?.surfaceId??'table'));
    const requested = target ? { x: target.x, y: target.y, yaw: target.yaw, surfaceId: target.surfaceId ?? 'table', z: target.z??(requestedSurface?surfaceBookZ(requestedSurface):undefined) } : null;
    const snapped = requested && snap && !state.allowUnreachablePlacement && requested.surfaceId==='table'
      ? findNearestValidDestination(state.dhTable, state.jointAngles, state.simulatedObject.position, state.simulatedObject.rotation.yaw, requested, degToRad(requested.yaw))
      : null;
    const normalized = requested ? { ...requested, ...(snapped ?? {}) } : null;
    const destination = normalized ? { ...normalized, yaw: degToRad(normalized.yaw) } : null;
    const plan = destination ? createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, destination, state.simulatedObject.rotation.yaw, true, state.placementSurfaces,state.simulatedObject.surfaceId??'table') : null;
    const pick = destination ? null : validatePickApproach(state.dhTable, state.simulatedObject.position, state.jointAngles, state.simulatedObject.rotation.yaw + Math.PI / 2);
    set({ dropTarget: normalized, autonomousPlan: null, taskPhase: 'IDLE', autonomousPhase: 'IDLE', pickStatus: 'IDLE', simulatedObject: { ...state.simulatedObject, reachability: { reachable: plan?.reachable ?? pick!.reachable, reason: plan?.reason ?? pick!.reason } } });
  },

  setBookPosition: (position, surfaceId) => {
    const state = get();
    if (state.isHoldingBook || state.isTaskPlaying) return;
    set({
      simulatedObject: { ...state.simulatedObject, position, surfaceId:surfaceId??state.simulatedObject.surfaceId??'table', graspState: 'on-table', state: 'onTable', isGrasped: false, isAttached: false, reachability: { reachable: false, reason: 'Position changed; plan again' } },
      autonomousPlan: null,
      isTaskPlaying: false,
      taskPhase: 'IDLE', autonomousPhase: 'IDLE', pickStatus: 'IDLE',
    });
  },

  setBookYawDegrees: (yawDegrees) => {
    const state = get();
    if (state.isTaskPlaying) return;
    const rotation = { ...state.simulatedObject.rotation, yaw: degToRad(yawDegrees) };
    const plan = state.dropTarget ? createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, { ...state.dropTarget, yaw: degToRad(state.dropTarget.yaw) }, rotation.yaw, true, state.placementSurfaces,state.simulatedObject.surfaceId??'table') : null;
    const pick = state.dropTarget ? null : validatePickApproach(state.dhTable, state.simulatedObject.position, state.jointAngles, rotation.yaw + Math.PI / 2);
    set({ autonomousPlan: null, taskPhase: 'IDLE', autonomousPhase: 'IDLE', pickStatus: 'IDLE', simulatedObject: { ...state.simulatedObject, rotation, reachability: { reachable: plan?.reachable ?? pick!.reachable, reason: plan?.reason ?? pick!.reason } } });
  },

  setObjectDragging: (isBeingDragged) => set((state) => ({
    simulatedObject: { ...state.simulatedObject, isBeingDragged },
  })),

  setObjectSelected: (isSelected) => set((state) => ({
    simulatedObject: { ...state.simulatedObject, isSelected },
  })),

  checkObjectReachability: () => {
    const state = get();
    const position = !state.allowUnreachablePlacement ? findNearestPickablePosition(state.dhTable, state.jointAngles, state.simulatedObject.position, state.simulatedObject.rotation.yaw, state.dropTarget ? { ...state.dropTarget, yaw: degToRad(state.dropTarget.yaw) } : null) : state.simulatedObject.position;
    const object = { ...state.simulatedObject, position };
    if (!state.dropTarget) {
      const pick = evaluatePickabilityCached({ preset:state.tablePreset,dhTable:state.dhTable,jointAngles:state.jointAngles,yawRad:state.simulatedObject.rotation.yaw,bookZ:position.z }, position.x, position.y);
      set({ autonomousPlan: null, simulatedObject: { ...object, reachability: { reachable: pick.reachable, reason: pick.reason } } });
      return;
    }
    const plan = createPickPlan(state.dhTable, position, state.jointAngles, { ...state.dropTarget, yaw: degToRad(state.dropTarget.yaw) }, state.simulatedObject.rotation.yaw, true, state.placementSurfaces,state.simulatedObject.surfaceId??'table');
    set({ autonomousPlan: plan, simulatedObject: { ...object, reachability: { reachable: plan.reachable, reason: plan.reason } } });
  },

  planAutonomousPick: () => {
    const state = get();
    if (!state.dropTarget) { set({ autonomousPlan: null, pickStatus: 'Set a destination (click the table or type X/Y)', taskPhase: 'IDLE', autonomousPhase: 'IDLE' }); return; }
    const plan = createPickPlan(state.dhTable, state.simulatedObject.position, state.jointAngles, { ...state.dropTarget, yaw: degToRad(state.dropTarget.yaw) }, state.simulatedObject.rotation.yaw, true, state.placementSurfaces,state.simulatedObject.surfaceId??'table');
    set({
      taskMode: 'autonomous', simulatorMode: 'autonomous', autonomousPlan: plan, gripperCloseWidth: plan.gripWidth, isTaskPlaying: false,
      taskPhase: plan.reachable ? 'PLANNED' : 'UNREACHABLE', trajectoryTime: 0,
      autonomousPhase: plan.reachable ? 'PLANNED' : 'UNREACHABLE', pickStatus: plan.reachable ? 'PLANNING: READY' : `FAILED: ${plan.reason}`,
      graspDebugLog: [...state.graspDebugLog, graspDebugEntry('PLANNING', state.fkResult, state.simulatedObject, state.isGripperOpen)],
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
      graspDebugLog: [...state.graspDebugLog, graspDebugEntry('PREGRASP', state.fkResult, state.simulatedObject, true)],
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
      gripperCloseWidth: BOOK.size.y,
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
      gripperCloseWidth: BOOK.size.y,
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

  playTask: () => set((state) => ({ taskMode: 'predefined', simulatorMode: 'predefined', isTaskPlaying: true, pickStatus: 'IDLE', simulatedObject: { ...state.simulatedObject, position: { ...PREDEFINED_BOOK_POSITION }, rotation: { roll: 0, pitch: 0, yaw: 0 }, graspState: 'on-table', state: 'onTable', isAttached: false, isGrasped: false } })),

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
      graspDebugLog: [],
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
        reachability: { reachable: initialPickability.reachable, reason: initialPickability.reason },
      },
    });
  },

  tickTask: (deltaSeconds) => {
    const state = get();
    if (!state.isTaskPlaying || state.taskPhase === 'COMPLETE' || state.taskPhase === 'ERROR') return;

    if (state.taskMode === 'autonomous' && state.graspDebugLog[state.graspDebugLog.length - 1]?.stage !== state.pickStatus) {
      const fingerGap = (state.isGripperOpen ? GRIPPER.openWidth : state.gripperCloseWidth + GRIPPER.fingerThickness) - GRIPPER.fingerThickness;
      set({ graspDebugLog: [...state.graspDebugLog, {
        stage: state.pickStatus,
        tcp: { ...state.fkResult.tcpPose.position },
        bookCenter: { ...state.simulatedObject.position },
        fingerGap,
      }] });
    }

    if (state.taskMode === 'autonomous' && state.autonomousPlan?.reachable) {
      const plan = state.autonomousPlan;
      const index = state.autonomousWaypointIndex;
      const waypoint = plan.waypoints[index];
      if (!waypoint) {
        set({ isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: 'FAILED: Missing retreat waypoint' });
        return;
      }
      if (waypoint.id === 'GRASP' && state.pickStatus === 'OPEN') set({ pickStatus: 'DESCEND' });
      if (waypoint.id === 'LIFT' && state.pickStatus === 'ATTACH') set({ pickStatus: 'LIFT' });
      if (waypoint.id === 'RETREAT' && state.pickStatus === 'RELEASE') set({ pickStatus: 'RETREAT' });
      if (waypoint.id === 'GRASP' && state.pickStatus === 'VERIFY') {
        const contactError = distance(state.fkResult.tcpPose.position, state.simulatedObject.position);
        if (contactError > GRASP_TOLERANCE) {
          set({ isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: `FAILED: alignment error ${contactError.toFixed(3)} m`, positionError: contactError });
        } else {
          set({ isGripperOpen: false, pickStatus: 'CLOSE' });
        }
        return;
      }
      if (waypoint.id === 'GRASP' && state.pickStatus === 'CLOSE') {
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
      const targetFk = computeForwardKinematics(state.dhTable, waypoint.jointAngles);
      const yawTrackingError = Math.abs(Math.atan2(Math.sin(fk.endEffectorPose.orientation.yaw - targetFk.endEffectorPose.orientation.yaw), Math.cos(fk.endEffectorPose.orientation.yaw - targetFk.endEffectorPose.orientation.yaw)));
      const reached = waypoint.id === 'GRASP' || waypoint.id === 'PLACE' ? eeError <= 0.0015 && yawTrackingError <= degToRad(0.1) : eeError <= END_EFFECTOR_TOLERANCE;
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

      if (reached && waypoint.id === 'GRASP') {
        set({ ...baseUpdate, pickStatus: 'VERIFY' });
        return;
      }

      if (reached && waypoint.id === 'PLACE') {
        const releasePositionRaw = transformPoint(fk.endEffectorPose.rotationMatrix, state.heldObjectLocalOffset);
        const releaseRotation = addEuler(fk.endEffectorPose.orientation, state.heldRotationOffset);
        const releaseMatrix = rpyToMatrix(releaseRotation.roll, releaseRotation.pitch, releaseRotation.yaw);
        const releaseHalfBottom = Math.abs(releaseMatrix[8]) * BOOK.size.x / 2 + Math.abs(releaseMatrix[9]) * BOOK.size.y / 2 + Math.abs(releaseMatrix[10]) * BOOK.size.z / 2;
        const target = state.dropTarget;
        const targetSurface=state.placementSurfaces.find((surface)=>surface.id===(target?.surfaceId??'table'));
        const targetZ=target?.z??(targetSurface?surfaceBookZ(targetSurface):NaN);
        const releasePosition = { ...releasePositionRaw, z: Math.max(releasePositionRaw.z, targetZ + releaseHalfBottom - BOOK.size.z/2 + 1e-6) };
        const declaredPose = target ? { x: target.x, y: target.y, z: targetZ } : null;
        const releaseError = declaredPose ? distance(releasePosition, declaredPose) : Infinity;
        const yawError = target ? Math.abs(Math.atan2(Math.sin(state.simulatedObject.rotation.yaw - degToRad(target.yaw)), Math.cos(state.simulatedObject.rotation.yaw - degToRad(target.yaw)))) : Infinity;
        if (!declaredPose || releaseError > 0.005 || yawError > degToRad(2)) {
          set({ ...baseUpdate, isTaskPlaying: false, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: `FAILED: book pose outside release tolerance (${releaseError.toFixed(3)} m; dx=${declaredPose ? (releasePosition.x-declaredPose.x).toFixed(3) : 'n/a'}, dy=${declaredPose ? (releasePosition.y-declaredPose.y).toFixed(3) : 'n/a'}, dz=${declaredPose ? (releasePosition.z-declaredPose.z).toFixed(3) : 'n/a'} m; ${(yawError * 180 / Math.PI).toFixed(1)} deg)`, placementError: releaseError });
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
            ...state.simulatedObject, position: declaredPose!,
            rotation: { ...state.simulatedObject.rotation, yaw: degToRad(target!.yaw), roll: 0, pitch: 0 },
            graspState: 'on-table', state: 'placed', isGrasped: false, isAttached: false,
            surfaceId: target?.surfaceId ?? 'table',
          },
        });
        return;
      }

      if (reached && index === plan.waypoints.length - 1) {
        set({ ...baseUpdate, isTaskPlaying: false, taskPhase: 'COMPLETE', autonomousPhase: 'COMPLETE', pickStatus: 'DONE',
          graspDebugLog: [...state.graspDebugLog, graspDebugEntry('DONE', fk, state.simulatedObject, state.isGripperOpen)] });
        return;
      }

      if (reached) {
        const nextIndex = index + 1;
        const nextWaypoint=plan.waypoints[nextIndex];
        const nextPhase: TaskPhase = nextWaypoint?.id==='GRASP'?'MOVING TO GRASP'
          : nextWaypoint?.id==='LIFT'?'LIFTING'
            : nextWaypoint?.id==='PLACE'?'LOWERING TO TARGET'
              : nextWaypoint?.id==='RETREAT'?'MOVING TO HOME':'MOVING TO TARGET';
        set({
          ...baseUpdate,
          autonomousWaypointIndex: nextIndex,
          autonomousSegmentElapsed: 0,
          autonomousStartAngles: [...angles],
          autonomousPhase: nextPhase,
          pickStatus: nextWaypoint?.id === 'GRASP' ? 'OPEN' : nextWaypoint?.id === 'LIFT' ? 'LIFT' : nextWaypoint?.id === 'PRE-PLACE' ? 'TRANSPORT' : nextWaypoint?.id === 'PLACE' ? 'LOWER' : nextWaypoint?.id === 'RETREAT' ? 'RETREAT' : 'TRANSPORT',
          taskPhase: nextPhase,
        });
        return;
      }

      let heldPosition = state.isHoldingBook
        ? transformPoint(fk.endEffectorPose.rotationMatrix, state.heldObjectLocalOffset)
        : state.simulatedObject.position;
      const heldRotation = state.isHoldingBook ? addEuler(fk.endEffectorPose.orientation, state.heldRotationOffset) : state.simulatedObject.rotation;
      if (state.isHoldingBook && state.autonomousPhase === 'LOWERING TO TARGET') {
        const rotation = rpyToMatrix(heldRotation.roll, heldRotation.pitch, heldRotation.yaw);
        const halfBottom = Math.abs(rotation[8]) * BOOK.size.x / 2 + Math.abs(rotation[9]) * BOOK.size.y / 2 + Math.abs(rotation[10]) * BOOK.size.z / 2;
        const targetSurface=state.placementSurfaces.find((surface)=>surface.id===(state.dropTarget?.surfaceId??'table'));
        if (targetSurface && heldPosition.z - halfBottom < targetSurface.z + (targetSurface.tray?.floorThickness??0) + 1e-6) heldPosition = { ...heldPosition, z: targetSurface.z + (targetSurface.tray?.floorThickness??0) + halfBottom + 1e-6 };
      }
      if (state.isHoldingBook && ['LIFTING', 'MOVING TO TARGET', 'LOWERING TO TARGET'].includes(state.autonomousPhase)) {
        const collision = heldBookCollision(heldPosition, heldRotation,state.placementSurfaces,state.autonomousPhase==='LOWERING TO TARGET'?state.dropTarget?.surfaceId:undefined);
        if (collision) {
          set({ ...baseUpdate, isTaskPlaying: false, isHoldingBook: false, isGripperOpen: true, taskPhase: 'ERROR', autonomousPhase: 'ERROR', pickStatus: `FAILED: ${collision.kind} (${(collision.penetrationM*1000).toFixed(1)} mm)`, simulatedObject: { ...state.simulatedObject, position: heldPosition, rotation: heldRotation, graspState: 'on-table', state: 'onTable', isAttached: false, isGrasped: false } });
          return;
        }
      }
      set({
        ...baseUpdate,
        isTaskPlaying: true,
        taskPhase: state.autonomousPhase,
        simulatedObject: state.isHoldingBook ? {
          ...state.simulatedObject,
          position: heldPosition,
          rotation: heldRotation,
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
      const bookError = distance(PREDEFINED_BOOK_POSITION, { ...target.position, z: PREDEFINED_BOOK_POSITION.z });
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
  setShowGraspDebug: (enabled) => set({ showGraspDebug: enabled }),
  setShowJointLabels: (enabled) => set({ showJointLabels: enabled }),
  setShowGraspRegion: (enabled) => set({ showGraspRegion: enabled }),
  setShowReachabilityOverlay: (enabled) => set({ showReachabilityOverlay: enabled }),
  setShowPerformanceDebug: (enabled) => set({ showPerformanceDebug: enabled }),
  setShowPath: (enabled) => set({ showPath: enabled }),
}));
