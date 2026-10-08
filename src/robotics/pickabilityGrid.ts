import { DHParameter } from '../types/robotics';
import { TABLE_PRESETS, TablePreset, TOOL_LENGTH } from '../robot/robotConfig';
import { BOOK } from './task5';
import { createPickPlan, DropTarget, evaluatePickabilityCell } from './autonomousPlanner';
import { PlacementSurface, surfaceBookZ } from './placementSurfaces';
import { Vector3D } from '../types/robotics';

export const PICKABILITY_REASON = {
  outside: 'Outside table bounds',
  unreachable: 'Book orientation not reachable here at any wrist angle',
  reachable: 'Reachable',
} as const;
export type PickabilityReasonCode = 0 | 1 | 2;
export type PickabilityWorkerConfig = { preset: TablePreset; dhTable: DHParameter[]; jointAngles: number[]; yawRad: number; bookZ: number; surface?:PlacementSurface; mode?:'pick'|'place'; sourcePosition?:Vector3D; sourceSurfaceId?:string; surfaces?:PlacementSurface[] };
export type PickabilityGridShape = { columns: number; rows: number; stepM: number; originX: number; originY: number; count: number };

export function pickabilityShape(preset: TablePreset, stepM: number, surface?:PlacementSurface): PickabilityGridShape {
  const table = TABLE_PRESETS[preset],width=surface?.size.width??table.width,depth=surface?.size.depth??table.depth,cx=surface?.center.x??table.center.x,cy=surface?.center.y??table.center.y;
  const columns = Math.ceil((width - stepM / 2) / stepM - 1e-9), rows = Math.ceil((depth - stepM / 2) / stepM - 1e-9);
  return { columns, rows, stepM, originX: cx - width / 2 + stepM / 2, originY: cy - depth / 2 + stepM / 2, count: columns * rows };
}

export function pickabilityReasonCode(reason: string): PickabilityReasonCode {
  if (reason === PICKABILITY_REASON.reachable) return 2;
  if (reason === PICKABILITY_REASON.outside || reason.startsWith('Outside ')) return 0;
  return 1;
}
export function pickabilityReason(code: number): string { return code === 2 ? PICKABILITY_REASON.reachable : code === 0 ? PICKABILITY_REASON.outside : PICKABILITY_REASON.unreachable; }

/** Shared by the worker and its equivalence tests. A state byte is 0=outside, 1=fits/unreachable, 2=reachable. */
export function evaluatePickabilityState(config: PickabilityWorkerConfig, x: number, y: number): { state: PickabilityReasonCode; reason: string } {
  if(config.mode==='place'&&config.surface&&config.sourcePosition){
    const destination:DropTarget={x,y,yaw:0,surfaceId:config.surface.id,z:surfaceBookZ(config.surface)};
    const plan=createPickPlan(config.dhTable,config.sourcePosition,config.jointAngles,destination,config.yawRad,true,config.surfaces??[config.surface],config.sourceSurfaceId??'table');
    return {state:pickabilityReasonCode(plan.reason),reason:plan.reason};
  }
  const cell = evaluatePickabilityCell(config.dhTable, config.jointAngles, { x, y, z: config.bookZ }, config.yawRad,undefined,undefined,config.surface,config.surfaces);
  return { state: pickabilityReasonCode(cell.reason), reason: cell.reason };
}

export function computePickabilityChunk(config: PickabilityWorkerConfig, shape: PickabilityGridShape, start: number, count: number) {
  const states = new Uint8Array(count), reasons = new Uint8Array(count);
  for (let i = 0; i < count; i++) {
    const index = start + i, ix = Math.floor(index / shape.rows), iy = index % shape.rows;
    const result = evaluatePickabilityState(config, shape.originX + ix * shape.stepM, shape.originY + iy * shape.stepM);
    states[i] = result.state; reasons[i] = result.state;
  }
  return { states, reasons };
}

export function pickabilityCacheKey(config: PickabilityWorkerConfig): string {
  const yawDeg = Math.round(config.yawRad * 180 / Math.PI);
  const dhHash = config.dhTable.map(({ a, alpha, d, theta, thetaMin, thetaMax }) => [a, alpha, d, theta, thetaMin, thetaMax].join(',')).join(';');
  const jointsHash = config.jointAngles.map((q) => q.toFixed(5)).join(',');
  const table = TABLE_PRESETS[config.preset],surface=config.surface;
  const environmentHash=(config.surfaces??[]).map((s)=>[s.id,s.shape,s.center.x,s.center.y,s.yaw,s.z,s.size.width,s.size.depth,s.size.innerRadius,s.size.outerRadius,s.support.type,s.support.radius,s.support.height,s.tray?.wallHeight,s.tray?.floorThickness].join(',')).join(';');
  return ['pickability-v3', config.mode??'pick',config.sourceSurfaceId??'',config.sourcePosition?`${config.sourcePosition.x},${config.sourcePosition.y},${config.sourcePosition.z}`:'',environmentHash,surface?.id??'table', surface?.shape??'rectangle',surface?.size.width??table.width,surface?.size.depth??table.depth,surface?.center.x??table.center.x,surface?.center.y??table.center.y,surface?.z??config.bookZ,surface?.yaw??0, dhHash, `tool=${TOOL_LENGTH}`, `book=${BOOK.size.x},${BOOK.size.y},${BOOK.size.z}`, `yaw=${yawDeg}`, `joints=${jointsHash}`, `z=${config.bookZ.toFixed(5)}`].join('|');
}

export function pickabilityConfigHash(config: PickabilityWorkerConfig): string {
  return pickabilityCacheKey(config);
}

const pointCache = new Map<string, { reachable: boolean; reason: string }>();
export function evaluatePickabilityCached(config: PickabilityWorkerConfig, x: number, y: number) {
  const key = `${pickabilityCacheKey(config)}|point=${x.toFixed(6)},${y.toFixed(6)}`;
  const cached = pointCache.get(key);
  if (cached) return cached;
  const cell = evaluatePickabilityState(config, x, y);
  const result = { reachable: cell.state === 2, reason: cell.reason };
  pointCache.set(key, result);
  if (pointCache.size > 4096) pointCache.delete(pointCache.keys().next().value as string);
  return result;
}
