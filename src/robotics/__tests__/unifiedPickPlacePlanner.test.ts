import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { evaluatePickabilityState } from '../pickabilityGrid';
import { planPickAndPlace } from '../autonomousPlanner';
import { BOOK, HOME_JOINT_ANGLES } from '../task5';
import { defaultTableSurface, surfaceBookZ } from '../placementSurfaces';

describe('unified pick and place planning', () => {
  it('uses the same complete candidate plan for the place overlay and execution plan', () => {
    const surface = defaultTableSurface();
    const surfaces = [surface];
    const source = { x: 0.8, y: -0.04, z: surfaceBookZ(surface) };
    const destination = { x: 0.61, y: 0, yaw: 0, surfaceId: surface.id, z: surfaceBookZ(surface) };
    const config = { preset: 'compact' as const, dhTable: INITIAL_DH_TABLE, jointAngles: HOME_JOINT_ANGLES, yawRad: 0, bookZ: source.z, surface, mode: 'place' as const, sourcePosition: source, sourceSurfaceId: surface.id, surfaces };
    const overlay = evaluatePickabilityState(config, destination.x, destination.y);
    const plan = planPickAndPlace(INITIAL_DH_TABLE, source, HOME_JOINT_ANGLES, destination, 0, true, surfaces, surface.id);
    console.log(`UNIFIED PLAN overlay=${overlay.reason} candidate=${plan.graspCandidate} waypoints=${plan.waypoints.map((waypoint) => waypoint.id).join(',')} residual=${(plan.worstIkResidual * 1000).toFixed(3)} mm`);
    expect(overlay.state).toBe(2);
    expect(plan.reachable, plan.reason).toBe(true);
    expect(plan.waypoints.map((waypoint) => waypoint.id)).toEqual(['PRE-GRASP', 'GRASP', 'LIFT', 'PRE-PLACE', 'PLACE', 'RETREAT']);
  });
});
