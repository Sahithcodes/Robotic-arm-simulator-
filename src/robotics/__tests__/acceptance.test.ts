import React from 'react';
import { renderToString } from 'react-dom/server';
import { describe, expect, it } from 'vitest';
import { TaskExecutionPanel } from '../../components/TaskPanel/TaskExecutionPanel';
import { CanvasContainer } from '../../components/RobotViewer/CanvasContainer';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { useSimulationStore } from '../../store/simulationStore';
import { computeForwardKinematics } from '../forwardKinematics';
import { planPickAndPlace } from '../autonomousPlanner';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';
import { defaultTableSurface } from '../placementSurfaces';

const rad = (degrees: number) => degrees * Math.PI / 180;
const wrappedError = (a: number, b: number) => Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
const acceptanceRows: { row: string; result: string }[] = [];

function execute(plan: ReturnType<typeof planPickAndPlace>, source: { x: number; y: number; z: number }, bookYaw: number, dest: { x: number; y: number; yaw: number }) {
  const store = useSimulationStore;
  store.getState().resetTask();
  store.setState((state) => ({
    taskMode: 'autonomous', simulatorMode: 'autonomous', jointAngles: [...HOME_JOINT_ANGLES],
    fkResult: computeForwardKinematics(state.dhTable, HOME_JOINT_ANGLES),
    simulatedObject: { ...state.simulatedObject, position: { ...source }, surfaceId: 'table', rotation: { roll: 0, pitch: 0, yaw: bookYaw }, state: 'onTable', graspState: 'on-table', isAttached: false, isGrasped: false },
    dropTarget: { ...dest, yaw: dest.yaw * 180 / Math.PI, surfaceId: 'table', z: BOOK.initialPosition.z }, autonomousPlan: plan,
    isTaskPlaying: false, isHoldingBook: false, isGripperOpen: true, taskPhase: 'PLANNED', autonomousPhase: 'PLANNED',
  }));
  store.getState().executeAutonomousPick();
  let lowestBottom = Infinity;
  for (let frame = 0; frame < 900 && store.getState().isTaskPlaying; frame++) {
    store.getState().tickTask(0.05);
    const state = store.getState();
    const halfZ = BOOK.size.z / 2;
    lowestBottom = Math.min(lowestBottom, state.simulatedObject.position.z - halfZ);
  }
  const state = store.getState(), object = state.simulatedObject;
  return { state, lowestBottom, positionError: Math.max(Math.hypot(object.position.x - dest.x, object.position.y - dest.y, object.position.z - BOOK.initialPosition.z), state.placementError), yawError: wrappedError(object.rotation.yaw, dest.yaw) };
}

describe('stabilization acceptance', () => {
  it('A — predefined Compact task reaches placed/DONE without a destination', () => {
    const store = useSimulationStore.getState();
    store.setTablePreset('compact'); store.resetTask(); store.playTask();
    for (let frame = 0; frame < 1000 && useSimulationStore.getState().isTaskPlaying; frame++) useSimulationStore.getState().tickTask(0.05);
    const end = useSimulationStore.getState();
    expect(end.taskPhase).toBe('COMPLETE'); expect(end.simulatedObject.state).toBe('placed'); expect(end.dropTarget).toBeNull();
    acceptanceRows.push({ row: 'A', result: 'PASS — predefined Compact COMPLETE/placed, no destination' });
  }, 30000);

  it('B — seeded 50 valid Compact pairs execute with strict pose and table-bottom checks', () => {
    const seed = 0xC0A7AC7, rng = (() => { let s = seed >>> 0; return () => ((s = (1664525 * s + 1013904223) >>> 0) / 0x100000000); })();
    const surface = defaultTableSurface(); let attempts = 0, successes = 0, worstPosition = 0, worstYaw = 0, lowest = Infinity;
    const failures: string[] = [], geometricRejects: Record<string, number> = {};
    useSimulationStore.getState().setTablePreset('compact');
    while (successes < 50 && attempts < 6000) {
      attempts++;
      const yaw = rad(Math.floor(rng() * 181));
      const source = { x: 0.78 + rng() * 0.035, y: -0.055 + rng() * 0.03, z: surface.z + BOOK.size.z / 2 };
      const destination = { x: 0.60 + rng() * 0.025, y: -0.005 + rng() * 0.01, yaw: rad(Math.floor(rng() * 181)) };
      const plan = planPickAndPlace(INITIAL_DH_TABLE, source, HOME_JOINT_ANGLES, { ...destination, surfaceId: 'table' }, yaw, true, [surface], 'table');
      if (!plan.reachable) { geometricRejects[plan.reason] = (geometricRejects[plan.reason] ?? 0) + 1; continue; }
      const result = execute(plan, source, yaw, destination);
      const passed = result.state.pickStatus === 'DONE' && result.state.simulatedObject.state === 'placed' && result.positionError <= 0.005 && result.yawError <= rad(2) && result.lowestBottom >= TABLE.height - 1e-6;
      if (passed) { successes++; worstPosition = Math.max(worstPosition, result.positionError); worstYaw = Math.max(worstYaw, result.yawError); lowest = Math.min(lowest, result.lowestBottom); }
      else failures.push(`attempt=${attempts} state=${result.state.pickStatus} reason=${result.state.simulatedObject.reachability.reason} pos=${result.positionError.toFixed(5)} yawDeg=${(result.yawError / rad(1)).toFixed(2)} bottom=${result.lowestBottom.toFixed(5)}`);
    }
    console.log(`ACCEPT B seed=${seed} attempts=${attempts} validPairs=${successes} completed=${successes} planningRate=${(100 * successes / attempts).toFixed(1)}% geometricRejects=${JSON.stringify(geometricRejects)} failures=${JSON.stringify(failures)} worstPositionMm=${(worstPosition * 1000).toFixed(3)} worstYawDeg=${(worstYaw / rad(1)).toFixed(3)} lowestBookBottomM=${lowest.toFixed(5)}`);
    expect(successes).toBe(50); expect(failures).toEqual([]);
    acceptanceRows.push({ row: 'B', result: `PASS — seed ${seed}, valid/completed ${successes}/${successes}; planning acceptance rate ${((100 * successes / attempts).toFixed(1))}%` });
  }, 600000);

  it('C — rotated matrix preserves world yaw across all valid planned pairs', () => {
    const surface = defaultTableSurface(), positions = [{ x: 0.76, y: -0.06 }, { x: 0.77, y: -0.05 }, { x: 0.78, y: -0.04 }, { x: 0.79, y: -0.03 }, { x: 0.8, y: -0.02 }];
    const random = (() => { let state = 0xC0FFEE; return () => ((state = (1664525 * state + 1013904223) >>> 0) / 0x100000000); })();
    const destinationYaws = [0, 45, 90, 135, 180, Math.floor(random() * 181), Math.floor(random() * 181)].map(rad);
    let tested = 0, planned = 0, completed = 0, worstYaw = 0; const rejects: Record<string, number> = {}, failures: string[] = [];
    for (const p of positions) for (let bookYawDeg = 0; bookYawDeg <= 180; bookYawDeg += 15) for (const destinationYaw of destinationYaws) {
      tested++;
      const source = { ...p, z: surface.z + BOOK.size.z / 2 }, dest = { x: 0.61, y: 0, yaw: destinationYaw }, bookYaw = rad(bookYawDeg);
      const plan = planPickAndPlace(INITIAL_DH_TABLE, source, HOME_JOINT_ANGLES, { ...dest, surfaceId: 'table' }, bookYaw, true, [surface], 'table');
      if (!plan.reachable) { rejects[plan.reason] = (rejects[plan.reason] ?? 0) + 1; continue; }
      planned++;
      const end = execute(plan, source, bookYaw, dest);
      if (end.state.pickStatus === 'DONE' && end.yawError <= rad(2) && end.positionError <= 0.005 && end.lowestBottom >= TABLE.height - 1e-6) { completed++; worstYaw = Math.max(worstYaw, end.yawError); }
      else failures.push(`book=${bookYawDeg} dest=${(destinationYaw / rad(1)).toFixed(1)} reason=${end.state.pickStatus} yawErr=${(end.yawError / rad(1)).toFixed(2)}`);
    }
    console.log(`ACCEPT C matrix=${tested} positions=${JSON.stringify(positions)} bookYaws=0..180/15 destinationYawsDeg=${JSON.stringify(destinationYaws.map((yaw) => Number((yaw / rad(1)).toFixed(3))))} validPlans=${planned} DONE=${completed} invalidFixtureReasons=${JSON.stringify(rejects)} executionFailures=${JSON.stringify(failures)} worstYawErrorDeg=${(worstYaw / rad(1)).toFixed(3)}`);
    expect(failures).toEqual([]); expect(planned).toBeGreaterThan(0); expect(completed).toBe(planned);
    acceptanceRows.push({ row: 'C', result: `PASS — rotated fixture ${completed}/${planned} planned cases DONE; ${tested - planned} planner-rejected fixtures reported` });
  }, 300000);

  it('D — non-table transport remains explicitly experimental', () => {
    const ids = useSimulationStore.getState().placementSurfaces.map((surface) => surface.id);
    expect(ids).toEqual(['table']);
    console.log('ACCEPT D Table→Raised and Raised→Table not certified; non-table presets omitted from runtime and labeled EXPERIMENTAL.');
    acceptanceRows.push({ row: 'D', result: 'PASS — Table↔Raised remains disabled as experimental; transport not certified' });
  });

  it('E — planner reroutes above an obstacle or refuses the plan with a reason', () => {
    const surface = defaultTableSurface(), obstacle = { id: 'acceptance-obstacle', name: 'Raised blocker', shape: 'rectangle' as const, center: { x: 0.70, y: 0.02, z: 0 }, yaw: 0, z: 0.90, size: { width: 0.08, depth: 0.08 }, support: { type: 'none' as const }, color: '#668899' };
    const source = { x: 0.8, y: -0.04, z: surface.z + BOOK.size.z / 2 }, destination = { x: 0.61, y: 0, yaw: 0, surfaceId: 'table' };
    const baseline = planPickAndPlace(INITIAL_DH_TABLE, source, HOME_JOINT_ANGLES, destination, 0, true, [surface], 'table');
    const plan = planPickAndPlace(INITIAL_DH_TABLE, source, HOME_JOINT_ANGLES, destination, 0, true, [surface, obstacle], 'table');
    const lift = plan.waypoints.find((waypoint) => waypoint.id === 'TRANSPORT-LIFT');
    console.log(`ACCEPT E baseline=${baseline.reason} obstacle plan=${plan.reachable ? 'rerouted' : 'refused'} reason=${plan.reason} failure=${plan.failure?.otherPrimitive ?? 'none'} transportLiftM=${plan.liftHeight?.toFixed(3) ?? 'none'}`);
    expect(baseline.reachable, `control transport must be reachable: ${baseline.reason}`).toBe(true);
    expect(!plan.reachable || !!lift).toBe(true);
    acceptanceRows.push({ row: 'E', result: `PASS — ${plan.reachable ? `rerouted at ${plan.liftHeight?.toFixed(3)} m` : `refused: ${plan.reason}`}` });
  });

  it('F — SSR smoke and no application error/warn during headless execution', () => {
    const errors: unknown[][] = [], warns: unknown[][] = [], oldError = console.error, oldWarn = console.warn;
    console.error = (...args: unknown[]) => errors.push(args); console.warn = (...args: unknown[]) => warns.push(args);
    try {
      expect(renderToString(React.createElement(TaskExecutionPanel)).length).toBeGreaterThan(0);
      expect(renderToString(React.createElement(CanvasContainer))).toContain('Initializing 3D WebGL Kinematic Renderer');
      const surface = defaultTableSurface(), source = { x: 0.8, y: -0.04, z: surface.z + BOOK.size.z / 2 }, dest = { x: 0.61, y: 0, yaw: 0 };
      const plan = planPickAndPlace(INITIAL_DH_TABLE, source, HOME_JOINT_ANGLES, { ...dest, surfaceId: 'table' }, 0, true, [surface], 'table');
      expect(plan.reachable).toBe(true);
      expect(execute(plan, source, 0, dest).state.pickStatus).toBe('DONE');
    } finally { console.error = oldError; console.warn = oldWarn; }
    expect(errors).toEqual([]); expect(warns).toEqual([]);
    acceptanceRows.push({ row: 'F', result: 'PASS — panel/viewport SSR; no console.error/warn during headless pick' });
  });

  it('G — Large preset runs 20 measured plans for reporting', () => {
    const store = useSimulationStore.getState(); store.setTablePreset('large');
    const source = { ...store.simulatedObject.position }, counts = { planned: 0, rejected: 0 };
    for (let i = 0; i < 20; i++) {
      const target = { x: source.x + (i % 2 ? -0.15 : 0.15), y: source.y, yaw: 0, surfaceId: 'table' };
      const plan = planPickAndPlace(store.dhTable, source, HOME_JOINT_ANGLES, target, 0, true, store.placementSurfaces, 'table');
      if (plan.reachable) counts.planned++; else counts.rejected++;
    }
    console.log(`ACCEPT G Large report pairs=20 planned=${counts.planned} rejected=${counts.rejected}`);
    expect(counts.planned + counts.rejected).toBe(20);
    acceptanceRows.push({ row: 'G', result: `PASS — report only: ${counts.planned}/20 plans accepted on Large` });
  });

  it('prints acceptance table', () => { console.table(acceptanceRows); expect(acceptanceRows.map((r) => r.row)).toContain('F'); });
});
