import { beforeEach, describe, expect, it } from 'vitest';
import { useSimulationStore } from '../src/store/simulationStore';

describe('autonomous pick execution', () => {
  beforeEach(() => { useSimulationStore.getState().setTablePreset('compact'); useSimulationStore.getState().resetTask(); useSimulationStore.getState().setDropTarget({x:.61,y:0,yaw:0},true); });

  it('keeps the object at its table position until spatially verified grasp', () => {
    const store = useSimulationStore.getState();
    const pickup = { ...store.simulatedObject.position };
    store.planAutonomousPick();
    expect(useSimulationStore.getState().autonomousPlan?.reachable).toBe(true);
    useSimulationStore.getState().executeAutonomousPick();

    for (let i = 0; i < 160 && useSimulationStore.getState().autonomousWaypointIndex < 1; i += 1) {
      useSimulationStore.getState().tickTask(0.05);
    }
    let state = useSimulationStore.getState();
    expect(state.simulatedObject.position).toEqual(pickup);
    expect(state.simulatedObject.isAttached).toBe(false);

    for (let i = 0; i < 100 && !useSimulationStore.getState().simulatedObject.isAttached; i += 1) {
      useSimulationStore.getState().tickTask(0.05);
    }
    state = useSimulationStore.getState();
    expect(state.taskPhase, `phase=${state.taskPhase}, waypoint=${state.autonomousWaypointIndex}, error=${state.positionError}, reason=${state.simulatedObject.reachability.reason}`).toBe('ATTACHED');
    expect(state.simulatedObject.isGrasped).toBe(true);
    expect(state.simulatedObject.isAttached).toBe(true);
    expect(state.simulatedObject.position).toEqual(pickup);
  });

  it('carries the same object and leaves it at the target after release', () => {
    const store = useSimulationStore.getState();
    store.planAutonomousPick();
    store.executeAutonomousPick();
    for (let i = 0; i < 500 && useSimulationStore.getState().isTaskPlaying; i += 1) {
      useSimulationStore.getState().tickTask(0.05);
    }

    const state = useSimulationStore.getState();
    expect(state.taskPhase, `waypoint=${state.autonomousWaypointIndex}, error=${state.positionError}, placement=${state.placementError}`).toBe('COMPLETE');
    expect(state.simulatedObject.isAttached).toBe(false);
    expect(state.simulatedObject.isGrasped).toBe(false);
    expect(state.dropTarget).toBeDefined();
    expect(state.simulatedObject.position.x).toBeCloseTo(state.dropTarget!.x, 2);
    expect(state.simulatedObject.position.y).toBeCloseTo(state.dropTarget!.y, 2);
  });

  it('replans and completes autonomous pickup at every reachable point in the 3x3 table grid', () => {
    const points = [
      { x: 0.78, y: -0.08 }, { x: 0.78, y: -0.04 }, { x: 0.78, y: 0 },
      { x: 0.79, y: -0.08 }, { x: 0.79, y: -0.04 }, { x: 0.79, y: 0 },
      { x: 0.80, y: -0.08 }, { x: 0.80, y: -0.04 }, { x: 0.80, y: 0 },
    ];
    for (const point of points) {
      useSimulationStore.getState().setBookPosition({ ...point, z: useSimulationStore.getState().simulatedObject.position.z });
      useSimulationStore.getState().setDropTarget({x:.61,y:0,yaw:0});
      useSimulationStore.getState().checkObjectReachability();
      expect(useSimulationStore.getState().autonomousPlan?.reachable, `plan failed at ${point.x}, ${point.y}`).toBe(true);
      useSimulationStore.getState().executeAutonomousPick();
      for (let frame = 0; frame < 400 && useSimulationStore.getState().isTaskPlaying; frame += 1) {
        useSimulationStore.getState().tickTask(0.05);
      }
      const result = useSimulationStore.getState();
      expect(result.taskPhase, `execution failed at ${point.x}, ${point.y}`).toBe('COMPLETE');
      expect(result.simulatedObject.isAttached).toBe(false);
      expect(result.dropTarget).toBeDefined();
      expect(result.simulatedObject.position.x).toBeCloseTo(result.dropTarget!.x, 2);
      expect(result.simulatedObject.position.y).toBeCloseTo(result.dropTarget!.y, 2);
    }
  });
});
