import { describe, expect, it } from 'vitest';
import { useSimulationStore } from '../../store/simulationStore';

describe('scene surface defaults', () => {
  it('starts and resets to Table only so unverified supports cannot obstruct pickup', () => {
    const store = useSimulationStore.getState();
    store.setTablePreset('compact');
    const surfaces = useSimulationStore.getState().placementSurfaces;
    console.log(`SCENE DEFAULT surfaces=${surfaces.map((surface) => surface.name).join(',')} tableTop=${surfaces[0]?.z.toFixed(4)} m`);
    expect(surfaces.map((surface) => surface.id)).toEqual(['table']);
    expect(surfaces[0].z).toBe(0.78);
  });
});
