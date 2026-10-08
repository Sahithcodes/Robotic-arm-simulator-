'use client';

import { useEffect, useRef, useState } from 'react';
import { shallow } from 'zustand/shallow';
import { useSimulationStore } from './simulationStore';

type SimulationState = ReturnType<typeof useSimulationStore.getState>;
export function useThrottledSimulationSelector<T>(selector: (state: SimulationState) => T, intervalMs = 100): T {
  const selectorRef = useRef(selector); selectorRef.current = selector;
  const [selected, setSelected] = useState(() => selector(useSimulationStore.getState()));
  useEffect(() => {
    let timer: ReturnType<typeof setTimeout> | null = null;
    const unsubscribe = useSimulationStore.subscribe(() => {
      if (timer) return;
      timer = setTimeout(() => {
        timer = null;
        const next = selectorRef.current(useSimulationStore.getState());
        setSelected((previous) => shallow(previous, next) ? previous : next);
      }, intervalMs);
    });
    return () => { unsubscribe(); if (timer) clearTimeout(timer); };
  }, [intervalMs]);
  return selected;
}
