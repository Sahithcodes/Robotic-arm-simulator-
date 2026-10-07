'use client';

import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';

export const Header: React.FC = () => {
  const resetToHome = useSimulationStore((state) => state.resetToHome);
  const simulatorMode = useSimulationStore((state) => state.simulatorMode);

  return (
    <header className="app-header">
      <div>
        <h1>6-DOF ROBOTIC ARM SIMULATOR</h1>
        <p>Task-Oriented Design &middot; Chang &amp; Park (2003) &middot; PUMA-type serial manipulator</p>
      </div>
      <div className="header-actions">
        <span className="mode-readout">MODE: {simulatorMode.toUpperCase()}</span>
        <button type="button" onClick={resetToHome} className="technical-button">HOME POSE</button>
      </div>
    </header>
  );
};
