'use client';

import React from 'react';
import { AppTab } from '../../types/robotics';
import { useSimulationStore } from '../../store/simulationStore';

const tabs: { id: AppTab; label: string; disabled?: boolean }[] = [
  { id: 'robot', label: 'ROBOT' },
  { id: 'kinematics', label: 'KINEMATICS' },
  { id: 'task', label: 'TASK 5' },
  { id: 'trajectory', label: 'TRAJECTORY', disabled: true },
  { id: 'execution', label: 'EXECUTION CHECK', disabled: true },
  { id: 'dynamics', label: 'DYNAMICS', disabled: true },
];

export const TabNavigation: React.FC = () => {
  const activeTab=useSimulationStore((s)=>s.activeTab),setActiveTab=useSimulationStore((s)=>s.setActiveTab);

  return (
    <div className="app-tabs">
      {tabs.map((tab) => {
        const isActive = activeTab === tab.id;
        return (
          <button
            key={tab.id}
            disabled={tab.disabled}
            onClick={() => !tab.disabled && setActiveTab(tab.id)}
            className={isActive ? 'active' : undefined}
          >
            <span>{tab.label}</span>
          </button>
        );
      })}
    </div>
  );
};
