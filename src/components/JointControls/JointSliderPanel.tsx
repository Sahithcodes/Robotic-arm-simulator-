'use client';

import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';
import { PUMA_GEOMETRY } from '../../robot/robotConfig';
import { radToDeg } from '../../robotics/transforms';

const names = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'];

export const JointSliderPanel: React.FC = () => {
  const { jointAngles, setJointAngleDegrees, resetToHome } = useSimulationStore();

  return (
    <section className="control-panel">
      <div className="panel-heading">
        <h2>MANUAL JOINT CONTROL</h2>
        <button type="button" onClick={resetToHome}>
          ZERO ALL
        </button>
      </div>
      <div className="units-note">DISPLAY: DEGREES | INTERNAL: RADIANS</div>
      <div className="joint-limits-note">SIMULATION JOINT LIMITS | CONFIGURABLE IN robotConfig</div>
      <div className="joint-list">
        {jointAngles.map((angle, index) => {
          const limits = PUMA_GEOMETRY.jointLimits[index];
          const degrees = radToDeg(angle);
          return (
            <div className="joint-row" key={names[index]}>
              <div className="joint-title">
                <span>J{index + 1}</span>
                <span>{names[index]}</span>
                <output>{degrees.toFixed(1)}deg</output>
              </div>
              <input
                type="range"
                min={limits.min}
                max={limits.max}
                step="0.1"
                value={degrees}
                onChange={(event) => setJointAngleDegrees(index, Number(event.target.value))}
              />
              <div className="joint-limits">
                <span>MIN {limits.min}deg</span>
                <span>MAX {limits.max}deg</span>
              </div>
            </div>
          );
        })}
      </div>
      <div className="state-note">KINEMATIC STATE: SYNCHRONIZED</div>
    </section>
  );
};
