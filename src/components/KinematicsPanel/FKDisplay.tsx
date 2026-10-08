'use client';

import React from 'react';
import { radToDeg } from '../../robotics/transforms';
import { useThrottledSimulationSelector } from '../../store/useThrottledSimulationSelector';

export const FKDisplay: React.FC = () => {
  const fkResult = useThrottledSimulationSelector((s) => s.fkResult);
  const { position, orientation } = fkResult.endEffectorPose;

  const rollDeg = radToDeg(orientation.roll);
  const pitchDeg = radToDeg(orientation.pitch);
  const yawDeg = radToDeg(orientation.yaw);

  return (
    <section className="control-panel">
      <div className="panel-heading">
        <h2>END-EFFECTOR CARTESIAN POSE</h2>
        <span className="technical-tag">T_0^6 FK RESULT</span>
      </div>

      <div className="readout-group">
        <div className="readout-label">CARTESIAN POSITION / WORLD FRAME</div>
        <div className="readout-grid">
          <ValueCell label="X" value={position.x.toFixed(4)} unit="m" />
          <ValueCell label="Y" value={position.y.toFixed(4)} unit="m" />
          <ValueCell label="Z" value={position.z.toFixed(4)} unit="m" />
        </div>
      </div>

      <div className="readout-group">
        <div className="readout-label">ORIENTATION / ROLL-PITCH-YAW</div>
        <div className="readout-grid">
          <ValueCell label="ROLL" value={rollDeg.toFixed(2)} unit="deg" secondary={`${orientation.roll.toFixed(3)} rad`} />
          <ValueCell label="PITCH" value={pitchDeg.toFixed(2)} unit="deg" secondary={`${orientation.pitch.toFixed(3)} rad`} />
          <ValueCell label="YAW" value={yawDeg.toFixed(2)} unit="deg" secondary={`${orientation.yaw.toFixed(3)} rad`} />
        </div>
      </div>
    </section>
  );
};

const ValueCell: React.FC<{ label: string; value: string; unit: string; secondary?: string }> = ({
  label,
  value,
  unit,
  secondary,
}) => (
  <div className="readout-cell">
    <span>{label}</span>
    <b>{value}</b>
    <em>{secondary ?? unit}</em>
  </div>
);
