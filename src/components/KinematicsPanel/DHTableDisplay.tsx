'use client';

import React from 'react';
import { useSimulationStore } from '../../store/simulationStore';
import { radToDeg } from '../../robotics/transforms';

export const DHTableDisplay: React.FC = () => {
  const { dhTable, jointAngles } = useSimulationStore();

  return (
    <section className="control-panel">
      <div className="panel-heading">
        <h2>STANDARD DENAVIT-HARTENBERG PARAMETERS</h2>
        <span className="technical-tag">A_i = Rz(theta) Tz(d) Tx(a) Rx(alpha)</span>
      </div>

      <div className="technical-table-wrap">
        <table className="technical-table">
          <thead>
            <tr>
              <th>Joint</th>
              <th>a_i m</th>
              <th>alpha_i</th>
              <th>d_i m</th>
              <th>theta_i</th>
            </tr>
          </thead>
          <tbody>
            {dhTable.map((row, index) => {
              const currentThetaRad = jointAngles[index];
              const currentThetaDeg = radToDeg(currentThetaRad);
              const alphaDeg = radToDeg(row.alpha);

              return (
                <tr key={row.jointIndex}>
                  <td>J{row.jointIndex}</td>
                  <td>{row.a.toFixed(3)}</td>
                  <td>
                    {row.alpha.toFixed(3)} rad / {alphaDeg.toFixed(1)}deg
                  </td>
                  <td>{row.d.toFixed(3)}</td>
                  <td className="table-active-value">
                    {currentThetaRad.toFixed(4)} rad / {currentThetaDeg.toFixed(1)}deg
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>

      <div className="state-note">Chang &amp; Park (2003): d1=0.70m, a2=0.43m, a3=0.37m, d6=0.10m</div>
    </section>
  );
};
