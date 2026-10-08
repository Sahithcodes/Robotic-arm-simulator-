'use client';

import React, { useState } from 'react';
import { Matrix4x4 } from '../../types/robotics';
import { useThrottledSimulationSelector } from '../../store/useThrottledSimulationSelector';

export const MatrixDisplay: React.FC = () => {
  const fkResult = useThrottledSimulationSelector((s) => s.fkResult);
  const [selectedMatrix, setSelectedMatrix] = useState<string>('T06');

  const renderMatrix = (m: Matrix4x4) => {
    return (
      <div className="matrix-grid">
        {m.map((val, index) => {
          const isTranslation = index % 4 === 3;
          return (
            <div key={index} className={isTranslation ? 'matrix-cell translation' : 'matrix-cell'}>
              {val.toFixed(4)}
            </div>
          );
        })}
      </div>
    );
  };

  let displayMatrix: Matrix4x4 = fkResult.endEffectorPose.rotationMatrix;
  let title = 'Homogeneous Matrix T_0^6';

  if (selectedMatrix.startsWith('A')) {
    const index = parseInt(selectedMatrix.substring(1)) - 1;
    if (index >= 0 && index < fkResult.individualTransforms.length) {
      displayMatrix = fkResult.individualTransforms[index];
      title = `Individual Transformation Matrix A_${index + 1} (T_${index}^${index + 1})`;
    }
  } else if (selectedMatrix.startsWith('T0')) {
    const index = parseInt(selectedMatrix.substring(2)) - 1;
    if (index >= 0 && index < fkResult.cumulativeTransforms.length) {
      displayMatrix = fkResult.cumulativeTransforms[index];
      title = `Cumulative Transformation Matrix T_0^${index + 1}`;
    }
  }

  return (
    <section className="control-panel">
      <div className="panel-heading">
        <h2>KINEMATIC TRANSFORMATION MATRICES</h2>
      </div>

      <div className="matrix-selector">
        <button
          type="button"
          onClick={() => setSelectedMatrix('T06')}
          className={selectedMatrix === 'T06' ? 'active' : undefined}
        >
          T_0^6
        </button>
        {[1, 2, 3, 4, 5, 6].map((index) => (
          <button
            key={`A${index}`}
            type="button"
            onClick={() => setSelectedMatrix(`A${index}`)}
            className={selectedMatrix === `A${index}` ? 'active' : undefined}
          >
            A_{index}
          </button>
        ))}
      </div>

      <div className="readout-label">{title}</div>
      {renderMatrix(displayMatrix)}

      <div className="state-note">Columns 1-3: orientation submatrix | Column 4: translation [X, Y, Z, 1]^T</div>
    </section>
  );
};
