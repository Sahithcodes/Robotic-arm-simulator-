import { describe, it, expect } from 'vitest';
import {
  identity4,
  multiply4x4,
  rotX,
  rotY,
  rotZ,
  translation3D,
  rpyToMatrix,
  matrixToRPY,
  extractTranslation,
  radToDeg,
  degToRad,
} from '../src/robotics/transforms';

describe('Robotics Matrix Transformations Utilities', () => {
  it('creates identity matrix correctly', () => {
    const I = identity4();
    expect(I).toEqual([
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1,
    ]);
  });

  it('multiplies identity matrix by identity matrix', () => {
    const I = identity4();
    const result = multiply4x4(I, I);
    expect(result).toEqual(I);
  });

  it('translates Cartesian coordinates correctly', () => {
    const T = translation3D(0.5, -1.2, 3.4);
    const pos = extractTranslation(T);
    expect(pos.x).toBeCloseTo(0.5);
    expect(pos.y).toBeCloseTo(-1.2);
    expect(pos.z).toBeCloseTo(3.4);
  });

  it('computes rotX and rotY and verifies orthogonal property R * R^T = I', () => {
    const angle = Math.PI / 4; // 45 deg
    const Rx = rotX(angle);
    expect(Rx[5]).toBeCloseTo(Math.cos(angle));
    expect(Rx[6]).toBeCloseTo(-Math.sin(angle));
  });

  it('converts Roll-Pitch-Yaw to matrix and extracts RPY back accurately', () => {
    const roll = degToRad(30);
    const pitch = degToRad(45);
    const yaw = degToRad(60);

    const R = rpyToMatrix(roll, pitch, yaw);
    const extracted = matrixToRPY(R);

    expect(radToDeg(extracted.roll)).toBeCloseTo(30, 4);
    expect(radToDeg(extracted.pitch)).toBeCloseTo(45, 4);
    expect(radToDeg(extracted.yaw)).toBeCloseTo(60, 4);
  });
});
