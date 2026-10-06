import { describe, it, expect } from 'vitest';
import { computeDHMatrix } from '../src/robotics/dh';
import { DHParameter } from '../src/types/robotics';

describe('Standard Denavit-Hartenberg (DH) Transformation Matrix', () => {
  it('computes A_i matrix for pure translation link (a=0, alpha=0, d=0.7, theta=0)', () => {
    const param: DHParameter = {
      jointIndex: 1,
      name: 'Link 1 Test',
      a: 0,
      alpha: 0,
      d: 0.70,
      theta: 0,
      isVariable: true,
      thetaMin: -Math.PI,
      thetaMax: Math.PI,
    };

    const A = computeDHMatrix(param);
    expect(A[3]).toBeCloseTo(0);
    expect(A[7]).toBeCloseTo(0);
    expect(A[11]).toBeCloseTo(0.70);
  });

  it('computes A_i matrix with link length a=0.43 and twist alpha=0', () => {
    const param: DHParameter = {
      jointIndex: 2,
      name: 'Link 2 Test',
      a: 0.43,
      alpha: 0,
      d: 0,
      theta: 0,
      isVariable: true,
      thetaMin: -Math.PI,
      thetaMax: Math.PI,
    };

    const A = computeDHMatrix(param);
    expect(A[0]).toBeCloseTo(1);
    expect(A[3]).toBeCloseTo(0.43);
    expect(A[7]).toBeCloseTo(0);
    expect(A[11]).toBeCloseTo(0);
  });
});
