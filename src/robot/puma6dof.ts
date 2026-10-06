import { DHParameter, FKResult, JointState } from '../types/robotics';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY } from './robotConfig';
import { computeForwardKinematics } from '../robotics/forwardKinematics';
import { degToRad, radToDeg } from '../robotics/transforms';

/**
 * PUMA 6-DOF Robot Kinematic Model Wrapper
 */
export class PUMA6DOF {
  private dhTable: DHParameter[];
  private jointAngles: number[];

  constructor(initialAngles?: number[]) {
    this.dhTable = JSON.parse(JSON.stringify(INITIAL_DH_TABLE));
    this.jointAngles = initialAngles && initialAngles.length === 6 
      ? [...initialAngles] 
      : [0, 0, 0, 0, 0, 0];
  }

  public getDHTable(): DHParameter[] {
    return this.dhTable;
  }

  public getJointAngles(): number[] {
    return [...this.jointAngles];
  }

  public setJointAngle(index: number, angleInRadians: number): FKResult {
    if (index >= 0 && index < 6) {
      const limit = PUMA_GEOMETRY.jointLimits[index];
      const minRad = degToRad(limit.min);
      const maxRad = degToRad(limit.max);
      
      // Clamp to joint limits
      const clamped = Math.max(minRad, Math.min(maxRad, angleInRadians));
      this.jointAngles[index] = clamped;
    }
    return this.getFKResult();
  }

  public setJointAngles(anglesInRadians: number[]): FKResult {
    if (anglesInRadians.length === 6) {
      for (let i = 0; i < 6; i++) {
        this.setJointAngle(i, anglesInRadians[i]);
      }
    }
    return this.getFKResult();
  }

  public getFKResult(): FKResult {
    return computeForwardKinematics(this.dhTable, this.jointAngles);
  }

  public getJointStates(): JointState[] {
    return this.dhTable.map((dh, idx) => ({
      index: idx + 1,
      name: dh.name,
      angle: this.jointAngles[idx],
      velocity: 0,
      acceleration: 0,
      minLimit: dh.thetaMin,
      maxLimit: dh.thetaMax,
    }));
  }
}
