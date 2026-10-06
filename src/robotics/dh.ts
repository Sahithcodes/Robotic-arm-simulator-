import { DHParameter, Matrix4x4 } from '../types/robotics';
import { sanitizeNumber } from './transforms';

/**
 * Standard Denavit-Hartenberg Transformation Matrix A_i = T_{i-1}^i(a_i, alpha_i, d_i, theta_i).
 * 
 * Transformation steps according to standard DH convention (Denavit & Hartenberg 1955):
 * 1. Rotate about Z_{i-1} by theta_i
 * 2. Translate along Z_{i-1} by d_i
 * 3. Translate along X_i by a_i
 * 4. Rotate about X_i by alpha_i
 * 
 * Resulting Matrix A_i:
 * [ cos(theta)   -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)   a*cos(theta) ]
 * [ sin(theta)    cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   a*sin(theta) ]
 * [     0               sin(alpha)              cos(alpha)             d       ]
 * [     0                   0                       0                  1       ]
 */
export function computeDHMatrix(param: DHParameter, currentTheta?: number): Matrix4x4 {
  const theta = currentTheta !== undefined ? currentTheta : param.theta;
  const { a, alpha, d } = param;

  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);

  return [
    sanitizeNumber(ct), sanitizeNumber(-st * ca), sanitizeNumber(st * sa),  sanitizeNumber(a * ct),
    sanitizeNumber(st), sanitizeNumber(ct * ca),  sanitizeNumber(-ct * sa), sanitizeNumber(a * st),
    0,                  sanitizeNumber(sa),       sanitizeNumber(ca),       sanitizeNumber(d),
    0,                  0,                        0,                        1,
  ];
}

/**
 * Updates dynamic theta values in a DH parameter table.
 */
export function updateDHParametersWithJointAngles(
  dhTable: DHParameter[],
  jointAngles: number[]
): DHParameter[] {
  return dhTable.map((param, index) => {
    if (index < jointAngles.length && param.isVariable) {
      return {
        ...param,
        theta: jointAngles[index],
      };
    }
    return param;
  });
}
