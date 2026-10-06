import { Matrix4x4, Vector3D, EulerAngles } from '../types/robotics';

/**
 * Returns a 4x4 Identity Matrix.
 */
export function identity4(): Matrix4x4 {
  return [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
}

/**
 * Multiplies two 4x4 matrices in row-major order: C = A * B.
 */
export function multiply4x4(a: Matrix4x4, b: Matrix4x4): Matrix4x4 {
  const result: Matrix4x4 = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0];
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) {
        sum += a[r * 4 + k] * b[k * 4 + c];
      }
      result[r * 4 + c] = sanitizeNumber(sum);
    }
  }
  return result;
}

/**
 * 4x4 Rotation Matrix about X-axis by angle (radians).
 */
export function rotX(angle: number): Matrix4x4 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
    1,  0,  0, 0,
    0,  c, -s, 0,
    0,  s,  c, 0,
    0,  0,  0, 1,
  ];
}

/**
 * 4x4 Rotation Matrix about Y-axis by angle (radians).
 */
export function rotY(angle: number): Matrix4x4 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
     c, 0, s, 0,
     0, 1, 0, 0,
    -s, 0, c, 0,
     0, 0, 0, 1,
  ];
}

/**
 * 4x4 Rotation Matrix about Z-axis by angle (radians).
 */
export function rotZ(angle: number): Matrix4x4 {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
    c, -s, 0, 0,
    s,  c, 0, 0,
    0,  0, 1, 0,
    0,  0, 0, 1,
  ];
}

/**
 * 4x4 Translation Matrix for vector (dx, dy, dz).
 */
export function translation3D(dx: number, dy: number, dz: number): Matrix4x4 {
  return [
    1, 0, 0, dx,
    0, 1, 0, dy,
    0, 0, 1, dz,
    0, 0, 0, 1,
  ];
}

/**
 * Converts Roll-Pitch-Yaw angles (extrinsic Z-Y-X / intrinsic X-Y-Z) to 4x4 homogeneous rotation matrix.
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
export function rpyToMatrix(roll: number, pitch: number, yaw: number): Matrix4x4 {
  const Rz = rotZ(yaw);
  const Ry = rotY(pitch);
  const Rx = rotX(roll);
  return multiply4x4(Rz, multiply4x4(Ry, Rx));
}

/**
 * Extracts Roll, Pitch, Yaw angles (radians) from a 4x4 homogeneous transformation matrix.
 * Uses robust atan2 formulation handling gimbal lock situations.
 */
export function matrixToRPY(m: Matrix4x4): EulerAngles {
  // Extract rotation submatrix elements
  const r11 = m[0], r12 = m[1], r13 = m[2];
  const r21 = m[4], r22 = m[5], r23 = m[6];
  const r31 = m[8], r32 = m[9], r33 = m[10];

  let roll: number;
  let pitch: number;
  let yaw: number;

  // Check for pitch gimbal lock at +/- 90 degrees
  const sy = Math.sqrt(r11 * r11 + r21 * r21);

  if (sy > 1e-6) {
    roll = Math.atan2(r32, r33);
    pitch = Math.atan2(-r31, sy);
    yaw = Math.atan2(r21, r11);
  } else {
    // Gimbal lock case (pitch = +/- 90 deg)
    roll = Math.atan2(-r23, r22);
    pitch = Math.atan2(-r31, sy);
    yaw = 0;
  }

  return {
    roll: sanitizeNumber(roll),
    pitch: sanitizeNumber(pitch),
    yaw: sanitizeNumber(yaw),
  };
}

/**
 * Extracts the 3D Cartesian position vector [x, y, z] from a 4x4 homogeneous transformation matrix.
 */
export function extractTranslation(m: Matrix4x4): Vector3D {
  return {
    x: sanitizeNumber(m[3]),
    y: sanitizeNumber(m[7]),
    z: sanitizeNumber(m[11]),
  };
}

/**
 * Ensures numbers are finite, not NaN/Infinity, and eliminates -0.
 */
export function sanitizeNumber(val: number): number {
  if (!Number.isFinite(val) || Number.isNaN(val)) {
    return 0;
  }
  const clean = Math.abs(val) < 1e-12 ? 0 : val;
  return Object.is(clean, -0) ? 0 : clean;
}

/**
 * Converts radians to degrees.
 */
export function radToDeg(rad: number): number {
  return sanitizeNumber(rad * (180 / Math.PI));
}

/**
 * Converts degrees to radians.
 */
export function degToRad(deg: number): number {
  return sanitizeNumber(deg * (Math.PI / 180));
}
