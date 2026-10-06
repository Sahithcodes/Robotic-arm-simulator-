import { FKResult, Vector3D } from '../types/robotics';
import { BOOK, GRIPPER, TABLE } from './task5';

export type FingerBox = { corners: Vector3D[] };

/** Corners of the rendered finger boxes transformed by the complete T06 FK pose. */
export function getFingerBoxCorners(fk: FKResult, opening: number): FingerBox[] {
  const m = fk.endEffectorPose.rotationMatrix;
  const axes = [
    { x: m[0], y: m[4], z: m[8] },
    { x: m[1], y: m[5], z: m[9] },
    { x: m[2], y: m[6], z: m[10] },
  ];
  const boxes: FingerBox[] = [];
  for (const side of [-1, 1]) {
    const center = {
      x: fk.endEffectorPose.position.x + axes[0].x * side * opening / 2 + axes[2].x * GRIPPER.fingerLength / 2,
      y: fk.endEffectorPose.position.y + axes[0].y * side * opening / 2 + axes[2].y * GRIPPER.fingerLength / 2,
      z: fk.endEffectorPose.position.z + axes[0].z * side * opening / 2 + axes[2].z * GRIPPER.fingerLength / 2,
    };
    const half = [GRIPPER.fingerThickness / 2, 0.008, GRIPPER.fingerLength / 2];
    const corners: Vector3D[] = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
      corners.push({
        x: center.x + axes[0].x * half[0] * sx + axes[1].x * half[1] * sy + axes[2].x * half[2] * sz,
        y: center.y + axes[0].y * half[0] * sx + axes[1].y * half[1] * sy + axes[2].y * half[2] * sz,
        z: center.z + axes[0].z * half[0] * sx + axes[1].z * half[1] * sy + axes[2].z * half[2] * sz,
      });
    }
    boxes.push({ corners });
  }
  return boxes;
}

/** Conservative oriented-box/AABB test against the tabletop volume. */
export function fingerBoxesIntersectTable(boxes: FingerBox[]): boolean {
  const bounds = {
    minX: TABLE.center.x - TABLE.width / 2, maxX: TABLE.center.x + TABLE.width / 2,
    minY: TABLE.center.y - TABLE.depth / 2, maxY: TABLE.center.y + TABLE.depth / 2,
  };
  return boxes.some(({ corners }) => {
    const xs = corners.map((p) => p.x), ys = corners.map((p) => p.y), zs = corners.map((p) => p.z);
    const overlapsXY = Math.max(...xs) >= bounds.minX && Math.min(...xs) <= bounds.maxX
      && Math.max(...ys) >= bounds.minY && Math.min(...ys) <= bounds.maxY;
    return overlapsXY && Math.min(...zs) < TABLE.height - 1e-6;
  });
}

/** Clearance between the book's narrower face and each finger's inner face. */
export function fingerBookSideClearance(fk: FKResult, bookYaw: number, opening: number): number {
  const m = fk.endEffectorPose.rotationMatrix;
  const fingerAxis = { x: m[0], y: m[4] };
  const bookX = { x: Math.cos(bookYaw), y: Math.sin(bookYaw) };
  const bookY = { x: -Math.sin(bookYaw), y: Math.cos(bookYaw) };
  const bookHalfAlongGrip = Math.abs(bookX.x * fingerAxis.x + bookX.y * fingerAxis.y) * BOOK.size.x / 2
    + Math.abs(bookY.x * fingerAxis.x + bookY.y * fingerAxis.y) * BOOK.size.y / 2;
  return opening / 2 - GRIPPER.fingerThickness / 2 - bookHalfAlongGrip;
}
