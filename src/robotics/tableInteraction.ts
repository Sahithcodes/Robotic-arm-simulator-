import { Vector3D } from '../types/robotics';

export type Ray3 = { origin: Vector3D; direction: Vector3D };

export function rayToTableXY(ray: Ray3, tableZ: number): { x: number; y: number } | null {
  if (Math.abs(ray.direction.z) < 1e-9) return null;
  const distance = (tableZ - ray.origin.z) / ray.direction.z;
  if (distance < 0) return null;
  return {
    x: ray.origin.x + ray.direction.x * distance,
    y: ray.origin.y + ray.direction.y * distance,
  };
}
