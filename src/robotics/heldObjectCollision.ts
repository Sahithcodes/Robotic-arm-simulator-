import { BOOK, TABLE } from './task5';
import { ROBOT_BASE_KEEP_OUT, ROBOT_MOUNT_HEIGHT, ROBOT_PEDESTAL } from '../robot/robotConfig';
import { EulerAngles, Vector3D } from '../types/robotics';
import { rpyToMatrix } from './transforms';

export type HeldObjectCollision = { kind: 'held-book/table' | 'held-book/base'; penetrationM: number };

export function heldBookCollision(position: Vector3D, rotation: EulerAngles): HeldObjectCollision | null {
  const m = rpyToMatrix(rotation.roll, rotation.pitch, rotation.yaw);
  const axes = [{x:m[0],y:m[4],z:m[8]},{x:m[1],y:m[5],z:m[9]},{x:m[2],y:m[6],z:m[10]}];
  const half = [BOOK.size.x/2, BOOK.size.y/2, BOOK.size.z/2];
  const corners: Vector3D[] = [];
  for (const sx of [-1,1]) for (const sy of [-1,1]) for (const sz of [-1,1]) {
    corners.push({x:position.x+axes[0].x*half[0]*sx+axes[1].x*half[1]*sy+axes[2].x*half[2]*sz,y:position.y+axes[0].y*half[0]*sx+axes[1].y*half[1]*sy+axes[2].y*half[2]*sz,z:position.z+axes[0].z*half[0]*sx+axes[1].z*half[1]*sy+axes[2].z*half[2]*sz});
  }
  const minZ=Math.min(...corners.map(p=>p.z)), maxZ=Math.max(...corners.map(p=>p.z));
  const minX=Math.min(...corners.map(p=>p.x)),maxX=Math.max(...corners.map(p=>p.x)),minY=Math.min(...corners.map(p=>p.y)),maxY=Math.max(...corners.map(p=>p.y));
  const tableOverlap=maxX>=TABLE.center.x-TABLE.width/2&&minX<=TABLE.center.x+TABLE.width/2&&maxY>=TABLE.center.y-TABLE.depth/2&&minY<=TABLE.center.y+TABLE.depth/2;
  if(tableOverlap&&minZ<TABLE.height)return {kind:'held-book/table',penetrationM:TABLE.height-minZ};
  if(maxZ>=ROBOT_MOUNT_HEIGHT&&minZ<=ROBOT_PEDESTAL.collisionTop){
    const localX=position.x-ROBOT_BASE_KEEP_OUT.center.x,localY=position.y-ROBOT_BASE_KEEP_OUT.center.y;
    const projectedX=Math.abs(localX*axes[0].x+localY*axes[0].y),projectedY=Math.abs(localX*axes[1].x+localY*axes[1].y);
    const dx=Math.max(0,projectedX-half[0]),dy=Math.max(0,projectedY-half[1]),penetration=ROBOT_PEDESTAL.columnRadius-Math.hypot(dx,dy);
    if(penetration>0)return {kind:'held-book/base',penetrationM:penetration};
  }
  return null;
}
