import { TABLE, BOOK } from './task5';
import { ROBOT_BASE_KEEP_OUT } from '../robot/robotConfig';
import { Vector3D } from '../types/robotics';

export type PlacementSurfaceShape = 'rectangle' | 'annular-sector';
export type PlacementSurface = {
  id: string;
  name: string;
  shape: PlacementSurfaceShape;
  center: Vector3D;
  yaw: number;
  z: number;
  size: { width: number; depth: number; innerRadius?: number; outerRadius?: number; startAngle?: number; endAngle?: number };
  support: { type: 'legs' | 'column' | 'none'; width?: number; depth?: number; radius?: number; height?: number };
  color: string;
  tray?: { wallHeight: number; floorThickness: number };
};

export function defaultPlacementSurfaces(): PlacementSurface[] {
  return [
    { id:'table', name:'Table', shape:'rectangle', center:{...TABLE.center}, yaw:0, z:TABLE.height, size:{width:TABLE.width,depth:TABLE.depth}, support:{type:'legs',width:.026,depth:.026,height:TABLE.height-.045}, color:'#3b4246' },
    { id:'raised-platform', name:'Raised platform', shape:'rectangle', center:{x:.71,y:.30,z:0}, yaw:0, z:TABLE.height+.15, size:{width:.22,depth:.18}, support:{type:'column',radius:.035,height:TABLE.height+.105}, color:'#667f8c' },
    { id:'lower-platform', name:'Lower platform', shape:'rectangle', center:{x:.36,y:-.22,z:0}, yaw:0, z:TABLE.height-.15, size:{width:.22,depth:.18}, support:{type:'column',radius:.035,height:TABLE.height-.195}, color:'#78856c' },
    { id:'shelf', name:'Shelf', shape:'rectangle', center:{x:.30,y:-.55,z:0}, yaw:0, z:1.05, size:{width:.30,depth:.20}, support:{type:'legs',width:.025,depth:.025,height:1.005}, color:'#8d745d' },
    { id:'tray', name:'Tray on table', shape:'rectangle', center:{x:.61,y:-.09,z:0}, yaw:0, z:TABLE.height, size:{width:.14,depth:.10}, support:{type:'none'}, color:'#638477', tray:{wallHeight:.04,floorThickness:.004} },
    { id:'floor', name:'Floor', shape:'rectangle', center:{x:.71,y:-.40,z:0}, yaw:0, z:0, size:{width:.34,depth:.20}, support:{type:'none'}, color:'#777b81' },
  ];
}

export function defaultTableSurface():PlacementSurface {
  return {id:'table',name:'Table',shape:'rectangle',center:{...TABLE.center},yaw:0,z:TABLE.height,size:{width:TABLE.width,depth:TABLE.depth},support:{type:'legs',width:.026,depth:.026,height:TABLE.height-.045},color:'#3b4246'};
}

export function surfaceBookZ(surface: PlacementSurface): number {
  if(surface.id==='table') return surface.z + BOOK.size.z / 2;
  return surface.z + (surface.tray?.floorThickness ?? 0) + BOOK.size.z / 2;
}

export function pointOnSurface(surface: PlacementSurface, p: {x:number;y:number}, margin=0): boolean {
  const dx=p.x-surface.center.x,dy=p.y-surface.center.y,c=Math.cos(surface.yaw),s=Math.sin(surface.yaw);
  const x=Math.abs(dx*c+dy*s),y=Math.abs(-dx*s+dy*c);
  if(surface.shape==='annular-sector') { const r=Math.hypot(dx,dy),angle=(Math.atan2(dy,dx)-surface.yaw+Math.PI*2)%(Math.PI*2),start=((surface.size.startAngle??0)+Math.PI*2)%(Math.PI*2),span=((surface.size.endAngle??Math.PI*2)-(surface.size.startAngle??0)+Math.PI*2)%(Math.PI*2)||Math.PI*2; return r>=((surface.size.innerRadius??0)+margin)&&r<=((surface.size.outerRadius??0)-margin)&&((angle-start+Math.PI*2)%(Math.PI*2))<=span; }
  return x<=surface.size.width/2-margin+1e-9&&y<=surface.size.depth/2-margin+1e-9;
}

export function clampPointToSurface(surface:PlacementSurface,p:{x:number;y:number},yaw=0):{x:number;y:number;z:number} {
  const relativeYaw=yaw-surface.yaw;
  const ex=Math.abs(Math.cos(relativeYaw))*BOOK.size.x/2+Math.abs(Math.sin(relativeYaw))*BOOK.size.y/2+.012;
  const ey=Math.abs(Math.sin(relativeYaw))*BOOK.size.x/2+Math.abs(Math.cos(relativeYaw))*BOOK.size.y/2+.012;
  const dx=p.x-surface.center.x,dy=p.y-surface.center.y,c=Math.cos(surface.yaw),s=Math.sin(surface.yaw);
  if(surface.shape==='annular-sector'){
    const minR=(surface.size.innerRadius??0)+Math.max(ex,ey),maxR=(surface.size.outerRadius??0)-Math.max(ex,ey),rawR=Math.hypot(dx,dy),start=surface.size.startAngle??0,end=surface.size.endAngle??Math.PI*2,angle=Math.atan2(dy,dx)-surface.yaw,normalized=(angle+Math.PI*2)%(Math.PI*2),span=(end-start+Math.PI*2)%(Math.PI*2)||Math.PI*2,local=Math.max(0,Math.min(span,((normalized-start+Math.PI*2)%(Math.PI*2)))),a=surface.yaw+start+local,r=Math.max(minR,Math.min(maxR,rawR));
    return {x:surface.center.x+r*Math.cos(a),y:surface.center.y+r*Math.sin(a),z:surfaceBookZ(surface)};
  }
  const lx=dx*c+dy*s,ly=-dx*s+dy*c;
  const x=Math.max(-surface.size.width/2+ex,Math.min(surface.size.width/2-ex,lx));
  const y=Math.max(-surface.size.depth/2+ey,Math.min(surface.size.depth/2-ey,ly));
  return {x:surface.center.x+x*c-y*s,y:surface.center.y+x*s+y*c,z:surfaceBookZ(surface)};
}

export function rayToNearestSurface(ray:{origin:Vector3D;direction:Vector3D},surfaces:PlacementSurface[],yaw=0):{point:{x:number;y:number;z:number};surfaceId:string}|null {
  let nearest=Infinity,result:{point:{x:number;y:number;z:number};surfaceId:string}|null=null;
  for(const surface of surfaces){if(Math.abs(ray.direction.z)<1e-9)continue;const t=(surface.z-ray.origin.z)/ray.direction.z;if(t<0||t>nearest+1e-9)continue;const p={x:ray.origin.x+ray.direction.x*t,y:ray.origin.y+ray.direction.y*t};if(!pointOnSurface(surface,p))continue;nearest=t;result={point:clampPointToSurface(surface,p,yaw),surfaceId:surface.id};}
  return result;
}

export function placementSurfaceError(surface: PlacementSurface, position: {x:number;y:number}, yaw=0): string | null {
  const ex=Math.abs(Math.cos(yaw))*BOOK.size.x/2+Math.abs(Math.sin(yaw))*BOOK.size.y/2;
  const ey=Math.abs(Math.sin(yaw))*BOOK.size.x/2+Math.abs(Math.cos(yaw))*BOOK.size.y/2;
  const c=Math.cos(surface.yaw),s=Math.sin(surface.yaw),dx=position.x-surface.center.x,dy=position.y-surface.center.y;
  if(surface.shape==='annular-sector') return pointOnSurface(surface,position,Math.max(ex,ey)+.012)?null:`Book footprint is off ${surface.name}`;
  const wallInset=surface.tray?.wallHeight?0.008:0,placementMargin=surface.tray?0:.012;
  if(Math.abs(dx*c+dy*s)+ex>surface.size.width/2-placementMargin-wallInset+1e-9||Math.abs(-dx*s+dy*c)+ey>surface.size.depth/2-placementMargin-wallInset+1e-9) return `Book footprint is off ${surface.name}`;
  const radius=Math.hypot(BOOK.size.x/2,BOOK.size.y/2)+.012;
  if(Math.hypot(position.x-ROBOT_BASE_KEEP_OUT.center.x,position.y-ROBOT_BASE_KEEP_OUT.center.y)<ROBOT_BASE_KEEP_OUT.radius+radius) return `${surface.name} is inside the robot base keep-out`;
  return null;
}

export function placementObstructionError(selected:PlacementSurface,position:{x:number;y:number;z:number},yaw:number,surfaces:PlacementSurface[]):string|null {
  const ex=Math.abs(Math.cos(yaw))*BOOK.size.x/2+Math.abs(Math.sin(yaw))*BOOK.size.y/2,ey=Math.abs(Math.sin(yaw))*BOOK.size.x/2+Math.abs(Math.cos(yaw))*BOOK.size.y/2;
  const minX=position.x-ex,maxX=position.x+ex,minY=position.y-ey,maxY=position.y+ey,minZ=position.z-BOOK.size.z/2,maxZ=position.z+BOOK.size.z/2;
  for(const other of surfaces){if(other.id===selected.id)continue;
    const overlaps=maxX>other.center.x-other.size.width/2+1e-9&&minX<other.center.x+other.size.width/2-1e-9&&maxY>other.center.y-other.size.depth/2+1e-9&&minY<other.center.y+other.size.depth/2-1e-9;
    if(!overlaps)continue;
    if(other.tray&&minZ<other.z+other.tray.wallHeight&&maxZ>other.z)return `Tray wall blocks placement on ${selected.name}`;
    if(!other.tray&&minZ<other.z&&maxZ>other.z-.025)return `Book intersects ${other.name}`;
    if(other.support.type==='column'&&other.support.radius&&minZ<(other.support.height??other.z)&&maxZ>0&&Math.hypot(position.x-other.center.x,position.y-other.center.y)<other.support.radius+Math.hypot(ex,ey))return `Book intersects ${other.name} support`;
  }
  return null;
}

export function validateSurfaceSet(surfaces: PlacementSurface[]): string | null {
  for(let i=0;i<surfaces.length;i++) for(let j=i+1;j<surfaces.length;j++) {
    const a=surfaces[i],b=surfaces[j];
    if((a.tray && b.id==='table')||(b.tray && a.id==='table')) continue;
    if(a.shape==='rectangle'&&b.shape==='rectangle'&&Math.abs(a.center.x-b.center.x)<(a.size.width+b.size.width)/2&&Math.abs(a.center.y-b.center.y)<(a.size.depth+b.size.depth)/2) return `${a.name} overlaps ${b.name}`;
  }
  for(const s of surfaces) if(s.id!=='table'&&Math.hypot(s.center.x-ROBOT_BASE_KEEP_OUT.center.x,s.center.y-ROBOT_BASE_KEEP_OUT.center.y)<ROBOT_BASE_KEEP_OUT.radius+Math.hypot(s.size.width/2,s.size.depth/2)) return `${s.name} overlaps the robot base keep-out`;
  return null;
}
