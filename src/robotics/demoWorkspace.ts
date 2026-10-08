import { DHParameter, Vector3D } from '../types/robotics';
import { HOME_JOINT_ANGLES } from './task5';
import { planPickAndPlace } from './autonomousPlanner';
import { PlacementSurface } from './placementSurfaces';

export type DemoWorkspace = {
  validCells: Vector3D[];
  bounds: { minX: number; maxX: number; minY: number; maxY: number } | null;
  largestRectangleCells: number;
  safeCells: Vector3D[];
};

/** Development/test sweep. Every included point is accepted by the complete pick/place planner. */
export function findDemoWorkspace(
  dh: DHParameter[], source: Vector3D, yaw: number, surface: PlacementSurface,
  options: { minX: number; maxX: number; minY: number; maxY: number; stepM?: number; safetyMarginM?: number },
): DemoWorkspace {
  const step = options.stepM ?? 0.01, margin = options.safetyMarginM ?? 0.02;
  const xs: number[] = [], ys: number[] = [];
  for(let x=options.minX;x<=options.maxX+1e-8;x+=step) xs.push(Number(x.toFixed(6)));
  for(let y=options.minY;y<=options.maxY+1e-8;y+=step) ys.push(Number(y.toFixed(6)));
  const valid = ys.map((y)=>xs.map((x)=>planPickAndPlace(dh,source,HOME_JOINT_ANGLES,{x,y,yaw:0,surfaceId:surface.id},yaw,true,[surface],surface.id).reachable));
  const validCells: Vector3D[]=[];
  for(let yi=0;yi<ys.length;yi++)for(let xi=0;xi<xs.length;xi++)if(valid[yi][xi])validCells.push({x:xs[xi],y:ys[yi],z:surface.z});
  let best={area:0,x0:0,x1:-1,y0:0,y1:-1};
  for(let x0=0;x0<xs.length;x0++)for(let x1=x0;x1<xs.length;x1++)for(let y0=0;y0<ys.length;y0++)for(let y1=y0;y1<ys.length;y1++){
    const area=(x1-x0+1)*(y1-y0+1);if(area<=best.area)continue;
    let all=true;for(let yi=y0;yi<=y1&&all;yi++)for(let xi=x0;xi<=x1;xi++)if(!valid[yi][xi]){all=false;break;}
    if(all)best={area,x0,x1,y0,y1};
  }
  if(!best.area)return {validCells,bounds:null,largestRectangleCells:0,safeCells:[]};
  const minX=xs[best.x0],maxX=xs[best.x1],minY=ys[best.y0],maxY=ys[best.y1];
  const safeCells=validCells.filter((p)=>p.x>=minX+margin-1e-8&&p.x<=maxX-margin+1e-8&&p.y>=minY+margin-1e-8&&p.y<=maxY-margin+1e-8);
  return {validCells,bounds:{minX,maxX,minY,maxY},largestRectangleCells:best.area,safeCells};
}
