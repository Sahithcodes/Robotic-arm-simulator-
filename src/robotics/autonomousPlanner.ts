import { DHParameter, Vector3D } from '../types/robotics';
import { PUMA_GEOMETRY, ROBOT_BASE_KEEP_OUT, ROBOT_MOUNT_HEIGHT, ROBOT_PEDESTAL, TCP_OFFSET } from '../robot/robotConfig';
import { computeForwardKinematics } from './forwardKinematics';
import { solveInverseKinematics, IKResult } from './inverseKinematics';
import { BOOK, GRIPPER, HOME_JOINT_ANGLES, TABLE } from './task5';
import { degToRad, rpyToMatrix } from './transforms';
import { smoothstep } from './task5';
export type PlannedWaypoint={id:string;position:Vector3D;jointAngles:number[]};
export type PickPlan={waypoints:PlannedWaypoint[];reachable:boolean;reason:string;pickableZone:Vector3D[];worstIkResidual:number};
export type PickabilityCell={position:Vector3D;fits:boolean;reachable:boolean;reason:string};
export const DEFAULT_DROP_POSITION={x:0.72,y:0,z:BOOK.initialPosition.z};
const MARGIN=.012, TABLE_MARGIN=.025;
function objectHalfExtents(yaw:number){const c=Math.abs(Math.cos(yaw)),s=Math.abs(Math.sin(yaw));return {x:c*BOOK.size.x/2+s*BOOK.size.y/2,y:s*BOOK.size.x/2+c*BOOK.size.y/2};}
function inBounds(p:Vector3D,yaw=0){const ext=objectHalfExtents(yaw);return Math.abs(p.x-TABLE.center.x)<=TABLE.width/2-ext.x-MARGIN+1e-9&&Math.abs(p.y-TABLE.center.y)<=TABLE.depth/2-ext.y-MARGIN+1e-9;}
export function isInsideBaseKeepOut(p:Vector3D){return Math.hypot(p.x-ROBOT_BASE_KEEP_OUT.center.x,p.y-ROBOT_BASE_KEEP_OUT.center.y)<ROBOT_BASE_KEEP_OUT.radius+Math.hypot(BOOK.size.x/2,BOOK.size.y/2);}
export function isValidObjectPosition(p:Vector3D,yaw=0){return inBounds(p,yaw)&&!isInsideBaseKeepOut(p)&&Math.abs(p.z-BOOK.initialPosition.z)<.015;}
function toolRotation(yaw:number,tilt=0){return rpyToMatrix(Math.PI-tilt,0,yaw);}
function flangeTarget(tcp:Vector3D,yaw:number,tilt=0):Vector3D {const r=toolRotation(yaw,tilt);return {x:tcp.x-r[2]*TCP_OFFSET.z,y:tcp.y-r[6]*TCP_OFFSET.z,z:tcp.z-r[10]*TCP_OFFSET.z};}
function axisTilt(fk:ReturnType<typeof computeForwardKinematics>){const m=fk.endEffectorPose.rotationMatrix;return Math.acos(Math.max(-1,Math.min(1,-m[10])));}
function hitsPedestal(dh:DHParameter[],q:number[]){const p=computeForwardKinematics(dh,q).jointPositions;for(let i=2;i<p.length;i++){const a=p[i-1],b=p[i],radius=i===2?.045:i===3?.04:i===4?.03:.025;for(let s=0;s<=20;s++){const t=s/20,z=a.z+(b.z-a.z)*t;if(z<ROBOT_MOUNT_HEIGHT||z>ROBOT_PEDESTAL.collisionTop)continue;const x=a.x+(b.x-a.x)*t-ROBOT_BASE_KEEP_OUT.center.x,y=a.y+(b.y-a.y)*t-ROBOT_BASE_KEEP_OUT.center.y;if(Math.hypot(x,y)<ROBOT_PEDESTAL.columnRadius+radius)return true;}}return false;}
function tableClear(dh:DHParameter[],q:number[],allowContact=false){const fk=computeForwardKinematics(dh,q),p=fk.jointPositions;const xmin=TABLE.center.x-TABLE.width/2,xmax=TABLE.center.x+TABLE.width/2,ymin=TABLE.center.y-TABLE.depth/2,ymax=TABLE.center.y+TABLE.depth/2;
 for(let i=1;i<p.length;i++){const a=p[i-1],b=p[i];for(let s=0;s<=10;s++){const t=s/10,x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t;if(x>=xmin&&x<=xmax&&y>=ymin&&y<=ymax&&z<TABLE.height+TABLE_MARGIN)return false;}}
 // Sample the complete finger segment in its world orientation.
 const m=fk.endEffectorPose.rotationMatrix;for(let s=0;s<=12;s++){const z=TCP_OFFSET.z*s/12;const x=fk.endEffectorPose.position.x+m[2]*z,y=fk.endEffectorPose.position.y+m[6]*z,w=fk.endEffectorPose.position.z+m[10]*z;if(x>=xmin&&x<=xmax&&y>=ymin&&y<=ymax&&w<TABLE.height+(allowContact?-.005:TABLE_MARGIN))return false;}
 return true;}
function poseCollision(dh:DHParameter[],q:number[],allowContact=false):'link-vs-base collision'|'collision with table'|null{if(hitsPedestal(dh,q))return 'link-vs-base collision';return tableClear(dh,q,allowContact)?null:'collision with table';}
function pathCollision(dh:DHParameter[],a:number[],b:number[],contactFinal=false):'link-vs-base collision'|'collision with table'|null{for(let i=1;i<=28;i++){const t=i/28,q=a.map((x,j)=>x+(b[j]-x)*smoothstep(t)),collision=poseCollision(dh,q,contactFinal&&i===28);if(collision)return collision;}return null;}
function seeds(current:number[],target:Vector3D){const bearing=Math.atan2(target.y,target.x);return [current,HOME_JOINT_ANGLES,[bearing,-.61,1.13,0,-.52,0],[bearing,-1.05,2.1,0,-.6,0],[bearing,-1.25,-1.8,0,1.4,0],[bearing,-.7,1.4,2.8,.52,0]];}
function solvePose(dh:DHParameter[],tcp:Vector3D,yaw:number,current:number[],maxTilt=degToRad(5)):{ik?:IKResult;tilt:number;reason:string}{const shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1;if(Math.hypot(tcp.x,tcp.y,tcp.z-shoulderZ)-TCP_OFFSET.z>PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3)return {tilt:0,reason:'outside workspace'};let best:{ik:IKResult;tilt:number;cost:number}|undefined;let positionCandidate=false,limitHit=false,tableCollision=false,baseCollision=false;const tiltStep=maxTilt<=degToRad(5)?degToRad(5):degToRad(10);for(let tilt=0;tilt<=maxTilt+1e-6;tilt+=tiltStep){const goal=flangeTarget(tcp,yaw,tilt),orientation=toolRotation(yaw,tilt),toolZAxis={x:orientation[2],y:orientation[6],z:orientation[10]};const candidateSeeds=seeds(current,goal);for(let si=0;si<candidateSeeds.length;si++){const ik=solveInverseKinematics(dh,{position:goal,toolZAxis,yaw},{initialAngles:candidateSeeds[si],positionTolerance:.0015,orientationTolerance:degToRad(2),maxIterations:150});limitHit ||= ik.hitLimit.some(Boolean);if(ik.residualPos<.008)positionCandidate=true;if(!ik.converged||ik.residualPos>.0015)continue;const fk=computeForwardKinematics(dh,ik.angles);if(axisTilt(fk)>maxTilt)continue;const collision=poseCollision(dh,ik.angles,true);if(collision){tableCollision ||= collision==='collision with table';baseCollision ||= collision==='link-vs-base collision';continue;}const cost=ik.angles.reduce((s,q,i)=>s+(q-current[i])**2,0)+tilt*tilt*4;if(!best||cost<best.cost)best={ik,tilt,cost};if(si===0)return {ik,tilt,reason:'Reachable'};}if(best)break;}const reason=baseCollision?'link-vs-base collision':tableCollision?'collision with table':limitHit?'joint limit':positionCandidate?'orientation unreachable':'outside workspace';return best?{ik:best.ik,tilt:best.tilt,reason:'Reachable'}:{tilt:0,reason};}
export function createPickPlan(dh:DHParameter[],objectPosition:Vector3D,currentAngles:number[],destination=DEFAULT_DROP_POSITION,yaw=0):PickPlan{
 if(!inBounds(objectPosition,yaw))return {waypoints:[],reachable:false,reason:'Outside table bounds',pickableZone:[],worstIkResidual:Infinity};
 if(isInsideBaseKeepOut(objectPosition))return {waypoints:[],reachable:false,reason:'Inside robot base keep-out',pickableZone:[],worstIkResidual:Infinity};
 if(!isValidObjectPosition(destination,yaw))return {waypoints:[],reachable:false,reason:'Invalid drop position',pickableZone:[],worstIkResidual:Infinity};
 const shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1;
 const distanceToShoulder=Math.hypot(objectPosition.x-ROBOT_BASE_KEEP_OUT.center.x,objectPosition.y-ROBOT_BASE_KEEP_OUT.center.y,objectPosition.z-shoulderZ);
 if(distanceToShoulder-TCP_OFFSET.z>PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3)return {waypoints:[],reachable:false,reason:'outside workspace',pickableZone:[],worstIkResidual:Infinity};
 const graspYaw=yaw+(BOOK.size.y<BOOK.size.x?Math.PI/2:0);
 const targets=[{id:'PRE-GRASP',p:{...objectPosition,z:objectPosition.z+.14},contact:false},{id:'GRASP',p:objectPosition,contact:true},{id:'LIFT',p:{...objectPosition,z:objectPosition.z+.18},contact:false},{id:'DROP-APPROACH',p:{...destination,z:destination.z+.14},contact:false},{id:'DROP',p:destination,contact:true},{id:'RETREAT',p:{...destination,z:destination.z+.18},contact:false}];
 let q=[...currentAngles],waypoints:PlannedWaypoint[]=[],worst=0,prev=[...currentAngles];
 for(const target of targets){const solved=solvePose(dh,target.p,graspYaw,q,degToRad(30));if(!solved.ik)return {waypoints:[],reachable:false,reason:`${solved.reason} at ${target.id}`,pickableZone:[],worstIkResidual:worst};const ik=solved.ik;const flange=flangeTarget(target.p,graspYaw,solved.tilt);const collision=pathCollision(dh,prev,ik.angles,target.contact);if(collision)return {waypoints:[],reachable:false,reason:`${collision} on path to ${target.id}`,pickableZone:[],worstIkResidual:Math.max(worst,ik.residualPos)};waypoints.push({id:target.id,position:flange,jointAngles:ik.angles});q=ik.angles;prev=ik.angles;worst=Math.max(worst,ik.residualPos);}
 return {waypoints,reachable:true,reason:'Reachable',pickableZone:[],worstIkResidual:worst};
}
export function clampObjectToTable(position:Vector3D,yaw=0):Vector3D{const ext=objectHalfExtents(yaw),x=TABLE.width/2-ext.x-MARGIN,y=TABLE.depth/2-ext.y-MARGIN;return {x:Math.max(TABLE.center.x-x,Math.min(TABLE.center.x+x,position.x)),y:Math.max(TABLE.center.y-y,Math.min(TABLE.center.y+y,position.y)),z:BOOK.initialPosition.z};}
export function evaluatePickabilityCell(dh:DHParameter[],currentAngles:number[],position:Vector3D,yaw:number):PickabilityCell{
 const fits=inBounds(position,yaw);
 if(!fits)return {position,fits:false,reachable:false,reason:'Outside table bounds'};
 const plan=createPickPlan(dh,position,currentAngles,DEFAULT_DROP_POSITION,yaw);
 return {position,fits:true,reachable:plan.reachable,reason:plan.reason};
}
export function createPickabilityGrid(dh:DHParameter[],currentAngles:number[],yaw:number,step=.01):PickabilityCell[]{
 const cells:PickabilityCell[]=[];
 const minX=TABLE.center.x-TABLE.width/2,maxX=TABLE.center.x+TABLE.width/2;
 const minY=TABLE.center.y-TABLE.depth/2,maxY=TABLE.center.y+TABLE.depth/2;
 for(let x=minX+step/2;x<maxX;x+=step)for(let y=minY+step/2;y<maxY;y+=step){
  cells.push(evaluatePickabilityCell(dh,currentAngles,{x,y,z:BOOK.initialPosition.z},yaw));
 }
 return cells;
}
export function validatePickApproach(dh:DHParameter[],position:Vector3D,currentAngles:number[],yaw:number){const grasp=solvePose(dh,position,yaw,currentAngles,degToRad(30));if(!grasp.ik)return {reachable:false,reason:grasp.reason,residualPos:Infinity};const prePosition={...position,z:position.z+.14};const pre=solvePose(dh,prePosition,yaw,grasp.ik.angles,degToRad(30));if(!pre.ik)return {reachable:false,reason:pre.reason,residualPos:grasp.ik.residualPos};const collision=pathCollision(dh,pre.ik.angles,grasp.ik.angles,true);return !collision?{reachable:true,reason:'Reachable',residualPos:Math.max(grasp.ik.residualPos,pre.ik.residualPos)}:{reachable:false,reason:collision,residualPos:Math.max(grasp.ik.residualPos,pre.ik.residualPos)};}
export function computePickableZone(dh:DHParameter[],currentAngles:number[],yawSamples=[0],step=.08){const zone:Vector3D[]=[];const x0=TABLE.center.x-TABLE.width/2+BOOK.size.x/2+MARGIN,x1=TABLE.center.x+TABLE.width/2-BOOK.size.x/2-MARGIN,y0=TABLE.center.y-TABLE.depth/2+BOOK.size.y/2+MARGIN,y1=TABLE.center.y+TABLE.depth/2-BOOK.size.y/2-MARGIN;for(let x=x0;x<=x1+1e-6;x+=step)for(let y=y0;y<=y1+1e-6;y+=step){const p={x,y,z:BOOK.initialPosition.z};if(isInsideBaseKeepOut(p))continue;if(yawSamples.every(yaw=>solvePose(dh,p,yaw+(BOOK.size.y<BOOK.size.x?Math.PI/2:0),currentAngles,degToRad(30)).ik))zone.push(p);}return zone;}
