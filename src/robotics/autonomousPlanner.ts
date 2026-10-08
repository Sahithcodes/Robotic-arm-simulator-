import { DHParameter, Matrix4x4, Vector3D } from '../types/robotics';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY, ROBOT_BASE_KEEP_OUT, ROBOT_MOUNT_HEIGHT, ROBOT_PEDESTAL, TCP_OFFSET } from '../robot/robotConfig';
import { computeForwardKinematics } from './forwardKinematics';
import { solveInverseKinematics, IKResult } from './inverseKinematics';
import { BOOK, GRIPPER, HOME_JOINT_ANGLES, TABLE } from './task5';
import { degToRad, rpyToMatrix } from './transforms';
import { smoothstep } from './task5';
import { PlacementSurface, defaultTableSurface, placementObstructionError, placementSurfaceError, surfaceBookZ } from './placementSurfaces';
import { getFingerBoxCorners } from './gripperGeometry';
export type PlannedWaypoint={id:string;position:Vector3D;jointAngles:number[];tiltDeg?:number};
export type PlanFailure={stage:string;kind:string;linkName:string|null;otherPrimitive:string|null;penetrationMm:number|null;jointAngles:number[]|null;tiltDeg:number|null;ikSeedUsed:number|null;sampleZMm?:number|null};
export type PickPlan={waypoints:PlannedWaypoint[];reachable:boolean;reason:string;pickableZone:Vector3D[];worstIkResidual:number;failure:PlanFailure|null;diagnostics:PlanFailure[];graspCandidate:string;gripWidth:number;liftHeight?:number};
export type PickabilityCell={position:Vector3D;fits:boolean;reachable:boolean;reason:string};
export type DropTarget={x:number;y:number;yaw:number;surfaceId?:string;z?:number};
export type GraspCandidate={name:string;toolYaw:number;width:number};
function canonicalBookYaw(yaw:number){return Math.atan2(Math.sin(2*yaw),Math.cos(2*yaw))/2;}
function wrapYaw(yaw:number){return Math.atan2(Math.sin(yaw),Math.cos(yaw));}
const MARGIN=.012, TABLE_MARGIN=.025;
// The finger tips stop 3 mm above the actual support top. On a 25 mm book
// resting there, this gives 22 mm of vertical overlap with its side faces.
// The TCP is the grasp datum at book center. At this height the modeled
// finger boxes clear the supporting surface; see heightCalibration.test.ts.
export const GRASP_TCP_SURFACE_CLEARANCE = BOOK.size.z / 2;
function surfaceTcpZ(surface:PlacementSurface){return surfaceBookZ(surface);}
function objectHalfExtents(yaw:number){const c=Math.abs(Math.cos(yaw)),s=Math.abs(Math.sin(yaw));return {x:c*BOOK.size.x/2+s*BOOK.size.y/2,y:s*BOOK.size.x/2+c*BOOK.size.y/2};}
function inBounds(p:Vector3D,yaw=0){const ext=objectHalfExtents(yaw);return Math.abs(p.x-TABLE.center.x)<=TABLE.width/2-ext.x-MARGIN+1e-9&&Math.abs(p.y-TABLE.center.y)<=TABLE.depth/2-ext.y-MARGIN+1e-9;}
export function isInsideBaseKeepOut(p:Vector3D){return Math.hypot(p.x-ROBOT_BASE_KEEP_OUT.center.x,p.y-ROBOT_BASE_KEEP_OUT.center.y)<ROBOT_BASE_KEEP_OUT.radius+Math.hypot(BOOK.size.x/2,BOOK.size.y/2);}
export function isValidObjectPosition(p:Vector3D,yaw=0){return inBounds(p,yaw)&&!isInsideBaseKeepOut(p)&&Math.abs(p.z-BOOK.initialPosition.z)<.015;}
export function validatePickSequence(dh:DHParameter[],position:Vector3D,current:number[],toolYaw:number,surfaces:PlacementSurface[]=[defaultTableSurface()],surfaceId='table'){
 let q=[...current],previous=[...current];
 const surface=surfaces.find((candidate)=>candidate.id===surfaceId)??defaultTableSurface();
 const grasp={...position,z:surfaceTcpZ(surface)};
 for(const target of [{p:{...grasp,z:grasp.z+.14},contact:false},{p:grasp,contact:true},{p:{...grasp,z:grasp.z+.18},contact:false}]){
  const solved=solvePose(dh,target.p,toolYaw,q,degToRad(30),0,surfaces,target.contact?surfaceId:undefined);if(!solved.ik)return {reachable:false,reason:solved.reason,diagnostics:solved.diagnostics};
  const collision=pathCollision(dh,previous,solved.ik.angles,target.contact,false,surfaces,target.contact?surfaceId:undefined);if(collision)return {reachable:false,reason:collision.kind,collision};
  q=solved.ik.angles;previous=q;
 }
 return {reachable:true,reason:'Reachable'};
}
function graspCandidates(bookYaw:number):GraspCandidate[] {
 const axisYaw=canonicalBookYaw(bookYaw),wideClearance=(GRIPPER.openWidth-GRIPPER.fingerThickness-BOOK.size.x)/2;
 return [{name:'narrow axis',toolYaw:axisYaw+Math.PI/2,width:BOOK.size.y},{name:'narrow axis, flipped wrist',toolYaw:axisYaw-Math.PI/2,width:BOOK.size.y},...(wideClearance>=.008?[{name:'wide axis',toolYaw:axisYaw,width:BOOK.size.x},{name:'wide axis, flipped wrist',toolYaw:axisYaw+Math.PI,width:BOOK.size.x}]:[])];
}
export function chooseGraspCandidate(dh:DHParameter[],position:Vector3D,current:number[],bookYaw:number,onSequenceValidation?:()=>void,surfaces:PlacementSurface[]=[defaultTableSurface()],surfaceId='table'):GraspCandidate|null {
 return graspCandidates(bookYaw).find((candidate)=>{onSequenceValidation?.();return validatePickSequence(dh,position,current,candidate.toolYaw,surfaces,surfaceId).reachable;})??null;
}
function toolRotation(yaw:number,tilt=0){return rpyToMatrix(Math.PI-tilt,0,yaw);}
function flangeTarget(tcp:Vector3D,yaw:number,tilt=0):Vector3D {const r=toolRotation(yaw,tilt);return {x:tcp.x-r[2]*TCP_OFFSET.z,y:tcp.y-r[6]*TCP_OFFSET.z,z:tcp.z-r[10]*TCP_OFFSET.z};}
function axisTilt(fk:ReturnType<typeof computeForwardKinematics>){const m=fk.endEffectorPose.rotationMatrix;return Math.acos(Math.max(-1,Math.min(1,-m[10])));}
const LINK_NAMES=['base-to-shoulder','upper-arm','forearm','wrist-1','wrist-2','tool-flange'];
type Collision={kind:string;linkName:string;otherPrimitive:string;penetrationMm:number;sampleZMm:number;jointAngles?:number[]};
function hitsPedestal(dh:DHParameter[],q:number[]):Collision|null{const p=computeForwardKinematics(dh,q).jointPositions;for(let i=2;i<p.length;i++){const a=p[i-1],b=p[i],radius=i===2?.045:i===3?.04:i===4?.03:.025;for(let s=0;s<=20;s++){const t=s/20,z=a.z+(b.z-a.z)*t;if(z<ROBOT_MOUNT_HEIGHT||z>ROBOT_PEDESTAL.collisionTop)continue;const x=a.x+(b.x-a.x)*t-ROBOT_BASE_KEEP_OUT.center.x,y=a.y+(b.y-a.y)*t-ROBOT_BASE_KEEP_OUT.center.y,dist=Math.hypot(x,y),penetration=ROBOT_PEDESTAL.columnRadius+radius-dist;if(penetration>0)return {kind:'link-vs-base collision',linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:'pedestal-column-cylinder (center=(0,0,z=590 mm); radius=95 mm; height=620 mm; z=280–900 mm)',penetrationMm:penetration*1000,sampleZMm:z*1000};}}return null;}
function tableClear(dh:DHParameter[],q:number[],allowContact=false,allowToolContact=false):Collision|null{const fk=computeForwardKinematics(dh,q),p=fk.jointPositions;const xmin=TABLE.center.x-TABLE.width/2,xmax=TABLE.center.x+TABLE.width/2,ymin=TABLE.center.y-TABLE.depth/2,ymax=TABLE.center.y+TABLE.depth/2,planeZ=TABLE.height+TABLE_MARGIN,primitiveName=`table-link-clearance-plane (size=${(TABLE.width*1000).toFixed(0)}x${(TABLE.depth*1000).toFixed(0)} mm; center=(${(TABLE.center.x*1000).toFixed(0)},${(TABLE.center.y*1000).toFixed(0)}) mm; z=${(planeZ*1000).toFixed(0)} mm; 25 mm link clearance above tabletop)`;
 for(let i=1;i<p.length;i++){const a=p[i-1],b=p[i];for(let s=0;s<=10;s++){const t=s/10,x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t,penetration=planeZ-z;if(x>=xmin&&x<=xmax&&y>=ymin&&y<=ymax&&penetration>0)return {kind:'collision with table',linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:primitiveName,penetrationMm:penetration*1000,sampleZMm:z*1000};}}
 // Sample the complete finger segment in its world orientation.
 if(!allowToolContact){const m=fk.endEffectorPose.rotationMatrix,toolPlaneZ=TABLE.height,toolPrimitive=`tabletop-surface (z=${(toolPlaneZ*1000).toFixed(0)} mm)`;for(let s=0;s<=12;s++){const z=TCP_OFFSET.z*s/12;const x=fk.endEffectorPose.position.x+m[2]*z,y=fk.endEffectorPose.position.y+m[6]*z,w=fk.endEffectorPose.position.z+m[10]*z,penetration=toolPlaneZ-w;if(x>=xmin&&x<=xmax&&y>=ymin&&y<=ymax&&penetration>0)return {kind:'collision with table',linkName:'tool/finger-axis centerline',otherPrimitive:toolPrimitive,penetrationMm:penetration*1000,sampleZMm:w*1000};}}
 return null;}
function extraSurfaceCollision(dh:DHParameter[],q:number[],surfaces:PlacementSurface[],allowedSurfaceId?:string):Collision|null {
 const fk=computeForwardKinematics(dh,q),points=fk.jointPositions,fingers=getFingerBoxCorners(fk,GRIPPER.openWidth);
 for(const surface of surfaces){if(surface.id==='table')continue;const allowTopContact=surface.id===allowedSurfaceId;
  const topZ=surface.z-.025;
  if(!allowTopContact)for(const {corners} of fingers){const xs=corners.map((p)=>p.x),ys=corners.map((p)=>p.y),zs=corners.map((p)=>p.z);if(Math.max(...xs)>=surface.center.x-surface.size.width/2&&Math.min(...xs)<=surface.center.x+surface.size.width/2&&Math.max(...ys)>=surface.center.y-surface.size.depth/2&&Math.min(...ys)<=surface.center.y+surface.size.depth/2&&Math.max(...zs)>topZ&&Math.min(...zs)<surface.z)return {kind:`collision with ${surface.name}`,linkName:'finger boxes',otherPrimitive:`${surface.name} top box`,penetrationMm:Math.max(0,(surface.z-Math.min(...zs))*1000),sampleZMm:Math.min(...zs)*1000};}
  for(let i=1;i<points.length;i++)for(let n=0;n<=10;n++){
   const t=n/10,p={x:points[i-1].x+(points[i].x-points[i-1].x)*t,y:points[i-1].y+(points[i].y-points[i-1].y)*t,z:points[i-1].z+(points[i].z-points[i-1].z)*t};
   if(!allowTopContact&&p.z<surface.z&&p.z>=surface.z-.025&&Math.abs(p.x-surface.center.x)<=surface.size.width/2+.03&&Math.abs(p.y-surface.center.y)<=surface.size.depth/2+.03) return {kind:surface.tray?'Tray wall blocks the approach':`collision with ${surface.name}`,linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:`${surface.name} top/support`,penetrationMm:(surface.z-p.z)*1000,sampleZMm:p.z*1000};
   if(surface.support.type==='column'&&surface.support.radius&&p.z<(surface.support.height??surface.z)&&Math.hypot(p.x-surface.center.x,p.y-surface.center.y)<surface.support.radius+.025)return {kind:`collision with ${surface.name} support`,linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:`${surface.name} support column`,penetrationMm:(surface.support.radius+.025-Math.hypot(p.x-surface.center.x,p.y-surface.center.y))*1000,sampleZMm:p.z*1000};
   if(surface.support.type==='legs'&&p.z<(surface.support.height??surface.z))for(const sx of [-1,1])for(const sy of [-1,1]){const x=surface.center.x+sx*(surface.size.width/2-.025),y=surface.center.y+sy*(surface.size.depth/2-.025);if(Math.abs(p.x-x)<(surface.support.width??.025)/2+.025&&Math.abs(p.y-y)<(surface.support.depth??.025)/2+.025)return {kind:`collision with ${surface.name} support`,linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:`${surface.name} support leg`,penetrationMm:25,sampleZMm:p.z*1000};}
   if(surface.tray&&p.z>=surface.z&&p.z<=surface.z+surface.tray.wallHeight&&Math.abs(p.x-surface.center.x)<=surface.size.width/2&&Math.abs(p.y-surface.center.y)<=surface.size.depth/2){const nearWall=Math.abs(Math.abs(p.x-surface.center.x)-surface.size.width/2)<.025||Math.abs(Math.abs(p.y-surface.center.y)-surface.size.depth/2)<.025;if(nearWall)return {kind:'Tray wall blocks the approach',linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:'Tray walls',penetrationMm:25,sampleZMm:p.z*1000};}
  }
 }
 return null;
}
function poseCollision(dh:DHParameter[],q:number[],allowContact=false,surfaces:PlacementSurface[]=[defaultTableSurface()],allowedSurfaceId?:string):Collision|null{const collision=hitsPedestal(dh,q)??tableClear(dh,q,allowContact,allowContact&&allowedSurfaceId==='table')??extraSurfaceCollision(dh,q,surfaces,allowedSurfaceId);return collision?{...collision,jointAngles:[...q]}:null;}
function pathCollision(dh:DHParameter[],a:number[],b:number[],contactFinal=false,contactStart=false,surfaces:PlacementSurface[]=[defaultTableSurface()],allowedSurfaceId?:string):Collision|null{for(let i=1;i<=28;i++){const t=i/28,q=a.map((x,j)=>x+(b[j]-x)*smoothstep(t)),collision=poseCollision(dh,q,false,surfaces,allowedSurfaceId);if(collision&&!(collision.linkName==='tool/finger-axis centerline'&&(contactFinal||contactStart)))return collision;}return null;}
function seeds(current:number[],target:Vector3D){const bearing=Math.atan2(target.y,target.x);return [current,HOME_JOINT_ANGLES,[bearing,-.61,1.13,0,-.52,0],[bearing,-1.05,2.1,0,-.6,0],[bearing,-1.25,-1.8,0,1.4,0],[bearing,-.7,1.4,2.8,.52,0]];}
function rot3(m:number[]){return [[m[0],m[1],m[2]],[m[4],m[5],m[6]],[m[8],m[9],m[10]]];}
function mul3(a:number[][],b:number[][]){return a.map((row,r)=>b[0].map((_,c)=>row.reduce((s,v,k)=>s+v*b[k][c],0)));}
export function analyticalIKSeeds(dh:DHParameter[],tcp:Vector3D,target:Matrix4x4,current:number[]):number[][]{
 const zAxis={x:target[2],y:target[6],z:target[10]},reach=PUMA_GEOMETRY.d6+TCP_OFFSET.z;
 const wc={x:tcp.x-zAxis.x*reach,y:tcp.y-zAxis.y*reach,z:tcp.z-zAxis.z*reach},shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1;
 const dx=wc.x-ROBOT_BASE_KEEP_OUT.center.x,dy=wc.y-ROBOT_BASE_KEEP_OUT.center.y,v=shoulderZ-wc.z,r=Math.hypot(dx,dy),bearing=Math.atan2(dy,dx),seedsOut:number[][]=[];
 for(const q1raw of [bearing,bearing+Math.PI]){
  const q1=Math.atan2(Math.sin(q1raw),Math.cos(q1raw)),u=dx*Math.cos(q1)+dy*Math.sin(q1),c3=(u*u+v*v-PUMA_GEOMETRY.l2**2-PUMA_GEOMETRY.l3**2)/(2*PUMA_GEOMETRY.l2*PUMA_GEOMETRY.l3);
  if(c3 < -1-1e-6 || c3 > 1+1e-6)continue;
  const elbow=Math.acos(Math.max(-1,Math.min(1,c3)));
  for(const q3 of [elbow,-elbow,elbow-2*Math.PI,-elbow+2*Math.PI]){
   const q2base=Math.atan2(v,u)-Math.atan2(PUMA_GEOMETRY.l3*Math.sin(q3),PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3*Math.cos(q3));
   for(const q2 of [q2base,q2base-2*Math.PI,q2base+2*Math.PI]){
   const q123=[q1,q2,q3];if(q123.some((q,i)=>q<degToRad(PUMA_GEOMETRY.jointLimits[i].min)-1e-6||q>degToRad(PUMA_GEOMETRY.jointLimits[i].max)+1e-6))continue;
   const fk03=computeForwardKinematics(dh,[...q123,0,0,0]),r03=rot3(fk03.cumulativeTransforms[2]),rt=rot3(target),r36=mul3(r03[0].map((_,i)=>r03.map(row=>row[i])),rt);
   const c5=Math.max(-1,Math.min(1,r36[2][2]));for(const q5 of [Math.acos(c5),-Math.acos(c5)]){
    const s5=Math.sin(q5);if(Math.abs(s5)<1e-7)continue;
    const q4=Math.atan2(-r36[1][2]/s5,-r36[0][2]/s5),q6=Math.atan2(-r36[2][1]/s5,r36[2][0]/s5);
    for(const d4 of [-2*Math.PI,0,2*Math.PI])for(const d6 of [-2*Math.PI,0,2*Math.PI]){
     const q=[q1,q2,q3,q4+d4,q5,q6+d6];if(q.every((a,i)=>a>=degToRad(PUMA_GEOMETRY.jointLimits[i].min)-1e-6&&a<=degToRad(PUMA_GEOMETRY.jointLimits[i].max)+1e-6))seedsOut.push(q);
    }
   }
   }
  }
 }
 return seedsOut.sort((a,b)=>a.reduce((s,q,i)=>s+(q-current[i])**2,0)-b.reduce((s,q,i)=>s+(q-current[i])**2,0));
}
function solvePose(dh:DHParameter[],tcp:Vector3D,yaw:number,current:number[],maxTilt=degToRad(5),minTilt=0,surfaces:PlacementSurface[]=[defaultTableSurface()],allowedSurfaceId?:string):{ik?:IKResult;tilt:number;ikSeedUsed?:number;reason:string;diagnostics:PlanFailure[]}{const shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1,diagnostics:PlanFailure[]=[];if(Math.hypot(tcp.x,tcp.y,tcp.z-shoulderZ)-TCP_OFFSET.z>PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3)return {tilt:0,reason:'outside workspace',diagnostics:[{stage:'IK',kind:'outside workspace',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:0,ikSeedUsed:null}]};let best:{ik:IKResult;tilt:number;cost:number;seed:number}|undefined;let positionCandidate=false,limitHit=false,tableCollision=false,baseCollision=false,surfaceCollision:Collision|null=null;const tiltStep=maxTilt<=degToRad(5)?degToRad(5):degToRad(10);for(let tilt=minTilt;tilt<=maxTilt+1e-6;tilt+=tiltStep){const goal=flangeTarget(tcp,yaw,tilt),orientation=toolRotation(yaw,tilt),toolZAxis={x:orientation[2],y:orientation[6],z:orientation[10]};const candidateSeeds=[...seeds(current,goal),...analyticalIKSeeds(dh,tcp,orientation,current)];for(let si=0;si<candidateSeeds.length;si++){const ik=solveInverseKinematics(dh,{position:goal,toolZAxis,yaw},{initialAngles:candidateSeeds[si],positionTolerance:.0015,orientationTolerance:degToRad(.5),maxIterations:150});limitHit ||= ik.hitLimit.some(Boolean);if(ik.residualPos<.008)positionCandidate=true;if(!ik.converged||ik.residualPos>.0015){diagnostics.push({stage:'IK',kind:ik.hitLimit.some(Boolean)?'joint limit':'IK non-convergence',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:[...ik.angles],tiltDeg:tilt*180/Math.PI,ikSeedUsed:si});continue;}const fk=computeForwardKinematics(dh,ik.angles);if(axisTilt(fk)>maxTilt){diagnostics.push({stage:'IK',kind:'orientation limit',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:[...ik.angles],tiltDeg:tilt*180/Math.PI,ikSeedUsed:si});continue;}const collision=poseCollision(dh,ik.angles,true,surfaces,allowedSurfaceId);if(collision){tableCollision ||= collision.kind==='collision with table';baseCollision ||= collision.kind==='link-vs-base collision';if(!tableCollision&&!baseCollision)surfaceCollision??=collision;diagnostics.push({stage:'POSE',kind:collision.kind,linkName:collision.linkName,otherPrimitive:collision.otherPrimitive,penetrationMm:collision.penetrationMm,sampleZMm:collision.sampleZMm,jointAngles:[...ik.angles],tiltDeg:tilt*180/Math.PI,ikSeedUsed:si});continue;}const cost=ik.angles.reduce((s,q,i)=>s+(q-current[i])**2,0)+tilt*tilt*4;if(!best||cost<best.cost)best={ik,tilt,cost,seed:si};if(si===0)return {ik,tilt,ikSeedUsed:si,reason:'Reachable',diagnostics};}if(best)break;}const reason=baseCollision?'link-vs-base collision':tableCollision?'collision with table':surfaceCollision?surfaceCollision.kind:limitHit?'joint limit':positionCandidate?'orientation unreachable':'outside workspace';return best?{ik:best.ik,tilt:best.tilt,ikSeedUsed:best.seed,reason:'Reachable',diagnostics}:{tilt:0,reason,diagnostics};}
function createPickPlanForCandidate(dh:DHParameter[],objectPosition:Vector3D,currentAngles:number[],destination:DropTarget|null|undefined,yaw:number,tiltEnabled:boolean,surfaces:PlacementSurface[],sourceSurfaceId:string,forcedCandidate?:GraspCandidate):PickPlan{
 const fail=(stage:string,kind:string,diagnostics:PlanFailure[]=[]):PickPlan=>{const expected=kind.includes('link-vs-base')?'link-vs-base collision':kind.includes('collision with table')?'collision with table':null;const chosen=(expected?[...diagnostics].reverse().find((d)=>d.kind===expected):undefined)??diagnostics.at(-1)??{stage,kind,linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:null,ikSeedUsed:null};return {waypoints:[],reachable:false,reason:kind==='Outside table bounds'?'Outside table bounds':kind==='Inside robot base keep-out'?'Inside robot base keep-out':kind,pickableZone:[],worstIkResidual:Infinity,failure:chosen,diagnostics,graspCandidate:'',gripWidth:BOOK.size.y};};
 const sourceSurface=surfaces.find((surface)=>surface.id===sourceSurfaceId)??surfaces.find((surface)=>surface.id==='table')!;
 const sourceError=placementSurfaceError(sourceSurface,objectPosition,yaw);
 if(sourceError)return fail('VALIDATION',sourceError);
 const sourceObstruction=placementObstructionError(sourceSurface,objectPosition,yaw,surfaces);
 if(sourceObstruction)return fail('VALIDATION',sourceObstruction);
 if(isInsideBaseKeepOut(objectPosition))return fail('VALIDATION','Inside robot base keep-out');
 if(!destination)return fail('DESTINATION','Set a destination first');
 const targetSurface=surfaces.find((surface)=>surface.id===(destination.surfaceId??'table'));
 if(!targetSurface)return fail('DESTINATION','Destination surface is unavailable');
 const place={x:destination.x,y:destination.y,z:destination.z??surfaceBookZ(targetSurface)};
 const destinationError=placementSurfaceError(targetSurface,place,destination.yaw);
 if(destinationError)return fail('DESTINATION',destinationError);
 const destinationObstruction=placementObstructionError(targetSurface,place,destination.yaw,surfaces);
 if(destinationObstruction)return fail('DESTINATION',destinationObstruction);
 if(isInsideBaseKeepOut(place))return fail('DESTINATION','Destination too close to the robot base');
 const separation=Math.hypot(place.x-objectPosition.x,place.y-objectPosition.y);
 const objectRadius=Math.hypot(BOOK.size.x/2,BOOK.size.y/2);
 if(separation<objectRadius*2+.02)return fail('DESTINATION','Destination overlaps the book');
 const shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1;
 const distanceToShoulder=Math.hypot(objectPosition.x-ROBOT_BASE_KEEP_OUT.center.x,objectPosition.y-ROBOT_BASE_KEEP_OUT.center.y,objectPosition.z-shoulderZ);
 if(distanceToShoulder-TCP_OFFSET.z>PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3)return fail('PRE-GRASP','outside workspace',[{stage:'PRE-GRASP',kind:'outside workspace',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:null,ikSeedUsed:null}]);
 const chosen=forcedCandidate??chooseGraspCandidate(dh,objectPosition,currentAngles,yaw,undefined,surfaces,sourceSurface.id);
 if(!chosen)return fail('GRASP','Book orientation not reachable here at any wrist angle');
 // Preserve the declared book-to-tool rotation from grasp through release.
 const relativeBookYaw=wrapYaw(yaw-chosen.toolYaw);
 const placeToolYaw=wrapYaw(destination.yaw-relativeBookYaw);
 const graspTcp={...objectPosition,z:surfaceTcpZ(sourceSurface)},placeTcp={...place,z:surfaceTcpZ(targetSurface)};
 const targets:{id:string;p:Vector3D;contact:boolean;yaw:number}[]=[{id:'PRE-GRASP',p:{...graspTcp,z:graspTcp.z+.14},contact:false,yaw:chosen.toolYaw},{id:'GRASP',p:graspTcp,contact:true,yaw:chosen.toolYaw},{id:'LIFT',p:{...graspTcp,z:graspTcp.z+.18},contact:false,yaw:chosen.toolYaw},{id:'PRE-PLACE',p:{...placeTcp,z:placeTcp.z+.14},contact:false,yaw:placeToolYaw},{id:'PLACE',p:placeTcp,contact:true,yaw:placeToolYaw},{id:'RETREAT',p:{...placeTcp,z:placeTcp.z+.20},contact:false,yaw:placeToolYaw}];
 const corridor={minX:Math.min(objectPosition.x,place.x),maxX:Math.max(objectPosition.x,place.x),minY:Math.min(objectPosition.y,place.y),maxY:Math.max(objectPosition.y,place.y)};
 const between=surfaces.filter((s)=>s.id!==sourceSurface.id&&s.id!==targetSurface.id&&s.center.x+s.size.width/2>=corridor.minX&&s.center.x-s.size.width/2<=corridor.maxX&&s.center.y+s.size.depth/2>=corridor.minY&&s.center.y-s.size.depth/2<=corridor.maxY);
 let liftHeight:number|undefined;
 if(between.length){liftHeight=Math.max(...between.map((s)=>s.z+(s.tray?.wallHeight??0)))+.065;targets.splice(3,0,{id:'TRANSPORT-LIFT',p:{x:place.x,y:place.y,z:liftHeight},contact:false,yaw:placeToolYaw});}
 let q=[...currentAngles],waypoints:PlannedWaypoint[]=[],worst=0,prev=[...currentAngles],graspTilt=0;const diagnostics:PlanFailure[]=[];
 for(const target of targets){const maxTilt=target.id==='PLACE'||target.id==='RETREAT'?graspTilt:degToRad(tiltEnabled?30:5);const minTilt=target.id==='PLACE'||target.id==='RETREAT'?graspTilt:0;const allowedSurface=target.id==='GRASP'?sourceSurface.id:target.id==='PLACE'?targetSurface.id:undefined;const solved=solvePose(dh,target.p,target.yaw,q,maxTilt,minTilt,surfaces,allowedSurface);diagnostics.push(...solved.diagnostics.map((d)=>({...d,stage:target.id})));if(!solved.ik){const why=target.id.includes('PLACE')||target.id==='RETREAT'?`${solved.reason==='outside workspace'?'Destination out of reach on '+targetSurface.name+' at this height':solved.reason} at ${target.id}`:`${solved.reason} at ${target.id}`;return fail(target.id,why,diagnostics);}const ik=solved.ik;if(target.id==='GRASP')graspTilt=solved.tilt;const flange=flangeTarget(target.p,target.yaw,solved.tilt);const collision=pathCollision(dh,prev,ik.angles,target.contact,target.id==='RETREAT',surfaces,allowedSurface);if(collision){const failure:PlanFailure={stage:target.id,kind:collision.kind,linkName:collision.linkName,otherPrimitive:collision.otherPrimitive,penetrationMm:collision.penetrationMm,sampleZMm:collision.sampleZMm,jointAngles:collision.jointAngles??[...ik.angles],tiltDeg:solved.tilt*180/Math.PI,ikSeedUsed:solved.ikSeedUsed??null};diagnostics.push(failure);return {...fail(target.id,`${collision.kind} on path to ${target.id}`,diagnostics),failure,worstIkResidual:Math.max(worst,ik.residualPos)};}waypoints.push({id:target.id,position:flange,jointAngles:ik.angles,tiltDeg:solved.tilt*180/Math.PI});q=ik.angles;prev=ik.angles;worst=Math.max(worst,ik.residualPos);}
 return {waypoints,reachable:true,reason:'Reachable',pickableZone:[],worstIkResidual:worst,failure:null,diagnostics,graspCandidate:chosen.name,gripWidth:chosen.width,liftHeight};
}
export function planPickAndPlace(dh:DHParameter[],objectPosition:Vector3D,currentAngles:number[],destination?:DropTarget|null,yaw=0,tiltEnabled=true,surfaces:PlacementSurface[]=[defaultTableSurface()],sourceSurfaceId='table'):PickPlan {
 const candidates=graspCandidates(yaw);
 let last:PickPlan|undefined;
 for(const candidate of candidates){const plan=createPickPlanForCandidate(dh,objectPosition,currentAngles,destination,yaw,tiltEnabled,surfaces,sourceSurfaceId,candidate);if(plan.reachable)return plan;last=plan;}
 return last??createPickPlanForCandidate(dh,objectPosition,currentAngles,destination,yaw,tiltEnabled,surfaces,sourceSurfaceId);
}
export const createPickPlan=planPickAndPlace;
export function clampObjectToTable(position:Vector3D,yaw=0):Vector3D{const ext=objectHalfExtents(yaw),x=TABLE.width/2-ext.x-MARGIN,y=TABLE.depth/2-ext.y-MARGIN;return {x:Math.max(TABLE.center.x-x,Math.min(TABLE.center.x+x,position.x)),y:Math.max(TABLE.center.y-y,Math.min(TABLE.center.y+y,position.y)),z:BOOK.initialPosition.z};}
export function findNearestPickablePosition(dh:DHParameter[],current:number[],position:Vector3D,bookYaw:number,destination?:DropTarget|null):Vector3D {
 const candidates:Vector3D[]=[];
 for(let x=TABLE.center.x-TABLE.width/2+.005;x<TABLE.center.x+TABLE.width/2;x+=.01)for(let y=TABLE.center.y-TABLE.depth/2+.005;y<TABLE.center.y+TABLE.depth/2;y+=.01){
  const p={x,y,z:BOOK.initialPosition.z};if(inBounds(p,bookYaw)&&!isInsideBaseKeepOut(p))candidates.push(p);
 }
 candidates.sort((a,b)=>Math.hypot(a.x-position.x,a.y-position.y)-Math.hypot(b.x-position.x,b.y-position.y));
 for(const p of candidates){
  if(destination?createPickPlan(dh,p,current,{...destination},bookYaw).reachable:!!chooseGraspCandidate(dh,p,current,bookYaw))return p;
 }
 return clampObjectToTable(position,bookYaw);
}
export function findNearestValidDestination(dh:DHParameter[],current:number[],bookPosition:Vector3D,bookYaw:number,requested:{x:number;y:number},destinationYaw:number):{x:number;y:number}|null {
 const candidates:Vector3D[]=[];
 for(let x=TABLE.center.x-TABLE.width/2+.005;x<TABLE.center.x+TABLE.width/2;x+=.01)for(let y=TABLE.center.y-TABLE.depth/2+.005;y<TABLE.center.y+TABLE.depth/2;y+=.01){
  const p={x,y,z:BOOK.initialPosition.z};if(inBounds(p,destinationYaw)&&!isInsideBaseKeepOut(p)&&Math.hypot(x-bookPosition.x,y-bookPosition.y)>=Math.hypot(BOOK.size.x/2,BOOK.size.y/2)*2+.02)candidates.push(p);
 }
 candidates.sort((a,b)=>Math.hypot(a.x-requested.x,a.y-requested.y)-Math.hypot(b.x-requested.x,b.y-requested.y));
 for(const p of candidates)if(createPickPlan(dh,bookPosition,current,{x:p.x,y:p.y,yaw:destinationYaw},bookYaw).reachable)return {x:p.x,y:p.y};
 return null;
}
export function evaluatePickabilityCell(dh:DHParameter[],currentAngles:number[],position:Vector3D,yaw:number,destination?:DropTarget|null,onSequenceValidation?:()=>void,surface?:PlacementSurface,environmentSurfaces?:PlacementSurface[]):PickabilityCell{
 const fits=surface?placementSurfaceError(surface,position,yaw)===null:inBounds(position,yaw);
 if(!fits)return {position,fits:false,reachable:false,reason:surface?`Outside ${surface.name} bounds`:'Outside table bounds'};
 const activeSurfaces=environmentSurfaces??(surface?[surface]:[defaultTableSurface()]);
 const pick=chooseGraspCandidate(dh,position,currentAngles,yaw,onSequenceValidation,activeSurfaces,surface?.id??'table');
 if(!pick&&surface){const detail=validatePickSequence(dh,position,currentAngles,canonicalBookYaw(yaw)+Math.PI/2,activeSurfaces,surface.id);return {position,fits:true,reachable:false,reason:detail.reason};}
 return {position,fits:true,reachable:!!pick,reason:pick?'Reachable':'Book orientation not reachable here at any wrist angle'};
}
export function createPickabilityGrid(dh:DHParameter[],currentAngles:number[],yaw:number,step=.01,destination?:DropTarget|null):PickabilityCell[]{
 const cells:PickabilityCell[]=[];
 const minX=TABLE.center.x-TABLE.width/2,maxX=TABLE.center.x+TABLE.width/2;
 const minY=TABLE.center.y-TABLE.depth/2,maxY=TABLE.center.y+TABLE.depth/2;
 for(let x=minX+step/2;x<maxX;x+=step)for(let y=minY+step/2;y<maxY;y+=step){
  cells.push(evaluatePickabilityCell(dh,currentAngles,{x,y,z:BOOK.initialPosition.z},yaw,destination));
 }
 return cells;
}
export function validatePickApproach(dh:DHParameter[],position:Vector3D,currentAngles:number[],yaw:number){const grasp=solvePose(dh,position,yaw,currentAngles,degToRad(30));if(!grasp.ik)return {reachable:false,reason:grasp.reason,residualPos:Infinity};const prePosition={...position,z:position.z+.14};const pre=solvePose(dh,prePosition,yaw,grasp.ik.angles,degToRad(30));if(!pre.ik)return {reachable:false,reason:pre.reason,residualPos:grasp.ik.residualPos};const collision=pathCollision(dh,pre.ik.angles,grasp.ik.angles,true);return !collision?{reachable:true,reason:'Reachable',residualPos:Math.max(grasp.ik.residualPos,pre.ik.residualPos)}:{reachable:false,reason:collision.kind,residualPos:Math.max(grasp.ik.residualPos,pre.ik.residualPos)};}
export function findDefaultPickPosition(dh:DHParameter[],currentAngles:number[],yaw=0):Vector3D {
 const candidates:Vector3D[]=[];
 for(let x=TABLE.center.x-TABLE.width/2+.005;x<TABLE.center.x+TABLE.width/2;x+=.01)
  for(let y=TABLE.center.y-TABLE.depth/2+.005;y<TABLE.center.y+TABLE.depth/2;y+=.01){
   const p={x,y,z:BOOK.initialPosition.z};if(inBounds(p,yaw)&&!isInsideBaseKeepOut(p))candidates.push(p);
  }
 // Keep the reset pose within the measured full-sequence workspace. The old
 // farthest-from-table-center heuristic selected an edge cell where the
 // center-aligned TCP could not reach pre-grasp.
 candidates.sort((a,b)=>Math.hypot(a.x-BOOK.initialPosition.x,a.y-BOOK.initialPosition.y)-Math.hypot(b.x-BOOK.initialPosition.x,b.y-BOOK.initialPosition.y));
 return candidates.find((p)=>chooseGraspCandidate(dh,p,currentAngles,yaw)!==null)??BOOK.initialPosition;
}
export function computePickableZone(dh:DHParameter[],currentAngles:number[],yawSamples=[0],step=.08){const zone:Vector3D[]=[];const x0=TABLE.center.x-TABLE.width/2+BOOK.size.x/2+MARGIN,x1=TABLE.center.x+TABLE.width/2-BOOK.size.x/2-MARGIN,y0=TABLE.center.y-TABLE.depth/2+BOOK.size.y/2+MARGIN,y1=TABLE.center.y+TABLE.depth/2-BOOK.size.y/2-MARGIN;for(let x=x0;x<=x1+1e-6;x+=step)for(let y=y0;y<=y1+1e-6;y+=step){const p={x,y,z:BOOK.initialPosition.z};if(isInsideBaseKeepOut(p))continue;if(yawSamples.every(yaw=>solvePose(dh,p,yaw+(BOOK.size.y<BOOK.size.x?Math.PI/2:0),currentAngles,degToRad(30)).ik))zone.push(p);}return zone;}

Object.assign(BOOK.initialPosition, {x:0.80,y:-0.04,z:TABLE.height+BOOK.size.z/2});
