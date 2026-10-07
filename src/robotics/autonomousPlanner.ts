import { DHParameter, Matrix4x4, Vector3D } from '../types/robotics';
import { INITIAL_DH_TABLE, PUMA_GEOMETRY, ROBOT_BASE_KEEP_OUT, ROBOT_MOUNT_HEIGHT, ROBOT_PEDESTAL, TCP_OFFSET } from '../robot/robotConfig';
import { computeForwardKinematics } from './forwardKinematics';
import { solveInverseKinematics, IKResult } from './inverseKinematics';
import { BOOK, GRIPPER, HOME_JOINT_ANGLES, TABLE } from './task5';
import { degToRad, rpyToMatrix } from './transforms';
import { smoothstep } from './task5';
export type PlannedWaypoint={id:string;position:Vector3D;jointAngles:number[];tiltDeg?:number};
export type PlanFailure={stage:string;kind:string;linkName:string|null;otherPrimitive:string|null;penetrationMm:number|null;jointAngles:number[]|null;tiltDeg:number|null;ikSeedUsed:number|null;sampleZMm?:number|null};
export type PickPlan={waypoints:PlannedWaypoint[];reachable:boolean;reason:string;pickableZone:Vector3D[];worstIkResidual:number;failure:PlanFailure|null;diagnostics:PlanFailure[];graspCandidate:string;gripWidth:number};
export type PickabilityCell={position:Vector3D;fits:boolean;reachable:boolean;reason:string};
export type DropTarget={x:number;y:number;yaw:number};
export type GraspCandidate={name:string;toolYaw:number;width:number};
function canonicalBookYaw(yaw:number){return Math.atan2(Math.sin(2*yaw),Math.cos(2*yaw))/2;}
const MARGIN=.012, TABLE_MARGIN=.025;
function objectHalfExtents(yaw:number){const c=Math.abs(Math.cos(yaw)),s=Math.abs(Math.sin(yaw));return {x:c*BOOK.size.x/2+s*BOOK.size.y/2,y:s*BOOK.size.x/2+c*BOOK.size.y/2};}
function inBounds(p:Vector3D,yaw=0){const ext=objectHalfExtents(yaw);return Math.abs(p.x-TABLE.center.x)<=TABLE.width/2-ext.x-MARGIN+1e-9&&Math.abs(p.y-TABLE.center.y)<=TABLE.depth/2-ext.y-MARGIN+1e-9;}
export function isInsideBaseKeepOut(p:Vector3D){return Math.hypot(p.x-ROBOT_BASE_KEEP_OUT.center.x,p.y-ROBOT_BASE_KEEP_OUT.center.y)<ROBOT_BASE_KEEP_OUT.radius+Math.hypot(BOOK.size.x/2,BOOK.size.y/2);}
export function isValidObjectPosition(p:Vector3D,yaw=0){return inBounds(p,yaw)&&!isInsideBaseKeepOut(p)&&Math.abs(p.z-BOOK.initialPosition.z)<.015;}
export function validatePickSequence(dh:DHParameter[],position:Vector3D,current:number[],toolYaw:number){
 let q=[...current],previous=[...current];
 for(const target of [{p:{...position,z:position.z+.14},contact:false},{p:position,contact:true},{p:{...position,z:position.z+.18},contact:false}]){
  const solved=solvePose(dh,target.p,toolYaw,q,degToRad(30));if(!solved.ik)return {reachable:false,reason:solved.reason};
  const collision=pathCollision(dh,previous,solved.ik.angles,target.contact);if(collision)return {reachable:false,reason:collision.kind};
  q=solved.ik.angles;previous=q;
 }
 return {reachable:true,reason:'Reachable'};
}
export function chooseGraspCandidate(dh:DHParameter[],position:Vector3D,current:number[],bookYaw:number):GraspCandidate|null {
 const axisYaw=canonicalBookYaw(bookYaw);
 const wideClearance=(GRIPPER.openWidth-GRIPPER.fingerThickness-BOOK.size.x)/2;
 const candidates:GraspCandidate[]=[
  {name:'narrow axis',toolYaw:axisYaw+Math.PI/2,width:BOOK.size.y},
  {name:'narrow axis, flipped wrist',toolYaw:axisYaw-Math.PI/2,width:BOOK.size.y},
  ...(wideClearance>=.008?[{name:'wide axis',toolYaw:axisYaw,width:BOOK.size.x},{name:'wide axis, flipped wrist',toolYaw:axisYaw+Math.PI,width:BOOK.size.x}]:[]),
 ];
 return candidates.find((candidate)=>validatePickSequence(dh,position,current,candidate.toolYaw).reachable)??null;
}
function toolRotation(yaw:number,tilt=0){return rpyToMatrix(Math.PI-tilt,0,yaw);}
function flangeTarget(tcp:Vector3D,yaw:number,tilt=0):Vector3D {const r=toolRotation(yaw,tilt);return {x:tcp.x-r[2]*TCP_OFFSET.z,y:tcp.y-r[6]*TCP_OFFSET.z,z:tcp.z-r[10]*TCP_OFFSET.z};}
function axisTilt(fk:ReturnType<typeof computeForwardKinematics>){const m=fk.endEffectorPose.rotationMatrix;return Math.acos(Math.max(-1,Math.min(1,-m[10])));}
const LINK_NAMES=['base-to-shoulder','upper-arm','forearm','wrist-1','wrist-2','tool-flange'];
type Collision={kind:'link-vs-base collision'|'collision with table';linkName:string;otherPrimitive:string;penetrationMm:number;sampleZMm:number;jointAngles?:number[]};
function hitsPedestal(dh:DHParameter[],q:number[]):Collision|null{const p=computeForwardKinematics(dh,q).jointPositions;for(let i=2;i<p.length;i++){const a=p[i-1],b=p[i],radius=i===2?.045:i===3?.04:i===4?.03:.025;for(let s=0;s<=20;s++){const t=s/20,z=a.z+(b.z-a.z)*t;if(z<ROBOT_MOUNT_HEIGHT||z>ROBOT_PEDESTAL.collisionTop)continue;const x=a.x+(b.x-a.x)*t-ROBOT_BASE_KEEP_OUT.center.x,y=a.y+(b.y-a.y)*t-ROBOT_BASE_KEEP_OUT.center.y,dist=Math.hypot(x,y),penetration=ROBOT_PEDESTAL.columnRadius+radius-dist;if(penetration>0)return {kind:'link-vs-base collision',linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:'pedestal-column-cylinder (center=(0,0,z=590 mm); radius=95 mm; height=620 mm; z=280–900 mm)',penetrationMm:penetration*1000,sampleZMm:z*1000};}}return null;}
function tableClear(dh:DHParameter[],q:number[],allowContact=false):Collision|null{const fk=computeForwardKinematics(dh,q),p=fk.jointPositions;const xmin=TABLE.center.x-TABLE.width/2,xmax=TABLE.center.x+TABLE.width/2,ymin=TABLE.center.y-TABLE.depth/2,ymax=TABLE.center.y+TABLE.depth/2,clearance=allowContact?-.005:TABLE_MARGIN,planeZ=TABLE.height+clearance,primitiveName=`table-top-clearance-plane (size=${(TABLE.width*1000).toFixed(0)}x${(TABLE.depth*1000).toFixed(0)} mm; center=(${(TABLE.center.x*1000).toFixed(0)},${(TABLE.center.y*1000).toFixed(0)}) mm; z=${(planeZ*1000).toFixed(0)} mm; ${(clearance*1000).toFixed(0)} mm offset from tabletop)`;
 for(let i=1;i<p.length;i++){const a=p[i-1],b=p[i];for(let s=0;s<=10;s++){const t=s/10,x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t,penetration=planeZ-z;if(x>=xmin&&x<=xmax&&y>=ymin&&y<=ymax&&penetration>0)return {kind:'collision with table',linkName:LINK_NAMES[i-1]??`link-${i}`,otherPrimitive:primitiveName,penetrationMm:penetration*1000,sampleZMm:z*1000};}}
 // Sample the complete finger segment in its world orientation.
 const m=fk.endEffectorPose.rotationMatrix;for(let s=0;s<=12;s++){const z=TCP_OFFSET.z*s/12;const x=fk.endEffectorPose.position.x+m[2]*z,y=fk.endEffectorPose.position.y+m[6]*z,w=fk.endEffectorPose.position.z+m[10]*z,penetration=planeZ-w;if(x>=xmin&&x<=xmax&&y>=ymin&&y<=ymax&&penetration>0)return {kind:'collision with table',linkName:'tool/finger-axis centerline',otherPrimitive:primitiveName,penetrationMm:penetration*1000,sampleZMm:w*1000};}
 return null;}
function poseCollision(dh:DHParameter[],q:number[],allowContact=false):Collision|null{const collision=hitsPedestal(dh,q)??tableClear(dh,q,allowContact);return collision?{...collision,jointAngles:[...q]}:null;}
function pathCollision(dh:DHParameter[],a:number[],b:number[],contactFinal=false,contactStart=false):Collision|null{for(let i=1;i<=28;i++){const t=i/28,q=a.map((x,j)=>x+(b[j]-x)*smoothstep(t)),collision=poseCollision(dh,q);if(collision&&!(collision.linkName==='tool/finger-axis centerline'&&(contactFinal||contactStart)))return collision;}return null;}
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
function solvePose(dh:DHParameter[],tcp:Vector3D,yaw:number,current:number[],maxTilt=degToRad(5),minTilt=0):{ik?:IKResult;tilt:number;ikSeedUsed?:number;reason:string;diagnostics:PlanFailure[]}{const shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1,diagnostics:PlanFailure[]=[];if(Math.hypot(tcp.x,tcp.y,tcp.z-shoulderZ)-TCP_OFFSET.z>PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3)return {tilt:0,reason:'outside workspace',diagnostics:[{stage:'IK',kind:'outside workspace',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:0,ikSeedUsed:null}]};let best:{ik:IKResult;tilt:number;cost:number;seed:number}|undefined;let positionCandidate=false,limitHit=false,tableCollision=false,baseCollision=false;const tiltStep=maxTilt<=degToRad(5)?degToRad(5):degToRad(10);for(let tilt=minTilt;tilt<=maxTilt+1e-6;tilt+=tiltStep){const goal=flangeTarget(tcp,yaw,tilt),orientation=toolRotation(yaw,tilt),toolZAxis={x:orientation[2],y:orientation[6],z:orientation[10]};const candidateSeeds=[...seeds(current,goal),...analyticalIKSeeds(dh,tcp,orientation,current)];for(let si=0;si<candidateSeeds.length;si++){const ik=solveInverseKinematics(dh,{position:goal,toolZAxis,yaw},{initialAngles:candidateSeeds[si],positionTolerance:.0015,orientationTolerance:degToRad(.5),maxIterations:150});limitHit ||= ik.hitLimit.some(Boolean);if(ik.residualPos<.008)positionCandidate=true;if(!ik.converged||ik.residualPos>.0015){diagnostics.push({stage:'IK',kind:ik.hitLimit.some(Boolean)?'joint limit':'IK non-convergence',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:[...ik.angles],tiltDeg:tilt*180/Math.PI,ikSeedUsed:si});continue;}const fk=computeForwardKinematics(dh,ik.angles);if(axisTilt(fk)>maxTilt){diagnostics.push({stage:'IK',kind:'orientation limit',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:[...ik.angles],tiltDeg:tilt*180/Math.PI,ikSeedUsed:si});continue;}const collision=poseCollision(dh,ik.angles,true);if(collision){tableCollision ||= collision.kind==='collision with table';baseCollision ||= collision.kind==='link-vs-base collision';diagnostics.push({stage:'POSE',kind:collision.kind,linkName:collision.linkName,otherPrimitive:collision.otherPrimitive,penetrationMm:collision.penetrationMm,sampleZMm:collision.sampleZMm,jointAngles:[...ik.angles],tiltDeg:tilt*180/Math.PI,ikSeedUsed:si});continue;}const cost=ik.angles.reduce((s,q,i)=>s+(q-current[i])**2,0)+tilt*tilt*4;if(!best||cost<best.cost)best={ik,tilt,cost,seed:si};if(si===0)return {ik,tilt,ikSeedUsed:si,reason:'Reachable',diagnostics};}if(best)break;}const reason=baseCollision?'link-vs-base collision':tableCollision?'collision with table':limitHit?'joint limit':positionCandidate?'orientation unreachable':'outside workspace';return best?{ik:best.ik,tilt:best.tilt,ikSeedUsed:best.seed,reason:'Reachable',diagnostics}:{tilt:0,reason,diagnostics};}
export function createPickPlan(dh:DHParameter[],objectPosition:Vector3D,currentAngles:number[],destination?:DropTarget|null,yaw=0,tiltEnabled=true):PickPlan{
 const fail=(stage:string,kind:string,diagnostics:PlanFailure[]=[]):PickPlan=>{const expected=kind.includes('link-vs-base')?'link-vs-base collision':kind.includes('collision with table')?'collision with table':null;const chosen=(expected?[...diagnostics].reverse().find((d)=>d.kind===expected):undefined)??diagnostics.at(-1)??{stage,kind,linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:null,ikSeedUsed:null};return {waypoints:[],reachable:false,reason:kind==='Outside table bounds'?'Outside table bounds':kind==='Inside robot base keep-out'?'Inside robot base keep-out':kind,pickableZone:[],worstIkResidual:Infinity,failure:chosen,diagnostics,graspCandidate:'',gripWidth:BOOK.size.y};};
 if(!inBounds(objectPosition,yaw))return fail('VALIDATION','Outside table bounds');
 if(isInsideBaseKeepOut(objectPosition))return fail('VALIDATION','Inside robot base keep-out');
 if(!destination)return fail('DESTINATION','Set a destination first');
 const place={x:destination.x,y:destination.y,z:TABLE.height+BOOK.size.z/2};
 if(!inBounds(place,destination.yaw))return fail('DESTINATION','Destination partly off the table');
 if(isInsideBaseKeepOut(place))return fail('DESTINATION','Destination too close to the robot base');
 const separation=Math.hypot(place.x-objectPosition.x,place.y-objectPosition.y);
 const objectRadius=Math.hypot(BOOK.size.x/2,BOOK.size.y/2);
 if(separation<objectRadius*2+.02)return fail('DESTINATION','Destination overlaps the book');
 const shoulderZ=ROBOT_MOUNT_HEIGHT+PUMA_GEOMETRY.d1;
 const distanceToShoulder=Math.hypot(objectPosition.x-ROBOT_BASE_KEEP_OUT.center.x,objectPosition.y-ROBOT_BASE_KEEP_OUT.center.y,objectPosition.z-shoulderZ);
 if(distanceToShoulder-TCP_OFFSET.z>PUMA_GEOMETRY.l2+PUMA_GEOMETRY.l3)return fail('PRE-GRASP','outside workspace',[{stage:'PRE-GRASP',kind:'outside workspace',linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:null,ikSeedUsed:null}]);
 const chosen=chooseGraspCandidate(dh,objectPosition,currentAngles,yaw);
 if(!chosen)return fail('GRASP','Book orientation not reachable here at any wrist angle');
 const placeToolYaw=destination.yaw+(chosen.toolYaw-canonicalBookYaw(yaw));
 const targets=[{id:'PRE-GRASP',p:{...objectPosition,z:objectPosition.z+.14},contact:false,yaw:chosen.toolYaw},{id:'GRASP',p:objectPosition,contact:true,yaw:chosen.toolYaw},{id:'LIFT',p:{...objectPosition,z:objectPosition.z+.18},contact:false,yaw:chosen.toolYaw},{id:'PRE-PLACE',p:{...place,z:place.z+.14},contact:false,yaw:placeToolYaw},{id:'PLACE',p:place,contact:true,yaw:placeToolYaw},{id:'RETREAT',p:{...place,z:place.z+.18},contact:false,yaw:placeToolYaw}];
 let q=[...currentAngles],waypoints:PlannedWaypoint[]=[],worst=0,prev=[...currentAngles],graspTilt=0;const diagnostics:PlanFailure[]=[];
 for(const target of targets){const maxTilt=target.id==='PLACE'||target.id==='RETREAT'?graspTilt:degToRad(tiltEnabled?30:5);const minTilt=target.id==='PLACE'||target.id==='RETREAT'?graspTilt:0;const solved=solvePose(dh,target.p,target.yaw,q,maxTilt,minTilt);diagnostics.push(...solved.diagnostics.map((d)=>({...d,stage:target.id})));if(!solved.ik){const why=target.id.includes('PLACE')||target.id==='RETREAT'?`${solved.reason==='outside workspace'?'Destination out of reach at this yaw':solved.reason} at ${target.id}`:`${solved.reason} at ${target.id}`;return fail(target.id,why,diagnostics);}const ik=solved.ik;if(target.id==='GRASP')graspTilt=solved.tilt;const flange=flangeTarget(target.p,target.yaw,solved.tilt);const collision=pathCollision(dh,prev,ik.angles,target.contact,target.id==='RETREAT');if(collision){const failure:PlanFailure={stage:target.id,kind:collision.kind,linkName:collision.linkName,otherPrimitive:collision.otherPrimitive,penetrationMm:collision.penetrationMm,sampleZMm:collision.sampleZMm,jointAngles:collision.jointAngles??[...ik.angles],tiltDeg:solved.tilt*180/Math.PI,ikSeedUsed:solved.ikSeedUsed??null};diagnostics.push(failure);return {...fail(target.id,`${collision.kind} on path to ${target.id}`,diagnostics),failure,worstIkResidual:Math.max(worst,ik.residualPos)};}waypoints.push({id:target.id,position:flange,jointAngles:ik.angles,tiltDeg:solved.tilt*180/Math.PI});q=ik.angles;prev=ik.angles;worst=Math.max(worst,ik.residualPos);}
 return {waypoints,reachable:true,reason:'Reachable',pickableZone:[],worstIkResidual:worst,failure:null,diagnostics,graspCandidate:chosen.name,gripWidth:chosen.width};
}
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
export function evaluatePickabilityCell(dh:DHParameter[],currentAngles:number[],position:Vector3D,yaw:number,destination?:DropTarget|null):PickabilityCell{
 const fits=inBounds(position,yaw);
 if(!fits)return {position,fits:false,reachable:false,reason:'Outside table bounds'};
 const pick=chooseGraspCandidate(dh,position,currentAngles,yaw);
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
 candidates.sort((a,b)=>Math.hypot(b.x-TABLE.center.x,b.y-TABLE.center.y)-Math.hypot(a.x-TABLE.center.x,a.y-TABLE.center.y));
 return candidates.find((p)=>chooseGraspCandidate(dh,p,currentAngles,yaw)!==null)??BOOK.initialPosition;
}
export function computePickableZone(dh:DHParameter[],currentAngles:number[],yawSamples=[0],step=.08){const zone:Vector3D[]=[];const x0=TABLE.center.x-TABLE.width/2+BOOK.size.x/2+MARGIN,x1=TABLE.center.x+TABLE.width/2-BOOK.size.x/2-MARGIN,y0=TABLE.center.y-TABLE.depth/2+BOOK.size.y/2+MARGIN,y1=TABLE.center.y+TABLE.depth/2-BOOK.size.y/2-MARGIN;for(let x=x0;x<=x1+1e-6;x+=step)for(let y=y0;y<=y1+1e-6;y+=step){const p={x,y,z:BOOK.initialPosition.z};if(isInsideBaseKeepOut(p))continue;if(yawSamples.every(yaw=>solvePose(dh,p,yaw+(BOOK.size.y<BOOK.size.x?Math.PI/2:0),currentAngles,degToRad(30)).ik))zone.push(p);}return zone;}

Object.assign(BOOK.initialPosition, findDefaultPickPosition(INITIAL_DH_TABLE, HOME_JOINT_ANGLES, 0));
