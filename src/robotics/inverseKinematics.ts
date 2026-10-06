import { DHParameter, Matrix4x4, Vector3D } from '../types/robotics';
import { computeForwardKinematics } from './forwardKinematics';
import { degToRad, multiply4x4, rpyToMatrix } from './transforms';
import { PUMA_GEOMETRY } from '../robot/robotConfig';

type IKTarget = { position: Vector3D; orientation?: { roll: number; pitch: number; yaw: number }; toolZAxis?: Vector3D; yaw?: number };
type IKOptions = { initialAngles?: number[]; maxIterations?: number; positionTolerance?: number; orientationTolerance?: number };
export type IKResult = { angles: number[]; positionError: number; orientationError: number; residualPos: number; residualAngle: number; hitLimit: boolean[]; converged: boolean; iterations: number };
const DEFAULT_ANGLES = [0, degToRad(-35), degToRad(55), 0, degToRad(-20), 0];
const DAMPING = 0.025;
const STEP_LIMIT = degToRad(7);
const rot = (m: Matrix4x4) => [[m[0],m[1],m[2]],[m[4],m[5],m[6]],[m[8],m[9],m[10]]];
function orientationVector(current: Matrix4x4, target: Matrix4x4 | null): number[] {
  if (!target) return [0,0,0];
  const a=rot(current), b=rot(target);
  const rel=Array.from({length:3},(_,r)=>Array.from({length:3},(_,c)=>b[r].reduce((s,v,k)=>s+v*a[c][k],0)));
  const angle=Math.acos(Math.max(-1,Math.min(1,(rel[0][0]+rel[1][1]+rel[2][2]-1)/2)));
  if(angle<1e-8)return [0,0,0];
  const scale=angle/(2*Math.sin(Math.min(angle,Math.PI-1e-6)));
  return [(rel[2][1]-rel[1][2])*scale,(rel[0][2]-rel[2][0])*scale,(rel[1][0]-rel[0][1])*scale];
}
function clamp(a:number[]) { return a.map((q,i)=>Math.max(degToRad(PUMA_GEOMETRY.jointLimits[i].min),Math.min(degToRad(PUMA_GEOMETRY.jointLimits[i].max),q))); }
function solve(a:number[][],b:number[]) { const n=b.length,m=a.map((r,i)=>[...r,b[i]]); for(let c=0;c<n;c++){let p=c;for(let r=c+1;r<n;r++)if(Math.abs(m[r][c])>Math.abs(m[p][c]))p=r;[m[c],m[p]]=[m[p],m[c]];const d=m[c][c]||1e-12;for(let j=c;j<=n;j++)m[c][j]/=d;for(let r=0;r<n;r++)if(r!==c){const f=m[r][c];for(let j=c;j<=n;j++)m[r][j]-=f*m[c][j];}}return m.map(r=>r[n]); }
function angleDifference(target:number,current:number){let d=target-current;while(d>Math.PI)d-=2*Math.PI;while(d< -Math.PI)d+=2*Math.PI;return d;}
function residual(dh:DHParameter[],q:number[],target:IKTarget, tr:Matrix4x4|null){const fk=computeForwardKinematics(dh,q),m=fk.endEffectorPose.rotationMatrix,p=fk.endEffectorPose.position;const pos=[target.position.x-p.x,target.position.y-p.y,target.position.z-p.z];let ori:number[];if(target.toolZAxis&&target.yaw!==undefined){const axis=[m[2],m[6],m[10]],axisError=[target.toolZAxis.x-axis[0],target.toolZAxis.y-axis[1],target.toolZAxis.z-axis[2]],yawError=angleDifference(target.yaw,fk.endEffectorPose.orientation.yaw);ori=[...axisError,yawError];}else ori=orientationVector(m,tr);return {e:[...pos,...ori.map(x=>x*.12)],p:Math.hypot(...pos),o:Math.hypot(...ori)};}
export function solveInverseKinematics(dh:DHParameter[],target:IKTarget,options:IKOptions={}):IKResult{
 let q=clamp(options.initialAngles??DEFAULT_ANGLES),last!:ReturnType<typeof residual>; const tr=target.orientation?rpyToMatrix(target.orientation.roll,target.orientation.pitch,target.orientation.yaw):null; const max=options.maxIterations??500,pt=options.positionTolerance??.002,ot=options.orientationTolerance??.09;
 for(let it=0;it<=max;it++){last=residual(dh,q,target,tr); if(last.p<=pt&&(!(tr||target.toolZAxis)||last.o<=ot))return result(q,last.p,last.o,it,true); if(it===max)break;
  const rows=last.e.length,jac=Array.from({length:rows},()=>Array(6).fill(0)),delta=1e-4;
  for(let j=0;j<6;j++){const qq=[...q];qq[j]+=delta;const e=residual(dh,qq,target,tr).e;for(let r=0;r<rows;r++)jac[r][j]=(e[r]-last.e[r])/delta;}
  const a=Array.from({length:rows},(_,r)=>Array.from({length:rows},(_,c)=>jac[r].reduce((s,v,k)=>s+v*jac[c][k],0)+(r===c?DAMPING*DAMPING:0)));
  const y=solve(a,last.e),step=Array.from({length:6},(_,j)=>jac.reduce((s,row,r)=>s+row[j]*y[r],0));
  q=clamp(q.map((v,j)=>v-Math.max(-STEP_LIMIT,Math.min(STEP_LIMIT,step[j]))));
 }
 return result(q,last.p,last.o,max,false);
}
function result(angles:number[],positionError:number,orientationError:number,iterations:number,converged:boolean):IKResult {const hitLimit=angles.map((q,i)=>{const l=PUMA_GEOMETRY.jointLimits[i];return Math.abs(q-degToRad(l.min))<1e-5||Math.abs(q-degToRad(l.max))<1e-5;});return {angles,positionError,orientationError,residualPos:positionError,residualAngle:orientationError,hitLimit,iterations,converged};}
export function transformPoint(matrix:Matrix4x4,point:Vector3D):Vector3D {const t=multiply4x4(matrix,[1,0,0,point.x,0,1,0,point.y,0,0,1,point.z,0,0,0,1]);return {x:t[3],y:t[7],z:t[11]};}
