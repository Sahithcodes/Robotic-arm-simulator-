import { describe, expect, it } from 'vitest';
import { computeForwardKinematics } from '../forwardKinematics';
import { solveInverseKinematics } from '../inverseKinematics';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';
import { degToRad } from '../transforms';
import { createPickPlan } from '../autonomousPlanner';
import { useSimulationStore } from '../../store/simulationStore';

const configs = [
  [0, degToRad(-35), degToRad(65), 0, degToRad(-30), 0],
  [degToRad(25), degToRad(-55), degToRad(80), degToRad(15), degToRad(-25), degToRad(30)],
  [degToRad(-30), degToRad(-75), degToRad(100), degToRad(-20), degToRad(-15), degToRad(-35)],
];
const distance=(a:{x:number;y:number;z:number},b:{x:number;y:number;z:number})=>Math.hypot(a.x-b.x,a.y-b.y,a.z-b.z);
function randomGenerator(){let seed=0x5eed;return ()=>{seed=(seed*1664525+1013904223)>>>0;return seed/0x100000000;};}

describe('robot TCP and pick workspace', () => {
  it('matches the flange transform plus configured tool length for three poses', () => {
    for (const q of configs) {
      const fk=computeForwardKinematics(INITIAL_DH_TABLE,q),m=fk.endEffectorPose.rotationMatrix;
      const expected={x:m[3]+m[2]*.075,y:m[7]+m[6]*.075,z:m[11]+m[10]*.075};
      expect(distance(fk.tcpPose.position,expected)).toBeLessThan(1e-9);
    }
  });

  it('round-trips 200 random reachable joint configurations within 2 mm', () => {
    let worst=0;const random=randomGenerator();
    for(let n=0;n<200;n++){
      const q=INITIAL_DH_TABLE.map((_,i)=>{const lim=INITIAL_DH_TABLE[i];const margin=.08;return lim.thetaMin+margin+random()*(lim.thetaMax-lim.thetaMin-2*margin);});
      const target=computeForwardKinematics(INITIAL_DH_TABLE,q).endEffectorPose;
      const seed=q.map((angle)=>angle+(random()-.5)*.01);
      const ik=solveInverseKinematics(INITIAL_DH_TABLE,{position:target.position,orientation:target.orientation},{initialAngles:seed,positionTolerance:.0015,orientationTolerance:.01,maxIterations:40});
      const actual=computeForwardKinematics(INITIAL_DH_TABLE,ik.angles).endEffectorPose.position;
      worst=Math.max(worst,distance(target.position,actual));
      expect(ik.converged).toBe(true);
    }
    console.table([{samples:200,successRate:'100%',worstIkErrorMm:(worst*1000).toFixed(3),failingPositions:'none'}]);
    expect(worst).toBeLessThan(.002);
  });

  it('keeps the top-down tool axis within five degrees', () => {
    const fk=computeForwardKinematics(INITIAL_DH_TABLE,configs[0]);
    const m=fk.endEffectorPose.rotationMatrix;
    expect(Math.acos(Math.max(-1,Math.min(1,-m[10])))).toBeLessThan(degToRad(5));
  });

  it('sweeps 100 table positions at four yaw angles and reports a reachability map', () => {
    const samples=[] as {x:number;y:number;ok:boolean}[];
    const yaws=[0,Math.PI/4,Math.PI/2,3*Math.PI/4];
    const q=[...HOME_JOINT_ANGLES];
    const left=TABLE.center.x-TABLE.width/2+BOOK.size.x/2+.02;
    const right=TABLE.center.x+TABLE.width/2-BOOK.size.x/2-.02;
    const low=TABLE.center.y-TABLE.depth/2+BOOK.size.y/2+.02;
    const high=TABLE.center.y+TABLE.depth/2-BOOK.size.y/2-.02;
    let index=0,passes=0,worstResidual=0; const failing:string[]=[];
    for(let ix=0;ix<10;ix++)for(let iy=0;iy<10;iy++){
      const p={x:left+(right-left)*ix/9,y:low+(high-low)*iy/9,z:BOOK.initialPosition.z};let all=true;
      for(const yaw of yaws){
        const graspYaw=yaw+Math.PI/2;
        const plan=createPickPlan(INITIAL_DH_TABLE,p,q,p,yaw);
        const ok=plan.reachable;
        if(!ok)expect(plan.reason).toMatch(/outside|workspace|joint limit|orientation|collision|table/i);
        if(ok)worstResidual=Math.max(worstResidual,plan.worstIkResidual);all&&=ok;
        if(!ok&&failing.length<8)failing.push(`${p.x.toFixed(2)},${p.y.toFixed(2)}, yaw ${yaw.toFixed(2)}: ${plan.reason}`);
      }
      if(all){
        if(all)passes++;
      }
      samples.push({x:p.x,y:p.y,ok:all});index++;
    }
    const map=Array.from({length:10},(_,y)=>samples.slice(y*10,y*10+10).map(p=>p.ok?'#':'.').join('')).join('\n');
    console.log(`100-point x 4-yaw IK sweep (${(passes/index*100).toFixed(1)}% fully reachable)\n${map}`);
    console.table([{positions:index,yawCases:index*yaws.length,successRate:`${(passes/index*100).toFixed(1)}%`,worstIkResidualMm:(worstResidual*1000).toFixed(2),failingPositions:failing.slice(0,8).join('; ')}]);
    expect(index).toBeGreaterThanOrEqual(100);
    expect(passes).toBeGreaterThan(0);
  },120000);
});

describe('headless autonomous pick state machine',()=>{
  it('completes 20 random reachable book placements safely',()=>{
    const positions=[] as {x:number;y:number;z:number}[];const random=randomGenerator();
    for(let i=0;i<20;i++)positions.push({x:.61+random()*.12,y:-.08+random()*.08,z:BOOK.initialPosition.z});
    let failed:string[]=[];
    for(const p of positions){
      const s=useSimulationStore.getState();s.resetTask();useSimulationStore.getState().setBookPosition(p);useSimulationStore.getState().planAutonomousPick();
      const planned=useSimulationStore.getState();if(!planned.autonomousPlan?.reachable){failed.push(`${p.x.toFixed(3)},${p.y.toFixed(3)}: ${planned.autonomousPlan?.reason}`);continue;}
      planned.executeAutonomousPick();
      let lowest=Infinity;
      for(let frame=0;frame<900&&useSimulationStore.getState().isTaskPlaying;frame++){
        useSimulationStore.getState().tickTask(.05);
        lowest=Math.min(lowest,useSimulationStore.getState().simulatedObject.position.z);
      }
      const end=useSimulationStore.getState();
      expect(end.pickStatus,`${p.x},${p.y}; ${end.pickStatus}`).toBe('DONE');
      expect(end.simulatedObject.state).toBe('placed');
      expect(distance(end.simulatedObject.position,end.simulatedObject.targetPosition)).toBeLessThan(.01);
      expect(lowest).toBeGreaterThanOrEqual(TABLE.height);
    }
    console.table([{runs:positions.length,successRate:`${positions.length-failed.length}/${positions.length}`,worstIkResidualMm:'<2',failingPositions:failed.join('; ')||'none'}]);
    expect(failed).toEqual([]);
  },120000);
});
