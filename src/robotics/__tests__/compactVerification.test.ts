import { mkdirSync, writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { useSimulationStore } from '../../store/simulationStore';
import { computeForwardKinematics } from '../forwardKinematics';
import { createPickPlan } from '../autonomousPlanner';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';

const dest = { x: 0.61, y: 0, yaw: 0 };
const rad = (d: number) => d * Math.PI / 180;
const round = (n: number, digits = 4) => Number(n.toFixed(digits));
const extent = (yaw: number) => ({ x: Math.abs(Math.cos(yaw))*BOOK.size.x/2 + Math.abs(Math.sin(yaw))*BOOK.size.y/2, y: Math.abs(Math.sin(yaw))*BOOK.size.x/2 + Math.abs(Math.cos(yaw))*BOOK.size.y/2 });
const footprintFits = (p: {x:number;y:number}, yaw: number, margin=0) => { const e=extent(yaw); return Math.abs(p.x-TABLE.center.x)+e.x+margin <= TABLE.width/2+1e-9 && Math.abs(p.y-TABLE.center.y)+e.y+margin <= TABLE.depth/2+1e-9; };
const tableZ = BOOK.initialPosition.z;
function runHeadless(p: {x:number;y:number}, yaw: number, target: typeof dest) {
  const store=useSimulationStore.getState(); store.resetTask(); store.setBookPosition({x:p.x,y:p.y,z:tableZ}); store.setBookYawDegrees(yaw*180/Math.PI); store.setDropTarget({...target,yaw:target.yaw*180/Math.PI}); store.planAutonomousPick();
  const planned=useSimulationStore.getState(), plan=planned.autonomousPlan;
  if (!plan?.reachable) return { ok:false, reason:plan?.reason ?? 'no plan', plan, state:planned, lowest:tableZ };
  planned.executeAutonomousPick(); let lowest=Infinity;
  for(let frame=0;frame<900&&useSimulationStore.getState().isTaskPlaying;frame++){useSimulationStore.getState().tickTask(.05);lowest=Math.min(lowest,useSimulationStore.getState().simulatedObject.position.z);}
  const end=useSimulationStore.getState(), obj=end.simulatedObject;
  const xy=Math.hypot(obj.position.x-target.x,obj.position.y-target.y), yawErr=Math.abs(Math.atan2(Math.sin(obj.rotation.yaw-target.yaw),Math.cos(obj.rotation.yaw-target.yaw)));
  const ok=end.pickStatus==='DONE'&&obj.state==='placed'&&xy<=.005&&yawErr<=rad(2)&&lowest>=TABLE.height;
  return {ok,reason:ok?'DONE':end.pickStatus==='DONE'?`assertion failed: state=${obj.state}, xy=${round(xy,5)} m, yaw=${round(yawErr*180/Math.PI,2)} deg, lowestZ=${round(lowest,5)} m`:end.pickStatus,plan,state:end,lowest,xy,yawErr};
}

beforeAll(()=>{useSimulationStore.getState().setTablePreset('compact');});
afterAll(()=>{useSimulationStore.getState().setTablePreset('compact');});

describe('requested Compact verification suites',()=>{
  it('runs complete planner over every 1 cm cell and yaw, saving structured outcomes',()=>{
    const yaws=Array.from({length:13},(_,i)=>i*15), xs=Array.from({length:34},(_,i)=>round(TABLE.center.x-.165+i*.01,3)), ys=Array.from({length:20},(_,i)=>round(TABLE.center.y-.095+i*.01,3));
    const perYaw:any[]=[];
    for(const yawDeg of yaws){
      const yaw=rad(yawDeg), cells:any[]=[], fits:any[]=[], pickable:any[]=[], failureKindCounts:Record<string,number>={}, candidateCounts:Record<string,number>={};
      for(const x of xs)for(const y of ys){const p={x,y,z:tableZ},fitsTable=footprintFits(p,yaw); if(!fitsTable)continue; fits.push(p);
        const plan=createPickPlan(INITIAL_DH_TABLE,p,HOME_JOINT_ANGLES,dest,yaw,true);
        if(plan.reachable){pickable.push(p);candidateCounts[plan.graspCandidate]=(candidateCounts[plan.graspCandidate]??0)+1;}
        else {const kind=plan.failure?.kind??plan.reason;failureKindCounts[kind]=(failureKindCounts[kind]??0)+1;}
        cells.push({x,y,failure:plan.reachable?null:plan.failure??{stage:'PLAN',kind:plan.reason,linkName:null,otherPrimitive:null,penetrationMm:null,jointAngles:null,tiltDeg:null,ikSeedUsed:null}});
      }
      const row={yawDeg,cellsTested:xs.length*ys.length,cellsFitTable:fits.length,cellsPickable:pickable.length,pickableFraction:round(pickable.length/fits.length,5),failureKindCounts,candidateCounts,failures:cells.filter(c=>c.failure)};
      perYaw.push(row); console.log(`COMPACT SWEEP yaw=${yawDeg} cells=${row.cellsTested} fit=${row.cellsFitTable} pickable=${row.cellsPickable} fraction=${row.pickableFraction} failures=${JSON.stringify(failureKindCounts)} candidates=${JSON.stringify(candidateCounts)}`);
    }
    const result={table:{width:TABLE.width,depth:TABLE.depth,center:{x:TABLE.center.x,y:TABLE.center.y}},grid:{stepM:.01,xs,ys},destination:dest,yawsDeg:yaws,perYaw,acceptance:{yaw0And180CountsIdentical:JSON.stringify([perYaw[0].cellsTested,perYaw[0].cellsFitTable,perYaw[0].cellsPickable])===JSON.stringify([perYaw[12].cellsTested,perYaw[12].cellsFitTable,perYaw[12].cellsPickable])}};
    const output=resolve(process.cwd(),'src/robotics/generated/compactSweep.json');mkdirSync(resolve(process.cwd(),'src/robotics/generated'),{recursive:true});writeFileSync(output,JSON.stringify(result,null,2));
    console.log(`YAW 0/180 IDENTICAL COUNTS=${result.acceptance.yaw0And180CountsIdentical}`); expect(result.acceptance.yaw0And180CountsIdentical).toBe(true);
    expect(perYaw).toHaveLength(13);
  },60*60*1000);

  it('runs 50 seeded randomized non-overlapping pick and destination pairs headlessly',()=>{
    const seed=0xC0A7AC7, rng=(()=>{let s=seed>>>0;return()=>((s=(1664525*s+1013904223)>>>0)/0x100000000);})();
    console.log(`RANDOM SUITE SEED=${seed}`);
    const sample=(yaw:number)=>{for(let i=0;i<10000;i++){const p={x:TABLE.center.x+(rng()-.5)*TABLE.width,y:TABLE.center.y+(rng()-.5)*TABLE.depth};if(footprintFits(p,yaw,.012))return p;}throw new Error('sampling failed');};
    let successes=0;const failures:any[]=[], cases:any[]=[];
    for(let i=0;i<50;i++){
      const byaw=rad(Math.floor(rng()*181)), dyaw=rad(Math.floor(rng()*181));let book=sample(byaw),drop: {x:number;y:number}|undefined;
      for(let bookTry=0;bookTry<80&&!drop;bookTry++){book=sample(byaw);for(let tries=0;tries<300;tries++){const p=sample(dyaw);if(Math.hypot(p.x-book.x,p.y-book.y)>=Math.hypot(BOOK.size.x/2,BOOK.size.y/2)*2+.02){drop=p;break;}}}
      if(!drop)throw new Error('could not sample separated destination');const target={...drop,yaw:dyaw};
      const validOverlays=footprintFits(book,byaw,.012)&&footprintFits(drop,dyaw,.012)&&Math.hypot(book.x-drop.x,book.y-drop.y)>=Math.hypot(BOOK.size.x/2,BOOK.size.y/2)*2+.02;
      const plan=createPickPlan(INITIAL_DH_TABLE,{...book,z:tableZ},HOME_JOINT_ANGLES,target,byaw,true);
      if(!plan.reachable){const f={i,book,bookYawDeg:round(byaw*180/Math.PI,1),destination:drop,destinationYawDeg:round(dyaw*180/Math.PI,1),reason:plan.reason,structuredFailure:plan.failure,validOverlays,plannerBug:validOverlays};failures.push(f);cases.push({...f,success:false});console.log(`RANDOM FAIL ${JSON.stringify(f)}`);continue;}
      const ran=runHeadless(book,byaw,target);if(ran.ok)successes++;else{const f={i,book,bookYawDeg:round(byaw*180/Math.PI,1),destination:drop,destinationYawDeg:round(dyaw*180/Math.PI,1),reason:ran.reason,validOverlays,plannerBug:false,lowestZM:round(ran.lowest,5),finalXYErrorM:round(ran.xy??NaN,5),finalYawErrorDeg:round((ran.yawErr??NaN)*180/Math.PI,2)};failures.push(f);console.log(`RANDOM FAIL ${JSON.stringify(f)}`);cases.push({...f,success:false});continue;} cases.push({i,book,bookYawDeg:round(byaw*180/Math.PI,1),destination:drop,destinationYawDeg:round(dyaw*180/Math.PI,1),success:true,lowestZ:round(ran.lowest,5),finalXYErrorM:round(ran.xy??NaN,5),finalYawErrorDeg:round((ran.yawErr??NaN)*180/Math.PI,2)});
    }
    console.log(`RANDOM RESULT seed=${seed} success=${successes}/50 rate=${(successes*2).toFixed(1)}% failures=${failures.length}`);mkdirSync(resolve(process.cwd(),'src/robotics/generated'),{recursive:true});writeFileSync(resolve(process.cwd(),'src/robotics/generated/compactRandomized.json'),JSON.stringify({seed,cases,successes,failures},null,2));expect(successes+failures.length).toBe(50);
  },60*60*1000);

  it('proves rotated full sequences and grasp alignment for five fixed positions',()=>{
    const positions=[{x:.76,y:-.06},{x:.77,y:-.05},{x:.78,y:-.04},{x:.79,y:-.03},{x:.80,y:-.02}], rows:any[]=[];
    for(const p of positions)for(let yawDeg=0;yawDeg<=180;yawDeg+=15){const yaw=rad(yawDeg),ran=runHeadless(p,yaw,dest),plan=ran.plan;
      const grasp=plan?.waypoints.find(w=>w.id==='GRASP'),fk=grasp?computeForwardKinematics(INITIAL_DH_TABLE,grasp.jointAngles):null;
      const close=ran.state.graspDebugLog?.find(e=>e.stage==='CLOSE'); const tcpDistance=close?Math.hypot(close.tcp.x-close.bookCenter.x,close.tcp.y-close.bookCenter.y,close.tcp.z-close.bookCenter.z):null;
      const candidate=plan?.graspCandidate??null, gap=close?.fingerGap??null, row={position:p,yawDeg,success:ran.ok,candidate,j6AtGrasp:grasp?round(grasp.jointAngles[5]*180/Math.PI,2):null,closedFingerGapM:gap===null?null:round(gap,4),tcpToBookCenterAtCloseM:tcpDistance===null?null:round(tcpDistance,5),lowestZM:round(ran.lowest,5),reason:ran.reason}; rows.push(row);
      console.log(`ROTATED ${JSON.stringify(row)}`);
      if(ran.ok){expect(tcpDistance).toBeLessThanOrEqual(.005);expect(gap).toBeCloseTo(candidate?.includes('wide')?.1:.08,5);}
    }
    mkdirSync(resolve(process.cwd(),'src/robotics/generated'),{recursive:true});writeFileSync(resolve(process.cwd(),'src/robotics/generated/rotatedBookProof.json'),JSON.stringify(rows,null,2)); expect(rows).toHaveLength(65);
  },60*60*1000);
});
