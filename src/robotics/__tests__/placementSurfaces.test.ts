import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';
import { createPickPlan, evaluatePickabilityCell } from '../autonomousPlanner';
import { defaultPlacementSurfaces, rayToNearestSurface, surfaceBookZ, validateSurfaceSet } from '../placementSurfaces';
import { pickabilityCacheKey, PickabilityWorkerConfig } from '../pickabilityGrid';
import { useSimulationStore } from '../../store/simulationStore';

describe('placement surfaces', () => {
  it('provides non-overlapping defaults and resting heights', () => {
    const surfaces=defaultPlacementSurfaces();
    expect(validateSurfaceSet(surfaces)).toBeNull();
    expect(surfaceBookZ(surfaces.find((surface)=>surface.id==='raised-platform')!)).toBeCloseTo(TABLE.height+.15+BOOK.size.z/2);
    expect(surfaceBookZ(surfaces.find((surface)=>surface.id==='lower-platform')!)).toBeCloseTo(TABLE.height-.15+BOOK.size.z/2);
    expect(surfaceBookZ(surfaces.find((surface)=>surface.id==='shelf')!)).toBeCloseTo(1.05+BOOK.size.z/2);
    expect(surfaceBookZ(surfaces.find((surface)=>surface.id==='tray')!)).toBeCloseTo(TABLE.height+.004+BOOK.size.z/2);
    expect(surfaceBookZ(surfaces.find((surface)=>surface.id==='floor')!)).toBeCloseTo(BOOK.size.z/2);
  });

  it('raycasts all surfaces and selects the nearest supported one', () => {
    const surfaces=defaultPlacementSurfaces();
    const tray=surfaces.find((surface)=>surface.id==='tray')!;
    const hit=rayToNearestSurface({origin:{x:tray.center.x,y:tray.center.y,z:1.4},direction:{x:0,y:0,z:-1}},surfaces);
    expect(hit?.surfaceId).toBe('tray');
    expect(hit?.point.z).toBeCloseTo(surfaceBookZ(surfaces.find((surface)=>surface.id==='tray')!));
  });

  it('invalidates pick and place cache keys when a surface moves or changes height', () => {
    const surface=defaultPlacementSurfaces()[0];
    const base:PickabilityWorkerConfig={preset:'compact',dhTable:INITIAL_DH_TABLE,jointAngles:HOME_JOINT_ANGLES,yawRad:0,bookZ:surfaceBookZ(surface),surface};
    expect(pickabilityCacheKey(base)).not.toBe(pickabilityCacheKey({...base,surface:{...surface,z:surface.z+.1}}));
    expect(pickabilityCacheKey(base)).not.toBe(pickabilityCacheKey({...base,surface:{...surface,center:{...surface.center,x:surface.center.x+.1}}}));
    const store=useSimulationStore.getState();
    const oldPlan=store.autonomousPlan,oldHeight=store.placementSurfaces.find((s)=>s.id==='raised-platform')!.z;
    useSimulationStore.setState({autonomousPlan:{reachable:true} as never});
    store.updatePlacementSurface('raised-platform',{z:oldHeight+.1});
    expect(useSimulationStore.getState().autonomousPlan).toBeNull();
    useSimulationStore.getState().updatePlacementSurface('raised-platform',{z:oldHeight});
    useSimulationStore.setState({autonomousPlan:oldPlan});
  });

  it('reports default-surface pick and place planning samples',()=>{
    const surfaces=defaultPlacementSurfaces().filter((surface)=>surface.id!=='floor');
    const records=surfaces.map((source,index)=>{const destination=surfaces[(index+1)%surfaces.length];const plan=createPickPlan(INITIAL_DH_TABLE,{x:source.center.x,y:source.center.y,z:surfaceBookZ(source)},HOME_JOINT_ANGLES,{x:destination.center.x,y:destination.center.y,yaw:0,surfaceId:destination.id,z:surfaceBookZ(destination)},0,true,[...defaultPlacementSurfaces()],source.id);return {source:source.name,destination:destination.name,reachable:plan.reachable,reason:plan.reason};});
    console.log('DEFAULT SURFACE PLAN SAMPLES',JSON.stringify(records));
  });

  it('rejects or adds a clearance waypoint for an obstacle in the transport corridor',()=>{
    const base=defaultPlacementSurfaces(),obstacle={id:'raised-blocker',name:'Raised blocker',shape:'rectangle' as const,center:{x:.70,y:.02,z:0},yaw:0,z:.90,size:{width:.08,depth:.08},support:{type:'none' as const},color:'#668899'};
    const baseline=createPickPlan(INITIAL_DH_TABLE,BOOK.initialPosition,HOME_JOINT_ANGLES,{x:.61,y:0,yaw:0,surfaceId:'table'},0,true,base,'table');
    const plan=createPickPlan(INITIAL_DH_TABLE,BOOK.initialPosition,HOME_JOINT_ANGLES,{x:.61,y:0,yaw:0,surfaceId:'table'},0,true,[...base,obstacle],'table');
    const lift=plan.waypoints.find((waypoint)=>waypoint.id==='TRANSPORT-LIFT');
    expect(baseline.reachable).toBe(true);
    expect(!plan.reachable||!!lift).toBe(true);
  });

  it('prints 3 cm yaw-zero area reachability estimates for every default surface',()=>{
    const surfaces=defaultPlacementSurfaces(),step=.03,source={...BOOK.initialPosition},records=[] as {surface:string;pickable:number;placeable:number;valid:number}[];
    for(const surface of surfaces){let valid=0,pickable=0,placeable=0;
      for(let x=surface.center.x-surface.size.width/2+step/2;x<surface.center.x+surface.size.width/2;x+=step)for(let y=surface.center.y-surface.size.depth/2+step/2;y<surface.center.y+surface.size.depth/2;y+=step){
        const pick=evaluatePickabilityCell(INITIAL_DH_TABLE,HOME_JOINT_ANGLES,{x,y,z:surfaceBookZ(surface)},0,null,undefined,surface,surfaces);if(!pick.fits)continue;valid++;if(pick.reachable)pickable++;
        if(createPickPlan(INITIAL_DH_TABLE,source,HOME_JOINT_ANGLES,{x,y,yaw:0,surfaceId:surface.id,z:surfaceBookZ(surface)},0,true,surfaces,'table').reachable)placeable++;
      }
      records.push({surface:surface.name,valid,pickable:valid?Math.round(1000*pickable/valid)/10:0,placeable:valid?Math.round(1000*placeable/valid)/10:0});
    }
    console.log('DEFAULT SURFACE REACHABILITY 3CM GRID',JSON.stringify(records));
  },120000);

  it('measures Floor pick reachability with the real planner', () => {
    const floor=defaultPlacementSurfaces().find((surface)=>surface.id==='floor')!;
    const step=.01, results=[] as ReturnType<typeof evaluatePickabilityCell>[];
    for(let x=floor.center.x-floor.size.width/2+step/2;x<floor.center.x+floor.size.width/2;x+=step)
      for(let y=floor.center.y-floor.size.depth/2+step/2;y<floor.center.y+floor.size.depth/2;y+=step)
        results.push(evaluatePickabilityCell(INITIAL_DH_TABLE,HOME_JOINT_ANGLES,{x,y,z:surfaceBookZ(floor)},0,null,undefined,floor));
    const fit=results.filter((result)=>result.fits),reachable=fit.filter((result)=>result.reachable).length;
    const reasons=[...new Set(fit.filter((result)=>!result.reachable).map((result)=>result.reason))];
    console.log(`FLOOR REACHABILITY: ${reachable}/${fit.length} valid grid samples (${fit.length?100*reachable/fit.length:0}%), reasons=${JSON.stringify(reasons)}`);
    expect(reachable).toBe(0);
  });
});
