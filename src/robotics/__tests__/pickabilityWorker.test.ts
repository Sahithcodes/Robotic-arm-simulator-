import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { BOOK, HOME_JOINT_ANGLES } from '../task5';
import { evaluatePickabilityCell } from '../autonomousPlanner';
import { evaluatePickabilityState, pickabilityCacheKey, pickabilityReason, PickabilityWorkerConfig } from '../pickabilityGrid';

describe('pickability worker contract',()=>{
  it('matches main-thread success and reason on 200 seeded random cells',()=>{
    const config:PickabilityWorkerConfig={preset:'compact',dhTable:INITIAL_DH_TABLE,jointAngles:HOME_JOINT_ANGLES,yawRad:Math.PI/4,bookZ:BOOK.initialPosition.z};
    let seed=0x71a5;const random=()=>((seed=(seed*1664525+1013904223)>>>0)/0x100000000);
    for(let i=0;i<200;i++){
      const x=.54+random()*.34,y=-.14+random()*.20,state=evaluatePickabilityState(config,x,y),main=evaluatePickabilityCell(config.dhTable,config.jointAngles,{x,y,z:config.bookZ},config.yawRad);
      expect({success:state.state===2,reason:pickabilityReason(state.state)}).toEqual({success:main.reachable,reason:main.reason});
    }
  },120000);
  it('invalidates cache keys for preset, yaw and book size changes',()=>{
    const config:PickabilityWorkerConfig={preset:'compact',dhTable:INITIAL_DH_TABLE,jointAngles:HOME_JOINT_ANGLES,yawRad:0,bookZ:BOOK.initialPosition.z};
    const base=pickabilityCacheKey(config);
    expect(pickabilityCacheKey({...config,preset:'large'})).not.toBe(base);
    expect(pickabilityCacheKey({...config,yawRad:Math.PI/2})).not.toBe(base);
    const old={...BOOK.size};
    try{BOOK.size.x=old.x+.001;expect(pickabilityCacheKey(config)).not.toBe(base);}finally{BOOK.size.x=old.x;BOOK.size.y=old.y;BOOK.size.z=old.z;}
  });
});
