import { describe, expect, it } from 'vitest';
import { INITIAL_DH_TABLE, ROBOT_BASE_KEEP_OUT, TCP_OFFSET } from '../../robot/robotConfig';
import { BOOK, HOME_JOINT_ANGLES } from '../task5';
import { solveInverseKinematics } from '../inverseKinematics';
import { degToRad, rpyToMatrix } from '../transforms';

describe('requested top-down reachability map', () => {
  it('prints a 2 cm IK map and failure counts at four yaws', () => {
    const yaws = [0, Math.PI / 4, Math.PI / 2, 3 * Math.PI / 4];
    const total = 46 * 61;
    for (const yaw of yaws) {
      const rows: string[] = [];
      const causeRows: string[] = [];
      let keepout = 0, jointLimit = 0, nonConverged = 0, reachable = 0;
      let minReach = Infinity, maxReach = 0;
      for (let iy = 0; iy <= 60; iy++) {
        const y = 0.60 - iy * 0.02;
        let row = '', causes = '';
        for (let ix = 0; ix <= 45; ix++) {
          const x = 0.10 + ix * 0.02;
          const radius = Math.hypot(x - ROBOT_BASE_KEEP_OUT.center.x, y - ROBOT_BASE_KEEP_OUT.center.y);
          const isKeepout = radius < ROBOT_BASE_KEEP_OUT.radius + Math.hypot(BOOK.size.x / 2, BOOK.size.y / 2);
          if (isKeepout) { keepout++; row += '.'; causes += 'K'; continue; }
          const rotation = rpyToMatrix(Math.PI, 0, yaw);
          const target = {
            position: { x: x - rotation[2] * TCP_OFFSET.z, y: y - rotation[6] * TCP_OFFSET.z, z: BOOK.initialPosition.z - rotation[10] * TCP_OFFSET.z },
            toolZAxis: { x: rotation[2], y: rotation[6], z: rotation[10] },
            yaw,
          };
          const bearing = Math.atan2(y, x);
          const seeds = [
            HOME_JOINT_ANGLES,
            [0, degToRad(-35), degToRad(55), 0, degToRad(-20), 0],
            [bearing, degToRad(-35), degToRad(70), 0, degToRad(-35), 0],
            [bearing, degToRad(-80), degToRad(120), 0, degToRad(-40), 0],
            [bearing, degToRad(-100), degToRad(-80), 0, degToRad(80), 0],
          ];
          let solved = false, hitLimit = false;
          for (const initialAngles of seeds) {
            const result = solveInverseKinematics(INITIAL_DH_TABLE, target, {
              initialAngles, maxIterations: 35, positionTolerance: 0.003, orientationTolerance: degToRad(5),
            });
            if (result.converged) { solved = true; break; }
            hitLimit ||= result.hitLimit.some(Boolean);
          }
          if (solved) {
            reachable++; minReach = Math.min(minReach, radius); maxReach = Math.max(maxReach, radius); row += '#'; causes += '#';
          } else {
            if (hitLimit) { jointLimit++; causes += 'L'; } else { nonConverged++; causes += 'I'; }
            row += '.';
          }
        }
        rows.push(row); causeRows.push(causes);
      }
      console.log(`TOP-DOWN ${yaw * 180 / Math.PI}°; # ${reachable}/${total}; IK failure causes keepout=${keepout}, joint-limit=${jointLimit}, non-convergence=${nonConverged}; horizontal reach ${minReach.toFixed(3)}–${maxReach.toFixed(3)} m\n${rows.join('\n')}\nCell causes (# reachable, K keep-out, L joint limit, I IK non-convergence):\n${causeRows.join('\n')}`);
      expect(reachable).toBeGreaterThan(0);
    }
  }, 120000);
});
