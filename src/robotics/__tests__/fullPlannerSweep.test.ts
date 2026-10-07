import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { INITIAL_DH_TABLE } from '../../robot/robotConfig';
import { BOOK, HOME_JOINT_ANGLES, TABLE } from '../task5';
import { createPickPlan, PlanFailure } from '../autonomousPlanner';
import { computeForwardKinematics } from '../forwardKinematics';
import { useSimulationStore } from '../../store/simulationStore';
import { beforeEach } from 'vitest';

beforeEach(() => useSimulationStore.getState().setTablePreset('large'));

const yawsDeg = [0, 30, 60, 90, 120, 150, 180];
const round = (n: number, digits = 3) => Number(n.toFixed(digits));
const casePoses = [{ x: 0.54, y: 0 }, { x: 0.366, y: -0.114 }];
const usedTilt = (plan: ReturnType<typeof createPickPlan>) => Math.max(0, ...plan.waypoints.map((w) => w.tiltDeg ?? 0));
function renderedUpperArmVerticalHalfHeightMm(q: number[] | null): number | null {
  if (!q) return null;
  const joints = computeForwardKinematics(INITIAL_DH_TABLE, q).jointPositions;
  const a = new THREE.Vector3(joints[1].x, joints[1].y, joints[1].z), b = new THREE.Vector3(joints[2].x, joints[2].y, joints[2].z);
  const direction = b.clone().sub(a).normalize();
  const rotation = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  const xAxis = new THREE.Vector3(1, 0, 0).applyQuaternion(rotation), zAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(rotation);
  const halfHeight = 0.036 * Math.abs(xAxis.z) + 0.025 * Math.abs(zAxis.z);
  return round(halfHeight * 1000, 2);
}

describe('instrumented planner cases and measured full-plan sweep', () => {
  it('validates the saved sweep and the book-shrunk sector on the 1 cm XY grid', () => {
    const saved = JSON.parse(readFileSync(resolve(process.cwd(), 'src/robotics/generated/sweep.json'), 'utf8')) as {
      grid: { xs: number[]; ys: number[]; stepM: number };
      yawsDeg: number[];
      perYaw: Record<string, { strict: boolean[]; enabled: boolean[]; tiltDeg: (number | null)[]; failureKind: (string | null)[] }>;
      intersection: boolean[];
      sector: { innerR: number; outerR: number; halfSpanDeg: number };
    };
    const sector = JSON.parse(readFileSync(resolve(process.cwd(), 'src/robotics/generated/sector.json'), 'utf8')) as { innerR: number; outerR: number; halfSpanDeg: number; areaM2: number };
    expect(saved.grid).toMatchObject({ stepM: 0.01 });
    expect(saved.grid.xs).toHaveLength(91);
    expect(saved.grid.ys).toHaveLength(121);
    expect(saved.yawsDeg).toEqual(yawsDeg);
    for (const yaw of yawsDeg) {
      expect(saved.perYaw[String(yaw)].strict).toHaveLength(91 * 121);
      expect(saved.perYaw[String(yaw)].enabled).toHaveLength(91 * 121);
      expect(saved.perYaw[String(yaw)].tiltDeg).toHaveLength(91 * 121);
      expect(saved.perYaw[String(yaw)].failureKind).toHaveLength(91 * 121);
    }
    const recomputedIntersection = Array.from({ length: 91 * 121 }, (_, i) => yawsDeg.every((yaw) => saved.perYaw[String(yaw)].enabled[i]));
    expect(saved.intersection).toEqual(recomputedIntersection);
    expect(saved.intersection.filter(Boolean)).toHaveLength(533);
    expect(sector).toMatchObject({ innerR: 0.516, outerR: 0.834, halfSpanDeg: 15, areaM2: 0.112413 });
    const hd = Math.hypot(BOOK.size.x / 2, BOOK.size.y / 2), r0 = sector.innerR + hd, r1 = sector.outerR - hd;
    const a1 = sector.halfSpanDeg - Math.asin(hd / r0) * 180 / Math.PI;
    let checked = 0, failures = 0;
    for (let ix = 0; ix < saved.grid.xs.length; ix++) for (let iy = 0; iy < saved.grid.ys.length; iy++) {
      const x = saved.grid.xs[ix], y = saved.grid.ys[iy], r = Math.hypot(x, y), a = Math.abs(Math.atan2(y, x) * 180 / Math.PI), index = ix * saved.grid.ys.length + iy;
      if (r >= r0 - 1e-9 && r <= r1 + 1e-9 && a <= a1 + 1e-9) { checked++; if (!saved.intersection[index]) failures++; }
    }
    console.log(`SAVED SECTOR CHECK: ${checked} shrunk-sector grid points, failures=${failures}`);
    expect(checked).toBe(386);
    expect(failures).toBe(0);
  });

  it('prints structured failure diagnostics for the two requested positions', () => {
    const rows: { position: string; yawDeg: number; tiltDeg: number; tried: string; stageKind: string; link: string; primitive: string; penetrationMm: string; seeds: string; failureRecords: object[] }[] = [];
    for (const position of casePoses) for (const yawDeg of [0, 45, 90, 135]) {
      const plan = createPickPlan(INITIAL_DH_TABLE, { ...position, z: BOOK.initialPosition.z }, HOME_JOINT_ANGLES, { x: 0.8, y: 0, yaw: 0 }, yawDeg * Math.PI / 180, true);
      for (const tilt of [0, 10, 20, 30]) {
        const attempts = plan.diagnostics.filter((d) => Math.abs((d.tiltDeg ?? -1) - tilt) < 1e-5);
        const pathFailure = plan.failure && Math.abs((plan.failure.tiltDeg ?? -1) - tilt) < 1e-5 ? [plan.failure] : [];
        const all = [...attempts, ...pathFailure];
        const unique = new Map<string, PlanFailure[]>();
        for (const failure of all) {
          const key = [failure.stage, failure.kind, failure.linkName, failure.otherPrimitive, failure.penetrationMm === null ? '' : round(failure.penetrationMm, 2)].join('|');
          unique.set(key, [...(unique.get(key) ?? []), failure]);
        }
        const tried = all.length > 0 || (plan.reachable && usedTilt(plan) >= tilt);
        rows.push({
          position: `(${position.x.toFixed(3)}, ${position.y.toFixed(3)})`, yawDeg, tiltDeg: tilt, tried: tried ? 'yes' : 'not reached',
          stageKind: [...unique.keys()].map((key) => key.split('|').slice(0, 2).join(': ')).join('; ') || (tried ? 'no recorded failure' : '—'),
          link: [...new Set([...unique.keys()].map((key) => key.split('|')[2] || '—'))].join('; '),
          primitive: [...new Set([...unique.keys()].map((key) => key.split('|')[3] || '—'))].join('; '),
          penetrationMm: [...new Set([...unique.keys()].map((key) => key.split('|')[4] || '—'))].join('; '),
          seeds: [...new Set(all.map((failure) => failure.ikSeedUsed === null ? 'path' : String(failure.ikSeedUsed)))].join(',') || '—',
          failureRecords: all.map(({ stage, kind, linkName, otherPrimitive, penetrationMm, jointAngles, tiltDeg, ikSeedUsed, sampleZMm }) => ({ stage, kind, linkName, otherPrimitive, penetrationMm: penetrationMm === null ? null : round(penetrationMm, 2), sampleZMm: sampleZMm == null ? null : round(sampleZMm, 1), renderedUpperArmHalfHeightMm: linkName === 'upper-arm' ? renderedUpperArmVerticalHalfHeightMm(jointAngles) : null, jointAngles, tiltDeg: tiltDeg === null ? null : round(tiltDeg), ikSeedUsed })),
        });
      }
      console.log(`CASE (${position.x.toFixed(3)}, ${position.y.toFixed(3)}) yaw=${yawDeg}° => ${plan.reason}; final failure=${JSON.stringify(plan.failure)}`);
    }
    console.log('PER-TILT STRUCTURED FAILURES');
    console.table(rows.map(({ failureRecords: _records, ...row }) => row));
    const diagnosticPath = resolve(process.cwd(), 'src/robotics/generated/caseDiagnostics.json');
    mkdirSync(dirname(diagnosticPath), { recursive: true });
    writeFileSync(diagnosticPath, JSON.stringify(rows, null, 2));
    expect(rows).toHaveLength(32);
  });

  it.skipIf(process.env.RUN_PICK_SWEEP !== '1')('sweeps the complete 1 cm workspace and derives sector data', () => {
    const xs = Array.from({ length: 91 }, (_, i) => round(0.10 + i * 0.01, 2));
    const ys = Array.from({ length: 121 }, (_, i) => round(-0.60 + i * 0.01, 2));
    const cells = xs.flatMap((x) => ys.map((y) => ({ x, y, z: BOOK.initialPosition.z })));
    const yawData: Record<string, { strict: boolean[]; enabled: boolean[]; tiltDeg: (number | null)[]; failureKind: (string | null)[] }> = {};
    const counts: { yaw: number; strict: number; enabled: number; ms: number }[] = [];
    for (const yawDeg of yawsDeg) {
      const yaw = yawDeg * Math.PI / 180;
      const strict: boolean[] = [], enabled: boolean[] = [], tiltDeg: (number | null)[] = [], failureKind: (string | null)[] = [];
      const start = performance.now();
      for (const position of cells) {
        const destination = { x: 0.8, y: 0, yaw: 0 };
        const noTilt = createPickPlan(INITIAL_DH_TABLE, position, HOME_JOINT_ANGLES, destination, yaw, false);
        const tilted = createPickPlan(INITIAL_DH_TABLE, position, HOME_JOINT_ANGLES, destination, yaw, true);
        strict.push(noTilt.reachable);
        enabled.push(tilted.reachable);
        tiltDeg.push(tilted.reachable ? round(usedTilt(tilted), 1) : null);
        failureKind.push(tilted.reachable ? null : tilted.failure?.kind ?? tilted.reason);
      }
      const ms = performance.now() - start;
      yawData[String(yawDeg)] = { strict, enabled, tiltDeg, failureKind };
      counts.push({ yaw: yawDeg, strict: strict.filter(Boolean).length, enabled: enabled.filter(Boolean).length, ms: round(ms, 1) });
      console.log(`SWEEP yaw=${yawDeg}° strict=${strict.filter(Boolean).length}/${cells.length} tilt-enabled=${enabled.filter(Boolean).length}/${cells.length} elapsed=${(ms / 1000).toFixed(1)}s`);
    }

    const intersection = cells.map((_, i) => yawsDeg.every((yaw) => yawData[String(yaw)].enabled[i]));
    const polarAngles = Array.from({ length: 72 }, (_, i) => -180 + i * 5);
    const polarRadii = Array.from({ length: 116 }, (_, i) => i * 0.01);
    const at = (x: number, y: number) => {
      const ix = Math.round((x - xs[0]) / 0.01), iy = Math.round((y - ys[0]) / 0.01);
      return ix >= 0 && ix < xs.length && iy >= 0 && iy < ys.length && Math.hypot(xs[ix] - x, ys[iy] - y) <= 0.0072 && intersection[ix * ys.length + iy];
    };
    const polar = polarRadii.map((r) => polarAngles.map((a) => {
      const angle = a * Math.PI / 180;
      return at(r * Math.cos(angle), r * Math.sin(angle)) ? '#' : '.';
    }).join(''));
    console.log('ALL-YAW POLAR MAP (radius rows 0.00 to 1.15 m by 0.01; angle columns -177.5 to +177.5° by 5°)\n' + polar.join('\n'));

    const radialBands = polarAngles.map((angleDeg) => {
      const ok = polarRadii.map((_, ri) => polar[ri][Math.round((angleDeg + 177.5) / 5)] === '#');
      let inner = -1, outer = -1;
      for (let i = 0; i < ok.length; i++) if (ok[i]) { inner = i; while (i + 1 < ok.length && ok[i + 1]) i++; outer = i; break; }
      return { angleDeg, innerR: inner < 0 ? null : round(polarRadii[inner], 2), outerR: outer < 0 ? null : round(polarRadii[outer], 2) };
    });
    console.log('ALL-YAW RADIAL BANDS\n' + JSON.stringify(radialBands, null, 2));

    const halfDiagonal = Math.hypot(BOOK.size.x / 2, BOOK.size.y / 2);
    let sector = { innerR: 0, outerR: 0, halfSpanDeg: 0, areaM2: 0 };
    for (let half = 180; half >= 1; half--) for (let centerInner = 0.27; centerInner <= 0.95; centerInner += 0.01) {
      const inner = centerInner - halfDiagonal;
      if (inner <= 0) continue;
      const inset = Math.asin(Math.min(1, halfDiagonal / centerInner)) * 180 / Math.PI;
      const centerHalf = half - inset;
      if (centerHalf <= 0) continue;
      let end = -1;
      for (let r = centerInner; r <= 1.15; r += 0.01) {
        let ringGood = true;
        for (let a = -centerHalf; a <= centerHalf + 1e-8; a += 2.5) {
          if (!at(r * Math.cos(a * Math.PI / 180), r * Math.sin(a * Math.PI / 180))) { ringGood = false; break; }
        }
        if (!ringGood) break;
        end = Math.round(r * 100);
      }
      if (end < 0) continue;
      const outer = end / 100 + halfDiagonal;
      const area = 0.5 * (outer * outer - inner * inner) * (2 * half * Math.PI / 180);
      if (area > sector.areaM2) sector = { innerR: round(inner, 3), outerR: round(outer, 3), halfSpanDeg: half, areaM2: round(area, 6) };
    }
    const intersectionCount = intersection.filter(Boolean).length;
    const shrunkInner = sector.innerR + halfDiagonal, shrunkOuter = sector.outerR - halfDiagonal;
    const shrunkHalfSpan = sector.halfSpanDeg - Math.asin(Math.min(1, halfDiagonal / shrunkInner)) * 180 / Math.PI;
    let shrunkSectorGridCells = 0, shrunkSectorFailures = 0;
    for (let i = 0; i < cells.length; i++) {
      const r = Math.hypot(cells[i].x, cells[i].y), a = Math.abs(Math.atan2(cells[i].y, cells[i].x) * 180 / Math.PI);
      if (r >= shrunkInner - 1e-9 && r <= shrunkOuter + 1e-9 && a <= shrunkHalfSpan + 1e-9) {
        shrunkSectorGridCells++;
        if (!intersection[i]) shrunkSectorFailures++;
      }
    }
    expect(shrunkSectorFailures).toBe(0);
    const rectangleCellCount = cells.filter((p) => p.x >= TABLE.center.x - TABLE.width / 2 && p.x < TABLE.center.x + TABLE.width / 2 && p.y >= TABLE.center.y - TABLE.depth / 2 && p.y < TABLE.center.y + TABLE.depth / 2).length;
    const rectanglePct = round(intersectionCount / rectangleCellCount * 100, 2);
    const sectorJson = { ...sector, derivedFrom: 'fullPlannerSweep.test.ts; 1 cm XY grid; all-yaw intersection at 0,30,60,90,120,150,180 degrees; sector candidates sampled every 1 cm radius and 2.5 degrees; book half-diagonal radial and side inset', allYawPickableCells: intersectionCount, currentRectangleCells: rectangleCellCount, rectanglePct, shrunkSectorGridCells, shrunkSectorFailures, areaComparisonM2: { derivedSector: sector.areaM2, currentRectangle: 0.24, oldRectangle: 0.068 } };
    const out = resolve(process.cwd(), 'src/robotics/generated/sweep.json');
    mkdirSync(dirname(out), { recursive: true });
    writeFileSync(out, JSON.stringify({ grid: { xs, ys, stepM: 0.01 }, yawsDeg, perYaw: yawData, counts, intersection, polar: { radiiM: polarRadii, anglesDeg: polarAngles, rows: polar }, sector: sectorJson }, null, 2));
    writeFileSync(resolve(process.cwd(), 'src/robotics/generated/sector.json'), JSON.stringify({ innerR: sector.innerR, outerR: sector.outerR, halfSpanDeg: sector.halfSpanDeg, areaM2: sector.areaM2, derivedFrom: sectorJson.derivedFrom }, null, 2));
    console.log('ALL-YAW SECTOR ' + JSON.stringify(sectorJson));
    expect(counts).toHaveLength(7);
  }, 60 * 60 * 1000);
});
