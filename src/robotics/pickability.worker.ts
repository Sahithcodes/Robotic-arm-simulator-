import { INITIAL_DH_TABLE, setTablePreset } from '../robot/robotConfig';
import { HOME_JOINT_ANGLES } from './task5';
import { PickabilityWorkerConfig, pickabilityShape, computePickabilityChunk } from './pickabilityGrid';

type StartMessage = { type: 'start'; jobId: number; config: PickabilityWorkerConfig; stepsM: number[]; chunkSize?: number };
type CancelMessage = { type: 'cancel'; jobId: number };
type WorkerMessage = StartMessage | CancelMessage;
const cancelled = new Set<number>();
const scope = self as unknown as { onmessage: ((event: MessageEvent<WorkerMessage>) => void) | null; postMessage: (message: unknown, transfer?: Transferable[]) => void };

scope.onmessage = (event: MessageEvent<WorkerMessage>) => {
  const message = event.data;
  if (message.type === 'cancel') { cancelled.add(message.jobId); return; }
  cancelled.delete(message.jobId);
  runStages(message);
};

function runStages(message: StartMessage) {
  const { jobId, config } = message;
  const chunkSize = Math.max(1, Math.min(message.chunkSize ?? 8, 16));
  setTablePreset(config.preset);
  const workerConfig = { ...config, dhTable: config.dhTable?.length ? config.dhTable : INITIAL_DH_TABLE, jointAngles: config.jointAngles?.length ? config.jointAngles : HOME_JOINT_ANGLES };
  let stageIndex = 0;
  const nextStage = () => {
    if (cancelled.has(jobId)) { cancelled.delete(jobId); return; }
    if (stageIndex >= message.stepsM.length) { scope.postMessage({ type: 'jobComplete', jobId,mode:config.mode??'pick',surfaceId:config.surface?.id }); return; }
    const stepM = message.stepsM[stageIndex++];
    const shape = pickabilityShape(config.preset, stepM,config.surface);
    scope.postMessage({ type: 'stageStart', jobId, stepM, shape, startedAt: performance.now(),mode:config.mode??'pick',surfaceId:config.surface?.id });
    let start = 0;
    const processChunk = () => {
      if (cancelled.has(jobId)) { cancelled.delete(jobId); return; }
      const count = Math.min(chunkSize, shape.count - start);
      const chunk = computePickabilityChunk(workerConfig, shape, start, count);
      scope.postMessage({ type: 'chunk', jobId, stepM, start, states: chunk.states, reasons: chunk.reasons }, [chunk.states.buffer, chunk.reasons.buffer]);
      start += count;
      if (start < shape.count) setTimeout(processChunk, 0);
      else {
        scope.postMessage({ type: 'stageComplete', jobId, stepM, count: shape.count, totalMs: performance.now() - (scope as any).__stageStartedAt,mode:config.mode??'pick',surfaceId:config.surface?.id });
        setTimeout(nextStage, 0);
      }
    };
    (scope as any).__stageStartedAt = performance.now();
    setTimeout(processChunk, 0);
  };
  nextStage();
}
