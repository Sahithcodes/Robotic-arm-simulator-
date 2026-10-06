'use client';

import React from 'react';
import { Header } from './Header';
import { TabNavigation } from './TabNavigation';
import { CanvasContainer } from '../RobotViewer/CanvasContainer';
import { JointSliderPanel } from '../JointControls/JointSliderPanel';
import { DHTableDisplay } from '../KinematicsPanel/DHTableDisplay';
import { FKDisplay } from '../KinematicsPanel/FKDisplay';
import { MatrixDisplay } from '../KinematicsPanel/MatrixDisplay';
import { useSimulationStore } from '../../store/simulationStore';
import { radToDeg } from '../../robotics/transforms';
import { TaskExecutionPanel } from '../TaskPanel/TaskExecutionPanel';

export const MainLayout: React.FC = () => {
  const { activeTab, fkResult, jointAngles } = useSimulationStore();
  const { position, orientation } = fkResult.endEffectorPose;

  return (
    <div className="flex flex-col h-screen overflow-hidden bg-[#171a1d] text-slate-100">
      <Header />
      <TabNavigation />

      <main className="flex-1 grid grid-cols-1 lg:grid-cols-12 gap-3 p-3 min-h-0 overflow-hidden">
        <section className="lg:col-span-8 flex flex-col h-full min-h-[420px]">
          <CanvasContainer />
        </section>

        <section className="lg:col-span-4 flex flex-col h-full overflow-y-auto space-y-3 pr-1">
          {activeTab === 'robot' && (
            <div className="flex flex-col space-y-3 h-full">
              <div className="flex-1 min-h-[320px]">
                <JointSliderPanel />
              </div>
              <div>
                <FKDisplay />
              </div>
            </div>
          )}

          {activeTab === 'kinematics' && (
            <div className="flex flex-col space-y-3">
              <DHTableDisplay />
              <FKDisplay />
              <MatrixDisplay />
            </div>
          )}

          {activeTab === 'task' && (
            <div className="flex flex-col space-y-3">
              <TaskExecutionPanel />
              <JointSliderPanel />
              <FKDisplay />
            </div>
          )}
        </section>
      </main>

      <footer className="telemetry-strip">
        <span>EE: X={position.x.toFixed(3)} Y={position.y.toFixed(3)} Z={position.z.toFixed(3)} m</span>
        <span>
          RPY: {radToDeg(orientation.roll).toFixed(1)}deg {radToDeg(orientation.pitch).toFixed(1)}deg{' '}
          {radToDeg(orientation.yaw).toFixed(1)}deg
        </span>
        <span>
          {jointAngles.map((angle, index) => `J${index + 1}=${radToDeg(angle).toFixed(0)}deg`).join('  ')}
        </span>
      </footer>
    </div>
  );
};
