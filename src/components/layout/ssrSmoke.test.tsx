import React from 'react';
import { renderToString } from 'react-dom/server';
import { describe, expect, it } from 'vitest';
import { TaskExecutionPanel } from '../TaskPanel/TaskExecutionPanel';
import { CanvasContainer } from '../RobotViewer/CanvasContainer';

describe('server rendering smoke checks', () => {
  it('renders the Task panel and viewport wrapper without creating a Canvas', () => {
    const task = renderToString(<TaskExecutionPanel />);
    const viewport = renderToString(<CanvasContainer />);
    expect(task.length).toBeGreaterThan(0);
    expect(viewport).toContain('Initializing 3D WebGL Kinematic Renderer');
  });
});
