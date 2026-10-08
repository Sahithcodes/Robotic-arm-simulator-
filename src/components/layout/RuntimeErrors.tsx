'use client';

import React, { Component, ErrorInfo, ReactNode, useEffect, useState } from 'react';

type RuntimeError = { message: string; stack: string };
const subscribers = new Set<(error: RuntimeError) => void>();
export function reportRuntimeError(error: RuntimeError) {
  console.error('[Runtime error]', error.message, error.stack);
  subscribers.forEach((subscriber) => subscriber(error));
}

export class ErrorBoundary extends Component<{ children: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() { return { failed: true }; }
  componentDidCatch(error: Error, info: ErrorInfo) {
    reportRuntimeError({ message: error.message, stack: error.stack ?? info.componentStack ?? '' });
  }
  render() { return this.state.failed ? <div role="alert" className="rounded border border-red-500 p-2 text-red-300">This panel failed. See Errors below.</div> : this.props.children; }
}

export function RuntimeErrors() {
  const [errors, setErrors] = useState<RuntimeError[]>([]);
  useEffect(() => {
    const capture = (error: RuntimeError) => setErrors((current) => [...current, error]);
    subscribers.add(capture);
    const onError = (event: ErrorEvent) => reportRuntimeError({ message: event.message || 'Browser error', stack: event.error?.stack ?? `${event.filename}:${event.lineno}:${event.colno}` });
    const onRejection = (event: PromiseRejectionEvent) => {
      const reason = event.reason;
      reportRuntimeError({ message: reason instanceof Error ? reason.message : String(reason), stack: reason instanceof Error ? reason.stack ?? '' : '' });
    };
    window.addEventListener('error', onError);
    window.addEventListener('unhandledrejection', onRejection);
    return () => { subscribers.delete(capture); window.removeEventListener('error', onError); window.removeEventListener('unhandledrejection', onRejection); };
  }, []);
  if (!errors.length) return null;
  const details = errors.map(({ message, stack }, index) => `${index + 1}. ${message}\n${stack}`).join('\n\n');
  return <details className="fixed bottom-2 right-2 z-50 max-w-md rounded border border-red-500 bg-slate-950 p-2 text-xs text-red-200"><summary>Errors ({errors.length})</summary><pre className="max-h-48 overflow-auto whitespace-pre-wrap">{details}</pre><button type="button" onClick={() => void navigator.clipboard?.writeText(details)}>Copy errors</button></details>;
}
