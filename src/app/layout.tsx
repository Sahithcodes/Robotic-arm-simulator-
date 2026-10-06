import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: '6-DOF Robotic Arm Simulator | Task-Oriented Design',
  description: 'Professional 3D Robotic Arm Simulator based on Chang & Park (2003) Task-Oriented Design methodology.',
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en" className="dark">
      <body className="antialiased bg-slate-950 text-slate-100">
        {children}
      </body>
    </html>
  );
}
