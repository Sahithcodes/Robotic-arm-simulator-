'use client';

import React, { useMemo } from 'react';
import * as THREE from 'three';
import { Html } from '@react-three/drei';
import { BOOK, GRIPPER } from '../../robotics/task5';
import { FKResult, Matrix4x4, Vector3D } from '../../types/robotics';

interface RobotKinematicChainProps {
  fkResult: FKResult;
  isGripperOpen: boolean;
  showRobotDebug: boolean;
  showJointLabels?: boolean;
}

export function toThreeMatrix4(m: Matrix4x4): THREE.Matrix4 {
  const threeMat = new THREE.Matrix4();
  threeMat.set(
    m[0], m[1], m[2], m[3],
    m[4], m[5], m[6], m[7],
    m[8], m[9], m[10], m[11],
    m[12], m[13], m[14], m[15]
  );
  return threeMat;
}

function v(point: Vector3D): THREE.Vector3 {
  return new THREE.Vector3(point.x, point.y, point.z);
}

export function createLinkBetween(startPoint: Vector3D, endPoint: Vector3D, radius: number) {
  const start = v(startPoint);
  const end = v(endPoint);
  const direction = end.clone().sub(start);
  const length = direction.length();
  const midpoint = start.clone().add(end).multiplyScalar(0.5);
  const quaternion = new THREE.Quaternion().setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    direction.clone().normalize()
  );

  return { midpoint, quaternion, length, radius };
}

const jointColors = ['#c3873a', '#7895a3', '#7f9274', '#87919b', '#7f8c93', '#9aa5aa'];
const steel = '#7c878d';
const darkSteel = '#465158';

function jointAxis(cumulativeTransforms: Matrix4x4[], jointIndex: number): THREE.Vector3 {
  if (jointIndex <= 0) return new THREE.Vector3(0, 0, 1);
  const transform = cumulativeTransforms[jointIndex - 1];
  return new THREE.Vector3(transform[2], transform[6], transform[10]).normalize();
}

export const RobotKinematicChain: React.FC<RobotKinematicChainProps> = ({
  fkResult,
  isGripperOpen,
  showRobotDebug,
  showJointLabels = false,
}) => {
  const { jointPositions, cumulativeTransforms } = fkResult;
  const jointDisplayPositions = [...jointPositions.slice(1)];
  jointDisplayPositions[5] = jointPositions[5];
  const fkLine = useMemo(
    () => new THREE.BufferGeometry().setFromPoints(jointPositions.map(v)),
    [jointPositions]
  );

  return (
    <group>
      <mesh position={[0, 0, 0.035]} castShadow receiveShadow>
        <cylinderGeometry args={[0.16, 0.2, 0.07, 36]} />
        <meshStandardMaterial color="#31383d" metalness={0.65} roughness={0.32} />
      </mesh>
      {/* Floor pedestal and shoulder column meet the DH mounting origin at z=0.28 m. */}
      <mesh position={[0, 0, 0.14]} castShadow receiveShadow>
        <cylinderGeometry args={[0.14, 0.17, 0.28, 32]} />
        <meshStandardMaterial color="#30383d" metalness={0.65} roughness={0.34} />
      </mesh>
      <mesh position={[0, 0, 0.63]} castShadow receiveShadow>
        <cylinderGeometry args={[0.075, 0.095, 0.70, 28]} />
        <meshStandardMaterial color="#414b51" metalness={0.62} roughness={0.34} />
      </mesh>
      <mesh position={[0, 0, 0.105]} castShadow receiveShadow>
        <cylinderGeometry args={[0.105, 0.13, 0.07, 32]} />
        <meshStandardMaterial color="#3f494f" metalness={0.68} roughness={0.3} />
      </mesh>

      <mesh position={[0, 0, 0.21]} castShadow receiveShadow>
        <cylinderGeometry args={[0.066, 0.078, 0.14, 24]} />
        <meshStandardMaterial color="#414b51" metalness={0.62} roughness={0.34} />
      </mesh>

      <ColumnLink start={jointPositions[0]} end={jointPositions[1]} radius={0.055} color="#4a5258" />
      {jointDisplayPositions.map((point, index) => (
        <JointHousing
          key={`joint-${index + 1}`}
          point={point}
          axis={jointAxis(cumulativeTransforms, index)}
          color={jointColors[index]}
          label={`J${index + 1}`}
          showLabel={showJointLabels}
          variant={index < 1 ? 'shoulder' : index < 3 ? 'arm' : 'wrist'}
        />
      ))}

      <BeamLink start={jointPositions[1]} end={jointPositions[2]} size={[0.072, 0.05]} color="#6f7f86" />
      <BeamLink start={jointPositions[2]} end={jointPositions[3]} size={[0.062, 0.046]} color="#75886f" />
      <ColumnLink start={jointPositions[3]} end={jointPositions[4]} radius={0.026} color="#727985" />
      <ColumnLink start={jointPositions[4]} end={jointPositions[5]} radius={0.022} color="#7e7686" />
      <BeamLink start={jointPositions[5]} end={jointPositions[6]} size={[0.03, 0.026]} color="#6b8588" />

      {cumulativeTransforms[5] && (
        <EndEffectorMesh
          transform={cumulativeTransforms[5]}
          isOpen={isGripperOpen}
          showRobotDebug={showRobotDebug}
        />
      )}

      {showRobotDebug && (
        <>
          <line>
            <primitive object={fkLine} attach="geometry" />
            <lineBasicMaterial color="#f8d477" />
          </line>
          {jointPositions.map((point, index) => (
            <mesh key={`origin-${index}`} position={[point.x, point.y, point.z]}>
              <sphereGeometry args={[index === 0 ? 0.018 : 0.014, 14, 10]} />
              <meshStandardMaterial color={index === 6 ? '#f43f5e' : '#f8d477'} />
            </mesh>
          ))}
        </>
      )}
    </group>
  );
};

const ColumnLink: React.FC<{ start: Vector3D; end: Vector3D; radius: number; color: string }> = ({
  start,
  end,
  radius,
  color,
}) => {
  const link = createLinkBetween(start, end, radius);
  if (link.length < 0.002) return null;

  return (
    <mesh position={link.midpoint} quaternion={link.quaternion} castShadow receiveShadow>
      <cylinderGeometry args={[radius, radius, link.length, 18]} />
      <meshStandardMaterial color={color} metalness={0.55} roughness={0.36} />
    </mesh>
  );
};

const BeamLink: React.FC<{ start: Vector3D; end: Vector3D; size: [number, number]; color: string }> = ({
  start,
  end,
  size,
  color,
}) => {
  const link = createLinkBetween(start, end, 0);
  if (link.length < 0.002) return null;

  return (
    <group position={link.midpoint} quaternion={link.quaternion}>
      <mesh castShadow receiveShadow>
        <boxGeometry args={[size[0], link.length, size[1]]} />
        <meshStandardMaterial color={color} metalness={0.58} roughness={0.34} />
      </mesh>
      <mesh position={[0, 0, size[1] * 0.58]} castShadow receiveShadow>
        <boxGeometry args={[size[0] * 0.72, link.length * 0.9, size[1] * 0.18]} />
        <meshStandardMaterial color="#9aa6ad" metalness={0.5} roughness={0.38} />
      </mesh>
    </group>
  );
};

const JointHousing: React.FC<{
  point: Vector3D;
  axis: THREE.Vector3;
  color: string;
  label: string;
  variant: 'shoulder' | 'arm' | 'wrist';
  showLabel: boolean;
}> = ({ point, axis, color, label, variant, showLabel }) => {
  const dimensions = {
    shoulder: { radius: 0.068, length: 0.105, labelOffset: new THREE.Vector3(0.055, 0.025, 0.072) },
    arm: { radius: 0.056, length: 0.085, labelOffset: new THREE.Vector3(0.045, 0.02, 0.06) },
    wrist: { radius: 0.033, length: 0.052, labelOffset: new THREE.Vector3(0.034, 0.018, 0.042) },
  }[variant];
  const quaternion = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis);

  return (
    <group position={[point.x, point.y, point.z]}>
      <mesh quaternion={quaternion} castShadow receiveShadow>
        <cylinderGeometry args={[dimensions.radius, dimensions.radius, dimensions.length, 26]} />
        <meshStandardMaterial color={color} metalness={0.72} roughness={0.25} />
      </mesh>
      <mesh castShadow receiveShadow>
        <sphereGeometry args={[dimensions.radius * 0.72, 18, 12]} />
        <meshStandardMaterial color={darkSteel} metalness={0.62} roughness={0.32} />
      </mesh>
      {showLabel && (
        <Html distanceFactor={6.5} position={dimensions.labelOffset} center>
          <div className="joint-mark">{label}</div>
        </Html>
      )}
    </group>
  );
};

const EndEffectorMesh: React.FC<{
  transform: Matrix4x4;
  isOpen: boolean;
  showRobotDebug: boolean;
}> = ({ transform, isOpen, showRobotDebug }) => {
  const matrix = toThreeMatrix4(transform);
  const opening = isOpen ? GRIPPER.openWidth : GRIPPER.closedWidth;
  const fingerX = opening / 2;

  return (
    <group matrix={matrix} matrixAutoUpdate={false}>
      {/* T06 is the end of the final d6 link; the mount and gripper attach here. */}
      <mesh castShadow receiveShadow>
        <boxGeometry args={[0.065, 0.04, 0.03]} />
        <meshStandardMaterial color={steel} metalness={0.82} roughness={0.22} />
      </mesh>
      <mesh position={[0, 0, GRIPPER.fingerLength * 0.18]} castShadow receiveShadow>
        <boxGeometry args={[0.03, 0.026, 0.07]} />
        <meshStandardMaterial color="#606b72" metalness={0.75} roughness={0.28} />
      </mesh>
      <mesh position={[-fingerX, 0, GRIPPER.fingerLength / 2]} castShadow receiveShadow>
        <boxGeometry args={[GRIPPER.fingerThickness, 0.016, GRIPPER.fingerLength]} />
        <meshStandardMaterial color="#d2d8dc" metalness={0.75} roughness={0.2} />
      </mesh>
      <mesh position={[fingerX, 0, GRIPPER.fingerLength / 2]} castShadow receiveShadow>
        <boxGeometry args={[GRIPPER.fingerThickness, 0.016, GRIPPER.fingerLength]} />
        <meshStandardMaterial color="#d2d8dc" metalness={0.75} roughness={0.2} />
      </mesh>

      {showRobotDebug && <axesHelper args={[0.12]} />}
    </group>
  );
};

export const BookMesh: React.FC<{ highlighted?: boolean }> = ({ highlighted = false }) => (
  <group>
    <mesh castShadow receiveShadow>
      <boxGeometry args={[BOOK.size.x, BOOK.size.y, BOOK.size.z]} />
      <meshStandardMaterial color={highlighted ? '#d15a4f' : '#b84646'} emissive={highlighted ? '#512018' : '#000000'} roughness={0.55} />
    </mesh>
    <mesh position={[0, 0, BOOK.size.z * 0.53]}>
      <boxGeometry args={[BOOK.size.x * 0.92, BOOK.size.y * 0.88, 0.003]} />
      <meshStandardMaterial color="#f2e5c5" roughness={0.7} />
    </mesh>
  </group>
);
