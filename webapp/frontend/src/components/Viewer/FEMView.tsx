import { useMemo } from 'react';
import { Line, Sphere, Text, Html } from '@react-three/drei';
import * as THREE from 'three';
import useStore from '../../stores/modelStore';

interface Props {
  showDeflected?: boolean;
  showRealistic?: boolean;
}

// Node point component
function NodePoint({ position }: { position: [number, number, number] }) {
  return (
    <Sphere args={[0.05]} position={position}>
      <meshStandardMaterial color="#333" />
    </Sphere>
  );
}

// Beam line component
function BeamLine({
  start,
  end,
  selected = false,
  label,
}: {
  start: [number, number, number];
  end: [number, number, number];
  selected?: boolean;
  label?: string;
}) {
  const midpoint = useMemo(
    () =>
      new THREE.Vector3(
        (start[0] + end[0]) / 2,
        (start[1] + end[1]) / 2,
        (start[2] + end[2]) / 2
      ),
    [start, end]
  );

  return (
    <group>
      <Line
        points={[start, end]}
        color={selected ? '#ff6600' : '#0066ff'}
        lineWidth={selected ? 3 : 2}
      />
      {label && (
        <Text
          position={[midpoint.x, midpoint.y + 0.2, midpoint.z]}
          fontSize={0.15}
          color="#666"
          anchorX="center"
          anchorY="bottom"
        >
          {label}
        </Text>
      )}
    </group>
  );
}

// Fixed support symbol
function FixedSupport({ position }: { position: [number, number, number] }) {
  return (
    <group position={position}>
      {/* Ground base */}
      <mesh position={[0, -0.05, 0]}>
        <boxGeometry args={[0.3, 0.1, 0.3]} />
        <meshStandardMaterial color="#666" />
      </mesh>
      {/* Hatching lines (simplified) */}
      <Line
        points={[
          [-0.15, -0.1, -0.15],
          [-0.25, -0.2, -0.25],
        ]}
        color="#444"
        lineWidth={1}
      />
      <Line
        points={[
          [0, -0.1, 0],
          [-0.1, -0.2, -0.1],
        ]}
        color="#444"
        lineWidth={1}
      />
      <Line
        points={[
          [0.15, -0.1, 0.15],
          [0.05, -0.2, 0.05],
        ]}
        color="#444"
        lineWidth={1}
      />
    </group>
  );
}

// Pinned support symbol
function PinnedSupport({ position }: { position: [number, number, number] }) {
  return (
    <group position={position}>
      <mesh rotation={[Math.PI, 0, 0]} position={[0, -0.1, 0]}>
        <coneGeometry args={[0.12, 0.2, 3]} />
        <meshStandardMaterial color="#666" />
      </mesh>
    </group>
  );
}

// Load arrow component - exported for use in Task 17.7
export function LoadArrow({
  position,
  dof,
  value,
}: {
  position: [number, number, number];
  dof: number;
  value: number;
}) {
  const direction = useMemo(() => {
    const dirs = [
      [1, 0, 0], // UX
      [0, 1, 0], // UY
      [0, 0, 1], // UZ
      [1, 0, 0], // RX
      [0, 1, 0], // RY
      [0, 0, 1], // RZ
    ];
    const dir = dirs[dof] || [0, -1, 0];
    return new THREE.Vector3(...dir).multiplyScalar(Math.sign(value));
  }, [dof, value]);

  const length = Math.min(Math.abs(value) * 0.1, 1.5);
  const color = value > 0 ? '#22c55e' : '#ef4444';

  return (
    <group position={position}>
      <arrowHelper
        args={[direction, new THREE.Vector3(0, 0, 0), length, color, 0.2, 0.1]}
      />
      <Html position={direction.clone().multiplyScalar(length * 1.2)} center>
        <div className="text-xs bg-white px-1 py-0.5 rounded shadow-sm border whitespace-nowrap">
          {Math.abs(value).toFixed(1)} kN
        </div>
      </Html>
    </group>
  );
}

export default function FEMView({ showDeflected: _showDeflected = false, showRealistic: _showRealistic = false }: Props) {
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  void _showDeflected; void _showRealistic; // Reserved for future implementation

  const { beams, boundaryConditions } = useStore();

  // Get unique node positions
  const nodePositions = useMemo(() => {
    const positions = new Map<string, [number, number, number]>();
    beams.forEach((beam) => {
      const startKey = beam.start.join(',');
      const endKey = beam.end.join(',');
      positions.set(startKey, beam.start as [number, number, number]);
      positions.set(endKey, beam.end as [number, number, number]);
    });
    return Array.from(positions.values());
  }, [beams]);

  if (beams.length === 0) {
    return (
      <Html center>
        <div className="text-gray-400 text-center p-4 bg-white rounded-lg shadow-sm">
          <p className="text-sm mb-1">No model data</p>
          <p className="text-xs">Create beams to see the model</p>
        </div>
      </Html>
    );
  }

  return (
    <group>
      {/* Beam elements */}
      {beams.map((beam) => (
        <BeamLine
          key={beam.id}
          start={beam.start as [number, number, number]}
          end={beam.end as [number, number, number]}
          label={`B${beam.id}`}
        />
      ))}

      {/* Nodes */}
      {nodePositions.map((pos, i) => (
        <NodePoint key={i} position={pos} />
      ))}

      {/* Boundary conditions - Fixed supports at start of first beam for now */}
      {boundaryConditions.length > 0 && beams.length > 0 && (
        <FixedSupport position={beams[0].start as [number, number, number]} />
      )}

      {/* If no BCs but we have beams, show a pinned support at first node */}
      {boundaryConditions.length === 0 && beams.length > 0 && nodePositions.length > 0 && (
        <PinnedSupport position={nodePositions[0]} />
      )}
    </group>
  );
}
