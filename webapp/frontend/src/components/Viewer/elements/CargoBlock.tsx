import { useMemo } from 'react';
import { Box, Text } from '@react-three/drei';
import * as THREE from 'three';
import type { Cargo } from '../../../types/model';
import CoGIndicator from './CoGIndicator';
import SupportSphere from './SupportSphere';

/**
 * 3D cargo block visualization
 * Semi-transparent cube with edges, CoG indicator, and support spheres
 */
export function CargoBlock({ cargo }: { cargo: Cargo }) {
  const dimensions = cargo.dimensions || [2, 2, 2];
  const [width, height, depth] = dimensions;

  // Create edges geometry for visibility
  const edgesGeometry = useMemo(() => {
    const boxGeom = new THREE.BoxGeometry(width, height, depth);
    return new THREE.EdgesGeometry(boxGeom);
  }, [width, height, depth]);

  return (
    <group position={cargo.cogPosition}>
      {/* Cargo cube - semi-transparent */}
      <Box args={dimensions as [number, number, number]}>
        <meshStandardMaterial
          color="#8B4513"
          opacity={0.6}
          transparent
          side={THREE.DoubleSide}
        />
      </Box>

      {/* Edges for visibility */}
      <lineSegments geometry={edgesGeometry}>
        <lineBasicMaterial color="#000" linewidth={1} />
      </lineSegments>

      {/* CoG indicator at center */}
      <CoGIndicator position={[0, 0, 0]} size={0.4} />

      {/* Cargo label */}
      <Text
        position={[0, height / 2 + 0.3, 0]}
        fontSize={0.2}
        color="#333"
        anchorX="center"
        anchorY="bottom"
      >
        {cargo.name} ({cargo.mass.toFixed(1)} mT)
      </Text>

      {/* Support spheres at connection points */}
      {cargo.connections.map((conn, i) => (
        <SupportSphere
          key={i}
          position={conn.cargoOffset || [0, -height / 2, 0]}
          radius={0.08}
        />
      ))}
    </group>
  );
}

export default CargoBlock;
