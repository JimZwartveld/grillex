import { useMemo, useState } from 'react';
import { Box, Text } from '@react-three/drei';
import * as THREE from 'three';
import type { Cargo } from '../../../types/model';
import CoGIndicator from './CoGIndicator';
import SupportSphere from './SupportSphere';

interface Props {
  cargo: Cargo;
  selected?: boolean;
  onClick?: () => void;
  onContextMenu?: (e: React.MouseEvent) => void;
}

/**
 * 3D cargo block visualization - Z-up coordinate system
 * Semi-transparent cube with edges, CoG indicator, and support spheres
 * Dimensions: [lengthX, widthY, heightZ] - height is vertical (Z-up)
 */
export function CargoBlock({ cargo, selected = false, onClick, onContextMenu }: Props) {
  const [hovered, setHovered] = useState(false);
  const dimensions = cargo.dimensions || [2, 2, 2];
  // For Z-up: [lengthX, widthY, heightZ]
  const [lengthX, widthY, heightZ] = dimensions;

  // Create edges geometry for visibility
  const edgesGeometry = useMemo(() => {
    const boxGeom = new THREE.BoxGeometry(lengthX, widthY, heightZ);
    return new THREE.EdgesGeometry(boxGeom);
  }, [lengthX, widthY, heightZ]);

  // Determine color based on selection/hover state
  const boxColor = selected ? '#ff6600' : hovered ? '#d4a373' : '#8B4513';
  const opacity = selected || hovered ? 0.8 : 0.6;

  return (
    <group position={cargo.cogPosition}>
      {/* Cargo cube - semi-transparent, interactive */}
      <Box
        args={dimensions as [number, number, number]}
        onClick={(e) => {
          e.stopPropagation();
          onClick?.();
        }}
        onContextMenu={(e) => {
          e.stopPropagation();
          if (e.nativeEvent) {
            onContextMenu?.(e.nativeEvent as unknown as React.MouseEvent);
          }
        }}
        onPointerOver={(e) => {
          e.stopPropagation();
          setHovered(true);
          document.body.style.cursor = 'pointer';
        }}
        onPointerOut={() => {
          setHovered(false);
          document.body.style.cursor = 'default';
        }}
      >
        <meshStandardMaterial
          color={boxColor}
          opacity={opacity}
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

      {/* Cargo label - positioned above cargo in +Z direction (Z-up) */}
      <Text
        position={[0, 0, heightZ / 2 + 0.3]}
        fontSize={0.2}
        color="#333"
        anchorX="center"
        anchorY="bottom"
      >
        {cargo.name} ({cargo.mass.toFixed(1)} mT)
      </Text>

      {/* Support spheres at connection points - bottom is -Z direction */}
      {cargo.connections.map((conn, i) => (
        <SupportSphere
          key={i}
          position={conn.cargoOffset || [0, 0, -heightZ / 2]}
          radius={0.08}
        />
      ))}
    </group>
  );
}

export default CargoBlock;
