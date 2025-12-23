import { Text } from '@react-three/drei';

/**
 * Axes helper for Z-up coordinate system.
 * X (red) - horizontal, along length
 * Y (green) - horizontal, transverse
 * Z (blue) - vertical (UP)
 */
export default function AxesHelper() {
  const axisLength = 2;
  const labelOffset = 0.3;

  return (
    <group>
      {/* Axes lines - Three.js default axes helper */}
      <axesHelper args={[axisLength]} />

      {/* X axis label (red - horizontal) */}
      <Text
        position={[axisLength + labelOffset, 0, 0]}
        fontSize={0.2}
        color="red"
        anchorX="center"
        anchorY="middle"
      >
        X
      </Text>

      {/* Y axis label (green - horizontal) */}
      <Text
        position={[0, axisLength + labelOffset, 0]}
        fontSize={0.2}
        color="green"
        anchorX="center"
        anchorY="middle"
      >
        Y
      </Text>

      {/* Z axis label (blue - vertical UP) */}
      <Text
        position={[0, 0, axisLength + labelOffset]}
        fontSize={0.2}
        color="blue"
        anchorX="center"
        anchorY="middle"
      >
        Z (up)
      </Text>
    </group>
  );
}
