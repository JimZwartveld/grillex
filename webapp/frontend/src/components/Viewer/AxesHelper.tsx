import { Text } from '@react-three/drei';

export default function AxesHelper() {
  const axisLength = 2;
  const labelOffset = 0.3;

  return (
    <group>
      {/* Axes lines */}
      <axesHelper args={[axisLength]} />

      {/* X axis label */}
      <Text
        position={[axisLength + labelOffset, 0, 0]}
        fontSize={0.2}
        color="red"
        anchorX="center"
        anchorY="middle"
      >
        X
      </Text>

      {/* Y axis label */}
      <Text
        position={[0, axisLength + labelOffset, 0]}
        fontSize={0.2}
        color="green"
        anchorX="center"
        anchorY="middle"
      >
        Y
      </Text>

      {/* Z axis label */}
      <Text
        position={[0, 0, axisLength + labelOffset]}
        fontSize={0.2}
        color="blue"
        anchorX="center"
        anchorY="middle"
      >
        Z
      </Text>
    </group>
  );
}
