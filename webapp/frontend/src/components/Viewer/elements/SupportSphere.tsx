import { Sphere } from '@react-three/drei';

/**
 * Spherical support for cargo connections
 * Metallic appearance to indicate load transfer point
 */
export function SupportSphere({
  position,
  radius = 0.1,
}: {
  position: [number, number, number];
  radius?: number;
}) {
  return (
    <Sphere args={[radius, 16, 16]} position={position}>
      <meshStandardMaterial color="#333" metalness={0.8} roughness={0.2} />
    </Sphere>
  );
}

export default SupportSphere;
