import { useRef, useMemo } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * Center of Gravity indicator - naval architecture style
 * A circle divided into 4 quadrants: black/white alternating
 * Always faces the camera (billboard)
 */
export function CoGIndicator({
  position = [0, 0, 0],
  size = 0.3,
}: {
  position?: [number, number, number];
  size?: number;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  const { camera } = useThree();

  // Custom shader material for quadrant pattern
  const material = useMemo(
    () =>
      new THREE.ShaderMaterial({
        uniforms: {},
        vertexShader: `
        varying vec2 vUv;
        void main() {
          vUv = uv;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
      `,
        fragmentShader: `
        varying vec2 vUv;
        void main() {
          // Quadrant pattern:
          // Top-left: black, Top-right: white
          // Bottom-left: white, Bottom-right: black
          bool left = vUv.x < 0.5;
          bool top = vUv.y > 0.5;
          bool isBlack = (left && top) || (!left && !top);

          // Circle mask
          vec2 center = vUv - 0.5;
          float dist = length(center);
          if (dist > 0.5) discard;

          // Cross lines
          float lineWidth = 0.03;
          bool onLine = abs(vUv.x - 0.5) < lineWidth || abs(vUv.y - 0.5) < lineWidth;

          // Outer ring
          bool onRing = dist > 0.45 && dist <= 0.5;

          if (onLine || onRing) {
            gl_FragColor = vec4(0.4, 0.4, 0.4, 1.0);  // Gray lines
          } else {
            gl_FragColor = isBlack ? vec4(0.0, 0.0, 0.0, 1.0) : vec4(1.0, 1.0, 1.0, 1.0);
          }
        }
      `,
        side: THREE.DoubleSide,
      }),
    []
  );

  // Billboard effect - always face camera
  useFrame(() => {
    if (meshRef.current) {
      meshRef.current.quaternion.copy(camera.quaternion);
    }
  });

  return (
    <mesh ref={meshRef} position={position} material={material}>
      <planeGeometry args={[size, size]} />
    </mesh>
  );
}

export default CoGIndicator;
