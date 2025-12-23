import { useMemo } from 'react';
import { Html } from '@react-three/drei';
import * as THREE from 'three';
import useStore from '../../stores/modelStore';
import { createProfileFromSection } from './geometry/sectionProfiles';
import CargoBlock from './elements/CargoBlock';

// Extruded beam component with realistic cross-section
function ExtrudedBeam({
  start,
  end,
  sectionName,
  sectionProps,
  selected = false,
}: {
  start: [number, number, number];
  end: [number, number, number];
  sectionName: string;
  sectionProps: { A: number; Iy: number; Iz: number; J: number };
  selected?: boolean;
}) {
  const geometry = useMemo(() => {
    // Get section profile
    const profile = createProfileFromSection(sectionName, sectionProps);

    // Calculate beam direction and length
    const startVec = new THREE.Vector3(...start);
    const endVec = new THREE.Vector3(...end);
    const direction = endVec.clone().sub(startVec);
    const length = direction.length();

    // Create extrusion settings
    const extrudeSettings: THREE.ExtrudeGeometryOptions = {
      steps: 1,
      depth: length,
      bevelEnabled: false,
    };

    // Create extruded geometry
    const geom = new THREE.ExtrudeGeometry(profile.shape, extrudeSettings);

    // The extrusion is along Z by default, we need to rotate it to align with beam axis
    // First rotate 90 degrees around X to make extrusion go along Y (up)
    // Then we'll position and rotate the mesh to match the beam direction

    return geom;
  }, [start, end, sectionName, sectionProps]);

  // Calculate transformation to position beam correctly
  const { position, quaternion } = useMemo(() => {
    const startVec = new THREE.Vector3(...start);
    const endVec = new THREE.Vector3(...end);
    const direction = endVec.clone().sub(startVec).normalize();

    // Create quaternion to rotate from Z-axis to beam direction
    const q = new THREE.Quaternion();
    const zAxis = new THREE.Vector3(0, 0, 1);
    q.setFromUnitVectors(zAxis, direction);

    return {
      position: startVec,
      quaternion: q,
    };
  }, [start, end]);

  return (
    <mesh geometry={geometry} position={position} quaternion={quaternion}>
      <meshStandardMaterial
        color={selected ? '#ff8800' : '#a0a0a0'}
        metalness={0.4}
        roughness={0.6}
      />
    </mesh>
  );
}

// Simplified beam for performance (uses cylinder instead of extrusion)
function SimplifiedBeam({
  start,
  end,
  selected = false,
}: {
  start: [number, number, number];
  end: [number, number, number];
  selected?: boolean;
}) {
  const { position, quaternion, length } = useMemo(() => {
    const startVec = new THREE.Vector3(...start);
    const endVec = new THREE.Vector3(...end);
    const midpoint = startVec.clone().add(endVec).multiplyScalar(0.5);
    const direction = endVec.clone().sub(startVec);
    const len = direction.length();

    // Rotate from Y-axis (cylinder default) to beam direction
    const q = new THREE.Quaternion();
    const yAxis = new THREE.Vector3(0, 1, 0);
    q.setFromUnitVectors(yAxis, direction.normalize());

    return {
      position: midpoint,
      quaternion: q,
      length: len,
    };
  }, [start, end]);

  return (
    <mesh position={position} quaternion={quaternion}>
      <cylinderGeometry args={[0.05, 0.05, length, 8]} />
      <meshStandardMaterial
        color={selected ? '#ff8800' : '#708090'}
        metalness={0.5}
        roughness={0.4}
      />
    </mesh>
  );
}

// Joint node sphere
function JointNode({ position }: { position: [number, number, number] }) {
  return (
    <mesh position={position}>
      <sphereGeometry args={[0.08, 16, 16]} />
      <meshStandardMaterial color="#444" metalness={0.6} roughness={0.3} />
    </mesh>
  );
}

// Fixed support base
function FixedSupportRealistic({ position }: { position: [number, number, number] }) {
  return (
    <group position={position}>
      <mesh position={[0, -0.08, 0]}>
        <boxGeometry args={[0.4, 0.16, 0.4]} />
        <meshStandardMaterial color="#555" metalness={0.3} roughness={0.7} />
      </mesh>
      {/* Anchor bolts */}
      {[
        [-0.12, -0.16, -0.12],
        [0.12, -0.16, -0.12],
        [-0.12, -0.16, 0.12],
        [0.12, -0.16, 0.12],
      ].map((pos, i) => (
        <mesh key={i} position={pos as [number, number, number]}>
          <cylinderGeometry args={[0.02, 0.02, 0.1, 8]} />
          <meshStandardMaterial color="#333" metalness={0.8} roughness={0.2} />
        </mesh>
      ))}
    </group>
  );
}

export default function RealisticView() {
  const { beams, sections, boundaryConditions, cargos, selectedBeamId } = useStore();

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

  // Get section properties by name
  const getSectionProps = (sectionName: string) => {
    const section = sections.find((s) => s.name === sectionName);
    if (section) {
      return { A: section.A, Iy: section.Iy, Iz: section.Iz, J: section.J };
    }
    // Default properties
    return { A: 0.005, Iy: 1e-5, Iz: 1e-6, J: 1e-7 };
  };

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

  // Use simplified beams for large models, detailed for small
  const useSimplified = beams.length > 50;

  return (
    <group>
      {/* Beams */}
      {beams.map((beam) =>
        useSimplified ? (
          <SimplifiedBeam
            key={beam.id}
            start={beam.start as [number, number, number]}
            end={beam.end as [number, number, number]}
            selected={beam.id === selectedBeamId}
          />
        ) : (
          <ExtrudedBeam
            key={beam.id}
            start={beam.start as [number, number, number]}
            end={beam.end as [number, number, number]}
            sectionName={beam.section}
            sectionProps={getSectionProps(beam.section)}
            selected={beam.id === selectedBeamId}
          />
        )
      )}

      {/* Joint nodes */}
      {nodePositions.map((pos, i) => (
        <JointNode key={i} position={pos} />
      ))}

      {/* Fixed supports */}
      {boundaryConditions.length > 0 && beams.length > 0 && (
        <FixedSupportRealistic position={beams[0].start as [number, number, number]} />
      )}

      {/* Cargo blocks */}
      {cargos.map((cargo) => (
        <CargoBlock key={cargo.id} cargo={cargo} />
      ))}
    </group>
  );
}
