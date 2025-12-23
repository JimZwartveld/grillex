import { useMemo, useCallback } from 'react';
import { Line, Sphere, Text, Html } from '@react-three/drei';
import * as THREE from 'three';
import useStore from '../../stores/modelStore';
import CargoBlock from './elements/CargoBlock';

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

// Clickable beam line component
function BeamLine({
  id,
  start,
  end,
  selected = false,
  label,
  onClick,
  onContextMenu,
}: {
  id: number;
  start: [number, number, number];
  end: [number, number, number];
  selected?: boolean;
  label?: string;
  onClick?: () => void;
  onContextMenu?: (e: React.MouseEvent) => void;
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

  // Create a clickable mesh along the beam
  const beamDir = useMemo(() => {
    const dir = new THREE.Vector3(
      end[0] - start[0],
      end[1] - start[1],
      end[2] - start[2]
    );
    return dir;
  }, [start, end]);

  const length = beamDir.length();
  const quaternion = useMemo(() => {
    const q = new THREE.Quaternion();
    q.setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      beamDir.clone().normalize()
    );
    return q;
  }, [beamDir]);

  // Suppress id warning
  void id;

  return (
    <group>
      {/* Visible line */}
      <Line
        points={[start, end]}
        color={selected ? '#ff6600' : '#0066ff'}
        lineWidth={selected ? 4 : 2}
      />

      {/* Invisible clickable cylinder */}
      <mesh
        position={[midpoint.x, midpoint.y, midpoint.z]}
        quaternion={quaternion}
        onClick={(e) => {
          e.stopPropagation();
          onClick?.();
        }}
        onContextMenu={(e) => {
          e.stopPropagation();
          // Pass the native event for position
          if (e.nativeEvent) {
            onContextMenu?.(e.nativeEvent as unknown as React.MouseEvent);
          }
        }}
        onPointerOver={(e) => {
          e.stopPropagation();
          document.body.style.cursor = 'pointer';
        }}
        onPointerOut={() => {
          document.body.style.cursor = 'default';
        }}
      >
        <cylinderGeometry args={[0.08, 0.08, length, 8]} />
        <meshBasicMaterial transparent opacity={0} />
      </mesh>

      {/* Label */}
      {label && (
        <Text
          position={[midpoint.x, midpoint.y + 0.25, midpoint.z]}
          fontSize={0.15}
          color={selected ? '#ff6600' : '#666'}
          anchorX="center"
          anchorY="bottom"
        >
          {label}
        </Text>
      )}
    </group>
  );
}

// Fixed support symbol - Z-up coordinate system
// Ground is at Z=0, support renders below the node
function FixedSupport({ position }: { position: [number, number, number] }) {
  return (
    <group position={position}>
      {/* Ground base - flat on XY plane, below node in Z */}
      <mesh position={[0, 0, -0.05]}>
        <boxGeometry args={[0.3, 0.3, 0.1]} />
        <meshStandardMaterial color="#666" />
      </mesh>
      {/* Hatching lines (simplified) - extending down in -Z direction */}
      <Line
        points={[
          [-0.15, -0.15, -0.1],
          [-0.25, -0.25, -0.2],
        ]}
        color="#444"
        lineWidth={1}
      />
      <Line
        points={[
          [0, 0, -0.1],
          [-0.1, -0.1, -0.2],
        ]}
        color="#444"
        lineWidth={1}
      />
      <Line
        points={[
          [0.15, 0.15, -0.1],
          [0.05, 0.05, -0.2],
        ]}
        color="#444"
        lineWidth={1}
      />
    </group>
  );
}

// Pinned support symbol - Z-up coordinate system
// Triangle points up (in +Z direction), base at ground
function PinnedSupport({ position }: { position: [number, number, number] }) {
  return (
    <group position={position}>
      {/* Cone pointing up (+Z), positioned below the node */}
      {/* Rotate -90 degrees around X to make cone point in +Z instead of +Y */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, -0.1]}>
        <coneGeometry args={[0.12, 0.2, 3]} />
        <meshStandardMaterial color="#666" />
      </mesh>
    </group>
  );
}

// Load arrow component
export function LoadArrow({
  position,
  dof,
  value,
}: {
  position: [number, number, number];
  dof: number;
  value: number;
}) {
  const { direction, length, color } = useMemo(() => {
    // DOF to direction mapping
    const dirs: [number, number, number][] = [
      [1, 0, 0],  // UX
      [0, 1, 0],  // UY
      [0, 0, 1],  // UZ
      [1, 0, 0],  // RX (moment)
      [0, 1, 0],  // RY (moment)
      [0, 0, 1],  // RZ (moment)
    ];
    const baseDir = dirs[dof] || [0, -1, 0];
    const dir = new THREE.Vector3(...baseDir);

    // Scale by sign of value
    if (value < 0) {
      dir.multiplyScalar(-1);
    }

    return {
      direction: dir,
      length: Math.min(Math.max(Math.abs(value) * 0.08, 0.3), 1.5),
      color: dof < 3 ? (value > 0 ? '#22c55e' : '#ef4444') : '#8b5cf6', // Purple for moments
    };
  }, [dof, value]);

  return (
    <group position={position}>
      <arrowHelper
        args={[direction, new THREE.Vector3(0, 0, 0), length, color, 0.15, 0.08]}
      />
      <Html position={direction.clone().multiplyScalar(length * 1.3)} center>
        <div className="text-xs bg-white px-1.5 py-0.5 rounded shadow-sm border border-gray-200 whitespace-nowrap font-medium">
          {Math.abs(value).toFixed(1)} {dof < 3 ? 'kN' : 'kNm'}
        </div>
      </Html>
    </group>
  );
}

// DOF string to index mapping
function dofToIndex(dof: string | number): number {
  if (typeof dof === 'number') return dof;
  const mapping: Record<string, number> = {
    'UX': 0, 'UY': 1, 'UZ': 2,
    'RX': 3, 'RY': 4, 'RZ': 5,
  };
  return mapping[dof] ?? 2; // Default to UZ
}

// Ground plane for capturing empty space clicks
function GroundPlane({ onContextMenu }: { onContextMenu: (position: [number, number, number], event: React.MouseEvent) => void }) {
  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}  // Rotate to XY plane for Z-up
      position={[0, 0, 0]}
      onContextMenu={(e) => {
        e.stopPropagation();
        const point = e.point;
        // Snap to nearest 0.5m grid
        const snappedPoint: [number, number, number] = [
          Math.round(point.x * 2) / 2,
          Math.round(point.y * 2) / 2,
          Math.round(point.z * 2) / 2,
        ];
        if (e.nativeEvent) {
          onContextMenu(snappedPoint, e.nativeEvent as unknown as React.MouseEvent);
        }
      }}
      onPointerOver={() => {
        document.body.style.cursor = 'crosshair';
      }}
      onPointerOut={() => {
        document.body.style.cursor = 'default';
      }}
    >
      <planeGeometry args={[100, 100]} />
      <meshBasicMaterial transparent opacity={0} />
    </mesh>
  );
}

export default function FEMView({ showDeflected: _showDeflected = false, showRealistic: _showRealistic = false }: Props) {
  void _showDeflected; void _showRealistic; // Reserved for future implementation

  const { beams, boundaryConditions, loadCases, cargos, selectedBeamId, selectBeam, openContextMenu, openAddContextMenu, contextMenu, activeLoadCaseId } = useStore();

  const handleGroundContextMenu = useCallback((position: [number, number, number], e: React.MouseEvent) => {
    e.preventDefault();
    openAddContextMenu(e.clientX, e.clientY, position);
  }, [openAddContextMenu]);

  const handleBeamClick = useCallback((beamId: number) => {
    selectBeam(selectedBeamId === beamId ? null : beamId);
  }, [selectedBeamId, selectBeam]);

  const handleBeamContextMenu = useCallback((beamId: number, e: React.MouseEvent) => {
    e.preventDefault();
    openContextMenu(e.clientX, e.clientY, 'beam', beamId);
  }, [openContextMenu]);

  const handleCargoContextMenu = useCallback((cargoId: number, e: React.MouseEvent) => {
    e.preventDefault();
    openContextMenu(e.clientX, e.clientY, 'cargo', cargoId);
  }, [openContextMenu]);

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

  // Get load data from load cases, filtered by activeLoadCaseId
  const loads = useMemo(() => {
    const result: Array<{
      position: [number, number, number];
      dof: number;
      value: number;
    }> = [];

    // Filter load cases by activeLoadCaseId (null = show all)
    const filteredLoadCases = activeLoadCaseId === null
      ? loadCases
      : loadCases.filter((lc, idx) => (lc.id ?? idx) === activeLoadCaseId);

    // Create loads at beam endpoints based on position matching
    filteredLoadCases.forEach((lc) => {
      lc.loads.forEach((load) => {
        // Find the node position for this load
        const nodePos = nodePositions[load.node_id];
        if (nodePos) {
          result.push({
            position: nodePos,
            dof: dofToIndex(load.dof),
            value: load.value,
          });
        }
      });
    });

    return result;
  }, [loadCases, nodePositions, activeLoadCaseId]);

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
      {/* Ground plane for right-click context menu on empty space */}
      <GroundPlane onContextMenu={handleGroundContextMenu} />

      {/* Beam elements */}
      {beams.map((beam) => (
        <BeamLine
          key={beam.id}
          id={beam.id}
          start={beam.start as [number, number, number]}
          end={beam.end as [number, number, number]}
          selected={beam.id === selectedBeamId}
          label={`B${beam.id}`}
          onClick={() => handleBeamClick(beam.id)}
          onContextMenu={(e) => handleBeamContextMenu(beam.id, e)}
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

      {/* Load arrows */}
      {loads.map((load, i) => (
        <LoadArrow
          key={i}
          position={load.position}
          dof={load.dof}
          value={load.value}
        />
      ))}

      {/* Cargo blocks */}
      {cargos.map((cargo) => (
        <CargoBlock
          key={cargo.id}
          cargo={cargo}
          selected={contextMenu.elementType === 'cargo' && contextMenu.elementId === cargo.id}
          onContextMenu={(e) => handleCargoContextMenu(cargo.id, e)}
        />
      ))}
    </group>
  );
}
