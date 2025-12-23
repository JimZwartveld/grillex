import { useMemo, useState } from 'react';
import { Line, Sphere, Html } from '@react-three/drei';
import useStore from '../../stores/modelStore';

// Deformation scale slider component
function DeformationScaleSlider({
  value,
  onChange,
}: {
  value: number;
  onChange: (v: number) => void;
}) {
  return (
    <div className="bg-white/95 rounded-lg shadow-lg p-3 backdrop-blur-sm">
      <label className="text-xs font-medium text-gray-600 block mb-2">
        Deformation Scale: {value}x
      </label>
      <input
        type="range"
        min="1"
        max="500"
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        className="w-32 h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
      />
    </div>
  );
}

// Displacement color legend
function DisplacementLegend({
  maxDisplacement,
}: {
  maxDisplacement: number;
}) {
  const colors = ['#22c55e', '#84cc16', '#eab308', '#f97316', '#ef4444'];
  const labels = [0, 0.25, 0.5, 0.75, 1].map((f) => (f * maxDisplacement * 1000).toFixed(2));

  return (
    <div className="bg-white/95 rounded-lg shadow-lg p-3 backdrop-blur-sm">
      <div className="text-xs font-medium text-gray-600 mb-2">
        Displacement (mm)
      </div>
      <div className="flex flex-col gap-0.5">
        {colors.map((color, i) => (
          <div key={i} className="flex items-center gap-2">
            <div
              className="w-4 h-3 rounded-sm"
              style={{ backgroundColor: color }}
            />
            <span className="text-xs text-gray-600">{labels[i]}</span>
          </div>
        )).reverse()}
      </div>
    </div>
  );
}

// Get color based on displacement magnitude
function getDisplacementColor(displacement: number, maxDisplacement: number): string {
  if (maxDisplacement === 0) return '#22c55e';

  const ratio = Math.min(displacement / maxDisplacement, 1);

  // Green -> Yellow -> Orange -> Red gradient
  if (ratio < 0.25) {
    return '#22c55e'; // Green
  } else if (ratio < 0.5) {
    return '#84cc16'; // Lime
  } else if (ratio < 0.75) {
    return '#eab308'; // Yellow/Orange
  } else if (ratio < 0.9) {
    return '#f97316'; // Orange
  } else {
    return '#ef4444'; // Red
  }
}

// Deflected node sphere
function DeflectedNodePoint({
  originalPosition,
  displacement,
  scale,
  color,
}: {
  originalPosition: [number, number, number];
  displacement: [number, number, number];
  scale: number;
  color: string;
}) {
  const position = useMemo(
    () =>
      [
        originalPosition[0] + displacement[0] * scale,
        originalPosition[1] + displacement[1] * scale,
        originalPosition[2] + displacement[2] * scale,
      ] as [number, number, number],
    [originalPosition, displacement, scale]
  );

  return (
    <Sphere args={[0.06]} position={position}>
      <meshStandardMaterial color={color} />
    </Sphere>
  );
}

// Deflected beam line
function DeflectedBeamLine({
  startPos,
  endPos,
  startDisp,
  endDisp,
  scale,
  color,
}: {
  startPos: [number, number, number];
  endPos: [number, number, number];
  startDisp: [number, number, number];
  endDisp: [number, number, number];
  scale: number;
  color: string;
}) {
  const points = useMemo(() => {
    // Create interpolated points along the beam for smoother visualization
    const numPoints = 10;
    const result: [number, number, number][] = [];

    for (let i = 0; i <= numPoints; i++) {
      const t = i / numPoints;

      // Interpolate original position
      const x = startPos[0] + t * (endPos[0] - startPos[0]);
      const y = startPos[1] + t * (endPos[1] - startPos[1]);
      const z = startPos[2] + t * (endPos[2] - startPos[2]);

      // Interpolate displacement (linear for now, could add cubic interpolation)
      const dx = startDisp[0] + t * (endDisp[0] - startDisp[0]);
      const dy = startDisp[1] + t * (endDisp[1] - startDisp[1]);
      const dz = startDisp[2] + t * (endDisp[2] - startDisp[2]);

      result.push([x + dx * scale, y + dy * scale, z + dz * scale]);
    }

    return result;
  }, [startPos, endPos, startDisp, endDisp, scale]);

  return <Line points={points} color={color} lineWidth={3} />;
}

// Original beam shown faded
function OriginalBeamLine({
  start,
  end,
}: {
  start: [number, number, number];
  end: [number, number, number];
}) {
  return <Line points={[start, end]} color="#999" lineWidth={1} opacity={0.4} transparent />;
}

// Original node shown faded
function OriginalNodePoint({ position }: { position: [number, number, number] }) {
  return (
    <Sphere args={[0.04]} position={position}>
      <meshStandardMaterial color="#999" transparent opacity={0.4} />
    </Sphere>
  );
}

export default function ResultsView() {
  const { beams, nodes, results, isAnalyzed } = useStore();
  const [scale, setScale] = useState(100);

  // Get unique node positions from beams
  const nodePositions = useMemo(() => {
    const positions = new Map<string, { position: [number, number, number]; nodeId: number }>();
    let nodeIndex = 0;

    beams.forEach((beam) => {
      const startKey = beam.start.join(',');
      const endKey = beam.end.join(',');

      if (!positions.has(startKey)) {
        positions.set(startKey, {
          position: beam.start as [number, number, number],
          nodeId: nodeIndex++,
        });
      }
      if (!positions.has(endKey)) {
        positions.set(endKey, {
          position: beam.end as [number, number, number],
          nodeId: nodeIndex++,
        });
      }
    });

    return Array.from(positions.values());
  }, [beams]);

  // Calculate maximum displacement for color scaling
  const maxDisplacement = useMemo(() => {
    if (!results?.displacements) return 0.01;

    let max = 0;
    Object.values(results.displacements).forEach((disp) => {
      if (disp && disp.length >= 3) {
        const magnitude = Math.sqrt(disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2);
        max = Math.max(max, magnitude);
      }
    });

    return max || 0.01;
  }, [results]);

  // Get displacement for a node position
  const getDisplacementAt = (position: [number, number, number]): [number, number, number] => {
    if (!results?.displacements) return [0, 0, 0];

    // Find matching node in nodes array
    const node = nodes.find(
      (n) =>
        Math.abs(n.position[0] - position[0]) < 0.001 &&
        Math.abs(n.position[1] - position[1]) < 0.001 &&
        Math.abs(n.position[2] - position[2]) < 0.001
    );

    if (node && results.displacements[node.id]) {
      const disp = results.displacements[node.id];
      return [disp[0] || 0, disp[1] || 0, disp[2] || 0];
    }

    // Fallback: find by index in nodePositions
    const posKey = position.join(',');
    const idx = nodePositions.findIndex((n) => n.position.join(',') === posKey);
    if (idx >= 0 && results.displacements[idx]) {
      const disp = results.displacements[idx];
      return [disp[0] || 0, disp[1] || 0, disp[2] || 0];
    }

    return [0, 0, 0];
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

  if (!isAnalyzed || !results) {
    return (
      <Html center>
        <div className="text-amber-600 text-center p-4 bg-white rounded-lg shadow-sm">
          <p className="text-sm mb-1">No analysis results</p>
          <p className="text-xs">Run analysis to see deflected shape</p>
        </div>
      </Html>
    );
  }

  return (
    <group>
      {/* UI Overlays */}
      <Html position={[-5, 4, 0]} style={{ pointerEvents: 'auto' }}>
        <DeformationScaleSlider value={scale} onChange={setScale} />
      </Html>

      <Html position={[5, -3, 0]} style={{ pointerEvents: 'none' }}>
        <DisplacementLegend maxDisplacement={maxDisplacement} />
      </Html>

      {/* Original shape (faded) */}
      <group>
        {beams.map((beam) => (
          <OriginalBeamLine
            key={`orig-${beam.id}`}
            start={beam.start as [number, number, number]}
            end={beam.end as [number, number, number]}
          />
        ))}
        {nodePositions.map((node, i) => (
          <OriginalNodePoint key={`orig-node-${i}`} position={node.position} />
        ))}
      </group>

      {/* Deflected shape */}
      <group>
        {beams.map((beam) => {
          const startDisp = getDisplacementAt(beam.start as [number, number, number]);
          const endDisp = getDisplacementAt(beam.end as [number, number, number]);
          const avgMagnitude =
            (Math.sqrt(startDisp[0] ** 2 + startDisp[1] ** 2 + startDisp[2] ** 2) +
              Math.sqrt(endDisp[0] ** 2 + endDisp[1] ** 2 + endDisp[2] ** 2)) /
            2;
          const color = getDisplacementColor(avgMagnitude, maxDisplacement);

          return (
            <DeflectedBeamLine
              key={`def-${beam.id}`}
              startPos={beam.start as [number, number, number]}
              endPos={beam.end as [number, number, number]}
              startDisp={startDisp}
              endDisp={endDisp}
              scale={scale}
              color={color}
            />
          );
        })}

        {nodePositions.map((node, i) => {
          const disp = getDisplacementAt(node.position);
          const magnitude = Math.sqrt(disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2);
          const color = getDisplacementColor(magnitude, maxDisplacement);

          return (
            <DeflectedNodePoint
              key={`def-node-${i}`}
              originalPosition={node.position}
              displacement={disp}
              scale={scale}
              color={color}
            />
          );
        })}
      </group>
    </group>
  );
}
