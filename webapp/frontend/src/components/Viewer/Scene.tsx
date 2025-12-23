import { OrbitControls, Grid, PerspectiveCamera } from '@react-three/drei';
import AxesHelper from './AxesHelper';
import FEMView from './FEMView';
import ResultsView from './ResultsView';
import RealisticView from './RealisticView';
import useStore from '../../stores/modelStore';

export default function Scene() {
  const { viewMode, beams } = useStore();

  // Calculate center of model for camera focus
  const modelCenter = beams.length > 0
    ? beams.reduce(
        (acc, beam) => [
          acc[0] + (beam.start[0] + beam.end[0]) / 2 / beams.length,
          acc[1] + (beam.start[1] + beam.end[1]) / 2 / beams.length,
          acc[2] + (beam.start[2] + beam.end[2]) / 2 / beams.length,
        ],
        [0, 0, 0]
      )
    : [0, 0, 0];

  // Calculate camera distance based on model size
  const modelSize = beams.length > 0
    ? Math.max(
        ...beams.map((b) =>
          Math.max(
            Math.abs(b.start[0] - modelCenter[0]),
            Math.abs(b.start[1] - modelCenter[1]),
            Math.abs(b.start[2] - modelCenter[2]),
            Math.abs(b.end[0] - modelCenter[0]),
            Math.abs(b.end[1] - modelCenter[1]),
            Math.abs(b.end[2] - modelCenter[2])
          )
        )
      )
    : 5;

  const cameraDistance = Math.max(modelSize * 3, 10);

  return (
    <>
      {/* Camera */}
      <PerspectiveCamera
        makeDefault
        position={[cameraDistance, cameraDistance * 0.8, cameraDistance]}
        fov={50}
      />

      {/* Controls */}
      <OrbitControls
        makeDefault
        target={modelCenter as [number, number, number]}
        enableDamping
        dampingFactor={0.1}
      />

      {/* Lighting */}
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} intensity={1} castShadow />
      <directionalLight position={[-10, -10, -5]} intensity={0.3} />

      {/* Ground grid */}
      <Grid
        position={[0, 0, 0]}
        args={[50, 50]}
        cellSize={1}
        cellThickness={0.5}
        cellColor="#d0d0d0"
        sectionSize={5}
        sectionThickness={1}
        sectionColor="#808080"
        fadeDistance={50}
        fadeStrength={1}
        followCamera={false}
        infiniteGrid
      />

      {/* Axes helper */}
      <AxesHelper />

      {/* Render view based on mode */}
      {viewMode === 'fem' && <FEMView />}
      {viewMode === 'results' && <ResultsView />}
      {viewMode === 'realistic' && <RealisticView />}
    </>
  );
}
