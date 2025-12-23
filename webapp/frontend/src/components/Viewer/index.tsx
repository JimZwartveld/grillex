import { Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { Loader } from '@react-three/drei';
import ViewModeSelector from './ViewModeSelector';
import Scene from './Scene';

export default function Viewer() {
  return (
    <div className="w-full h-full relative bg-gradient-to-b from-slate-100 to-slate-200">
      {/* View mode selector */}
      <ViewModeSelector />

      {/* Three.js canvas */}
      <Canvas
        shadows
        gl={{ antialias: true, alpha: true }}
        dpr={[1, 2]}
      >
        <Suspense fallback={null}>
          <Scene />
        </Suspense>
      </Canvas>

      {/* Loading indicator */}
      <Loader />

      {/* Controls hint */}
      <div className="absolute bottom-4 left-4 text-xs text-gray-500 bg-white bg-opacity-80 px-2 py-1 rounded">
        Left-click: Rotate | Right-click: Pan | Scroll: Zoom
      </div>
    </div>
  );
}
