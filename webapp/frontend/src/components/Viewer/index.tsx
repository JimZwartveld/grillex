import { Suspense, useCallback } from 'react';
import { Canvas } from '@react-three/fiber';
import { Loader } from '@react-three/drei';
import ViewModeSelector from './ViewModeSelector';
import Scene from './Scene';
import ContextMenu from './ContextMenu';
import BeamPropertiesDialog from './BeamPropertiesDialog';
import SupportPropertiesDialog from './SupportPropertiesDialog';
import CargoPropertiesDialog from './CargoPropertiesDialog';
import useStore from '../../stores/modelStore';

export default function Viewer() {
  const {
    beams,
    boundaryConditions,
    cargos,
    nodes,
    contextMenu,
    propertiesDialog,
    openPropertiesDialog,
    closePropertiesDialog,
    fetchModelState,
  } = useStore();

  // Get the currently selected element for properties dialog
  const getSelectedBeam = () => {
    if (propertiesDialog.elementType === 'beam' && propertiesDialog.elementId !== null) {
      return beams.find((b) => b.id === propertiesDialog.elementId) || null;
    }
    return null;
  };

  const getSelectedSupport = () => {
    if (propertiesDialog.elementType === 'support' && propertiesDialog.elementId !== null) {
      const bc = boundaryConditions.find((b) => b.node_id === propertiesDialog.elementId);
      if (bc) {
        const node = nodes.find((n) => n.id === bc.node_id);
        return {
          support: bc,
          nodePosition: node?.position as [number, number, number] || null,
        };
      }
    }
    return { support: null, nodePosition: null };
  };

  const getSelectedCargo = () => {
    if (propertiesDialog.elementType === 'cargo' && propertiesDialog.elementId !== null) {
      return cargos.find((c) => c.id === propertiesDialog.elementId) || null;
    }
    return null;
  };

  // Handle opening properties from context menu
  const handleProperties = useCallback(() => {
    if (contextMenu.elementType && contextMenu.elementId !== null) {
      openPropertiesDialog(contextMenu.elementType, contextMenu.elementId);
    }
  }, [contextMenu, openPropertiesDialog]);

  // Handle delete from context menu
  const handleDelete = useCallback(async () => {
    // TODO: Implement delete functionality via API
    console.log('Delete:', contextMenu.elementType, contextMenu.elementId);
    // For now, just refresh the model state
    await fetchModelState();
  }, [contextMenu, fetchModelState]);

  const { support, nodePosition } = getSelectedSupport();

  return (
    <div className="w-full h-full relative bg-gradient-to-b from-slate-100 to-slate-200">
      {/* View mode selector */}
      <ViewModeSelector />

      {/* Three.js canvas */}
      <Canvas
        shadows
        gl={{ antialias: true, alpha: true }}
        dpr={[1, 2]}
        onContextMenu={(e) => e.preventDefault()}
      >
        <Suspense fallback={null}>
          <Scene />
        </Suspense>
      </Canvas>

      {/* Loading indicator */}
      <Loader />

      {/* Controls hint */}
      <div className="absolute bottom-4 left-4 text-xs text-gray-500 bg-white bg-opacity-80 px-2 py-1 rounded">
        Left-click: Select | Right-click: Context menu | Scroll: Zoom
      </div>

      {/* Context menu */}
      <ContextMenu onProperties={handleProperties} onDelete={handleDelete} />

      {/* Properties dialogs */}
      <BeamPropertiesDialog
        beam={getSelectedBeam()}
        isOpen={propertiesDialog.isOpen && propertiesDialog.elementType === 'beam'}
        onClose={closePropertiesDialog}
      />
      <SupportPropertiesDialog
        support={support}
        nodePosition={nodePosition}
        isOpen={propertiesDialog.isOpen && propertiesDialog.elementType === 'support'}
        onClose={closePropertiesDialog}
      />
      <CargoPropertiesDialog
        cargo={getSelectedCargo()}
        isOpen={propertiesDialog.isOpen && propertiesDialog.elementType === 'cargo'}
        onClose={closePropertiesDialog}
      />
    </div>
  );
}
