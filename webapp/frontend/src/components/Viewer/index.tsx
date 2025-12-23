import { Suspense, useCallback, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { Loader } from '@react-three/drei';
import ViewModeSelector from './ViewModeSelector';
import Scene from './Scene';
import ContextMenu from './ContextMenu';
import AddContextMenu from './AddContextMenu';
import BeamPropertiesDialog from './BeamPropertiesDialog';
import SupportPropertiesDialog from './SupportPropertiesDialog';
import CargoPropertiesDialog from './CargoPropertiesDialog';
import SectionEditDialog from './SectionEditDialog';
import MaterialEditDialog from './MaterialEditDialog';
import AddBeamDialog from '../LeftPanel/AddBeamDialog';
import AddSupportDialog from '../LeftPanel/AddSupportDialog';
import AddLoadDialog from '../LeftPanel/AddLoadDialog';
import AddCargoDialog from '../LeftPanel/AddCargoDialog';
import useStore from '../../stores/modelStore';
import api from '../../api/client';

export default function Viewer() {
  const {
    beams,
    boundaryConditions,
    cargos,
    nodes,
    materials,
    sections,
    contextMenu,
    propertiesDialog,
    openPropertiesDialog,
    closePropertiesDialog,
    fetchModelState,
  } = useStore();

  // State for section/material edit dialogs
  const [sectionDialogOpen, setSectionDialogOpen] = useState(false);
  const [materialDialogOpen, setMaterialDialogOpen] = useState(false);
  const [editingBeamId, setEditingBeamId] = useState<number | null>(null);

  // State for add dialogs (triggered from right-click context menu)
  const [addDialogState, setAddDialogState] = useState<{
    type: 'beam' | 'support' | 'load' | 'cargo' | null;
    position: [number, number, number] | null;
  }>({ type: null, position: null });

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

  // Get beam for section/material edit
  const getEditingBeam = () => {
    if (editingBeamId !== null) {
      return beams.find((b) => b.id === editingBeamId) || null;
    }
    return null;
  };

  const getEditingSection = () => {
    const beam = getEditingBeam();
    if (beam) {
      return sections.find((s) => s.name === beam.section) || null;
    }
    return null;
  };

  const getEditingMaterial = () => {
    const beam = getEditingBeam();
    if (beam) {
      return materials.find((m) => m.name === beam.material) || null;
    }
    return null;
  };

  // Handle opening properties from context menu
  const handleProperties = useCallback(() => {
    if (contextMenu.elementType && contextMenu.elementId !== null) {
      openPropertiesDialog(contextMenu.elementType, contextMenu.elementId);
    }
  }, [contextMenu, openPropertiesDialog]);

  // Handle edit section from context menu
  const handleEditSection = useCallback(() => {
    if (contextMenu.elementType === 'beam' && contextMenu.elementId !== null) {
      setEditingBeamId(contextMenu.elementId);
      setSectionDialogOpen(true);
    }
  }, [contextMenu]);

  // Handle edit material from context menu
  const handleEditMaterial = useCallback(() => {
    if (contextMenu.elementType === 'beam' && contextMenu.elementId !== null) {
      setEditingBeamId(contextMenu.elementId);
      setMaterialDialogOpen(true);
    }
  }, [contextMenu]);

  // Handle delete from context menu
  const handleDelete = useCallback(async () => {
    const { elementType, elementId } = contextMenu;
    if (!elementType || elementId === null) return;

    // Confirm deletion
    const elementName = elementType === 'beam' ? `Beam ${elementId}` :
                       elementType === 'cargo' ? `Cargo ${elementId}` :
                       elementType === 'support' ? 'Support' : 'Element';

    if (!confirm(`Are you sure you want to delete ${elementName}?`)) {
      return;
    }

    try {
      let response;
      if (elementType === 'beam') {
        response = await api.tools.deleteBeam(elementId);
      } else if (elementType === 'cargo') {
        response = await api.tools.deleteCargo(elementId);
      } else if (elementType === 'support') {
        // For supports, we need to find the node position
        const bc = boundaryConditions.find((b) => b.node_id === elementId);
        if (bc) {
          const node = nodes.find((n) => n.id === bc.node_id);
          if (node) {
            response = await api.tools.removeBoundaryCondition(
              node.position as [number, number, number],
              'ALL'
            );
          }
        }
      }

      if (response?.success) {
        await fetchModelState();
      } else {
        console.error('Delete failed:', response?.error);
        alert(response?.error || 'Failed to delete element');
      }
    } catch (err) {
      console.error('Delete error:', err);
      alert('An error occurred while deleting');
    }
  }, [contextMenu, boundaryConditions, nodes, fetchModelState]);

  // Handlers for add context menu actions
  const handleAddBeam = useCallback((position: [number, number, number]) => {
    setAddDialogState({ type: 'beam', position });
  }, []);

  const handleAddSupport = useCallback((position: [number, number, number]) => {
    setAddDialogState({ type: 'support', position });
  }, []);

  const handleAddLoad = useCallback((position: [number, number, number]) => {
    setAddDialogState({ type: 'load', position });
  }, []);

  const handleAddCargo = useCallback((position: [number, number, number]) => {
    setAddDialogState({ type: 'cargo', position });
  }, []);

  const closeAddDialog = useCallback(() => {
    setAddDialogState({ type: null, position: null });
  }, []);

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

      {/* Context menu for existing elements */}
      <ContextMenu
        onProperties={handleProperties}
        onDelete={handleDelete}
        onEditSection={handleEditSection}
        onEditMaterial={handleEditMaterial}
      />

      {/* Context menu for adding new elements */}
      <AddContextMenu
        onAddBeam={handleAddBeam}
        onAddSupport={handleAddSupport}
        onAddLoad={handleAddLoad}
        onAddCargo={handleAddCargo}
      />

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

      {/* Section/Material edit dialogs */}
      <SectionEditDialog
        section={getEditingSection()}
        beamId={editingBeamId}
        isOpen={sectionDialogOpen}
        onClose={() => {
          setSectionDialogOpen(false);
          setEditingBeamId(null);
        }}
      />
      <MaterialEditDialog
        material={getEditingMaterial()}
        beamId={editingBeamId}
        isOpen={materialDialogOpen}
        onClose={() => {
          setMaterialDialogOpen(false);
          setEditingBeamId(null);
        }}
      />

      {/* Add dialogs (triggered from right-click context menu) */}
      <AddBeamDialog
        isOpen={addDialogState.type === 'beam'}
        onClose={closeAddDialog}
        initialPosition={addDialogState.position}
      />
      <AddSupportDialog
        isOpen={addDialogState.type === 'support'}
        onClose={closeAddDialog}
        initialPosition={addDialogState.position ?? undefined}
      />
      <AddLoadDialog
        isOpen={addDialogState.type === 'load'}
        onClose={closeAddDialog}
        initialPosition={addDialogState.position}
      />
      <AddCargoDialog
        isOpen={addDialogState.type === 'cargo'}
        onClose={closeAddDialog}
        initialPosition={addDialogState.position}
      />
    </div>
  );
}
