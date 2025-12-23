import { useState, useEffect, useMemo } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
  initialPosition?: [number, number, number] | null;
}

type CreationMode = 'nodes' | 'coordinates';

export default function AddBeamDialog({ isOpen, onClose, initialPosition }: Props) {
  const { nodes, materials, sections, fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [creationMode, setCreationMode] = useState<CreationMode>(
    nodes.length >= 2 ? 'nodes' : 'coordinates'
  );

  // Compute default end position: 6m in +X from start
  const defaultEndX = initialPosition ? String(initialPosition[0] + 6) : '6';
  const defaultEndY = initialPosition ? String(initialPosition[1]) : '0';
  const defaultEndZ = initialPosition ? String(initialPosition[2]) : '0';

  const [formData, setFormData] = useState({
    // Node selection mode
    startNodeId: '',
    endNodeId: '',
    // Coordinate mode
    startX: initialPosition ? String(initialPosition[0]) : '0',
    startY: initialPosition ? String(initialPosition[1]) : '0',
    startZ: initialPosition ? String(initialPosition[2]) : '0',
    endX: defaultEndX,
    endY: defaultEndY,
    endZ: defaultEndZ,
    // Common
    material: '',
    section: '',
  });

  // Build node options with position labels
  const nodeOptions = useMemo(() => {
    return nodes.map((n) => ({
      value: String(n.id),
      label: `Node ${n.id} (${n.position[0].toFixed(2)}, ${n.position[1].toFixed(2)}, ${n.position[2].toFixed(2)})`,
    }));
  }, [nodes]);

  // Update form when initialPosition changes
  useEffect(() => {
    if (initialPosition) {
      // If we have an initial position, switch to coordinates mode
      setCreationMode('coordinates');
      setFormData((prev) => ({
        ...prev,
        startX: String(initialPosition[0]),
        startY: String(initialPosition[1]),
        startZ: String(initialPosition[2]),
        endX: String(initialPosition[0] + 6),
        endY: String(initialPosition[1]),
        endZ: String(initialPosition[2]),
      }));
    }
  }, [initialPosition]);

  // Update mode when dialog opens based on available nodes
  useEffect(() => {
    if (isOpen && !initialPosition) {
      setCreationMode(nodes.length >= 2 ? 'nodes' : 'coordinates');
    }
  }, [isOpen, nodes.length, initialPosition]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      // If no material exists, create default Steel
      if (materials.length === 0) {
        await toolsApi.execute('add_material', {
          name: 'Steel',
          E: 210e6,
          nu: 0.3,
          rho: 7.85e-3,
        });
      }

      // If no section exists, create default IPE300
      if (sections.length === 0) {
        await toolsApi.execute('add_section', {
          name: 'IPE300',
          A: 0.00538,
          Iy: 8.36e-5,
          Iz: 6.04e-6,
          J: 2.01e-7,
        });
      }

      const materialName = formData.material || materials[0]?.name || 'Steel';
      const sectionName = formData.section || sections[0]?.name || 'IPE300';

      let startPosition: [number, number, number];
      let endPosition: [number, number, number];

      if (creationMode === 'nodes') {
        // Get positions from selected nodes
        const startNode = nodes.find((n) => String(n.id) === formData.startNodeId);
        const endNode = nodes.find((n) => String(n.id) === formData.endNodeId);

        if (!startNode || !endNode) {
          return; // Should not happen with proper validation
        }

        startPosition = startNode.position;
        endPosition = endNode.position;
      } else {
        // Use entered coordinates
        startPosition = [
          parseFloat(formData.startX),
          parseFloat(formData.startY),
          parseFloat(formData.startZ),
        ];
        endPosition = [
          parseFloat(formData.endX),
          parseFloat(formData.endY),
          parseFloat(formData.endZ),
        ];
      }

      const response = await toolsApi.execute('create_beam', {
        start_position: startPosition,
        end_position: endPosition,
        material: materialName,
        section: sectionName,
      });

      if (response.success) {
        await fetchModelState();
        onClose();
        // Reset form
        setFormData({
          startNodeId: '',
          endNodeId: '',
          startX: '0',
          startY: '0',
          startZ: '0',
          endX: '6',
          endY: '0',
          endZ: '0',
          material: '',
          section: '',
        });
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  const materialOptions = materials.map((m) => ({ value: m.name, label: m.name }));
  const sectionOptions = sections.map((s) => ({ value: s.name, label: s.name }));

  // Validation
  const isValid =
    creationMode === 'nodes'
      ? formData.startNodeId && formData.endNodeId && formData.startNodeId !== formData.endNodeId
      : true; // Coordinate mode always valid

  return (
    <Dialog title="Add Beam" isOpen={isOpen} onClose={onClose}>
      <form onSubmit={handleSubmit} className="space-y-4">
        {/* Creation mode toggle */}
        {nodes.length >= 2 && (
          <div className="flex rounded-lg border border-gray-200 overflow-hidden">
            <button
              type="button"
              className={`flex-1 px-3 py-2 text-sm font-medium transition-colors ${
                creationMode === 'nodes'
                  ? 'bg-blue-600 text-white'
                  : 'bg-gray-50 text-gray-700 hover:bg-gray-100'
              }`}
              onClick={() => setCreationMode('nodes')}
            >
              Select Nodes
            </button>
            <button
              type="button"
              className={`flex-1 px-3 py-2 text-sm font-medium transition-colors ${
                creationMode === 'coordinates'
                  ? 'bg-blue-600 text-white'
                  : 'bg-gray-50 text-gray-700 hover:bg-gray-100'
              }`}
              onClick={() => setCreationMode('coordinates')}
            >
              Enter Coordinates
            </button>
          </div>
        )}

        {creationMode === 'nodes' ? (
          <>
            {/* Node selection mode */}
            <Select
              label="Start Node"
              options={nodeOptions}
              value={formData.startNodeId}
              onChange={(e) => setFormData({ ...formData, startNodeId: e.target.value })}
              placeholder="Select start node"
            />

            <Select
              label="End Node"
              options={nodeOptions.filter((o) => o.value !== formData.startNodeId)}
              value={formData.endNodeId}
              onChange={(e) => setFormData({ ...formData, endNodeId: e.target.value })}
              placeholder="Select end node"
            />

            {formData.startNodeId && formData.endNodeId && formData.startNodeId === formData.endNodeId && (
              <p className="text-xs text-red-500">Start and end nodes must be different</p>
            )}
          </>
        ) : (
          <>
            {/* Coordinate entry mode */}
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                Start Position (m)
              </label>
              <div className="grid grid-cols-3 gap-2">
                <Input
                  placeholder="X"
                  value={formData.startX}
                  onChange={(e) => setFormData({ ...formData, startX: e.target.value })}
                  type="number"
                  step="0.01"
                />
                <Input
                  placeholder="Y"
                  value={formData.startY}
                  onChange={(e) => setFormData({ ...formData, startY: e.target.value })}
                  type="number"
                  step="0.01"
                />
                <Input
                  placeholder="Z"
                  value={formData.startZ}
                  onChange={(e) => setFormData({ ...formData, startZ: e.target.value })}
                  type="number"
                  step="0.01"
                />
              </div>
            </div>

            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">
                End Position (m)
              </label>
              <div className="grid grid-cols-3 gap-2">
                <Input
                  placeholder="X"
                  value={formData.endX}
                  onChange={(e) => setFormData({ ...formData, endX: e.target.value })}
                  type="number"
                  step="0.01"
                />
                <Input
                  placeholder="Y"
                  value={formData.endY}
                  onChange={(e) => setFormData({ ...formData, endY: e.target.value })}
                  type="number"
                  step="0.01"
                />
                <Input
                  placeholder="Z"
                  value={formData.endZ}
                  onChange={(e) => setFormData({ ...formData, endZ: e.target.value })}
                  type="number"
                  step="0.01"
                />
              </div>
            </div>
          </>
        )}

        <Select
          label="Material"
          options={materialOptions}
          value={formData.material}
          onChange={(e) => setFormData({ ...formData, material: e.target.value })}
          placeholder={materials.length === 0 ? 'Will create Steel' : 'Select material'}
        />

        <Select
          label="Section"
          options={sectionOptions}
          value={formData.section}
          onChange={(e) => setFormData({ ...formData, section: e.target.value })}
          placeholder={sections.length === 0 ? 'Will create IPE300' : 'Select section'}
        />

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Cancel
          </Button>
          <Button type="submit" variant="primary" disabled={isSubmitting || !isValid}>
            {isSubmitting ? 'Creating...' : 'Create Beam'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
