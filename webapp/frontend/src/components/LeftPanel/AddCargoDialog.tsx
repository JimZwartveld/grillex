import { useState, useEffect } from 'react';
import { Dialog, Button, Input } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
  initialPosition?: [number, number, number] | null;
}

export default function AddCargoDialog({ isOpen, onClose, initialPosition }: Props) {
  const { fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [formData, setFormData] = useState({
    name: 'Cargo 1',
    // COG position
    cogX: initialPosition ? String(initialPosition[0]) : '3',
    cogY: initialPosition ? String(initialPosition[1]) : '0',
    cogZ: initialPosition ? String(initialPosition[2]) : '0.5',
    // Dimensions (length, width, height)
    dimX: '2',
    dimY: '2',
    dimZ: '1',
    // Mass in metric tonnes
    mass: '10',
  });

  // Update form when initialPosition changes
  useEffect(() => {
    if (initialPosition) {
      setFormData(prev => ({
        ...prev,
        cogX: String(initialPosition[0]),
        cogY: String(initialPosition[1]),
        cogZ: String(initialPosition[2] + 0.5), // Offset Z up by half the default height
      }));
    }
  }, [initialPosition]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      const response = await toolsApi.execute('add_cargo', {
        name: formData.name,
        cog_position: [
          parseFloat(formData.cogX),
          parseFloat(formData.cogY),
          parseFloat(formData.cogZ),
        ],
        dimensions: [
          parseFloat(formData.dimX),
          parseFloat(formData.dimY),
          parseFloat(formData.dimZ),
        ],
        mass: parseFloat(formData.mass),
      });

      if (response.success) {
        await fetchModelState();
        onClose();
        // Reset form with incremented name
        const currentNum = parseInt(formData.name.replace(/\D/g, '')) || 1;
        setFormData({
          name: `Cargo ${currentNum + 1}`,
          cogX: '3',
          cogY: '0',
          cogZ: '0.5',
          dimX: '2',
          dimY: '2',
          dimZ: '1',
          mass: '10',
        });
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Dialog title="Add Cargo" isOpen={isOpen} onClose={onClose}>
      <form onSubmit={handleSubmit} className="space-y-4">
        {/* Name */}
        <Input
          label="Cargo Name"
          value={formData.name}
          onChange={(e) => setFormData({ ...formData, name: e.target.value })}
        />

        {/* COG Position */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Center of Gravity Position (m)
          </label>
          <div className="grid grid-cols-3 gap-2">
            <Input
              placeholder="X"
              value={formData.cogX}
              onChange={(e) => setFormData({ ...formData, cogX: e.target.value })}
              type="number"
              step="0.1"
            />
            <Input
              placeholder="Y"
              value={formData.cogY}
              onChange={(e) => setFormData({ ...formData, cogY: e.target.value })}
              type="number"
              step="0.1"
            />
            <Input
              placeholder="Z"
              value={formData.cogZ}
              onChange={(e) => setFormData({ ...formData, cogZ: e.target.value })}
              type="number"
              step="0.1"
            />
          </div>
        </div>

        {/* Dimensions */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Dimensions (m)
          </label>
          <div className="grid grid-cols-3 gap-2">
            <div>
              <Input
                placeholder="Length (X)"
                value={formData.dimX}
                onChange={(e) => setFormData({ ...formData, dimX: e.target.value })}
                type="number"
                step="0.1"
                min="0.1"
              />
              <span className="text-xs text-gray-400 mt-0.5 block">Length (X)</span>
            </div>
            <div>
              <Input
                placeholder="Width (Y)"
                value={formData.dimY}
                onChange={(e) => setFormData({ ...formData, dimY: e.target.value })}
                type="number"
                step="0.1"
                min="0.1"
              />
              <span className="text-xs text-gray-400 mt-0.5 block">Width (Y)</span>
            </div>
            <div>
              <Input
                placeholder="Height (Z)"
                value={formData.dimZ}
                onChange={(e) => setFormData({ ...formData, dimZ: e.target.value })}
                type="number"
                step="0.1"
                min="0.1"
              />
              <span className="text-xs text-gray-400 mt-0.5 block">Height (Z)</span>
            </div>
          </div>
        </div>

        {/* Mass */}
        <Input
          label="Mass (mT)"
          value={formData.mass}
          onChange={(e) => setFormData({ ...formData, mass: e.target.value })}
          type="number"
          step="0.1"
          min="0"
        />

        <p className="text-xs text-gray-500">
          The cargo will be visualized as a box with the specified dimensions.
          The COG position is at the center of the box.
        </p>

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Cancel
          </Button>
          <Button type="submit" variant="primary" disabled={isSubmitting}>
            {isSubmitting ? 'Adding...' : 'Add Cargo'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
