import { useState } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
}

const SUPPORT_TYPES = [
  { value: 'fixed', label: 'Fixed (all DOFs restrained)' },
  { value: 'pinned', label: 'Pinned (translations restrained)' },
  { value: 'roller_x', label: 'Roller X (free in X)' },
  { value: 'roller_y', label: 'Roller Y (free in Y)' },
  { value: 'roller_z', label: 'Roller Z (free in Z)' },
];

export default function AddSupportDialog({ isOpen, onClose }: Props) {
  const { fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [formData, setFormData] = useState({
    x: '0',
    y: '0',
    z: '0',
    type: 'fixed',
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      const position = [
        parseFloat(formData.x),
        parseFloat(formData.y),
        parseFloat(formData.z),
      ];

      let response;
      if (formData.type === 'fixed') {
        response = await toolsApi.execute('fix_node', { position });
      } else if (formData.type === 'pinned') {
        response = await toolsApi.execute('pin_node', { position });
      } else {
        // Roller - fix all except one direction
        const freeDir = formData.type.replace('roller_', '').toUpperCase();
        const dofs = ['UX', 'UY', 'UZ'].filter((d) => d !== `U${freeDir}`);

        for (const dof of dofs) {
          await toolsApi.execute('fix_dof', { position, dof, value: 0 });
        }
        // Also fix rotations for rollers
        for (const dof of ['RX', 'RY', 'RZ']) {
          await toolsApi.execute('fix_dof', { position, dof, value: 0 });
        }
        response = { success: true };
      }

      if (response?.success) {
        await fetchModelState();
        onClose();
        setFormData({ x: '0', y: '0', z: '0', type: 'fixed' });
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Dialog title="Add Support" isOpen={isOpen} onClose={onClose}>
      <form onSubmit={handleSubmit} className="space-y-4">
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Position (m)
          </label>
          <div className="grid grid-cols-3 gap-2">
            <Input
              placeholder="X"
              value={formData.x}
              onChange={(e) => setFormData({ ...formData, x: e.target.value })}
              type="number"
              step="0.01"
            />
            <Input
              placeholder="Y"
              value={formData.y}
              onChange={(e) => setFormData({ ...formData, y: e.target.value })}
              type="number"
              step="0.01"
            />
            <Input
              placeholder="Z"
              value={formData.z}
              onChange={(e) => setFormData({ ...formData, z: e.target.value })}
              type="number"
              step="0.01"
            />
          </div>
        </div>

        <Select
          label="Support Type"
          options={SUPPORT_TYPES}
          value={formData.type}
          onChange={(e) => setFormData({ ...formData, type: e.target.value })}
        />

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Cancel
          </Button>
          <Button type="submit" variant="primary" disabled={isSubmitting}>
            {isSubmitting ? 'Creating...' : 'Add Support'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
