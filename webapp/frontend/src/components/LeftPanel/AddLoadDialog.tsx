import { useState } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
}

const DOF_OPTIONS = [
  { value: 'UX', label: 'UX (Force in X)' },
  { value: 'UY', label: 'UY (Force in Y)' },
  { value: 'UZ', label: 'UZ (Force in Z)' },
  { value: 'RX', label: 'RX (Moment about X)' },
  { value: 'RY', label: 'RY (Moment about Y)' },
  { value: 'RZ', label: 'RZ (Moment about Z)' },
];

export default function AddLoadDialog({ isOpen, onClose }: Props) {
  const { fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [formData, setFormData] = useState({
    x: '6',
    y: '0',
    z: '0',
    dof: 'UZ',
    value: '-10',
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      const response = await toolsApi.execute('add_point_load', {
        position: [
          parseFloat(formData.x),
          parseFloat(formData.y),
          parseFloat(formData.z),
        ],
        dof: formData.dof,
        value: parseFloat(formData.value),
      });

      if (response.success) {
        await fetchModelState();
        onClose();
        setFormData({ x: '6', y: '0', z: '0', dof: 'UZ', value: '-10' });
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Dialog title="Add Point Load" isOpen={isOpen} onClose={onClose}>
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
          label="Direction / DOF"
          options={DOF_OPTIONS}
          value={formData.dof}
          onChange={(e) => setFormData({ ...formData, dof: e.target.value })}
        />

        <Input
          label="Value (kN or kNm)"
          value={formData.value}
          onChange={(e) => setFormData({ ...formData, value: e.target.value })}
          type="number"
          step="0.1"
          placeholder="Negative = downward/negative direction"
        />

        <p className="text-xs text-gray-500">
          Tip: Use negative values for downward loads (e.g., -10 kN in UZ for gravity)
        </p>

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Cancel
          </Button>
          <Button type="submit" variant="primary" disabled={isSubmitting}>
            {isSubmitting ? 'Adding...' : 'Add Load'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
