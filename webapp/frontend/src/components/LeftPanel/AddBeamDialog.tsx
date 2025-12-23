import { useState } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
}

export default function AddBeamDialog({ isOpen, onClose }: Props) {
  const { materials, sections, fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [formData, setFormData] = useState({
    startX: '0',
    startY: '0',
    startZ: '0',
    endX: '6',
    endY: '0',
    endZ: '0',
    material: '',
    section: '',
  });

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

      const materialName = formData.material || (materials[0]?.name || 'Steel');
      const sectionName = formData.section || (sections[0]?.name || 'IPE300');

      const response = await toolsApi.execute('create_beam', {
        start_position: [
          parseFloat(formData.startX),
          parseFloat(formData.startY),
          parseFloat(formData.startZ),
        ],
        end_position: [
          parseFloat(formData.endX),
          parseFloat(formData.endY),
          parseFloat(formData.endZ),
        ],
        material: materialName,
        section: sectionName,
      });

      if (response.success) {
        await fetchModelState();
        onClose();
        // Reset form
        setFormData({
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

  return (
    <Dialog title="Add Beam" isOpen={isOpen} onClose={onClose}>
      <form onSubmit={handleSubmit} className="space-y-4">
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
          <Button type="submit" variant="primary" disabled={isSubmitting}>
            {isSubmitting ? 'Creating...' : 'Create Beam'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
