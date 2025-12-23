import { useState, useEffect } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import api from '../../api/client';
import type { Beam } from '../../types/model';

interface Props {
  beam: Beam | null;
  isOpen: boolean;
  onClose: () => void;
}

export default function BeamPropertiesDialog({ beam, isOpen, onClose }: Props) {
  const { materials, sections, fetchModelState } = useStore();
  const [formData, setFormData] = useState({
    startX: '0',
    startY: '0',
    startZ: '0',
    endX: '0',
    endY: '0',
    endZ: '0',
    material: '',
    section: '',
  });
  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Populate form when beam changes
  useEffect(() => {
    if (beam) {
      setFormData({
        startX: beam.start[0].toString(),
        startY: beam.start[1].toString(),
        startZ: beam.start[2].toString(),
        endX: beam.end[0].toString(),
        endY: beam.end[1].toString(),
        endZ: beam.end[2].toString(),
        material: beam.material,
        section: beam.section,
      });
      setIsEditing(false);
      setError(null);
    }
  }, [beam]);

  if (!beam) return null;

  const materialOptions = materials.map((m) => ({ value: m.name, label: m.name }));
  const sectionOptions = sections.map((s) => ({ value: s.name, label: s.name }));

  const hasChanges =
    formData.material !== beam.material || formData.section !== beam.section;

  const handleSave = async () => {
    if (!hasChanges) {
      setIsEditing(false);
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.updateBeam(
        beam.id,
        formData.material !== beam.material ? formData.material : undefined,
        formData.section !== beam.section ? formData.section : undefined
      );

      if (response.success) {
        // Refresh model state to get updated data
        await fetchModelState();
        setIsEditing(false);
      } else {
        setError(response.error || 'Failed to update beam');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  const handleCancel = () => {
    // Reset form to original values
    setFormData({
      startX: beam.start[0].toString(),
      startY: beam.start[1].toString(),
      startZ: beam.start[2].toString(),
      endX: beam.end[0].toString(),
      endY: beam.end[1].toString(),
      endZ: beam.end[2].toString(),
      material: beam.material,
      section: beam.section,
    });
    setIsEditing(false);
    setError(null);
  };

  return (
    <Dialog title={`Beam ${beam.id} Properties`} isOpen={isOpen} onClose={onClose}>
      <div className="space-y-4">
        {/* Error message */}
        {error && (
          <div className="bg-red-50 border border-red-200 text-red-700 px-3 py-2 rounded text-sm">
            {error}
          </div>
        )}

        {/* Read-only beam ID */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Beam ID
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {beam.id}
          </div>
        </div>

        {/* Length (calculated, read-only) */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Length (m)
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {beam.length.toFixed(3)}
          </div>
        </div>

        {/* Start Position */}
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
              disabled
            />
            <Input
              placeholder="Y"
              value={formData.startY}
              onChange={(e) => setFormData({ ...formData, startY: e.target.value })}
              type="number"
              step="0.01"
              disabled
            />
            <Input
              placeholder="Z"
              value={formData.startZ}
              onChange={(e) => setFormData({ ...formData, startZ: e.target.value })}
              type="number"
              step="0.01"
              disabled
            />
          </div>
        </div>

        {/* End Position */}
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
              disabled
            />
            <Input
              placeholder="Y"
              value={formData.endY}
              onChange={(e) => setFormData({ ...formData, endY: e.target.value })}
              type="number"
              step="0.01"
              disabled
            />
            <Input
              placeholder="Z"
              value={formData.endZ}
              onChange={(e) => setFormData({ ...formData, endZ: e.target.value })}
              type="number"
              step="0.01"
              disabled
            />
          </div>
        </div>

        {/* Material */}
        <Select
          label="Material"
          options={materialOptions}
          value={formData.material}
          onChange={(e) => setFormData({ ...formData, material: e.target.value })}
          disabled={!isEditing || isSaving}
        />

        {/* Section */}
        <Select
          label="Section"
          options={sectionOptions}
          value={formData.section}
          onChange={(e) => setFormData({ ...formData, section: e.target.value })}
          disabled={!isEditing || isSaving}
        />

        {/* Section Properties (read-only) */}
        {sections.find((s) => s.name === formData.section) && (
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Section Properties
            </label>
            <div className="grid grid-cols-2 gap-2 text-sm">
              <div className="bg-gray-50 px-3 py-2 rounded border">
                <span className="text-gray-500">A:</span>{' '}
                {sections.find((s) => s.name === formData.section)?.A.toExponential(3)} m²
              </div>
              <div className="bg-gray-50 px-3 py-2 rounded border">
                <span className="text-gray-500">Iy:</span>{' '}
                {sections.find((s) => s.name === formData.section)?.Iy.toExponential(3)} m⁴
              </div>
              <div className="bg-gray-50 px-3 py-2 rounded border">
                <span className="text-gray-500">Iz:</span>{' '}
                {sections.find((s) => s.name === formData.section)?.Iz.toExponential(3)} m⁴
              </div>
              <div className="bg-gray-50 px-3 py-2 rounded border">
                <span className="text-gray-500">J:</span>{' '}
                {sections.find((s) => s.name === formData.section)?.J.toExponential(3)} m⁴
              </div>
            </div>
          </div>
        )}

        {/* Material Properties (read-only) */}
        {materials.find((m) => m.name === formData.material) && (
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Material Properties
            </label>
            <div className="grid grid-cols-2 gap-2 text-sm">
              <div className="bg-gray-50 px-3 py-2 rounded border">
                <span className="text-gray-500">E:</span>{' '}
                {(materials.find((m) => m.name === formData.material)?.E || 0 / 1e6).toFixed(0)} kN/m²
              </div>
              <div className="bg-gray-50 px-3 py-2 rounded border">
                <span className="text-gray-500">ν:</span>{' '}
                {materials.find((m) => m.name === formData.material)?.nu}
              </div>
              <div className="bg-gray-50 px-3 py-2 rounded border col-span-2">
                <span className="text-gray-500">ρ:</span>{' '}
                {materials.find((m) => m.name === formData.material)?.rho} mT/m³
              </div>
            </div>
          </div>
        )}

        <div className="flex justify-end gap-2 pt-2">
          {isEditing ? (
            <>
              <Button type="button" variant="secondary" onClick={handleCancel} disabled={isSaving}>
                Cancel
              </Button>
              <Button
                type="button"
                variant="primary"
                onClick={handleSave}
                disabled={isSaving || !hasChanges}
              >
                {isSaving ? 'Saving...' : 'Save Changes'}
              </Button>
            </>
          ) : (
            <>
              <Button type="button" variant="secondary" onClick={onClose}>
                Close
              </Button>
              <Button type="button" variant="primary" onClick={() => setIsEditing(true)}>
                Edit
              </Button>
            </>
          )}
        </div>
      </div>
    </Dialog>
  );
}
