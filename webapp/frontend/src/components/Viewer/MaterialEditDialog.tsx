import { useState, useEffect } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import api from '../../api/client';
import type { Material } from '../../types/model';

interface Props {
  material: Material | null;
  beamId: number | null;
  isOpen: boolean;
  onClose: () => void;
}

export default function MaterialEditDialog({ material, beamId, isOpen, onClose }: Props) {
  const { materials, fetchModelState } = useStore();
  const [mode, setMode] = useState<'select' | 'edit'>('select');
  const [selectedMaterial, setSelectedMaterial] = useState('');
  const [formData, setFormData] = useState({
    E: '',
    nu: '',
    rho: '',
  });
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (material && isOpen) {
      setSelectedMaterial(material.name);
      setFormData({
        E: material.E.toExponential(4),
        nu: material.nu.toString(),
        rho: material.rho.toExponential(4),
      });
      setMode('select');
      setError(null);
    }
  }, [material, isOpen]);

  if (!material) return null;

  const materialOptions = materials.map((m) => ({ value: m.name, label: m.name }));

  const handleChangeMaterial = async () => {
    if (beamId === null || selectedMaterial === material.name) return;

    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.updateBeam(beamId, selectedMaterial, undefined);
      if (response.success) {
        await fetchModelState();
        onClose();
      } else {
        setError(response.error || 'Failed to update beam material');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  const handleEditMaterial = async () => {
    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.updateMaterial(
        material.name,
        parseFloat(formData.E) || undefined,
        parseFloat(formData.nu) || undefined,
        parseFloat(formData.rho) || undefined
      );

      if (response.success) {
        await fetchModelState();
        onClose();
      } else {
        setError(response.error || 'Failed to update material');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <Dialog title={`Edit Material: ${material.name}`} isOpen={isOpen} onClose={onClose}>
      <div className="space-y-4">
        {error && (
          <div className="bg-red-50 border border-red-200 text-red-700 px-3 py-2 rounded text-sm">
            {error}
          </div>
        )}

        {/* Mode tabs */}
        <div className="flex border-b border-gray-200">
          <button
            className={`px-4 py-2 text-sm font-medium ${
              mode === 'select'
                ? 'border-b-2 border-blue-500 text-blue-600'
                : 'text-gray-500 hover:text-gray-700'
            }`}
            onClick={() => setMode('select')}
          >
            Change Material
          </button>
          <button
            className={`px-4 py-2 text-sm font-medium ${
              mode === 'edit'
                ? 'border-b-2 border-blue-500 text-blue-600'
                : 'text-gray-500 hover:text-gray-700'
            }`}
            onClick={() => setMode('edit')}
          >
            Edit Properties
          </button>
        </div>

        {mode === 'select' ? (
          <>
            <Select
              label="Select Material"
              options={materialOptions}
              value={selectedMaterial}
              onChange={(e) => setSelectedMaterial(e.target.value)}
              disabled={isSaving}
            />
            <p className="text-xs text-gray-500">
              Change the material assigned to this beam. All beams using this material will remain unchanged.
            </p>
            <div className="flex justify-end gap-2 pt-2">
              <Button variant="secondary" onClick={onClose} disabled={isSaving}>
                Cancel
              </Button>
              <Button
                variant="primary"
                onClick={handleChangeMaterial}
                disabled={isSaving || selectedMaterial === material.name}
              >
                {isSaving ? 'Saving...' : 'Apply'}
              </Button>
            </div>
          </>
        ) : (
          <>
            <p className="text-xs text-amber-600 bg-amber-50 px-3 py-2 rounded">
              Warning: Editing material properties will affect ALL beams using "{material.name}"
            </p>
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Young's Modulus (E) [kN/m²]
                </label>
                <Input
                  type="text"
                  value={formData.E}
                  onChange={(e) => setFormData({ ...formData, E: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Poisson's Ratio (ν) [-]
                </label>
                <Input
                  type="text"
                  value={formData.nu}
                  onChange={(e) => setFormData({ ...formData, nu: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Density (ρ) [mT/m³]
                </label>
                <Input
                  type="text"
                  value={formData.rho}
                  onChange={(e) => setFormData({ ...formData, rho: e.target.value })}
                  disabled={isSaving}
                />
              </div>
            </div>
            <div className="flex justify-end gap-2 pt-2">
              <Button variant="secondary" onClick={onClose} disabled={isSaving}>
                Cancel
              </Button>
              <Button variant="primary" onClick={handleEditMaterial} disabled={isSaving}>
                {isSaving ? 'Saving...' : 'Save Changes'}
              </Button>
            </div>
          </>
        )}
      </div>
    </Dialog>
  );
}
