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
  const { materials, beams, fetchModelState } = useStore();
  const [mode, setMode] = useState<'select' | 'create' | 'edit'>('select');
  const [selectedMaterial, setSelectedMaterial] = useState('');
  const [newMaterialName, setNewMaterialName] = useState('');
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
      setNewMaterialName('');
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

  // Count how many beams use the current material
  const beamsUsingMaterial = beams.filter((b) => b.material === material.name).length;

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

  const handleCreateMaterial = async () => {
    if (!newMaterialName.trim()) {
      setError('Please enter a material name');
      return;
    }

    if (materials.some((m) => m.name === newMaterialName.trim())) {
      setError('A material with this name already exists');
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      // Create new material
      const createResponse = await api.tools.execute('add_material', {
        name: newMaterialName.trim(),
        E: parseFloat(formData.E) || 210e6,
        nu: parseFloat(formData.nu) || 0.3,
        rho: parseFloat(formData.rho) || 7.85e-3,
      });

      if (!createResponse.success) {
        setError(createResponse.error || 'Failed to create material');
        return;
      }

      // Assign to beam if beamId is provided
      if (beamId !== null) {
        const updateResponse = await api.tools.updateBeam(beamId, newMaterialName.trim(), undefined);
        if (!updateResponse.success) {
          setError(updateResponse.error || 'Material created but failed to assign to beam');
          await fetchModelState();
          return;
        }
      }

      await fetchModelState();
      onClose();
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
    <Dialog title={`Material: ${material.name}`} isOpen={isOpen} onClose={onClose}>
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
            Change
          </button>
          <button
            className={`px-4 py-2 text-sm font-medium ${
              mode === 'create'
                ? 'border-b-2 border-blue-500 text-blue-600'
                : 'text-gray-500 hover:text-gray-700'
            }`}
            onClick={() => setMode('create')}
          >
            Create New
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

        {mode === 'select' && (
          <>
            <Select
              label="Select Material"
              options={materialOptions}
              value={selectedMaterial}
              onChange={(e) => setSelectedMaterial(e.target.value)}
              disabled={isSaving}
            />
            <p className="text-xs text-gray-500">
              Assign a different material to this beam. Other beams keep their current material.
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
        )}

        {mode === 'create' && (
          <>
            <p className="text-xs text-blue-600 bg-blue-50 px-3 py-2 rounded">
              Create a new material and assign it to this beam only.
            </p>
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                New Material Name
              </label>
              <Input
                type="text"
                value={newMaterialName}
                onChange={(e) => setNewMaterialName(e.target.value)}
                placeholder="e.g., Steel-Custom"
                disabled={isSaving}
              />
            </div>
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
              <Button
                variant="primary"
                onClick={handleCreateMaterial}
                disabled={isSaving || !newMaterialName.trim()}
              >
                {isSaving ? 'Creating...' : 'Create & Assign'}
              </Button>
            </div>
          </>
        )}

        {mode === 'edit' && (
          <>
            <p className="text-xs text-amber-600 bg-amber-50 px-3 py-2 rounded">
              Warning: This will modify "{material.name}" for all {beamsUsingMaterial} beam{beamsUsingMaterial !== 1 ? 's' : ''} using it.
              Use "Create New" if you want a unique material for this beam.
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
