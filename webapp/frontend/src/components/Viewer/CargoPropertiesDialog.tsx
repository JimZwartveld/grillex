import { useState, useEffect } from 'react';
import { Dialog, Button, Input } from '../common';
import useStore from '../../stores/modelStore';
import api from '../../api/client';
import type { Cargo } from '../../types/model';

interface Props {
  cargo: Cargo | null;
  isOpen: boolean;
  onClose: () => void;
}

export default function CargoPropertiesDialog({ cargo, isOpen, onClose }: Props) {
  const { fetchModelState } = useStore();
  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [formData, setFormData] = useState({
    name: '',
    mass: '',
  });

  // Reset form when cargo changes
  useEffect(() => {
    if (cargo) {
      setFormData({
        name: cargo.name,
        mass: cargo.mass.toString(),
      });
      setIsEditing(false);
      setError(null);
    }
  }, [cargo]);

  if (!cargo) return null;

  const hasChanges =
    formData.name !== cargo.name ||
    parseFloat(formData.mass) !== cargo.mass;

  const handleSave = async () => {
    if (!hasChanges) {
      setIsEditing(false);
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      const newName = formData.name !== cargo.name ? formData.name : undefined;
      const newMass = parseFloat(formData.mass) !== cargo.mass ? parseFloat(formData.mass) : undefined;

      const response = await api.tools.updateCargo(cargo.id, newName, newMass);

      if (response.success) {
        await fetchModelState();
        setIsEditing(false);
      } else {
        setError(response.error || 'Failed to update cargo');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  const handleCancel = () => {
    setFormData({
      name: cargo.name,
      mass: cargo.mass.toString(),
    });
    setIsEditing(false);
    setError(null);
  };

  const handleDelete = async () => {
    if (!confirm(`Are you sure you want to delete cargo "${cargo.name}"?`)) {
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.deleteCargo(cargo.id);
      if (response.success) {
        await fetchModelState();
        onClose();
      } else {
        setError(response.error || 'Failed to delete cargo');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <Dialog title={`Cargo: ${cargo.name}`} isOpen={isOpen} onClose={onClose}>
      <div className="space-y-4">
        {/* Error message */}
        {error && (
          <div className="bg-red-50 border border-red-200 text-red-700 px-3 py-2 rounded text-sm">
            {error}
          </div>
        )}

        {/* Cargo ID */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Cargo ID
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {cargo.id}
          </div>
        </div>

        {/* Name */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Name
          </label>
          {isEditing ? (
            <Input
              value={formData.name}
              onChange={(e) => setFormData({ ...formData, name: e.target.value })}
              disabled={isSaving}
            />
          ) : (
            <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
              {cargo.name}
            </div>
          )}
        </div>

        {/* Mass */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Mass (mT)
          </label>
          {isEditing ? (
            <Input
              type="number"
              step="0.01"
              value={formData.mass}
              onChange={(e) => setFormData({ ...formData, mass: e.target.value })}
              disabled={isSaving}
            />
          ) : (
            <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
              {cargo.mass.toFixed(2)}
            </div>
          )}
        </div>

        {/* CoG Position (read-only) */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Center of Gravity Position (m)
          </label>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">X:</span> {cargo.cogPosition[0].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Y:</span> {cargo.cogPosition[1].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Z:</span> {cargo.cogPosition[2].toFixed(3)}
            </div>
          </div>
          {isEditing && (
            <p className="text-xs text-gray-500 mt-1">
              Note: CoG position cannot be changed after creation.
            </p>
          )}
        </div>

        {/* Dimensions (read-only) - Z-up: [Length X, Width Y, Height Z] */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Dimensions (m)
          </label>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Length (X):</span> {cargo.dimensions[0].toFixed(2)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Width (Y):</span> {cargo.dimensions[1].toFixed(2)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Height (Z):</span> {cargo.dimensions[2].toFixed(2)}
            </div>
          </div>
        </div>

        {/* Connections */}
        {cargo.connections.length > 0 && (
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Connections ({cargo.connections.length})
            </label>
            <div className="space-y-2 max-h-32 overflow-y-auto">
              {cargo.connections.map((conn, i) => (
                <div
                  key={i}
                  className="bg-gray-50 px-3 py-2 rounded border text-sm"
                >
                  <div className="flex justify-between">
                    <span className="text-gray-500">Node {conn.node_id}</span>
                    <span>
                      Offset: [{conn.cargoOffset.map((v) => v.toFixed(2)).join(', ')}]
                    </span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}

        <div className="flex justify-between pt-2">
          <div>
            {isEditing && (
              <Button
                type="button"
                variant="danger"
                onClick={handleDelete}
                disabled={isSaving}
              >
                Delete
              </Button>
            )}
          </div>
          <div className="flex gap-2">
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
      </div>
    </Dialog>
  );
}
