import { useState, useEffect } from 'react';
import { Dialog, Button } from '../common';
import useStore from '../../stores/modelStore';
import api from '../../api/client';
import type { BoundaryCondition } from '../../types/model';

interface Props {
  support: BoundaryCondition | null;
  nodePosition: [number, number, number] | null;
  isOpen: boolean;
  onClose: () => void;
}

const DOF_NAMES = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ'];
const DOF_LABELS = ['Translation X', 'Translation Y', 'Translation Z', 'Rotation X', 'Rotation Y', 'Rotation Z'];

export default function SupportPropertiesDialog({ support, nodePosition, isOpen, onClose }: Props) {
  const { boundaryConditions, fetchModelState } = useStore();
  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [pendingChanges, setPendingChanges] = useState<{ dof: number; add: boolean }[]>([]);

  // Reset state when dialog opens/closes
  useEffect(() => {
    if (isOpen) {
      setIsEditing(false);
      setError(null);
      setPendingChanges([]);
    }
  }, [isOpen, support]);

  if (!support || !nodePosition) return null;

  // Find all BCs at this node
  const bcsAtNode = boundaryConditions.filter((bc) => bc.node_id === support.node_id);
  const fixedDofs = new Set(bcsAtNode.map((bc) => bc.dof));

  // Get effective fixed state (original + pending changes)
  const getEffectiveFixedState = (dofIndex: number) => {
    const originallyFixed = fixedDofs.has(dofIndex);
    const change = pendingChanges.find((c) => c.dof === dofIndex);
    if (change) {
      return change.add;
    }
    return originallyFixed;
  };

  const toggleDof = (dofIndex: number) => {
    const originallyFixed = fixedDofs.has(dofIndex);
    const existingChange = pendingChanges.find((c) => c.dof === dofIndex);

    if (existingChange) {
      // Remove the pending change (revert to original)
      setPendingChanges(pendingChanges.filter((c) => c.dof !== dofIndex));
    } else {
      // Add a pending change (toggle from original)
      setPendingChanges([...pendingChanges, { dof: dofIndex, add: !originallyFixed }]);
    }
  };

  const handleSave = async () => {
    if (pendingChanges.length === 0) {
      setIsEditing(false);
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      // Apply changes in sequence
      for (const change of pendingChanges) {
        const dofName = DOF_NAMES[change.dof];
        if (change.add) {
          // Add BC - use fix_dof tool
          const response = await api.tools.execute('fix_dof', {
            position: nodePosition,
            dof: dofName,
            value: 0.0,
          });
          if (!response.success) {
            throw new Error(response.error || `Failed to fix ${dofName}`);
          }
        } else {
          // Remove BC
          const response = await api.tools.removeBoundaryCondition(nodePosition, dofName);
          if (!response.success) {
            throw new Error(response.error || `Failed to remove ${dofName} constraint`);
          }
        }
      }

      // Refresh model state
      await fetchModelState();
      setPendingChanges([]);
      setIsEditing(false);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  const handleCancel = () => {
    setPendingChanges([]);
    setIsEditing(false);
    setError(null);
  };

  const handleRemoveAll = async () => {
    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.removeBoundaryCondition(nodePosition, 'ALL');
      if (response.success) {
        await fetchModelState();
        onClose();
      } else {
        setError(response.error || 'Failed to remove boundary conditions');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  // Determine support type
  const getSupportType = () => {
    const effectiveFixed = DOF_NAMES.filter((_, i) => getEffectiveFixedState(i));
    if (effectiveFixed.length === 6) return 'Fixed (all DOFs restrained)';
    if (effectiveFixed.length === 3 && effectiveFixed.every((d) => ['UX', 'UY', 'UZ'].includes(d))) {
      return 'Pinned (translations restrained)';
    }
    if (effectiveFixed.length === 0) return 'None (no DOFs restrained)';
    return `Custom (${effectiveFixed.length} DOFs restrained)`;
  };

  return (
    <Dialog title="Support Properties" isOpen={isOpen} onClose={onClose}>
      <div className="space-y-4">
        {/* Error message */}
        {error && (
          <div className="bg-red-50 border border-red-200 text-red-700 px-3 py-2 rounded text-sm">
            {error}
          </div>
        )}

        {/* Node Position */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Position (m)
          </label>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">X:</span> {nodePosition[0].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Y:</span> {nodePosition[1].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Z:</span> {nodePosition[2].toFixed(3)}
            </div>
          </div>
        </div>

        {/* Node ID */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Node ID
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {support.node_id}
          </div>
        </div>

        {/* Fixed DOFs */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Fixed Degrees of Freedom
          </label>
          <div className="grid grid-cols-3 gap-2">
            {DOF_NAMES.map((dofName, dofIndex) => {
              const isFixed = getEffectiveFixedState(dofIndex);
              const hasChange = pendingChanges.some((c) => c.dof === dofIndex);
              const bc = bcsAtNode.find((b) => b.dof === dofIndex);

              return (
                <button
                  key={dofIndex}
                  type="button"
                  onClick={() => isEditing && toggleDof(dofIndex)}
                  disabled={!isEditing || isSaving}
                  className={`px-3 py-2 rounded border text-sm text-center transition-colors ${
                    isFixed
                      ? hasChange
                        ? 'bg-green-100 border-green-300 text-green-700'
                        : 'bg-blue-50 border-blue-200 text-blue-700'
                      : hasChange
                      ? 'bg-red-100 border-red-300 text-red-700 line-through'
                      : 'bg-gray-50 border-gray-200 text-gray-400'
                  } ${isEditing ? 'cursor-pointer hover:opacity-80' : 'cursor-default'}`}
                  title={isEditing ? `Click to ${isFixed ? 'unfix' : 'fix'} ${DOF_LABELS[dofIndex]}` : DOF_LABELS[dofIndex]}
                >
                  {dofName}
                  {isFixed && bc && !hasChange && (
                    <span className="block text-xs">= {bc.value.toFixed(4)}</span>
                  )}
                  {hasChange && (
                    <span className="block text-xs">{isFixed ? '(adding)' : '(removing)'}</span>
                  )}
                </button>
              );
            })}
          </div>
          {isEditing && (
            <p className="text-xs text-gray-500 mt-2">
              Click on a DOF to toggle its fixed state. Green = will be added, Red = will be removed.
            </p>
          )}
        </div>

        {/* Support Type Description */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Support Type
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {getSupportType()}
          </div>
        </div>

        <div className="flex justify-between pt-2">
          <div>
            {isEditing && (
              <Button
                type="button"
                variant="danger"
                onClick={handleRemoveAll}
                disabled={isSaving}
              >
                Remove All
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
                  disabled={isSaving || pendingChanges.length === 0}
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
