import { useState, useEffect, useMemo } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
  initialPosition?: [number, number, number] | null;
}

const DOF_OPTIONS = [
  { value: 'UX', label: 'UX (Force in X)' },
  { value: 'UY', label: 'UY (Force in Y)' },
  { value: 'UZ', label: 'UZ (Force in Z)' },
  { value: 'RX', label: 'RX (Moment about X)' },
  { value: 'RY', label: 'RY (Moment about Y)' },
  { value: 'RZ', label: 'RZ (Moment about Z)' },
];

export default function AddLoadDialog({ isOpen, onClose, initialPosition }: Props) {
  const { fetchModelState, loadCases, activeLoadCaseId } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [formData, setFormData] = useState({
    x: initialPosition ? String(initialPosition[0]) : '6',
    y: initialPosition ? String(initialPosition[1]) : '0',
    z: initialPosition ? String(initialPosition[2]) : '0',
    dof: 'UZ',
    value: '-10',
    loadCaseId: '',
  });

  // Build load case options
  const loadCaseOptions = useMemo(() => {
    const options = loadCases.map((lc, idx) => ({
      value: String(lc.id ?? idx),
      label: lc.name,
    }));
    // Add "Create New" option at the end
    options.push({ value: '__new__', label: '+ Create New Load Case' });
    return options;
  }, [loadCases]);

  // Update form when initialPosition changes or when dialog opens
  useEffect(() => {
    if (isOpen) {
      // Pre-select active load case if one is selected
      const defaultLoadCaseId = activeLoadCaseId !== null
        ? String(activeLoadCaseId)
        : loadCases.length > 0
          ? String(loadCases[0].id ?? 0)
          : '';

      setFormData(prev => ({
        ...prev,
        x: initialPosition ? String(initialPosition[0]) : prev.x,
        y: initialPosition ? String(initialPosition[1]) : prev.y,
        z: initialPosition ? String(initialPosition[2]) : prev.z,
        loadCaseId: defaultLoadCaseId,
      }));
    }
  }, [initialPosition, isOpen, activeLoadCaseId, loadCases]);

  const [showNewLoadCaseForm, setShowNewLoadCaseForm] = useState(false);
  const [newLoadCaseName, setNewLoadCaseName] = useState('');
  const [newLoadCaseType, setNewLoadCaseType] = useState('permanent');

  const handleLoadCaseChange = (value: string) => {
    if (value === '__new__') {
      setShowNewLoadCaseForm(true);
    } else {
      setFormData({ ...formData, loadCaseId: value });
      setShowNewLoadCaseForm(false);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      let loadCaseId = formData.loadCaseId;

      // Create new load case if needed
      if (showNewLoadCaseForm && newLoadCaseName.trim()) {
        const lcResponse = await toolsApi.execute('add_load_case', {
          name: newLoadCaseName.trim(),
          load_case_type: newLoadCaseType,
        });
        if (lcResponse.success && lcResponse.data?.result) {
          const result = lcResponse.data.result as { id: number };
          loadCaseId = String(result.id);
        } else {
          setIsSubmitting(false);
          return;
        }
      }

      const response = await toolsApi.execute('add_point_load', {
        position: [
          parseFloat(formData.x),
          parseFloat(formData.y),
          parseFloat(formData.z),
        ],
        dof: formData.dof,
        value: parseFloat(formData.value),
        load_case_id: loadCaseId ? parseInt(loadCaseId, 10) : undefined,
      });

      if (response.success) {
        await fetchModelState();
        onClose();
        setFormData({ x: '6', y: '0', z: '0', dof: 'UZ', value: '-10', loadCaseId: '' });
        setShowNewLoadCaseForm(false);
        setNewLoadCaseName('');
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

        {/* Load Case Selection */}
        <div className="border-t border-gray-100 pt-4">
          <Select
            label="Load Case"
            options={loadCaseOptions}
            value={showNewLoadCaseForm ? '__new__' : formData.loadCaseId}
            onChange={(e) => handleLoadCaseChange(e.target.value)}
          />

          {showNewLoadCaseForm && (
            <div className="mt-3 p-3 bg-gray-50 rounded-lg space-y-3">
              <Input
                label="New Load Case Name"
                value={newLoadCaseName}
                onChange={(e) => setNewLoadCaseName(e.target.value)}
                placeholder="e.g., Dead Load"
                autoFocus
              />
              <Select
                label="Type"
                options={[
                  { value: 'permanent', label: 'Permanent' },
                  { value: 'variable', label: 'Variable' },
                  { value: 'accidental', label: 'Accidental' },
                  { value: 'seismic', label: 'Seismic' },
                ]}
                value={newLoadCaseType}
                onChange={(e) => setNewLoadCaseType(e.target.value)}
              />
            </div>
          )}
        </div>

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Cancel
          </Button>
          <Button
            type="submit"
            variant="primary"
            disabled={isSubmitting || (showNewLoadCaseForm && !newLoadCaseName.trim()) || (!showNewLoadCaseForm && !formData.loadCaseId && loadCases.length > 0)}
          >
            {isSubmitting ? 'Adding...' : 'Add Load'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
