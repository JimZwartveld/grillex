import { useState, useMemo, useEffect } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
  initialPosition?: [number, number, number];
}

// DOF labels and descriptions
const DOF_INFO = {
  UX: { label: 'UX', description: 'Translation in X', color: 'red' },
  UY: { label: 'UY', description: 'Translation in Y', color: 'green' },
  UZ: { label: 'UZ', description: 'Translation in Z', color: 'blue' },
  RX: { label: 'RX', description: 'Rotation about X', color: 'red' },
  RY: { label: 'RY', description: 'Rotation about Y', color: 'green' },
  RZ: { label: 'RZ', description: 'Rotation about Z', color: 'blue' },
};

type DOFKey = keyof typeof DOF_INFO;

interface DOFState {
  fixed: boolean;
  value: number;
}

// Preset configurations
const PRESETS: Record<string, Record<DOFKey, boolean>> = {
  fixed: { UX: true, UY: true, UZ: true, RX: true, RY: true, RZ: true },
  pinned: { UX: true, UY: true, UZ: true, RX: false, RY: false, RZ: false },
  roller_x: { UX: false, UY: true, UZ: true, RX: true, RY: true, RZ: true },
  roller_y: { UX: true, UY: false, UZ: true, RX: true, RY: true, RZ: true },
  roller_z: { UX: true, UY: true, UZ: false, RX: true, RY: true, RZ: true },
  translations_only: { UX: true, UY: true, UZ: true, RX: false, RY: false, RZ: false },
  rotations_only: { UX: false, UY: false, UZ: false, RX: true, RY: true, RZ: true },
};

const SUPPORT_TYPES = [
  { value: 'fixed', label: 'Fixed (all DOFs restrained)' },
  { value: 'pinned', label: 'Pinned (translations restrained)' },
  { value: 'roller_x', label: 'Roller X (free in X direction)' },
  { value: 'roller_y', label: 'Roller Y (free in Y direction)' },
  { value: 'roller_z', label: 'Roller Z (free in Z direction)' },
  { value: 'custom', label: 'Custom (configure DOFs manually)' },
];

// Checkbox component for individual DOF
function DOFCheckbox({
  dof,
  checked,
  value,
  onChange,
  onValueChange,
  disabled = false,
}: {
  dof: DOFKey;
  checked: boolean;
  value: number;
  onChange: (checked: boolean) => void;
  onValueChange: (value: number) => void;
  disabled?: boolean;
}) {
  const info = DOF_INFO[dof];
  const isTranslation = dof.startsWith('U');

  return (
    <div className={`flex items-center gap-2 p-2 rounded ${checked ? 'bg-gray-50' : ''}`}>
      <input
        type="checkbox"
        id={`dof-${dof}`}
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        disabled={disabled}
        className="w-4 h-4 text-blue-600 rounded border-gray-300 focus:ring-blue-500"
      />
      <label
        htmlFor={`dof-${dof}`}
        className={`flex-1 text-sm font-medium ${disabled ? 'text-gray-400' : 'text-gray-700'}`}
      >
        <span
          className="inline-block w-6 text-center font-mono"
          style={{ color: info.color }}
        >
          {info.label}
        </span>
        <span className="text-gray-500 text-xs ml-1">({info.description})</span>
      </label>
      {checked && (
        <div className="flex items-center gap-1">
          <input
            type="number"
            value={value}
            onChange={(e) => onValueChange(parseFloat(e.target.value) || 0)}
            className="w-20 px-2 py-1 text-xs border border-gray-200 rounded"
            step={isTranslation ? '0.001' : '0.01'}
            placeholder={isTranslation ? '0 m' : '0 rad'}
          />
          <span className="text-xs text-gray-400">{isTranslation ? 'm' : 'rad'}</span>
        </div>
      )}
    </div>
  );
}

export default function AddSupportDialog({ isOpen, onClose, initialPosition }: Props) {
  const { fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [formData, setFormData] = useState({
    x: initialPosition ? String(initialPosition[0]) : '0',
    y: initialPosition ? String(initialPosition[1]) : '0',
    z: initialPosition ? String(initialPosition[2]) : '0',
    type: 'fixed',
  });

  // Update form when initialPosition changes
  useEffect(() => {
    if (initialPosition) {
      setFormData(prev => ({
        ...prev,
        x: String(initialPosition[0]),
        y: String(initialPosition[1]),
        z: String(initialPosition[2]),
      }));
    }
  }, [initialPosition]);

  // Individual DOF states
  const [dofStates, setDofStates] = useState<Record<DOFKey, DOFState>>({
    UX: { fixed: true, value: 0 },
    UY: { fixed: true, value: 0 },
    UZ: { fixed: true, value: 0 },
    RX: { fixed: true, value: 0 },
    RY: { fixed: true, value: 0 },
    RZ: { fixed: true, value: 0 },
  });

  // Update DOF states when preset changes
  const handleTypeChange = (type: string) => {
    setFormData({ ...formData, type });

    if (type !== 'custom' && PRESETS[type]) {
      const preset = PRESETS[type];
      setDofStates((prev) => {
        const newStates = { ...prev };
        for (const dof of Object.keys(preset) as DOFKey[]) {
          newStates[dof] = { ...newStates[dof], fixed: preset[dof] };
        }
        return newStates;
      });
    }
  };

  // Check if current DOF config matches any preset
  const matchingPreset = useMemo(() => {
    for (const [presetName, presetDofs] of Object.entries(PRESETS)) {
      const matches = (Object.keys(presetDofs) as DOFKey[]).every(
        (dof) => dofStates[dof].fixed === presetDofs[dof]
      );
      if (matches) return presetName;
    }
    return 'custom';
  }, [dofStates]);

  const handleDOFChange = (dof: DOFKey, fixed: boolean) => {
    setDofStates((prev) => ({
      ...prev,
      [dof]: { ...prev[dof], fixed },
    }));
    // Switch to custom if user manually changes DOFs
    if (formData.type !== 'custom') {
      setFormData((prev) => ({ ...prev, type: 'custom' }));
    }
  };

  const handleDOFValueChange = (dof: DOFKey, value: number) => {
    setDofStates((prev) => ({
      ...prev,
      [dof]: { ...prev[dof], value },
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      const position = [
        parseFloat(formData.x),
        parseFloat(formData.y),
        parseFloat(formData.z),
      ];

      // Apply each fixed DOF
      const fixedDofs = (Object.keys(dofStates) as DOFKey[]).filter(
        (dof) => dofStates[dof].fixed
      );

      for (const dof of fixedDofs) {
        await toolsApi.execute('fix_dof', {
          position,
          dof,
          value: dofStates[dof].value,
        });
      }

      await fetchModelState();
      onClose();
      // Reset form
      setFormData({ x: '0', y: '0', z: '0', type: 'fixed' });
      setDofStates({
        UX: { fixed: true, value: 0 },
        UY: { fixed: true, value: 0 },
        UZ: { fixed: true, value: 0 },
        RX: { fixed: true, value: 0 },
        RY: { fixed: true, value: 0 },
        RZ: { fixed: true, value: 0 },
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  // Count fixed DOFs for summary
  const fixedCount = Object.values(dofStates).filter((s) => s.fixed).length;

  return (
    <Dialog title="Add Support" isOpen={isOpen} onClose={onClose}>
      <form onSubmit={handleSubmit} className="space-y-4">
        {/* Position input */}
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

        {/* Support type dropdown */}
        <Select
          label="Support Type"
          options={SUPPORT_TYPES}
          value={matchingPreset !== 'custom' ? matchingPreset : formData.type}
          onChange={(e) => handleTypeChange(e.target.value)}
        />

        {/* DOF configuration */}
        <div>
          <div className="flex items-center justify-between mb-2">
            <label className="block text-sm font-medium text-gray-700">
              DOF Configuration
            </label>
            <span className="text-xs text-gray-500">
              {fixedCount}/6 DOFs restrained
            </span>
          </div>

          <div className="border border-gray-200 rounded-lg divide-y divide-gray-100">
            {/* Translations */}
            <div className="p-2 bg-gray-50">
              <span className="text-xs font-medium text-gray-500 uppercase">
                Translations
              </span>
            </div>
            {(['UX', 'UY', 'UZ'] as DOFKey[]).map((dof) => (
              <DOFCheckbox
                key={dof}
                dof={dof}
                checked={dofStates[dof].fixed}
                value={dofStates[dof].value}
                onChange={(checked) => handleDOFChange(dof, checked)}
                onValueChange={(value) => handleDOFValueChange(dof, value)}
              />
            ))}

            {/* Rotations */}
            <div className="p-2 bg-gray-50">
              <span className="text-xs font-medium text-gray-500 uppercase">
                Rotations
              </span>
            </div>
            {(['RX', 'RY', 'RZ'] as DOFKey[]).map((dof) => (
              <DOFCheckbox
                key={dof}
                dof={dof}
                checked={dofStates[dof].fixed}
                value={dofStates[dof].value}
                onChange={(checked) => handleDOFChange(dof, checked)}
                onValueChange={(value) => handleDOFValueChange(dof, value)}
              />
            ))}
          </div>
        </div>

        {/* Quick preset buttons */}
        <div>
          <label className="block text-xs text-gray-500 mb-2">Quick Presets</label>
          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              onClick={() => handleTypeChange('fixed')}
              className={`px-2 py-1 text-xs rounded border ${
                matchingPreset === 'fixed'
                  ? 'bg-blue-100 border-blue-300 text-blue-700'
                  : 'bg-white border-gray-200 text-gray-600 hover:bg-gray-50'
              }`}
            >
              Fixed
            </button>
            <button
              type="button"
              onClick={() => handleTypeChange('pinned')}
              className={`px-2 py-1 text-xs rounded border ${
                matchingPreset === 'pinned'
                  ? 'bg-blue-100 border-blue-300 text-blue-700'
                  : 'bg-white border-gray-200 text-gray-600 hover:bg-gray-50'
              }`}
            >
              Pinned
            </button>
            <button
              type="button"
              onClick={() => handleTypeChange('roller_x')}
              className={`px-2 py-1 text-xs rounded border ${
                matchingPreset === 'roller_x'
                  ? 'bg-blue-100 border-blue-300 text-blue-700'
                  : 'bg-white border-gray-200 text-gray-600 hover:bg-gray-50'
              }`}
            >
              Roller X
            </button>
            <button
              type="button"
              onClick={() => handleTypeChange('roller_y')}
              className={`px-2 py-1 text-xs rounded border ${
                matchingPreset === 'roller_y'
                  ? 'bg-blue-100 border-blue-300 text-blue-700'
                  : 'bg-white border-gray-200 text-gray-600 hover:bg-gray-50'
              }`}
            >
              Roller Y
            </button>
            <button
              type="button"
              onClick={() => handleTypeChange('roller_z')}
              className={`px-2 py-1 text-xs rounded border ${
                matchingPreset === 'roller_z'
                  ? 'bg-blue-100 border-blue-300 text-blue-700'
                  : 'bg-white border-gray-200 text-gray-600 hover:bg-gray-50'
              }`}
            >
              Roller Z
            </button>
          </div>
        </div>

        {/* Submit buttons */}
        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Cancel
          </Button>
          <Button
            type="submit"
            variant="primary"
            disabled={isSubmitting || fixedCount === 0}
          >
            {isSubmitting ? 'Creating...' : 'Add Support'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
