import { useState } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

interface Props {
  isOpen: boolean;
  onClose: () => void;
}

const LOAD_CASE_TYPE_OPTIONS = [
  { value: 'permanent', label: 'Permanent (Dead Load)' },
  { value: 'variable', label: 'Variable (Live Load)' },
  { value: 'accidental', label: 'Accidental' },
  { value: 'seismic', label: 'Seismic' },
];

export default function AddLoadCaseDialog({ isOpen, onClose }: Props) {
  const { fetchModelState } = useStore();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [formData, setFormData] = useState({
    name: '',
    type: 'permanent',
  });

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!formData.name.trim()) {
      setError('Name is required');
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      const response = await toolsApi.execute('add_load_case', {
        name: formData.name.trim(),
        load_case_type: formData.type,
      });

      if (response.success) {
        await fetchModelState();
        onClose();
        setFormData({ name: '', type: 'permanent' });
      } else {
        setError(response.error || 'Failed to create load case');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleClose = () => {
    setFormData({ name: '', type: 'permanent' });
    setError(null);
    onClose();
  };

  return (
    <Dialog title="Add Load Case" isOpen={isOpen} onClose={handleClose}>
      <form onSubmit={handleSubmit} className="space-y-4">
        {error && (
          <div className="bg-red-50 border border-red-200 text-red-700 px-3 py-2 rounded text-sm">
            {error}
          </div>
        )}

        <Input
          label="Load Case Name"
          value={formData.name}
          onChange={(e) => setFormData({ ...formData, name: e.target.value })}
          placeholder="e.g., Dead Load, Live Load 1"
          autoFocus
        />

        <Select
          label="Load Case Type"
          options={LOAD_CASE_TYPE_OPTIONS}
          value={formData.type}
          onChange={(e) => setFormData({ ...formData, type: e.target.value })}
        />

        <p className="text-xs text-gray-500">
          Load case types are used for load combination according to design codes.
        </p>

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={handleClose}>
            Cancel
          </Button>
          <Button type="submit" variant="primary" disabled={isSubmitting || !formData.name.trim()}>
            {isSubmitting ? 'Creating...' : 'Create Load Case'}
          </Button>
        </div>
      </form>
    </Dialog>
  );
}
