import { useState, useEffect } from 'react';
import { Dialog, Button, Input, Select } from '../common';
import useStore from '../../stores/modelStore';
import api from '../../api/client';
import type { Section } from '../../types/model';

interface Props {
  section: Section | null;
  beamId: number | null;
  isOpen: boolean;
  onClose: () => void;
}

export default function SectionEditDialog({ section, beamId, isOpen, onClose }: Props) {
  const { sections, beams, fetchModelState } = useStore();
  const [mode, setMode] = useState<'select' | 'create' | 'edit'>('select');
  const [selectedSection, setSelectedSection] = useState('');
  const [newSectionName, setNewSectionName] = useState('');
  const [formData, setFormData] = useState({
    A: '',
    Iy: '',
    Iz: '',
    J: '',
  });
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (section && isOpen) {
      setSelectedSection(section.name);
      setNewSectionName('');
      setFormData({
        A: section.A.toExponential(4),
        Iy: section.Iy.toExponential(4),
        Iz: section.Iz.toExponential(4),
        J: section.J.toExponential(4),
      });
      setMode('select');
      setError(null);
    }
  }, [section, isOpen]);

  if (!section) return null;

  const sectionOptions = sections.map((s) => ({ value: s.name, label: s.name }));

  // Count how many beams use the current section
  const beamsUsingSection = beams.filter((b) => b.section === section.name).length;

  const handleChangeSection = async () => {
    if (beamId === null || selectedSection === section.name) return;

    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.updateBeam(beamId, undefined, selectedSection);
      if (response.success) {
        await fetchModelState();
        onClose();
      } else {
        setError(response.error || 'Failed to update beam section');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  const handleCreateSection = async () => {
    if (!newSectionName.trim()) {
      setError('Please enter a section name');
      return;
    }

    if (sections.some((s) => s.name === newSectionName.trim())) {
      setError('A section with this name already exists');
      return;
    }

    setIsSaving(true);
    setError(null);

    try {
      // Create new section
      const createResponse = await api.tools.execute('add_section', {
        name: newSectionName.trim(),
        A: parseFloat(formData.A) || 0.01,
        Iy: parseFloat(formData.Iy) || 1e-4,
        Iz: parseFloat(formData.Iz) || 1e-4,
        J: parseFloat(formData.J) || 1e-4,
      });

      if (!createResponse.success) {
        setError(createResponse.error || 'Failed to create section');
        return;
      }

      // Assign to beam if beamId is provided
      if (beamId !== null) {
        const updateResponse = await api.tools.updateBeam(beamId, undefined, newSectionName.trim());
        if (!updateResponse.success) {
          setError(updateResponse.error || 'Section created but failed to assign to beam');
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

  const handleEditSection = async () => {
    setIsSaving(true);
    setError(null);

    try {
      const response = await api.tools.updateSection(
        section.name,
        parseFloat(formData.A) || undefined,
        parseFloat(formData.Iy) || undefined,
        parseFloat(formData.Iz) || undefined,
        parseFloat(formData.J) || undefined
      );

      if (response.success) {
        await fetchModelState();
        onClose();
      } else {
        setError(response.error || 'Failed to update section');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <Dialog title={`Section: ${section.name}`} isOpen={isOpen} onClose={onClose}>
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
              label="Select Section"
              options={sectionOptions}
              value={selectedSection}
              onChange={(e) => setSelectedSection(e.target.value)}
              disabled={isSaving}
            />
            <p className="text-xs text-gray-500">
              Assign a different section to this beam. Other beams keep their current section.
            </p>
            <div className="flex justify-end gap-2 pt-2">
              <Button variant="secondary" onClick={onClose} disabled={isSaving}>
                Cancel
              </Button>
              <Button
                variant="primary"
                onClick={handleChangeSection}
                disabled={isSaving || selectedSection === section.name}
              >
                {isSaving ? 'Saving...' : 'Apply'}
              </Button>
            </div>
          </>
        )}

        {mode === 'create' && (
          <>
            <p className="text-xs text-blue-600 bg-blue-50 px-3 py-2 rounded">
              Create a new section and assign it to this beam only.
            </p>
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-1">
                New Section Name
              </label>
              <Input
                type="text"
                value={newSectionName}
                onChange={(e) => setNewSectionName(e.target.value)}
                placeholder="e.g., IPE300-Custom"
                disabled={isSaving}
              />
            </div>
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Area (A) [m²]
                </label>
                <Input
                  type="text"
                  value={formData.A}
                  onChange={(e) => setFormData({ ...formData, A: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Iy [m⁴]
                </label>
                <Input
                  type="text"
                  value={formData.Iy}
                  onChange={(e) => setFormData({ ...formData, Iy: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Iz [m⁴]
                </label>
                <Input
                  type="text"
                  value={formData.Iz}
                  onChange={(e) => setFormData({ ...formData, Iz: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  J [m⁴]
                </label>
                <Input
                  type="text"
                  value={formData.J}
                  onChange={(e) => setFormData({ ...formData, J: e.target.value })}
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
                onClick={handleCreateSection}
                disabled={isSaving || !newSectionName.trim()}
              >
                {isSaving ? 'Creating...' : 'Create & Assign'}
              </Button>
            </div>
          </>
        )}

        {mode === 'edit' && (
          <>
            <p className="text-xs text-amber-600 bg-amber-50 px-3 py-2 rounded">
              Warning: This will modify "{section.name}" for all {beamsUsingSection} beam{beamsUsingSection !== 1 ? 's' : ''} using it.
              Use "Create New" if you want a unique section for this beam.
            </p>
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Area (A) [m²]
                </label>
                <Input
                  type="text"
                  value={formData.A}
                  onChange={(e) => setFormData({ ...formData, A: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Iy [m⁴]
                </label>
                <Input
                  type="text"
                  value={formData.Iy}
                  onChange={(e) => setFormData({ ...formData, Iy: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Iz [m⁴]
                </label>
                <Input
                  type="text"
                  value={formData.Iz}
                  onChange={(e) => setFormData({ ...formData, Iz: e.target.value })}
                  disabled={isSaving}
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  J [m⁴]
                </label>
                <Input
                  type="text"
                  value={formData.J}
                  onChange={(e) => setFormData({ ...formData, J: e.target.value })}
                  disabled={isSaving}
                />
              </div>
            </div>
            <div className="flex justify-end gap-2 pt-2">
              <Button variant="secondary" onClick={onClose} disabled={isSaving}>
                Cancel
              </Button>
              <Button variant="primary" onClick={handleEditSection} disabled={isSaving}>
                {isSaving ? 'Saving...' : 'Save Changes'}
              </Button>
            </div>
          </>
        )}
      </div>
    </Dialog>
  );
}
