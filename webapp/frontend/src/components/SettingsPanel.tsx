import { useState, useMemo } from 'react';
import { X, Search, RotateCcw } from 'lucide-react';
import useStore, { type AppSettings } from '../stores/modelStore';

// Settings category definition
interface SettingDefinition {
  id: keyof AppSettings;
  name: string;
  description: string;
  type: 'toggle' | 'number' | 'color' | 'select';
  min?: number;
  max?: number;
  step?: number;
  options?: { value: string | number | boolean; label: string }[];
}

interface SettingsCategory {
  id: string;
  name: string;
  settings: SettingDefinition[];
}

const SETTINGS_CONFIG: SettingsCategory[] = [
  {
    id: 'display',
    name: 'Display',
    settings: [
      {
        id: 'gridSnap',
        name: 'Grid Snap',
        description: 'Snap new elements to grid',
        type: 'toggle',
      },
      {
        id: 'gridSize',
        name: 'Grid Size',
        description: 'Grid spacing in meters',
        type: 'number',
        min: 0.1,
        max: 10,
        step: 0.1,
      },
      {
        id: 'showNodes',
        name: 'Show Nodes',
        description: 'Display node spheres',
        type: 'toggle',
      },
      {
        id: 'nodeSize',
        name: 'Node Size',
        description: 'Size of node markers in meters',
        type: 'number',
        min: 0.01,
        max: 0.5,
        step: 0.01,
      },
      {
        id: 'showLabels',
        name: 'Show Labels',
        description: 'Display element labels',
        type: 'toggle',
      },
      {
        id: 'beamColor',
        name: 'Beam Color',
        description: 'Default color for beams',
        type: 'color',
      },
      {
        id: 'selectedBeamColor',
        name: 'Selected Beam Color',
        description: 'Color for selected beams',
        type: 'color',
      },
      {
        id: 'backgroundColor',
        name: 'Background Color',
        description: 'Viewer background color',
        type: 'color',
      },
    ],
  },
  {
    id: 'analysis',
    name: 'Analysis',
    settings: [
      {
        id: 'autoAnalyze',
        name: 'Auto-Analyze',
        description: 'Automatically run analysis after model changes',
        type: 'toggle',
      },
      {
        id: 'tolerance',
        name: 'Solver Tolerance',
        description: 'Numerical tolerance for solver convergence',
        type: 'number',
        min: 1e-15,
        max: 1e-6,
        step: 1e-12,
      },
    ],
  },
  {
    id: 'viewer',
    name: 'Viewer',
    settings: [
      {
        id: 'deformationScale',
        name: 'Deformation Scale',
        description: 'Scale factor for displaying deformations',
        type: 'number',
        min: 1,
        max: 1000,
        step: 10,
      },
      {
        id: 'showOriginal',
        name: 'Show Original Shape',
        description: 'Display undeformed shape overlay',
        type: 'toggle',
      },
    ],
  },
];

// Individual setting row component
function SettingRow({ setting }: { setting: SettingDefinition }) {
  const { settings, updateSetting } = useStore();
  const value = settings[setting.id];

  const handleChange = (newValue: boolean | number | string) => {
    updateSetting(setting.id, newValue as AppSettings[typeof setting.id]);
  };

  return (
    <div className="flex items-center justify-between py-3 border-b border-gray-100 last:border-0">
      <div className="flex-1 min-w-0 pr-4">
        <div className="text-sm font-medium text-gray-900">{setting.name}</div>
        <div className="text-xs text-gray-500">{setting.description}</div>
      </div>
      <div className="flex-shrink-0">
        {setting.type === 'toggle' && (
          <button
            type="button"
            onClick={() => handleChange(!value)}
            className={`relative inline-flex h-6 w-11 flex-shrink-0 cursor-pointer rounded-full border-2 border-transparent transition-colors duration-200 ease-in-out focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2 ${
              value ? 'bg-blue-600' : 'bg-gray-200'
            }`}
          >
            <span
              className={`pointer-events-none inline-block h-5 w-5 transform rounded-full bg-white shadow ring-0 transition duration-200 ease-in-out ${
                value ? 'translate-x-5' : 'translate-x-0'
              }`}
            />
          </button>
        )}
        {setting.type === 'number' && (
          <input
            type="number"
            value={value as number}
            min={setting.min}
            max={setting.max}
            step={setting.step}
            onChange={(e) => handleChange(parseFloat(e.target.value) || 0)}
            className="w-24 px-2 py-1 text-sm border border-gray-300 rounded focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
          />
        )}
        {setting.type === 'color' && (
          <div className="flex items-center gap-2">
            <input
              type="color"
              value={value as string}
              onChange={(e) => handleChange(e.target.value)}
              className="w-8 h-8 p-0 border border-gray-300 rounded cursor-pointer"
            />
            <input
              type="text"
              value={value as string}
              onChange={(e) => handleChange(e.target.value)}
              className="w-20 px-2 py-1 text-sm border border-gray-300 rounded focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 font-mono"
              pattern="^#[0-9A-Fa-f]{6}$"
            />
          </div>
        )}
        {setting.type === 'select' && setting.options && (
          <select
            value={String(value)}
            onChange={(e) => handleChange(e.target.value)}
            className="px-2 py-1 text-sm border border-gray-300 rounded focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
          >
            {setting.options.map((opt) => (
              <option key={String(opt.value)} value={String(opt.value)}>
                {opt.label}
              </option>
            ))}
          </select>
        )}
      </div>
    </div>
  );
}

export default function SettingsPanel() {
  const { settingsOpen, closeSettings, resetSettings } = useStore();
  const [search, setSearch] = useState('');

  // Filter settings based on search
  const filteredCategories = useMemo(() => {
    if (!search.trim()) return SETTINGS_CONFIG;

    const lower = search.toLowerCase();
    return SETTINGS_CONFIG.map((category) => ({
      ...category,
      settings: category.settings.filter(
        (s) =>
          s.name.toLowerCase().includes(lower) ||
          s.description.toLowerCase().includes(lower)
      ),
    })).filter((category) => category.settings.length > 0);
  }, [search]);

  const handleReset = () => {
    if (confirm('Reset all settings to defaults?')) {
      resetSettings();
    }
  };

  if (!settingsOpen) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center">
      {/* Backdrop */}
      <div
        className="absolute inset-0 bg-black bg-opacity-50"
        onClick={closeSettings}
      />

      {/* Panel */}
      <div className="relative bg-white rounded-lg shadow-xl w-full max-w-lg mx-4 max-h-[85vh] flex flex-col">
        {/* Header */}
        <div className="flex items-center justify-between p-4 border-b flex-shrink-0">
          <h2 className="text-lg font-semibold text-gray-800">Settings</h2>
          <div className="flex items-center gap-2">
            <button
              onClick={handleReset}
              className="p-2 text-gray-400 hover:text-gray-600 rounded-lg hover:bg-gray-100"
              title="Reset to defaults"
            >
              <RotateCcw className="w-4 h-4" />
            </button>
            <button
              onClick={closeSettings}
              className="p-2 text-gray-400 hover:text-gray-600 rounded-lg hover:bg-gray-100"
            >
              <X className="w-5 h-5" />
            </button>
          </div>
        </div>

        {/* Search */}
        <div className="p-4 border-b flex-shrink-0">
          <div className="relative">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-400" />
            <input
              type="text"
              placeholder="Search settings..."
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              className="w-full pl-9 pr-4 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
              autoFocus
            />
          </div>
        </div>

        {/* Settings content */}
        <div className="flex-1 overflow-y-auto p-4">
          {filteredCategories.length === 0 ? (
            <div className="text-center text-gray-500 py-8">
              No settings found matching "{search}"
            </div>
          ) : (
            filteredCategories.map((category) => (
              <div key={category.id} className="mb-6 last:mb-0">
                <h3 className="text-sm font-semibold text-gray-500 uppercase tracking-wide mb-2">
                  {category.name}
                </h3>
                <div className="bg-gray-50 rounded-lg px-4">
                  {category.settings.map((setting) => (
                    <SettingRow key={setting.id} setting={setting} />
                  ))}
                </div>
              </div>
            ))
          )}
        </div>
      </div>
    </div>
  );
}
