import useStore from '../../stores/modelStore';

const VIEW_MODES = [
  { value: 'fem', label: 'FEM View', description: 'Elements and nodes' },
  { value: 'results', label: 'Results View', description: 'Deflected shape' },
  { value: 'realistic', label: 'Model View', description: '3D geometry' },
];

export default function ViewModeSelector() {
  const { viewMode, setViewMode, isAnalyzed } = useStore();

  return (
    <div className="absolute top-4 left-4 z-10">
      <select
        className="bg-white border border-gray-300 rounded-md px-3 py-2 text-sm shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
        value={viewMode}
        onChange={(e) => setViewMode(e.target.value as 'fem' | 'results' | 'realistic')}
      >
        {VIEW_MODES.map((mode) => (
          <option
            key={mode.value}
            value={mode.value}
            disabled={mode.value === 'results' && !isAnalyzed}
          >
            {mode.label}
          </option>
        ))}
      </select>
    </div>
  );
}
