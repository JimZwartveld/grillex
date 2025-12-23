import { useState } from 'react';
import { ChevronDown, Plus, List, Eye, EyeOff } from 'lucide-react';
import useStore from '../../stores/modelStore';
import AddLoadCaseDialog from './AddLoadCaseDialog';

// Load case type colors
const LOAD_CASE_TYPE_COLORS: Record<string, string> = {
  permanent: 'bg-gray-500',
  variable: 'bg-blue-500',
  accidental: 'bg-orange-500',
  seismic: 'bg-red-500',
};

// Load case type labels
const LOAD_CASE_TYPE_LABELS: Record<string, string> = {
  permanent: 'Permanent',
  variable: 'Variable',
  accidental: 'Accidental',
  seismic: 'Seismic',
};

export default function LoadCaseSelector() {
  const { loadCases, activeLoadCaseId, setActiveLoadCase } = useStore();
  const [isOpen, setIsOpen] = useState(false);
  const [addDialogOpen, setAddDialogOpen] = useState(false);

  // Get active load case name
  const activeLoadCase = loadCases.find((lc, idx) =>
    activeLoadCaseId === null ? false : (lc.id ?? idx) === activeLoadCaseId
  );

  const handleSelectLoadCase = (loadCaseId: number | null) => {
    setActiveLoadCase(loadCaseId);
    setIsOpen(false);
  };

  return (
    <div className="px-3 py-2 border-b border-gray-100">
      <div className="flex items-center justify-between mb-2">
        <label className="text-xs font-semibold text-gray-500 uppercase tracking-wide flex items-center gap-1">
          <List className="w-3 h-3" />
          Load Cases
        </label>
        <button
          onClick={() => setAddDialogOpen(true)}
          className="text-xs text-blue-600 hover:text-blue-700 flex items-center gap-0.5"
        >
          <Plus className="w-3 h-3" />
          New
        </button>
      </div>

      {loadCases.length === 0 ? (
        <div className="text-xs text-gray-400 text-center py-2">
          No load cases defined
        </div>
      ) : (
        <div className="relative">
          {/* Dropdown button */}
          <button
            onClick={() => setIsOpen(!isOpen)}
            className="w-full flex items-center justify-between px-3 py-2 bg-gray-50 hover:bg-gray-100 rounded-lg border border-gray-200 text-sm"
          >
            <div className="flex items-center gap-2">
              {activeLoadCaseId === null ? (
                <>
                  <Eye className="w-4 h-4 text-green-500" />
                  <span>All Load Cases</span>
                </>
              ) : activeLoadCase ? (
                <>
                  <span
                    className={`w-2 h-2 rounded-full ${
                      LOAD_CASE_TYPE_COLORS[activeLoadCase.type] || 'bg-gray-400'
                    }`}
                  />
                  <span>{activeLoadCase.name}</span>
                </>
              ) : (
                <span className="text-gray-400">Select load case</span>
              )}
            </div>
            <ChevronDown
              className={`w-4 h-4 text-gray-400 transition-transform ${
                isOpen ? 'rotate-180' : ''
              }`}
            />
          </button>

          {/* Dropdown menu */}
          {isOpen && (
            <div className="absolute z-10 w-full mt-1 bg-white rounded-lg shadow-lg border border-gray-200 py-1 max-h-48 overflow-y-auto">
              {/* Show All option */}
              <button
                onClick={() => handleSelectLoadCase(null)}
                className={`w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2 ${
                  activeLoadCaseId === null ? 'bg-blue-50' : ''
                }`}
              >
                <Eye className="w-4 h-4 text-green-500" />
                <span>Show All</span>
                {activeLoadCaseId === null && (
                  <span className="ml-auto text-xs text-blue-600">✓</span>
                )}
              </button>

              <div className="border-t border-gray-100 my-1" />

              {/* Load case options */}
              {loadCases.map((lc, idx) => {
                const lcId = lc.id ?? idx;
                const isActive = activeLoadCaseId === lcId;

                return (
                  <button
                    key={lcId}
                    onClick={() => handleSelectLoadCase(lcId)}
                    className={`w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2 ${
                      isActive ? 'bg-blue-50' : ''
                    }`}
                  >
                    <span
                      className={`w-2 h-2 rounded-full ${
                        LOAD_CASE_TYPE_COLORS[lc.type] || 'bg-gray-400'
                      }`}
                    />
                    <div className="flex-1 min-w-0">
                      <div className="truncate">{lc.name}</div>
                      <div className="text-xs text-gray-400">
                        {LOAD_CASE_TYPE_LABELS[lc.type] || lc.type} • {lc.loads.length} loads
                      </div>
                    </div>
                    {isActive && (
                      <span className="text-xs text-blue-600">✓</span>
                    )}
                  </button>
                );
              })}
            </div>
          )}
        </div>
      )}

      {/* Hint about active filtering */}
      {activeLoadCaseId !== null && (
        <div className="mt-2 flex items-center gap-1 text-xs text-amber-600">
          <EyeOff className="w-3 h-3" />
          <span>Filtering: only showing loads from selected case</span>
        </div>
      )}

      {/* Add Load Case Dialog */}
      <AddLoadCaseDialog
        isOpen={addDialogOpen}
        onClose={() => setAddDialogOpen(false)}
      />
    </div>
  );
}
