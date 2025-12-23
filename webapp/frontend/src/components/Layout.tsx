import { ChevronLeft, ChevronRight, Settings } from 'lucide-react';
import useStore from '../stores/modelStore';
import LeftPanel from './LeftPanel';
import RightPanel from './RightPanel';
import Viewer from './Viewer';
import SettingsPanel from './SettingsPanel';

export default function Layout() {
  const { leftPanelCollapsed, rightPanelCollapsed, toggleLeftPanel, toggleRightPanel, error, setError, openSettings } = useStore();

  return (
    <div className="h-screen flex flex-col bg-white">
      {/* Header */}
      <header className="h-12 bg-gray-800 text-white flex items-center justify-between px-4 flex-shrink-0">
        <h1 className="text-lg font-semibold">Grillex FEM Solver</h1>
        <button
          onClick={openSettings}
          className="p-2 hover:bg-gray-700 rounded-lg transition-colors"
          title="Settings"
        >
          <Settings className="w-5 h-5" />
        </button>
      </header>

      {/* Settings Panel */}
      <SettingsPanel />

      {/* Error banner */}
      {error && (
        <div className="bg-red-100 border-b border-red-300 px-4 py-2 flex items-center justify-between">
          <span className="text-red-700 text-sm">{error}</span>
          <button
            onClick={() => setError(null)}
            className="text-red-500 hover:text-red-700 text-sm"
          >
            Dismiss
          </button>
        </div>
      )}

      {/* Main content */}
      <div className="flex-1 flex overflow-hidden">
        {/* Left panel */}
        <div
          className={`bg-white border-r border-gray-200 flex-shrink-0 transition-all duration-300 ${
            leftPanelCollapsed ? 'w-0' : 'w-72'
          } overflow-hidden`}
        >
          {!leftPanelCollapsed && <LeftPanel />}
        </div>

        {/* Left panel toggle */}
        <button
          onClick={toggleLeftPanel}
          className="w-6 flex-shrink-0 bg-gray-100 hover:bg-gray-200 flex items-center justify-center border-r border-gray-200"
          title={leftPanelCollapsed ? 'Expand left panel' : 'Collapse left panel'}
        >
          {leftPanelCollapsed ? (
            <ChevronRight className="w-4 h-4 text-gray-500" />
          ) : (
            <ChevronLeft className="w-4 h-4 text-gray-500" />
          )}
        </button>

        {/* Center viewer */}
        <div className="flex-1 min-w-0">
          <Viewer />
        </div>

        {/* Right panel toggle */}
        <button
          onClick={toggleRightPanel}
          className="w-6 flex-shrink-0 bg-gray-100 hover:bg-gray-200 flex items-center justify-center border-l border-gray-200"
          title={rightPanelCollapsed ? 'Expand right panel' : 'Collapse right panel'}
        >
          {rightPanelCollapsed ? (
            <ChevronLeft className="w-4 h-4 text-gray-500" />
          ) : (
            <ChevronRight className="w-4 h-4 text-gray-500" />
          )}
        </button>

        {/* Right panel */}
        <div
          className={`bg-white border-l border-gray-200 flex-shrink-0 transition-all duration-300 ${
            rightPanelCollapsed ? 'w-0' : 'w-80'
          } overflow-hidden`}
        >
          {!rightPanelCollapsed && <RightPanel />}
        </div>
      </div>

      {/* Footer */}
      <footer className="h-6 bg-gray-100 border-t border-gray-200 flex items-center px-4 flex-shrink-0">
        <span className="text-xs text-gray-500">Grillex 2.0 - FEM Structural Analysis</span>
      </footer>
    </div>
  );
}
