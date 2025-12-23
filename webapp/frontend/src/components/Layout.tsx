import { ChevronLeft, ChevronRight, Layers, MessageSquare, BarChart2 } from 'lucide-react';
import useStore from '../stores/modelStore';
import LeftPanel from './LeftPanel';

const Viewer = () => {
  const { viewMode } = useStore();

  return (
    <div className="h-full flex items-center justify-center bg-gray-100">
      <div className="text-center text-gray-500">
        <Layers className="w-16 h-16 mx-auto mb-4 text-gray-300" />
        <p className="text-lg">3D Viewer</p>
        <p className="text-sm">Mode: {viewMode}</p>
        <p className="text-sm mt-2">Three.js viewer will be implemented in Task 17.6</p>
      </div>
    </div>
  );
};

const RightPanel = () => {
  const { activeRightTab, setActiveRightTab, chatMessages, results, isAnalyzed } = useStore();

  return (
    <div className="h-full flex flex-col">
      {/* Tab buttons */}
      <div className="flex border-b border-gray-200">
        <button
          onClick={() => setActiveRightTab('results')}
          className={`flex-1 px-4 py-3 text-sm font-medium flex items-center justify-center gap-2 ${
            activeRightTab === 'results'
              ? 'text-blue-600 border-b-2 border-blue-600 bg-blue-50'
              : 'text-gray-600 hover:text-gray-800 hover:bg-gray-50'
          }`}
        >
          <BarChart2 className="w-4 h-4" />
          Results
        </button>
        <button
          onClick={() => setActiveRightTab('chat')}
          className={`flex-1 px-4 py-3 text-sm font-medium flex items-center justify-center gap-2 ${
            activeRightTab === 'chat'
              ? 'text-blue-600 border-b-2 border-blue-600 bg-blue-50'
              : 'text-gray-600 hover:text-gray-800 hover:bg-gray-50'
          }`}
        >
          <MessageSquare className="w-4 h-4" />
          Chat
        </button>
      </div>

      {/* Tab content */}
      <div className="flex-1 overflow-y-auto p-4">
        {activeRightTab === 'results' ? (
          <div>
            {isAnalyzed && results ? (
              <div className="space-y-4">
                <h3 className="text-sm font-medium text-gray-700">Analysis Results</h3>
                <p className="text-sm">Max displacement: {results.max_displacement?.toFixed(6)} m</p>
                <p className="text-sm">Max reaction: {results.max_reaction?.toFixed(2)} kN</p>
              </div>
            ) : (
              <p className="text-sm text-gray-400">No analysis results yet. Run analysis first.</p>
            )}
          </div>
        ) : (
          <div className="space-y-2">
            {chatMessages.length === 0 ? (
              <p className="text-sm text-gray-400">
                Start a conversation with the AI assistant to build and analyze your model.
              </p>
            ) : (
              chatMessages.map((msg) => (
                <div
                  key={msg.id}
                  className={`p-3 rounded-lg ${
                    msg.role === 'user'
                      ? 'bg-blue-100 text-blue-900 ml-8'
                      : 'bg-gray-100 text-gray-900 mr-8'
                  }`}
                >
                  <p className="text-sm whitespace-pre-wrap">{msg.content}</p>
                </div>
              ))
            )}
          </div>
        )}
      </div>

      {/* Chat input - placeholder */}
      {activeRightTab === 'chat' && (
        <div className="p-4 border-t border-gray-200">
          <p className="text-xs text-gray-400 text-center">
            Chat input will be implemented in Task 17.5
          </p>
        </div>
      )}
    </div>
  );
};

export default function Layout() {
  const { leftPanelCollapsed, rightPanelCollapsed, toggleLeftPanel, toggleRightPanel, error, setError } = useStore();

  return (
    <div className="h-screen flex flex-col bg-white">
      {/* Header */}
      <header className="h-12 bg-gray-800 text-white flex items-center px-4 flex-shrink-0">
        <h1 className="text-lg font-semibold">Grillex FEM Solver</h1>
      </header>

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
