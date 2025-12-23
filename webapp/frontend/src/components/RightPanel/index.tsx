import { MessageSquare, BarChart2 } from 'lucide-react';
import useStore from '../../stores/modelStore';
import ResultsTab from './ResultsTab';
import ChatTab from './ChatTab';

export default function RightPanel() {
  const { activeRightTab, setActiveRightTab, isAnalyzed } = useStore();

  return (
    <div className="h-full flex flex-col bg-white">
      {/* Tab buttons */}
      <div className="flex border-b border-gray-200 flex-shrink-0">
        <button
          onClick={() => setActiveRightTab('results')}
          className={`flex-1 px-4 py-3 text-sm font-medium flex items-center justify-center gap-2 transition-colors ${
            activeRightTab === 'results'
              ? 'text-blue-600 border-b-2 border-blue-600 bg-blue-50'
              : 'text-gray-600 hover:text-gray-800 hover:bg-gray-50'
          }`}
        >
          <BarChart2 className="w-4 h-4" />
          Results
          {isAnalyzed && (
            <span className="w-2 h-2 bg-green-500 rounded-full" title="Analysis complete" />
          )}
        </button>
        <button
          onClick={() => setActiveRightTab('chat')}
          className={`flex-1 px-4 py-3 text-sm font-medium flex items-center justify-center gap-2 transition-colors ${
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
      <div className="flex-1 overflow-hidden">
        {activeRightTab === 'results' ? <ResultsTab /> : <ChatTab />}
      </div>
    </div>
  );
}
