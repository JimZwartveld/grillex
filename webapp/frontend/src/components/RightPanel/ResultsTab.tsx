import { BarChart2, ArrowDown, ArrowUp, Target, AlertTriangle, RefreshCw } from 'lucide-react';
import useStore from '../../stores/modelStore';
import api from '../../api/client';

interface ResultsTableProps {
  title: string;
  icon: React.ReactNode;
  data: Array<{
    label: string;
    value: string;
    unit: string;
  }>;
}

function ResultsTable({ title, icon, data }: ResultsTableProps) {
  return (
    <div className="bg-white rounded-lg border border-gray-100 overflow-hidden">
      <div className="px-3 py-2 bg-gray-50 border-b border-gray-100 flex items-center gap-2">
        {icon}
        <span className="text-sm font-medium text-gray-700">{title}</span>
      </div>
      <div className="divide-y divide-gray-50">
        {data.map((row, i) => (
          <div key={i} className="px-3 py-2 flex justify-between items-center">
            <span className="text-xs text-gray-600">{row.label}</span>
            <span className="text-sm font-mono text-gray-800">
              {row.value} <span className="text-gray-400">{row.unit}</span>
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}

// Warning banner for stale results
function ResultsWarningBanner({ warning, onRerunAnalysis }: { warning: string; onRerunAnalysis: () => void }) {
  return (
    <div className="bg-yellow-50 border-l-4 border-yellow-400 p-3 mb-4">
      <div className="flex items-start gap-2">
        <AlertTriangle className="w-5 h-5 text-yellow-500 flex-shrink-0 mt-0.5" />
        <div className="flex-1 min-w-0">
          <p className="text-sm text-yellow-700">{warning}</p>
          <button
            onClick={onRerunAnalysis}
            className="mt-2 inline-flex items-center gap-1.5 px-3 py-1.5 text-xs font-medium text-yellow-700 bg-yellow-100 hover:bg-yellow-200 rounded-md transition-colors"
          >
            <RefreshCw className="w-3.5 h-3.5" />
            Re-run Analysis
          </button>
        </div>
      </div>
    </div>
  );
}

export default function ResultsTab() {
  const { isAnalyzed, results, beams, nodes, resultWarning, fetchModelState } = useStore();

  const handleRerunAnalysis = async () => {
    const response = await api.tools.execute('analyze', {});
    if (response.success) {
      await fetchModelState();
    }
  };

  // Show warning if results are stale
  if (resultWarning) {
    return (
      <div className="h-full overflow-y-auto p-4">
        <ResultsWarningBanner warning={resultWarning} onRerunAnalysis={handleRerunAnalysis} />
        <div className="text-center text-gray-400 mt-8">
          <BarChart2 className="w-12 h-12 mx-auto mb-3 text-gray-300" />
          <p className="text-sm mb-1">Results invalidated</p>
          <p className="text-xs">Re-run analysis to get updated results</p>
        </div>
      </div>
    );
  }

  if (!isAnalyzed || !results) {
    return (
      <div className="h-full flex items-center justify-center p-4">
        <div className="text-center text-gray-400">
          <BarChart2 className="w-12 h-12 mx-auto mb-3 text-gray-300" />
          <p className="text-sm mb-1">No analysis results yet</p>
          <p className="text-xs">Run analysis to see results</p>
        </div>
      </div>
    );
  }

  // Format displacement data
  const displacementData = Object.entries(results.displacements || {}).slice(0, 10).map(
    ([nodeId, disps]) => ({
      label: `Node ${nodeId}`,
      value: Array.isArray(disps)
        ? Math.max(...disps.map(Math.abs)).toExponential(3)
        : '0',
      unit: 'm',
    })
  );

  // Format reaction data
  const reactionData = Object.entries(results.reactions || {}).slice(0, 10).map(
    ([nodeId, reactions]) => ({
      label: `Node ${nodeId}`,
      value: Array.isArray(reactions)
        ? Math.max(...reactions.map(Math.abs)).toFixed(2)
        : '0',
      unit: 'kN',
    })
  );

  return (
    <div className="h-full overflow-y-auto p-4 space-y-4">
      {/* Summary */}
      <div className="grid grid-cols-2 gap-3">
        <div className="bg-blue-50 rounded-lg p-3 text-center">
          <ArrowDown className="w-5 h-5 mx-auto mb-1 text-blue-500" />
          <p className="text-xs text-blue-600 mb-1">Max Displacement</p>
          <p className="text-sm font-mono font-medium text-blue-800">
            {results.max_displacement?.toExponential(3) || '0'} m
          </p>
        </div>
        <div className="bg-green-50 rounded-lg p-3 text-center">
          <ArrowUp className="w-5 h-5 mx-auto mb-1 text-green-500" />
          <p className="text-xs text-green-600 mb-1">Max Reaction</p>
          <p className="text-sm font-mono font-medium text-green-800">
            {results.max_reaction?.toFixed(2) || '0'} kN
          </p>
        </div>
      </div>

      {/* Model info */}
      <div className="bg-gray-50 rounded-lg p-3">
        <div className="flex items-center gap-2 mb-2">
          <Target className="w-4 h-4 text-gray-500" />
          <span className="text-sm font-medium text-gray-700">Model Summary</span>
        </div>
        <div className="grid grid-cols-2 gap-2 text-xs">
          <div className="flex justify-between">
            <span className="text-gray-500">Nodes:</span>
            <span className="font-medium">{nodes.length}</span>
          </div>
          <div className="flex justify-between">
            <span className="text-gray-500">Beams:</span>
            <span className="font-medium">{beams.length}</span>
          </div>
        </div>
      </div>

      {/* Displacements table */}
      {displacementData.length > 0 && (
        <ResultsTable
          title="Nodal Displacements"
          icon={<ArrowDown className="w-4 h-4 text-blue-500" />}
          data={displacementData}
        />
      )}

      {/* Reactions table */}
      {reactionData.length > 0 && (
        <ResultsTable
          title="Support Reactions"
          icon={<ArrowUp className="w-4 h-4 text-green-500" />}
          data={reactionData}
        />
      )}

      {/* Note about more results */}
      {(Object.keys(results.displacements || {}).length > 10 ||
        Object.keys(results.reactions || {}).length > 10) && (
        <p className="text-xs text-gray-400 text-center">
          Showing first 10 results. Full data available via API.
        </p>
      )}
    </div>
  );
}
