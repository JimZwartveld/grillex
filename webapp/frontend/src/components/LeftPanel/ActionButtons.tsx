import { useState } from 'react';
import { Plus, Play, RefreshCw, Box, Anchor, ArrowDown } from 'lucide-react';
import { Button } from '../common';
import AddBeamDialog from './AddBeamDialog';
import AddSupportDialog from './AddSupportDialog';
import AddLoadDialog from './AddLoadDialog';
import useStore from '../../stores/modelStore';
import { toolsApi } from '../../api/client';

type DialogType = 'beam' | 'support' | 'load' | null;

export default function ActionButtons() {
  const [activeDialog, setActiveDialog] = useState<DialogType>(null);
  const [isAnalyzing, setIsAnalyzing] = useState(false);
  const { beams, fetchModelState, resetModel } = useStore();

  const handleNewModel = async () => {
    const response = await toolsApi.execute('create_model', { name: 'New Model' });
    if (response.success) {
      await fetchModelState();
    }
  };

  const handleAnalyze = async () => {
    setIsAnalyzing(true);
    try {
      const response = await toolsApi.execute('analyze', {});
      if (response.success) {
        await fetchModelState();
      }
    } finally {
      setIsAnalyzing(false);
    }
  };

  const handleReset = async () => {
    if (confirm('Are you sure you want to reset the model? All changes will be lost.')) {
      await resetModel();
    }
  };

  return (
    <div className="p-3 border-b border-gray-200">
      <div className="space-y-2">
        <div className="flex gap-2">
          <Button
            onClick={handleNewModel}
            className="flex-1"
            size="sm"
          >
            <Plus className="w-4 h-4 mr-1" />
            New
          </Button>
          <Button
            onClick={handleReset}
            size="sm"
            variant="danger"
          >
            <RefreshCw className="w-4 h-4" />
          </Button>
        </div>

        <div className="pt-2 border-t border-gray-100">
          <p className="text-xs text-gray-400 uppercase tracking-wider mb-2">Add Elements</p>
          <div className="grid grid-cols-3 gap-2">
            <Button
              onClick={() => setActiveDialog('beam')}
              size="sm"
              title="Add Beam"
            >
              <Box className="w-4 h-4" />
            </Button>
            <Button
              onClick={() => setActiveDialog('support')}
              size="sm"
              title="Add Support"
            >
              <Anchor className="w-4 h-4" />
            </Button>
            <Button
              onClick={() => setActiveDialog('load')}
              size="sm"
              title="Add Load"
            >
              <ArrowDown className="w-4 h-4" />
            </Button>
          </div>
        </div>

        <div className="pt-2 border-t border-gray-100">
          <Button
            onClick={handleAnalyze}
            variant="primary"
            className="w-full"
            disabled={beams.length === 0 || isAnalyzing}
          >
            <Play className="w-4 h-4 mr-1" />
            {isAnalyzing ? 'Analyzing...' : 'Run Analysis'}
          </Button>
        </div>
      </div>

      {/* Dialogs */}
      <AddBeamDialog
        isOpen={activeDialog === 'beam'}
        onClose={() => setActiveDialog(null)}
      />
      <AddSupportDialog
        isOpen={activeDialog === 'support'}
        onClose={() => setActiveDialog(null)}
      />
      <AddLoadDialog
        isOpen={activeDialog === 'load'}
        onClose={() => setActiveDialog(null)}
      />
    </div>
  );
}
