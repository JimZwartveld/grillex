import ActionButtons from './ActionButtons';
import ModelTree from './ModelTree';
import LoadCaseSelector from './LoadCaseSelector';

export default function LeftPanel() {
  return (
    <div className="h-full flex flex-col bg-white">
      <ActionButtons />
      <LoadCaseSelector />
      <div className="flex-1 overflow-y-auto border-t border-gray-100">
        <ModelTree />
      </div>
    </div>
  );
}
