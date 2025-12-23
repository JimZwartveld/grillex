import { Dialog, Button } from '../common';
import useStore from '../../stores/modelStore';
import type { BoundaryCondition } from '../../types/model';

interface Props {
  support: BoundaryCondition | null;
  nodePosition: [number, number, number] | null;
  isOpen: boolean;
  onClose: () => void;
}

const DOF_NAMES = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ'];

export default function SupportPropertiesDialog({ support, nodePosition, isOpen, onClose }: Props) {
  const { boundaryConditions } = useStore();

  if (!support || !nodePosition) return null;

  // Find all BCs at this node
  const bcsAtNode = boundaryConditions.filter((bc) => bc.node_id === support.node_id);

  return (
    <Dialog title="Support Properties" isOpen={isOpen} onClose={onClose}>
      <div className="space-y-4">
        {/* Node Position */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Position (m)
          </label>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">X:</span> {nodePosition[0].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Y:</span> {nodePosition[1].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Z:</span> {nodePosition[2].toFixed(3)}
            </div>
          </div>
        </div>

        {/* Node ID */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Node ID
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {support.node_id}
          </div>
        </div>

        {/* Fixed DOFs */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Fixed Degrees of Freedom
          </label>
          <div className="grid grid-cols-3 gap-2">
            {[0, 1, 2, 3, 4, 5].map((dofIndex) => {
              const bc = bcsAtNode.find((b) => b.dof === dofIndex);
              const isFixed = !!bc;
              return (
                <div
                  key={dofIndex}
                  className={`px-3 py-2 rounded border text-sm text-center ${
                    isFixed
                      ? 'bg-blue-50 border-blue-200 text-blue-700'
                      : 'bg-gray-50 border-gray-200 text-gray-400'
                  }`}
                >
                  {DOF_NAMES[dofIndex]}
                  {isFixed && (
                    <span className="block text-xs">
                      = {bc?.value.toFixed(4)}
                    </span>
                  )}
                </div>
              );
            })}
          </div>
        </div>

        {/* Support Type Description */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Support Type
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {bcsAtNode.length === 6
              ? 'Fixed (all DOFs restrained)'
              : bcsAtNode.length === 3 && bcsAtNode.every((bc) => bc.dof < 3)
              ? 'Pinned (translations restrained)'
              : `Custom (${bcsAtNode.length} DOFs restrained)`}
          </div>
        </div>

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Close
          </Button>
        </div>
      </div>
    </Dialog>
  );
}
