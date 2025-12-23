import { Dialog, Button } from '../common';
import type { Cargo } from '../../types/model';

interface Props {
  cargo: Cargo | null;
  isOpen: boolean;
  onClose: () => void;
}

export default function CargoPropertiesDialog({ cargo, isOpen, onClose }: Props) {
  if (!cargo) return null;

  return (
    <Dialog title={`Cargo: ${cargo.name}`} isOpen={isOpen} onClose={onClose}>
      <div className="space-y-4">
        {/* Cargo ID and Name */}
        <div className="grid grid-cols-2 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Cargo ID
            </label>
            <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
              {cargo.id}
            </div>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Name
            </label>
            <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
              {cargo.name}
            </div>
          </div>
        </div>

        {/* Mass */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Mass
          </label>
          <div className="text-sm text-gray-600 bg-gray-50 px-3 py-2 rounded border">
            {cargo.mass.toFixed(2)} mT
          </div>
        </div>

        {/* CoG Position */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Center of Gravity Position (m)
          </label>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">X:</span> {cargo.cogPosition[0].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Y:</span> {cargo.cogPosition[1].toFixed(3)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Z:</span> {cargo.cogPosition[2].toFixed(3)}
            </div>
          </div>
        </div>

        {/* Dimensions */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-2">
            Dimensions (m)
          </label>
          <div className="grid grid-cols-3 gap-2 text-sm">
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Width:</span> {cargo.dimensions[0].toFixed(2)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Height:</span> {cargo.dimensions[1].toFixed(2)}
            </div>
            <div className="bg-gray-50 px-3 py-2 rounded border text-center">
              <span className="text-gray-500">Depth:</span> {cargo.dimensions[2].toFixed(2)}
            </div>
          </div>
        </div>

        {/* Connections */}
        {cargo.connections.length > 0 && (
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Connections ({cargo.connections.length})
            </label>
            <div className="space-y-2 max-h-32 overflow-y-auto">
              {cargo.connections.map((conn, i) => (
                <div
                  key={i}
                  className="bg-gray-50 px-3 py-2 rounded border text-sm"
                >
                  <div className="flex justify-between">
                    <span className="text-gray-500">Node {conn.node_id}</span>
                    <span>
                      Offset: [{conn.cargoOffset.map((v) => v.toFixed(2)).join(', ')}]
                    </span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}

        <div className="flex justify-end gap-2 pt-2">
          <Button type="button" onClick={onClose}>
            Close
          </Button>
        </div>
      </div>
    </Dialog>
  );
}
