import { useEffect, useRef } from 'react';
import { Plus, Anchor, ArrowDown, Package } from 'lucide-react';
import useStore from '../../stores/modelStore';

interface Props {
  onAddBeam: (position: [number, number, number]) => void;
  onAddSupport: (position: [number, number, number]) => void;
  onAddLoad: (position: [number, number, number]) => void;
  onAddCargo: (position: [number, number, number]) => void;
}

export default function AddContextMenu({ onAddBeam, onAddSupport, onAddLoad, onAddCargo }: Props) {
  const { addContextMenu, closeAddContextMenu } = useStore();
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(e.target as Node)) {
        closeAddContextMenu();
      }
    };

    if (addContextMenu.isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [addContextMenu.isOpen, closeAddContextMenu]);

  // Close on escape
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        closeAddContextMenu();
      }
    };

    if (addContextMenu.isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      return () => document.removeEventListener('keydown', handleKeyDown);
    }
  }, [addContextMenu.isOpen, closeAddContextMenu]);

  if (!addContextMenu.isOpen || !addContextMenu.worldPosition) {
    return null;
  }

  const position = addContextMenu.worldPosition;
  const formattedPosition = `(${position[0].toFixed(2)}, ${position[1].toFixed(2)}, ${position[2].toFixed(2)})`;

  return (
    <div
      ref={menuRef}
      className="fixed z-50 bg-white rounded-lg shadow-lg border border-gray-200 py-1 min-w-[200px]"
      style={{
        left: addContextMenu.x,
        top: addContextMenu.y,
      }}
    >
      {/* Header showing position */}
      <div className="px-3 py-1.5 text-xs font-semibold text-gray-500 border-b border-gray-100">
        <span className="flex items-center gap-1">
          <Plus className="w-3 h-3" />
          Add at {formattedPosition}
        </span>
      </div>

      {/* Add Beam */}
      <button
        className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
        onClick={() => {
          onAddBeam(position);
          closeAddContextMenu();
        }}
      >
        <svg className="w-4 h-4 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 12h16" />
        </svg>
        Add Beam from here...
      </button>

      {/* Add Support */}
      <button
        className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
        onClick={() => {
          onAddSupport(position);
          closeAddContextMenu();
        }}
      >
        <Anchor className="w-4 h-4 text-green-500" />
        Add Support at this point
      </button>

      {/* Add Load */}
      <button
        className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
        onClick={() => {
          onAddLoad(position);
          closeAddContextMenu();
        }}
      >
        <ArrowDown className="w-4 h-4 text-red-500" />
        Add Load at this point
      </button>

      <div className="border-t border-gray-100 my-1" />

      {/* Add Cargo */}
      <button
        className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
        onClick={() => {
          onAddCargo(position);
          closeAddContextMenu();
        }}
      >
        <Package className="w-4 h-4 text-amber-600" />
        Add Cargo here
      </button>
    </div>
  );
}
