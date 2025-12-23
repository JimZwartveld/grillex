import { useEffect, useRef } from 'react';
import {
  Edit2,
  Trash2,
  Box,
  Layers,
  ArrowDown,
  Anchor,
  Package,
  Move,
} from 'lucide-react';
import useStore from '../../stores/modelStore';

export interface ContextMenuState {
  isOpen: boolean;
  x: number;
  y: number;
  elementType: 'beam' | 'support' | 'load' | 'cargo' | null;
  elementId: number | null;
}

interface Props {
  onProperties: () => void;
  onDelete: () => void;
  onEditSection?: () => void;
  onEditMaterial?: () => void;
  // Additional beam options
  onAddLoadToBeam?: () => void;
  onAddSupportAtEnd?: () => void;
  // Support options
  onEditDOFConstraints?: () => void;
  // Load options
  onEditLoadValue?: () => void;
  // Cargo options
  onMoveCargo?: () => void;
}

// Reusable menu item component
function MenuItem({
  icon,
  label,
  onClick,
  danger = false,
}: {
  icon: React.ReactNode;
  label: string;
  onClick: () => void;
  danger?: boolean;
}) {
  return (
    <button
      className={`w-full px-3 py-2 text-left text-sm flex items-center gap-2 ${
        danger
          ? 'text-red-600 hover:bg-red-50'
          : 'hover:bg-blue-50'
      }`}
      onClick={onClick}
    >
      {icon}
      {label}
    </button>
  );
}

function MenuDivider() {
  return <div className="border-t border-gray-100 my-1" />;
}

export default function ContextMenu({
  onProperties,
  onDelete,
  onEditSection,
  onEditMaterial,
  onAddLoadToBeam,
  onAddSupportAtEnd,
  onEditDOFConstraints,
  onEditLoadValue,
  onMoveCargo,
}: Props) {
  const { contextMenu, closeContextMenu } = useStore();
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(e.target as Node)) {
        closeContextMenu();
      }
    };

    if (contextMenu.isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [contextMenu.isOpen, closeContextMenu]);

  // Close on escape
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        closeContextMenu();
      }
    };

    if (contextMenu.isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      return () => document.removeEventListener('keydown', handleKeyDown);
    }
  }, [contextMenu.isOpen, closeContextMenu]);

  if (!contextMenu.isOpen || !contextMenu.elementType) {
    return null;
  }

  const getElementLabel = () => {
    switch (contextMenu.elementType) {
      case 'beam':
        return `Beam ${contextMenu.elementId}`;
      case 'support':
        return 'Support';
      case 'load':
        return 'Load';
      case 'cargo':
        return `Cargo ${contextMenu.elementId}`;
      default:
        return 'Element';
    }
  };

  const isBeam = contextMenu.elementType === 'beam';
  const isSupport = contextMenu.elementType === 'support';
  const isLoad = contextMenu.elementType === 'load';
  const isCargo = contextMenu.elementType === 'cargo';

  const handleAction = (action: () => void) => {
    action();
    closeContextMenu();
  };

  // Render beam-specific menu items
  const renderBeamMenu = () => (
    <>
      <MenuItem
        icon={<Edit2 className="w-4 h-4" />}
        label="Properties..."
        onClick={() => handleAction(onProperties)}
      />
      <MenuDivider />
      {onEditSection && (
        <MenuItem
          icon={<Box className="w-4 h-4" />}
          label="Change Section..."
          onClick={() => handleAction(onEditSection)}
        />
      )}
      {onEditMaterial && (
        <MenuItem
          icon={<Layers className="w-4 h-4" />}
          label="Change Material..."
          onClick={() => handleAction(onEditMaterial)}
        />
      )}
      <MenuDivider />
      {onAddLoadToBeam && (
        <MenuItem
          icon={<ArrowDown className="w-4 h-4 text-red-500" />}
          label="Add Load to Beam..."
          onClick={() => handleAction(onAddLoadToBeam)}
        />
      )}
      {onAddSupportAtEnd && (
        <MenuItem
          icon={<Anchor className="w-4 h-4 text-green-500" />}
          label="Add Support at End..."
          onClick={() => handleAction(onAddSupportAtEnd)}
        />
      )}
      <MenuDivider />
      <MenuItem
        icon={<Trash2 className="w-4 h-4" />}
        label="Delete Beam"
        onClick={() => handleAction(onDelete)}
        danger
      />
    </>
  );

  // Render support-specific menu items
  const renderSupportMenu = () => (
    <>
      <MenuItem
        icon={<Edit2 className="w-4 h-4" />}
        label="Properties..."
        onClick={() => handleAction(onProperties)}
      />
      {onEditDOFConstraints && (
        <>
          <MenuDivider />
          <MenuItem
            icon={<Anchor className="w-4 h-4" />}
            label="Edit DOF Constraints..."
            onClick={() => handleAction(onEditDOFConstraints)}
          />
        </>
      )}
      <MenuDivider />
      <MenuItem
        icon={<Trash2 className="w-4 h-4" />}
        label="Remove Support"
        onClick={() => handleAction(onDelete)}
        danger
      />
    </>
  );

  // Render load-specific menu items
  const renderLoadMenu = () => (
    <>
      <MenuItem
        icon={<Edit2 className="w-4 h-4" />}
        label="Properties..."
        onClick={() => handleAction(onProperties)}
      />
      {onEditLoadValue && (
        <>
          <MenuDivider />
          <MenuItem
            icon={<ArrowDown className="w-4 h-4" />}
            label="Edit Load Value..."
            onClick={() => handleAction(onEditLoadValue)}
          />
        </>
      )}
      <MenuDivider />
      <MenuItem
        icon={<Trash2 className="w-4 h-4" />}
        label="Delete Load"
        onClick={() => handleAction(onDelete)}
        danger
      />
    </>
  );

  // Render cargo-specific menu items
  const renderCargoMenu = () => (
    <>
      <MenuItem
        icon={<Edit2 className="w-4 h-4" />}
        label="Properties..."
        onClick={() => handleAction(onProperties)}
      />
      {onMoveCargo && (
        <>
          <MenuDivider />
          <MenuItem
            icon={<Move className="w-4 h-4" />}
            label="Move Cargo..."
            onClick={() => handleAction(onMoveCargo)}
          />
        </>
      )}
      <MenuDivider />
      <MenuItem
        icon={<Trash2 className="w-4 h-4" />}
        label="Delete Cargo"
        onClick={() => handleAction(onDelete)}
        danger
      />
    </>
  );

  // Render the appropriate menu based on element type
  const renderMenuItems = () => {
    if (isBeam) return renderBeamMenu();
    if (isSupport) return renderSupportMenu();
    if (isLoad) return renderLoadMenu();
    if (isCargo) return renderCargoMenu();

    // Fallback for unknown element type
    return (
      <>
        <MenuItem
          icon={<Edit2 className="w-4 h-4" />}
          label="Properties..."
          onClick={() => handleAction(onProperties)}
        />
        <MenuDivider />
        <MenuItem
          icon={<Trash2 className="w-4 h-4" />}
          label="Delete"
          onClick={() => handleAction(onDelete)}
          danger
        />
      </>
    );
  };

  return (
    <div
      ref={menuRef}
      className="fixed z-50 bg-white rounded-lg shadow-lg border border-gray-200 py-1 min-w-[200px]"
      style={{
        left: contextMenu.x,
        top: contextMenu.y,
      }}
    >
      {/* Header showing element type */}
      <div className="px-3 py-1.5 text-xs font-semibold text-gray-500 border-b border-gray-100 flex items-center gap-1">
        {isBeam && <Box className="w-3 h-3 text-blue-500" />}
        {isSupport && <Anchor className="w-3 h-3 text-green-500" />}
        {isLoad && <ArrowDown className="w-3 h-3 text-red-500" />}
        {isCargo && <Package className="w-3 h-3 text-amber-600" />}
        {getElementLabel()}
      </div>

      {renderMenuItems()}
    </div>
  );
}
