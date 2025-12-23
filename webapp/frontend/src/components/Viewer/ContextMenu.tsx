import { useEffect, useRef } from 'react';
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
}

export default function ContextMenu({ onProperties, onDelete, onEditSection, onEditMaterial }: Props) {
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

  return (
    <div
      ref={menuRef}
      className="fixed z-50 bg-white rounded-lg shadow-lg border border-gray-200 py-1 min-w-[180px]"
      style={{
        left: contextMenu.x,
        top: contextMenu.y,
      }}
    >
      {/* Header showing element type */}
      <div className="px-3 py-1.5 text-xs font-semibold text-gray-500 border-b border-gray-100">
        {getElementLabel()}
      </div>

      {/* Menu items */}
      <button
        className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
        onClick={() => {
          onProperties();
          closeContextMenu();
        }}
      >
        <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M11 5H6a2 2 0 00-2 2v11a2 2 0 002 2h11a2 2 0 002-2v-5m-1.414-9.414a2 2 0 112.828 2.828L11.828 15H9v-2.828l8.586-8.586z" />
        </svg>
        Properties...
      </button>

      {/* Beam-specific options */}
      {isBeam && onEditSection && (
        <button
          className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
          onClick={() => {
            onEditSection();
            closeContextMenu();
          }}
        >
          <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 5a1 1 0 011-1h14a1 1 0 011 1v2a1 1 0 01-1 1H5a1 1 0 01-1-1V5zM4 13a1 1 0 011-1h6a1 1 0 011 1v6a1 1 0 01-1 1H5a1 1 0 01-1-1v-6z" />
          </svg>
          Edit Section...
        </button>
      )}

      {isBeam && onEditMaterial && (
        <button
          className="w-full px-3 py-2 text-left text-sm hover:bg-blue-50 flex items-center gap-2"
          onClick={() => {
            onEditMaterial();
            closeContextMenu();
          }}
        >
          <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547M8 4h8l-1 1v5.172a2 2 0 00.586 1.414l5 5c1.26 1.26.367 3.414-1.415 3.414H4.828c-1.782 0-2.674-2.154-1.414-3.414l5-5A2 2 0 009 10.172V5L8 4z" />
          </svg>
          Edit Material...
        </button>
      )}

      <div className="border-t border-gray-100 my-1" />

      <button
        className="w-full px-3 py-2 text-left text-sm text-red-600 hover:bg-red-50 flex items-center gap-2"
        onClick={() => {
          onDelete();
          closeContextMenu();
        }}
      >
        <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
        </svg>
        Delete
      </button>
    </div>
  );
}
