import { useState } from 'react';
import { ChevronDown, ChevronRight, Box, CircleDot, Layers, Anchor, ArrowDown } from 'lucide-react';
import useStore from '../../stores/modelStore';

interface TreeNodeProps {
  label: string;
  count: number;
  icon: React.ReactNode;
  children?: React.ReactNode;
  defaultExpanded?: boolean;
}

function TreeNode({ label, count, icon, children, defaultExpanded = false }: TreeNodeProps) {
  const [expanded, setExpanded] = useState(defaultExpanded);

  return (
    <div className="mb-1">
      <button
        onClick={() => setExpanded(!expanded)}
        className="w-full flex items-center gap-2 px-2 py-1.5 text-left hover:bg-gray-100 rounded-md"
      >
        <span className="text-gray-400">
          {expanded ? <ChevronDown className="w-4 h-4" /> : <ChevronRight className="w-4 h-4" />}
        </span>
        <span className="text-gray-600">{icon}</span>
        <span className="flex-1 text-sm font-medium text-gray-700">{label}</span>
        <span className="text-xs text-gray-400 bg-gray-100 px-1.5 py-0.5 rounded">
          {count}
        </span>
      </button>
      {expanded && children && (
        <div className="ml-6 mt-1 space-y-0.5">{children}</div>
      )}
    </div>
  );
}

interface TreeLeafProps {
  label: string;
  onClick?: () => void;
  selected?: boolean;
}

function TreeLeaf({ label, onClick, selected }: TreeLeafProps) {
  return (
    <button
      onClick={onClick}
      className={`w-full text-left px-2 py-1 text-sm rounded-md ${
        selected
          ? 'bg-blue-100 text-blue-700'
          : 'text-gray-600 hover:bg-gray-50'
      }`}
    >
      {label}
    </button>
  );
}

function formatPosition(pos: number[]): string {
  return `(${pos.map((v) => v.toFixed(2)).join(', ')})`;
}

export default function ModelTree() {
  const { nodes, beams, materials, sections, boundaryConditions, loadCases, name } = useStore();

  return (
    <div className="p-3">
      <div className="mb-4">
        <h3 className="text-xs font-semibold text-gray-400 uppercase tracking-wider">
          Model
        </h3>
        <p className="text-sm font-medium text-gray-800 truncate mt-1">{name}</p>
      </div>

      <div className="space-y-1">
        <TreeNode
          label="Materials"
          count={materials.length}
          icon={<Box className="w-4 h-4" />}
        >
          {materials.length === 0 ? (
            <p className="text-xs text-gray-400 px-2 py-1">No materials defined</p>
          ) : (
            materials.map((m) => (
              <TreeLeaf key={m.name} label={`${m.name} (E=${(m.E / 1e6).toFixed(0)} MPa)`} />
            ))
          )}
        </TreeNode>

        <TreeNode
          label="Sections"
          count={sections.length}
          icon={<Layers className="w-4 h-4" />}
        >
          {sections.length === 0 ? (
            <p className="text-xs text-gray-400 px-2 py-1">No sections defined</p>
          ) : (
            sections.map((s) => (
              <TreeLeaf key={s.name} label={`${s.name} (A=${(s.A * 1e4).toFixed(2)} cm²)`} />
            ))
          )}
        </TreeNode>

        <TreeNode
          label="Nodes"
          count={nodes.length}
          icon={<CircleDot className="w-4 h-4" />}
        >
          {nodes.length === 0 ? (
            <p className="text-xs text-gray-400 px-2 py-1">No nodes</p>
          ) : (
            nodes.slice(0, 20).map((n) => (
              <TreeLeaf
                key={n.id}
                label={`Node ${n.id}: ${formatPosition(n.position)}`}
              />
            ))
          )}
          {nodes.length > 20 && (
            <p className="text-xs text-gray-400 px-2 py-1">... and {nodes.length - 20} more</p>
          )}
        </TreeNode>

        <TreeNode
          label="Beams"
          count={beams.length}
          icon={<Box className="w-4 h-4" />}
          defaultExpanded
        >
          {beams.length === 0 ? (
            <p className="text-xs text-gray-400 px-2 py-1">No beams defined</p>
          ) : (
            beams.slice(0, 20).map((b) => (
              <TreeLeaf
                key={b.id}
                label={`Beam ${b.id}: ${b.section} (L=${b.length.toFixed(2)}m)`}
              />
            ))
          )}
          {beams.length > 20 && (
            <p className="text-xs text-gray-400 px-2 py-1">... and {beams.length - 20} more</p>
          )}
        </TreeNode>

        <TreeNode
          label="Supports"
          count={boundaryConditions.length}
          icon={<Anchor className="w-4 h-4" />}
        >
          {boundaryConditions.length === 0 ? (
            <p className="text-xs text-gray-400 px-2 py-1">No supports defined</p>
          ) : (
            boundaryConditions.map((bc, i) => (
              <TreeLeaf key={i} label={`Node ${bc.node_id}: DOF ${bc.dof} = ${bc.value}`} />
            ))
          )}
        </TreeNode>

        <TreeNode
          label="Load Cases"
          count={loadCases.length}
          icon={<ArrowDown className="w-4 h-4" />}
        >
          {loadCases.length === 0 ? (
            <p className="text-xs text-gray-400 px-2 py-1">No load cases defined</p>
          ) : (
            loadCases.map((lc, i) => (
              <TreeLeaf key={i} label={`${lc.name} (${lc.loads.length} loads)`} />
            ))
          )}
        </TreeNode>
      </div>
    </div>
  );
}
