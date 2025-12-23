/**
 * TypeScript types for Grillex FEM model data
 */

export interface Node {
  id: number;
  position: [number, number, number];
  dof_active: boolean[];
}

export interface Beam {
  id: number;
  start: [number, number, number];
  end: [number, number, number];
  section: string;
  material: string;
  length: number;
}

export interface Material {
  name: string;
  E: number;
  nu: number;
  rho: number;
}

export interface Section {
  name: string;
  A: number;
  Iy: number;
  Iz: number;
  J: number;
}

export interface BoundaryCondition {
  node_id: number;
  dof: number;
  value: number;
}

export interface Load {
  node_id: number;
  dof: number;
  value: number;
}

export interface LoadCase {
  name: string;
  type: string;
  loads: Load[];
}

export interface AnalysisResults {
  displacements: Record<number, number[]>;
  reactions: Record<number, number[]>;
  max_displacement: number;
  max_reaction: number;
}

export interface CargoConnection {
  node_id: number;
  cargoOffset: [number, number, number];
}

export interface Cargo {
  id: number;
  name: string;
  cogPosition: [number, number, number];
  dimensions: [number, number, number];
  mass: number;
  connections: CargoConnection[];
}

export interface ModelState {
  name: string;
  nodes: Node[];
  beams: Beam[];
  materials: Material[];
  sections: Section[];
  boundaryConditions: BoundaryCondition[];
  loadCases: LoadCase[];
  cargos: Cargo[];
  results: AnalysisResults | null;
  isAnalyzed: boolean;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  toolCalls?: ToolCall[];
}

export interface ToolCall {
  name: string;
  input: Record<string, unknown>;
  result?: unknown;
}

export type ModelEventType =
  | 'model_updated'
  | 'model_created'
  | 'analysis_complete'
  | 'modal_analysis_complete'
  | 'nonlinear_analysis_complete'
  | 'error'
  | 'tool_executed'
  // Model modification events
  | 'beam_added' | 'beam_updated' | 'beam_deleted'
  | 'material_added' | 'material_updated' | 'material_deleted'
  | 'section_added' | 'section_updated' | 'section_deleted'
  | 'bc_added' | 'bc_deleted'
  | 'load_added' | 'load_deleted'
  | 'cargo_added' | 'cargo_updated' | 'cargo_deleted'
  | 'spring_added' | 'spring_deleted';

export interface ModelEvent {
  event_type: ModelEventType;
  data: Record<string, unknown>;
  timestamp: string;
}

export interface ToolResult {
  success: boolean;
  result?: unknown;
  error?: string;
}
