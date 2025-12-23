/**
 * TypeScript types for Grillex FEM model data
 *
 * These types are aligned with the C++ Model class structure from
 * cpp/include/grillex/model.hpp for consistency and easier maintenance.
 */

/**
 * Node representation matching C++ Node class
 */
export interface Node {
  id: number;
  /** Position [x, y, z] in meters */
  position: [number, number, number];
  /** DOF active flags: [UX, UY, UZ, RX, RY, RZ, WARP] */
  dof_active: boolean[];
  /** Number of active DOFs */
  num_dofs?: number;
}

/**
 * Beam element representation matching C++ BeamElement class
 */
export interface Beam {
  id: number;
  /** Start position [x, y, z] in meters */
  start: [number, number, number];
  /** End position [x, y, z] in meters */
  end: [number, number, number];
  /** Section name reference */
  section: string;
  /** Material name reference */
  material: string;
  /** Beam length in meters */
  length: number;
  /** Node ID at start (optional for backwards compatibility) */
  node_i_id?: number;
  /** Node ID at end (optional for backwards compatibility) */
  node_j_id?: number;
  /** Roll angle about beam axis in radians */
  roll_angle?: number;
  /** Beam formulation type */
  formulation?: 'euler_bernoulli' | 'timoshenko';
  /** Whether warping DOF is enabled */
  warping_enabled?: boolean;
}

/**
 * Material representation matching C++ Material class
 */
export interface Material {
  /** Material name */
  name: string;
  /** Young's modulus in kN/m² */
  E: number;
  /** Poisson's ratio (dimensionless) */
  nu: number;
  /** Density in mT/m³ */
  rho: number;
  /** Shear modulus in kN/m² (computed from E and nu) */
  G?: number;
}

/**
 * Cross-section representation matching C++ Section class
 */
export interface Section {
  /** Section name */
  name: string;
  /** Cross-sectional area in m² */
  A: number;
  /** Moment of inertia about local y-axis in m⁴ */
  Iy: number;
  /** Moment of inertia about local z-axis in m⁴ */
  Iz: number;
  /** Torsional constant in m⁴ */
  J: number;
  /** Warping constant in m⁶ (optional) */
  Iw?: number;
  /** Shear area factor for y-direction (optional) */
  ky?: number;
  /** Shear area factor for z-direction (optional) */
  kz?: number;
}

/**
 * Boundary condition matching C++ BoundaryCondition
 */
export interface BoundaryCondition {
  /** Node ID where BC is applied */
  node_id: number;
  /** DOF index (0-6) or DOF string */
  dof: number | string;
  /** Prescribed value */
  value: number;
  /** Whether this is a prescribed displacement (vs constraint) */
  is_prescribed?: boolean;
}

/**
 * Point load at a node
 */
export interface Load {
  /** Node ID where load is applied */
  node_id: number;
  /** DOF index (0-5) or DOF string */
  dof: number | string;
  /** Load value in kN (forces) or kNm (moments) */
  value: number;
}

/**
 * Line load along a beam element
 */
export interface LineLoad {
  /** Beam ID where load is applied */
  beam_id: number;
  /** Direction (local or global) */
  direction: 'local_y' | 'local_z' | 'global_x' | 'global_y' | 'global_z';
  /** Load value at start in kN/m */
  value_start: number;
  /** Load value at end in kN/m */
  value_end: number;
}

/**
 * Acceleration load (body force)
 */
export interface Acceleration {
  /** Acceleration components [ax, ay, az] in m/s² */
  components: [number, number, number];
}

/**
 * Load case matching C++ LoadCase class
 */
export interface LoadCase {
  /** Load case ID */
  id?: number;
  /** Load case name */
  name: string;
  /** Load case type for combination purposes */
  type: LoadCaseType | string;
  /** Point loads in this case */
  loads: Load[];
  /** Line loads in this case (optional) */
  line_loads?: LineLoad[];
  /** Accelerations in this case (optional) */
  accelerations?: Acceleration[];
}

/** Standard load case types per Eurocode */
export type LoadCaseType = 'permanent' | 'variable' | 'accidental' | 'seismic';

/**
 * Spring element representation
 */
export interface Spring {
  /** Spring ID */
  id: number;
  /** First node ID */
  node1: number;
  /** Second node ID */
  node2: number;
  /** Stiffness values per DOF */
  stiffness: number[];
  /** Whether this is a nonlinear spring */
  isNonlinear?: boolean;
  /** Whether this spring has a gap */
  hasGap?: boolean;
}

/**
 * Analysis results with displacement and reaction data
 */
export interface AnalysisResults {
  /** Displacements by node ID */
  displacements: Record<number, number[]>;
  /** Reactions by node ID */
  reactions: Record<number, number[]>;
  /** Maximum displacement magnitude */
  max_displacement: number;
  /** Maximum reaction magnitude */
  max_reaction: number;
}

/**
 * Connection point from cargo to structure
 */
export interface CargoConnection {
  /** Node ID on the structure */
  node_id: number;
  /** Offset from cargo COG [x, y, z] in meters */
  cargoOffset: [number, number, number];
}

/**
 * Cargo item representation
 */
export interface Cargo {
  /** Cargo ID */
  id: number;
  /** Cargo name */
  name: string;
  /** Center of gravity position [x, y, z] in meters */
  cogPosition: [number, number, number];
  /** Bounding box dimensions [length_x, width_y, height_z] in meters */
  dimensions: [number, number, number];
  /** Mass in metric tonnes (mT) */
  mass: number;
  /** Connection points to structure */
  connections: CargoConnection[];
}

/**
 * Constraint (MPC or rigid link)
 */
export interface Constraint {
  /** Constraint ID */
  id: number;
  /** Constraint type */
  type: 'mpc' | 'rigid_link';
  /** Master node ID */
  master_node: number;
  /** Slave node IDs */
  slave_nodes: number[];
  /** Constrained DOFs */
  dofs: number[];
}

/**
 * Complete model state matching C++ Model class
 */
export interface ModelState {
  /** Model name */
  name: string;
  /** All nodes in the model */
  nodes: Node[];
  /** All beam elements */
  beams: Beam[];
  /** All materials */
  materials: Material[];
  /** All cross-sections */
  sections: Section[];
  /** All boundary conditions */
  boundaryConditions: BoundaryCondition[];
  /** All load cases */
  loadCases: LoadCase[];
  /** All cargo items */
  cargos: Cargo[];
  /** All spring elements (optional) */
  springs?: Spring[];
  /** All constraints (optional) */
  constraints?: Constraint[];
  /** Analysis results (null if not analyzed) */
  results: AnalysisResults | null;
  /** Whether model has been analyzed */
  isAnalyzed: boolean;
}

/**
 * Chat message in the conversation
 */
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  toolCalls?: ToolCall[];
}

/**
 * Tool call record
 */
export interface ToolCall {
  name: string;
  input: Record<string, unknown>;
  result?: unknown;
}

/**
 * Event types for model updates via SSE
 */
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
  | 'load_case_added' | 'load_case_updated' | 'load_case_deleted'
  | 'cargo_added' | 'cargo_updated' | 'cargo_deleted'
  | 'spring_added' | 'spring_deleted'
  | 'constraint_added' | 'constraint_deleted';

/**
 * Model event from SSE
 */
export interface ModelEvent {
  event_type: ModelEventType;
  data: Record<string, unknown>;
  timestamp: string;
}

/**
 * Tool execution result
 */
export interface ToolResult {
  success: boolean;
  result?: unknown;
  error?: string;
  suggestion?: string;
}

/**
 * DOF index enumeration (matching C++ DOFIndex)
 */
export enum DOFIndex {
  UX = 0,
  UY = 1,
  UZ = 2,
  RX = 3,
  RY = 4,
  RZ = 5,
  WARP = 6,
}

/**
 * DOF name string literals
 */
export type DOFName = 'UX' | 'UY' | 'UZ' | 'RX' | 'RY' | 'RZ' | 'WARP';

/**
 * Convert DOF name to index
 */
export function dofNameToIndex(name: DOFName): DOFIndex {
  const mapping: Record<DOFName, DOFIndex> = {
    UX: DOFIndex.UX,
    UY: DOFIndex.UY,
    UZ: DOFIndex.UZ,
    RX: DOFIndex.RX,
    RY: DOFIndex.RY,
    RZ: DOFIndex.RZ,
    WARP: DOFIndex.WARP,
  };
  return mapping[name];
}

/**
 * Convert DOF index to name
 */
export function dofIndexToName(index: DOFIndex): DOFName {
  const mapping: Record<DOFIndex, DOFName> = {
    [DOFIndex.UX]: 'UX',
    [DOFIndex.UY]: 'UY',
    [DOFIndex.UZ]: 'UZ',
    [DOFIndex.RX]: 'RX',
    [DOFIndex.RY]: 'RY',
    [DOFIndex.RZ]: 'RZ',
    [DOFIndex.WARP]: 'WARP',
  };
  return mapping[index];
}
