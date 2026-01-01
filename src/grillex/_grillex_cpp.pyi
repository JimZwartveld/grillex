"""
Type stubs for the Grillex C++ extension module.

This file provides type hints for IDE support (autocomplete, type checking).
"""

from typing import List, Tuple, Optional, overload
from enum import IntEnum
import numpy as np
from numpy.typing import NDArray


# ==============================================================================
# Enums
# ==============================================================================

class DOFIndex(IntEnum):
    """Degree of freedom indices."""
    UX = 0
    UY = 1
    UZ = 2
    RX = 3
    RY = 4
    RZ = 5
    WARP = 6


class BeamFormulation(IntEnum):
    """Beam element formulation types."""
    EulerBernoulli = 0
    Timoshenko = 1


class SolverMethod(IntEnum):
    """Linear solver methods."""
    SparseLU = 0
    SimplicialLDLT = 1
    ConjugateGradient = 2


class LoadCaseType(IntEnum):
    """Load case types for combination factors."""
    Permanent = 0
    Variable = 1
    Environmental = 2
    Accidental = 3


class ReleaseCombo4DOF(IntEnum):
    """4-bit release combinations for axial and shear."""
    FIXED_FIXED_FIXED_FIXED = 0
    FIXED_FIXED_FREE_FIXED = 1
    FIXED_FIXED_FIXED_FREE = 2
    FIXED_FIXED_FREE_FREE = 3
    FREE_FIXED_FIXED_FIXED = 4
    FREE_FIXED_FREE_FIXED = 5
    FIXED_FREE_FIXED_FREE = 6
    FREE_FIXED_FREE_FREE = 7
    FREE_FREE_FIXED_FIXED = 8
    FREE_FREE_FREE_FIXED = 9
    FREE_FREE_FIXED_FREE = 10
    FREE_FREE_FREE_FREE = 15


class ReleaseCombo2DOF(IntEnum):
    """2-bit release combinations for moment and torsion."""
    FIXED_FIXED = 0
    FIXED_FREE = 1
    FREE_FIXED = 2
    FREE_FREE = 3


class ReleaseComboWarping(IntEnum):
    """4-bit release combinations for warping (theta and phi at each end)."""
    FIXED_FIXED_FIXED_FIXED = 0
    FIXED_FIXED_FREE_FREE = 3  # Cantilever
    FIXED_FREE_FIXED_FREE = 5  # Pure St. Venant
    FREE_FREE_FIXED_FIXED = 12  # Reverse cantilever
    FREE_FREE_FREE_FREE = 15  # All free


# ==============================================================================
# Core Data Structures
# ==============================================================================

class Node:
    """3D point with up to 7 DOFs (including warping)."""
    id: int
    x: float
    y: float
    z: float
    dof_active: List[bool]

    def __init__(self, id: int, x: float, y: float, z: float) -> None: ...
    def coords(self) -> NDArray[np.float64]: ...
    def distance_to(self, other: 'Node') -> float: ...


class NodeRegistry:
    """Node management with automatic merging."""
    def get_or_create_node(self, x: float, y: float, z: float) -> Node: ...
    def get_node(self, id: int) -> Optional[Node]: ...
    def find_node_at(self, x: float, y: float, z: float) -> Optional[Node]: ...
    def size(self) -> int: ...


class Material:
    """Material properties."""
    name: str
    E: float  # Young's modulus [kN/m²]
    nu: float  # Poisson's ratio
    G: float  # Shear modulus [kN/m²]
    rho: float  # Density [mT/m³]

    def __init__(self, name: str, E: float, nu: float, rho: float = 0.0) -> None: ...


class Section:
    """Cross-section properties for beams."""
    name: str
    A: float  # Area [m²]
    Iy: float  # Moment of inertia about y [m⁴]
    Iz: float  # Moment of inertia about z [m⁴]
    J: float  # Torsional constant [m⁴]
    Iw: float  # Warping constant [m⁶]
    omega_max: float  # Maximum sectorial coordinate [m²]
    Asy: float  # Shear area in y [m²]
    Asz: float  # Shear area in z [m²]

    def __init__(self, name: str, A: float, Iy: float, Iz: float, J: float) -> None: ...
    def set_warping_properties(self, Iw: float, omega_max: float) -> None: ...
    def set_shear_areas(self, Asy: float, Asz: float) -> None: ...


class LocalAxes:
    """Local coordinate system for beam elements."""
    x: NDArray[np.float64]
    y: NDArray[np.float64]
    z: NDArray[np.float64]

    @staticmethod
    def from_nodes(node_i: Node, node_j: Node, roll: float = 0.0) -> 'LocalAxes': ...


class EndRelease:
    """End release (pin/hinge) configuration."""
    axial: bool
    shear_y: bool
    shear_z: bool
    torsion: bool
    moment_y: bool
    moment_z: bool
    warping: bool

    def __init__(self) -> None: ...
    def all_fixed(self) -> bool: ...
    def all_released(self) -> bool: ...


class BeamConfig:
    """Configuration for beam element creation."""
    formulation: BeamFormulation
    use_warping: bool
    release_i: EndRelease
    release_j: EndRelease

    def __init__(self) -> None: ...


# ==============================================================================
# Phase 7: Internal Actions Structs
# ==============================================================================

class DisplacementLine:
    """Displacements and rotations at a position along the beam."""
    x: float  # Position along beam [0, L] in meters
    u: float  # Axial displacement [m]
    v: float  # Lateral displacement in y [m]
    w: float  # Lateral displacement in z [m]
    theta_x: float  # Twist rotation (torsion) [rad]
    theta_y: float  # Bending rotation about y [rad]
    theta_z: float  # Bending rotation about z [rad]
    phi_prime: float  # Warping parameter (rate of twist) [rad]

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, x: float) -> None: ...


class EndForces:
    """Element end forces in local coordinates."""
    N: float  # Axial force [kN] (positive = tension)
    Vy: float  # Shear force in local y [kN]
    Vz: float  # Shear force in local z [kN]
    Mx: float  # Torsion moment [kN·m]
    My: float  # Bending moment about local y [kN·m]
    Mz: float  # Bending moment about local z [kN·m]
    B: float  # Bimoment [kN·m²] (for 14-DOF warping elements)

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self, N: float, Vy: float, Vz: float,
        Mx: float, My: float, Mz: float, B: float = 0.0
    ) -> None: ...

    def to_vector6(self) -> NDArray[np.float64]: ...
    def to_vector7(self) -> NDArray[np.float64]: ...


class InternalActions:
    """Internal actions at a position along the beam."""
    x: float  # Position along beam [0, L] in meters
    N: float  # Axial force [kN]
    Vy: float  # Shear force in y [kN]
    Vz: float  # Shear force in z [kN]
    Mx: float  # Torsion moment [kN·m]
    My: float  # Moment about y [kN·m]
    Mz: float  # Moment about z [kN·m]

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, x: float) -> None: ...
    @overload
    def __init__(
        self, x: float, N: float, Vy: float, Vz: float,
        Mx: float, My: float, Mz: float
    ) -> None: ...


class WarpingInternalActions(InternalActions):
    """Warping-specific internal actions (extends InternalActions)."""
    B: float  # Bimoment [kN·m²]
    Mx_sv: float  # St. Venant torsion component [kN·m]
    Mx_w: float  # Warping torsion component [kN·m]
    sigma_w_max: float  # Maximum warping normal stress [kN/m²]

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, x: float) -> None: ...


class ActionExtreme:
    """Extremum location and value for moment/shear along beam elements."""
    x: float  # Position along beam [m]
    value: float  # Value at extremum

    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, x: float, value: float) -> None: ...


class DistributedLoad:
    """Distributed load in local coordinates."""
    q_start: float  # Load at start [kN/m]
    q_end: float  # Load at end [kN/m]

    def __init__(self, q_start: float = 0.0, q_end: float = 0.0) -> None: ...


# ==============================================================================
# Beam Element
# ==============================================================================

class BeamElementBase:
    """Abstract base class for beam elements."""
    def compute_local_stiffness(self) -> NDArray[np.float64]: ...
    def compute_local_mass(self) -> NDArray[np.float64]: ...
    def compute_transformation(self) -> NDArray[np.float64]: ...
    def num_dofs(self) -> int: ...
    def get_formulation(self) -> BeamFormulation: ...
    def has_warping(self) -> bool: ...


class BeamElement(BeamElementBase):
    """3D beam element (Euler-Bernoulli or Timoshenko, 12 or 14 DOF)."""
    id: int
    node_i: Node
    node_j: Node
    material: Material
    section: Section
    local_axes: LocalAxes
    length: float
    offset_i: NDArray[np.float64]
    offset_j: NDArray[np.float64]
    releases: Tuple[EndRelease, EndRelease]
    config: BeamConfig

    # Stiffness and mass matrices
    def local_stiffness_matrix(self) -> NDArray[np.float64]: ...
    def transformation_matrix(self) -> NDArray[np.float64]: ...
    def global_stiffness_matrix(self) -> NDArray[np.float64]: ...
    def local_mass_matrix(self) -> NDArray[np.float64]: ...
    def global_mass_matrix(self) -> NDArray[np.float64]: ...

    # Warping matrices
    def local_stiffness_matrix_warping(self) -> NDArray[np.float64]: ...
    def local_mass_matrix_warping(self) -> NDArray[np.float64]: ...
    def transformation_matrix_warping(self) -> NDArray[np.float64]: ...
    def global_stiffness_matrix_warping(self) -> NDArray[np.float64]: ...
    def global_mass_matrix_warping(self) -> NDArray[np.float64]: ...

    # Offset handling
    def set_offsets(self, offset_i: NDArray[np.float64], offset_j: NDArray[np.float64]) -> None: ...
    def has_offsets(self) -> bool: ...
    def effective_length(self) -> float: ...
    def offset_transformation_matrix(self) -> NDArray[np.float64]: ...
    def offset_transformation_matrix_warping(self) -> NDArray[np.float64]: ...

    # Geometry
    def direction_vector(self) -> NDArray[np.float64]: ...

    # Equivalent nodal forces
    def equivalent_nodal_forces(
        self,
        w_start: NDArray[np.float64],
        w_end: NDArray[np.float64]
    ) -> NDArray[np.float64]: ...

    # Displacements
    def get_element_displacements_local(
        self,
        global_displacements: NDArray[np.float64],
        dof_handler: 'DOFHandler'
    ) -> NDArray[np.float64]: ...

    def get_displacements_at(
        self,
        x: float,
        global_displacements: NDArray[np.float64],
        dof_handler: 'DOFHandler'
    ) -> DisplacementLine: ...

    # End forces (Task 7.1)
    def compute_end_forces(
        self,
        global_displacements: NDArray[np.float64],
        dof_handler: 'DOFHandler'
    ) -> Tuple[EndForces, EndForces]: ...

    # Distributed load queries (Task 7.0)
    def get_distributed_load_y(self, load_case: 'LoadCase') -> DistributedLoad: ...
    def get_distributed_load_z(self, load_case: 'LoadCase') -> DistributedLoad: ...
    def get_distributed_load_axial(self, load_case: 'LoadCase') -> DistributedLoad: ...

    # Internal actions (Task 7.2)
    def get_internal_actions(
        self,
        x: float,
        global_displacements: NDArray[np.float64],
        dof_handler: 'DOFHandler',
        load_case: Optional['LoadCase'] = None
    ) -> InternalActions: ...

    def find_moment_extremes(
        self,
        axis: str,
        global_displacements: NDArray[np.float64],
        dof_handler: 'DOFHandler',
        load_case: Optional['LoadCase'] = None
    ) -> Tuple[ActionExtreme, ActionExtreme]: ...

    # Warping internal actions (Task 7.2b)
    def get_warping_internal_actions(
        self,
        x: float,
        global_displacements: NDArray[np.float64],
        dof_handler: 'DOFHandler'
    ) -> WarpingInternalActions: ...

    def compute_warping_stress(self, bimoment: float) -> float: ...


def create_beam_element(
    id: int,
    node_i: Node,
    node_j: Node,
    material: Material,
    section: Section,
    config: BeamConfig = ...,
    roll: float = 0.0
) -> BeamElement: ...


def are_elements_collinear(
    elem1: BeamElement,
    elem2: BeamElement,
    tolerance: float = 1e-6
) -> bool: ...


# ==============================================================================
# DOF Handling
# ==============================================================================

class WarpingDOFInfo:
    """Information about warping DOF for element at node."""
    element_id: int
    node_id: int
    global_dof: int
    coupling_group: int


class WarpingCoupling:
    """Group of coupled warping DOFs for collinear elements."""
    dof_infos: List[WarpingDOFInfo]
    master_dof: int


class DOFHandler:
    """Global DOF numbering with element-specific warping support."""
    def assign_dofs(
        self,
        nodes: List[Node],
        elements: List[BeamElement]
    ) -> None: ...

    def get_global_dof(
        self,
        node_id: int,
        local_dof: DOFIndex
    ) -> int: ...

    def get_warping_dof(
        self,
        element_id: int,
        node_id: int
    ) -> int: ...

    def num_dofs(self) -> int: ...
    def num_free_dofs(self) -> int: ...
    def num_fixed_dofs(self) -> int: ...


# ==============================================================================
# Assembly
# ==============================================================================

class Assembler:
    """Assembles global stiffness and mass matrices from elements."""
    def assemble_stiffness(
        self,
        elements: List[BeamElement],
        dof_handler: DOFHandler
    ) -> NDArray[np.float64]: ...

    def assemble_mass(
        self,
        elements: List[BeamElement],
        dof_handler: DOFHandler
    ) -> NDArray[np.float64]: ...


# ==============================================================================
# Boundary Conditions
# ==============================================================================

class FixedDOF:
    """Represents a fixed degree of freedom with prescribed value."""
    global_dof: int
    value: float

    def __init__(self, global_dof: int, value: float = 0.0) -> None: ...


class BCHandler:
    """Boundary condition handler for managing fixed DOFs."""
    def add_fixed_dof(
        self,
        node_id: int,
        dof: DOFIndex,
        value: float = 0.0
    ) -> None: ...

    def fix_node(self, node_id: int) -> None: ...

    def apply_to_system(
        self,
        K: NDArray[np.float64],
        F: NDArray[np.float64],
        dof_handler: DOFHandler
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]: ...

    def get_fixed_dofs(self) -> List[FixedDOF]: ...


# ==============================================================================
# Solver
# ==============================================================================

class LinearSolver:
    """Linear solver for finite element systems (K * u = F)."""
    def __init__(self, method: SolverMethod = SolverMethod.SparseLU) -> None: ...

    def solve(
        self,
        K: NDArray[np.float64],
        F: NDArray[np.float64]
    ) -> NDArray[np.float64]: ...


# ==============================================================================
# Loads
# ==============================================================================

class NodalLoad:
    """Concentrated force/moment at a node."""
    node_id: int
    dof: DOFIndex
    value: float

    def __init__(self, node_id: int, dof: DOFIndex, value: float) -> None: ...


class LineLoad:
    """Distributed load along a beam element."""
    element_id: int
    w_start: NDArray[np.float64]
    w_end: NDArray[np.float64]

    def __init__(
        self,
        element_id: int,
        w_start: NDArray[np.float64],
        w_end: NDArray[np.float64]
    ) -> None: ...


class LoadCase:
    """Load case containing all loads for a specific scenario."""
    name: str
    load_type: LoadCaseType

    def __init__(
        self,
        name: str = "Default",
        load_type: LoadCaseType = LoadCaseType.Permanent
    ) -> None: ...

    def add_nodal_load(self, node_id: int, dof: DOFIndex, value: float) -> None: ...
    def add_line_load(
        self,
        element_id: int,
        w_start: NDArray[np.float64],
        w_end: NDArray[np.float64]
    ) -> None: ...
    def set_acceleration(self, ax: float, ay: float, az: float) -> None: ...
    def remove_nodal_load(self, index: int) -> bool: ...
    def remove_line_load(self, index: int) -> bool: ...
    def clear(self) -> None: ...
    def is_empty(self) -> bool: ...
    def get_nodal_loads(self) -> List[NodalLoad]: ...
    def get_line_loads(self) -> List[LineLoad]: ...


class LoadCaseResult:
    """Results for a single load case analysis."""
    load_case_name: str
    displacements: NDArray[np.float64]
    reactions: NDArray[np.float64]


class LoadCombinationTerm:
    """Term in a load combination (load case + factor)."""
    load_case_name: str
    factor: float

    def __init__(self, load_case_name: str, factor: float) -> None: ...


class LoadCombination:
    """Load combination for code-based analysis."""
    name: str
    terms: List[LoadCombinationTerm]

    def __init__(self, name: str) -> None: ...
    def add_term(self, load_case_name: str, factor: float) -> None: ...


# ==============================================================================
# Constraints
# ==============================================================================

class EqualityConstraint:
    """Simple equality constraint between two DOFs."""
    master_dof: int
    slave_dof: int
    coefficient: float

    def __init__(
        self,
        master_dof: int,
        slave_dof: int,
        coefficient: float = 1.0
    ) -> None: ...


class RigidLink:
    """Rigid link constraint between master and slave nodes."""
    master_node_id: int
    slave_node_id: int

    def __init__(self, master_node_id: int, slave_node_id: int) -> None: ...


class ReducedSystem:
    """Result of system reduction with MPC constraints."""
    K_reduced: NDArray[np.float64]
    F_reduced: NDArray[np.float64]
    T: NDArray[np.float64]  # Transformation matrix


class ConstraintHandler:
    """Multi-point constraint (MPC) handler."""
    def add_equality_constraint(
        self,
        master_dof: int,
        slave_dof: int,
        coefficient: float = 1.0
    ) -> None: ...

    def add_rigid_link(
        self,
        master_node_id: int,
        slave_node_id: int,
        dof_handler: DOFHandler
    ) -> None: ...

    def reduce_system(
        self,
        K: NDArray[np.float64],
        F: NDArray[np.float64]
    ) -> ReducedSystem: ...


# ==============================================================================
# Model (Top-level)
# ==============================================================================

class Model:
    """Top-level orchestration class for complete structural analysis workflow."""
    boundary_conditions: BCHandler

    def __init__(self) -> None: ...

    # Node management
    def get_or_create_node(self, x: float, y: float, z: float) -> Node: ...
    def get_node(self, id: int) -> Optional[Node]: ...
    def find_node_at(self, x: float, y: float, z: float) -> Optional[Node]: ...

    # Material and section creation
    def create_material(
        self,
        name: str,
        E: float,
        nu: float,
        rho: float = 0.0
    ) -> Material: ...

    def create_section(
        self,
        name: str,
        A: float,
        Iy: float,
        Iz: float,
        J: float
    ) -> Section: ...

    # Element creation
    def create_beam(
        self,
        node_i: Node,
        node_j: Node,
        material: Material,
        section: Section,
        config: BeamConfig = ...
    ) -> BeamElement: ...

    # Load cases
    def get_default_load_case(self) -> LoadCase: ...
    def create_load_case(
        self,
        name: str,
        load_type: LoadCaseType = LoadCaseType.Permanent
    ) -> LoadCase: ...

    # Analysis
    def analyze(self) -> bool: ...

    # Results
    def get_displacements(self) -> NDArray[np.float64]: ...
    def get_reactions(self) -> NDArray[np.float64]: ...
    def get_dof_handler(self) -> DOFHandler: ...
    def get_element(self, id: int) -> Optional[BeamElement]: ...
