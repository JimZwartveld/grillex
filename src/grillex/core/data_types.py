"""
Core data types for Grillex structural analysis.

This module provides Python-friendly access to the C++ core data structures:
- Node: Point in 3D space with up to 7 DOFs (including warping)
- NodeRegistry: Manager for nodes with automatic merging
- Material: Material properties (E, G, nu, rho)
- Section: Cross-section properties for beams
- LocalAxes: Local coordinate system for beam elements
- BeamElement: 3D Euler-Bernoulli beam element
- BeamConfig: Configuration for beam element creation
- BeamElementBase: Abstract base class for beam elements
- create_beam_element: Factory function for creating beam elements
- are_elements_collinear: Helper for detecting collinear beams at nodes
- DOFHandler: Global DOF numbering with element-specific warping support
- WarpingDOFInfo: Information about warping DOF for element at node
- WarpingCoupling: Group of coupled warping DOFs for collinear elements
- Assembler: Assembles global stiffness and mass matrices from elements
- DOFIndex: Enum for local DOF indices (UX, UY, UZ, RX, RY, RZ, WARP)
- FixedDOF: Represents a fixed degree of freedom with prescribed value
- BCHandler: Boundary condition handler for managing fixed DOFs
- SolverMethod: Enum for linear solver methods (SparseLU, SimplicialLDLT, ConjugateGradient)
- LinearSolver: Linear solver for finite element systems (K * u = F)
- LoadCaseType: Enum for load case types (Permanent, Variable, Environmental, Accidental)
- NodalLoad: Concentrated force/moment at a node
- LineLoad: Distributed load along a beam element
- LoadCase: Load case containing all loads for a specific scenario
- LoadCaseResult: Results for a single load case analysis
- Model: Top-level orchestration class for complete structural analysis workflow
"""

from grillex._grillex_cpp import (
    Node,
    NodeRegistry,
    Material,
    Section,
    LocalAxes,
    BeamElement,
    BeamFormulation,
    EndRelease,
    BeamConfig,
    BeamElementBase,
    create_beam_element,
    are_elements_collinear,
    DOFHandler,
    WarpingDOFInfo,
    WarpingCoupling,
    Assembler,
    DOFIndex,
    FixedDOF,
    BCHandler,
    SolverMethod,
    LinearSolver,
    LoadCaseType,
    NodalLoad,
    LineLoad,
    LoadCase,
    LoadCaseResult,
    Model
)

__all__ = [
    'Node', 'NodeRegistry', 'Material', 'Section', 'LocalAxes',
    'BeamElement', 'BeamFormulation', 'EndRelease',
    'BeamConfig', 'BeamElementBase', 'create_beam_element',
    'are_elements_collinear',
    'DOFHandler', 'WarpingDOFInfo', 'WarpingCoupling',
    'Assembler',
    'DOFIndex', 'FixedDOF', 'BCHandler',
    'SolverMethod', 'LinearSolver',
    'LoadCaseType', 'NodalLoad', 'LineLoad', 'LoadCase', 'LoadCaseResult',
    'Model'
]
