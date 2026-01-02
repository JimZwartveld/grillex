"""Core data structures and model classes for Grillex."""

from .data_types import (
    Node, NodeRegistry, Material, Section, LocalAxes,
    BeamElement, BeamFormulation, EndRelease,
    BeamConfig, BeamElementBase, create_beam_element,
    are_elements_collinear,
    DOFHandler, WarpingDOFInfo, WarpingCoupling,
    Assembler,
    DOFIndex, FixedDOF, BCHandler,
    SolverMethod, LinearSolver,
    LoadCaseType, NodalLoad, LineLoad, DistributedLoad,
    LoadCase, LoadCaseResult, LoadCombinationResult,
    LoadCombinationTerm, LoadCombination,
    Model,
    EqualityConstraint, RigidLink, ReducedSystem, ConstraintHandler,
    # Phase 7: Internal Actions
    EndForces, InternalActions, WarpingInternalActions, ActionExtreme,
    ReleaseCombo4DOF, ReleaseCombo2DOF, ReleaseComboWarping, DisplacementLine,
    # Phase 8: Additional Element Types
    SpringElement, PointMass, PlateElement, PlateElement8, PlateElement9, PlateElementTri, LoadingCondition,
    # Phase 15: Nonlinear Springs
    SpringBehavior, NonlinearSolverResult, NonlinearInitialState,
    NonlinearSolverSettings, NonlinearSolver,
    # Phase 11: Error Handling & Diagnostics
    ErrorCode, GrillexError, WarningCode, WarningSeverity,
    GrillexWarning, WarningList,
    # Phase 11 (Task 11.3): Singularity Diagnostics
    RigidBodyModeType, RigidBodyModeInfo, DOFParticipation,
    SingularityDiagnostics, SingularityAnalyzerSettings, SingularityAnalyzer,
    # Phase 16: Eigenvalue Analysis
    EigensolverMethod, EigensolverSettings, ModeResult,
    EigensolverResult, EigenvalueSolver,
)

from .model_wrapper import (
    ActionType,
    Beam,
    BeamLineLoad,
    StructuralModel,
    MeshStatistics
)

from .cargo import (
    Cargo,
    CargoConnection
)

from .plate import (
    Plate,
    EdgeMeshControl,
    PlateBeamCoupling,
    SupportCurve
)

from .element_types import (
    PlateElementType,
    ELEMENT_TYPE_INFO,
    get_element_type,
    get_element_info,
    get_available_element_types,
    create_plate_element,
    is_quad_element,
    is_triangle_element,
    supports_shear_deformation,
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
    'LoadCaseType', 'NodalLoad', 'LineLoad', 'DistributedLoad',
    'LoadCase', 'LoadCaseResult', 'LoadCombinationResult',
    'LoadCombinationTerm', 'LoadCombination',
    'Model',
    'EqualityConstraint', 'RigidLink', 'ReducedSystem', 'ConstraintHandler',
    # Phase 7: Internal Actions
    'EndForces', 'InternalActions', 'WarpingInternalActions', 'ActionExtreme',
    'ReleaseCombo4DOF', 'ReleaseCombo2DOF', 'ReleaseComboWarping', 'DisplacementLine',
    # Phase 8: Additional Element Types
    'SpringElement', 'PointMass', 'PlateElement', 'PlateElement8', 'PlateElement9', 'PlateElementTri', 'LoadingCondition',
    # Phase 15: Nonlinear Springs
    'SpringBehavior', 'NonlinearSolverResult', 'NonlinearInitialState',
    'NonlinearSolverSettings', 'NonlinearSolver',
    # Phase 9: Cargo Modelling
    'Cargo', 'CargoConnection',
    # Phase 11: Error Handling & Diagnostics
    'ErrorCode', 'GrillexError', 'WarningCode', 'WarningSeverity',
    'GrillexWarning', 'WarningList',
    # Phase 11 (Task 11.3): Singularity Diagnostics
    'RigidBodyModeType', 'RigidBodyModeInfo', 'DOFParticipation',
    'SingularityDiagnostics', 'SingularityAnalyzerSettings', 'SingularityAnalyzer',
    # Phase 16: Eigenvalue Analysis
    'EigensolverMethod', 'EigensolverSettings', 'ModeResult',
    'EigensolverResult', 'EigenvalueSolver',
    # Phase 19: Plate Meshing
    'Plate', 'EdgeMeshControl', 'PlateBeamCoupling', 'SupportCurve',
    'ActionType', 'Beam', 'BeamLineLoad', 'StructuralModel', 'MeshStatistics',
    # Phase 19 (Task 19.6): Element Type Infrastructure
    'PlateElementType', 'ELEMENT_TYPE_INFO',
    'get_element_type', 'get_element_info', 'get_available_element_types',
    'create_plate_element', 'is_quad_element', 'is_triangle_element',
    'supports_shear_deformation',
]