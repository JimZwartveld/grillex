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
    LoadCase, LoadCaseResult,
    LoadCombinationTerm, LoadCombination,
    Model,
    EqualityConstraint, RigidLink, ReducedSystem, ConstraintHandler,
    # Phase 7: Internal Actions
    EndForces, InternalActions, WarpingInternalActions, ActionExtreme,
    ReleaseCombo4DOF, ReleaseCombo2DOF, ReleaseComboWarping, DisplacementLine,
    # Phase 8: Additional Element Types
    SpringElement, PointMass, PlateElement, LoadingCondition,
    # Phase 11: Error Handling & Diagnostics
    ErrorCode, GrillexError, WarningCode, WarningSeverity,
    GrillexWarning, WarningList,
)

from .model_wrapper import (
    Beam,
    StructuralModel
)

from .cargo import (
    Cargo,
    CargoConnection
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
    'LoadCase', 'LoadCaseResult',
    'LoadCombinationTerm', 'LoadCombination',
    'Model',
    'EqualityConstraint', 'RigidLink', 'ReducedSystem', 'ConstraintHandler',
    # Phase 7: Internal Actions
    'EndForces', 'InternalActions', 'WarpingInternalActions', 'ActionExtreme',
    'ReleaseCombo4DOF', 'ReleaseCombo2DOF', 'ReleaseComboWarping', 'DisplacementLine',
    # Phase 8: Additional Element Types
    'SpringElement', 'PointMass', 'PlateElement', 'LoadingCondition',
    # Phase 9: Cargo Modelling
    'Cargo', 'CargoConnection',
    # Phase 11: Error Handling & Diagnostics
    'ErrorCode', 'GrillexError', 'WarningCode', 'WarningSeverity',
    'GrillexWarning', 'WarningList',
    'Beam', 'StructuralModel'
]