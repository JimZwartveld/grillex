"""Pydantic response models for API endpoints."""

from typing import Any, Dict, List, Optional
from pydantic import BaseModel, Field


class ToolResponse(BaseModel):
    """Response from a tool execution.

    Attributes:
        success: Whether the tool executed successfully.
        result: The result data (varies by tool).
        error: Error message if success is False.
        suggestion: Suggested fix if there was an error.
    """

    success: bool
    result: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    suggestion: Optional[str] = None


class NodeState(BaseModel):
    """State of a node in the model.

    Attributes:
        id: Node ID.
        position: [x, y, z] coordinates in meters.
    """

    id: int
    position: List[float]


class MaterialState(BaseModel):
    """State of a material in the model.

    Attributes:
        name: Material name.
        E: Young's modulus in kN/m².
        nu: Poisson's ratio.
        rho: Density in mT/m³.
        G: Shear modulus in kN/m² (computed).
    """

    name: str
    E: Optional[float] = None
    nu: Optional[float] = None
    rho: Optional[float] = None
    G: Optional[float] = None


class SectionState(BaseModel):
    """State of a section in the model.

    Attributes:
        name: Section name.
        A: Cross-sectional area in m².
        Iy: Moment of inertia about y in m⁴.
        Iz: Moment of inertia about z in m⁴.
        J: Torsional constant in m⁴.
        Iw: Warping constant in m⁶.
        ky: Shear area factor y.
        kz: Shear area factor z.
    """

    name: str
    A: Optional[float] = None
    Iy: Optional[float] = None
    Iz: Optional[float] = None
    J: Optional[float] = None
    Iw: Optional[float] = None
    ky: Optional[float] = None
    kz: Optional[float] = None


class BeamState(BaseModel):
    """State of a beam element in the model.

    Attributes:
        id: Beam ID.
        start: Start position [x, y, z] in meters.
        end: End position [x, y, z] in meters.
        section: Section name.
        material: Material name.
        length: Beam length in meters.
    """

    id: int
    start: List[float]
    end: List[float]
    section: str
    material: str
    length: float


class LoadState(BaseModel):
    """State of a load in the model.

    Attributes:
        node_id: Node ID where load is applied.
        dof: Degree of freedom (e.g., "UZ").
        value: Load magnitude in kN or kN/m.
    """

    node_id: int
    dof: str
    value: float


class BoundaryConditionState(BaseModel):
    """State of a boundary condition in the model.

    Attributes:
        node_id: Node ID where BC is applied.
        dof: Degree of freedom (e.g., "UZ").
        value: Prescribed value.
    """

    node_id: int
    dof: str
    value: float


class SpringState(BaseModel):
    """State of a spring element in the model.

    Attributes:
        id: Spring ID.
        node1: First node ID.
        node2: Second node ID.
        isNonlinear: Whether the spring is nonlinear.
        hasGap: Whether the spring has a gap.
    """

    id: int
    node1: int
    node2: int
    isNonlinear: bool = False
    hasGap: bool = False


class DisplacementResult(BaseModel):
    """Displacement result at a node.

    Attributes:
        nodeId: Node ID.
        position: Node position [x, y, z].
        displacements: Dictionary of DOF -> displacement value.
    """

    nodeId: int
    position: List[float]
    displacements: Dict[str, float]


class ReactionResult(BaseModel):
    """Reaction result at a supported node.

    Attributes:
        position: Node position [x, y, z].
        reactions: Dictionary of DOF -> reaction value.
    """

    position: List[float]
    reactions: Dict[str, float]


class ResultsState(BaseModel):
    """Analysis results state.

    Attributes:
        displacements: List of displacement results.
        reactions: List of reaction results.
    """

    displacements: List[DisplacementResult] = Field(default_factory=list)
    reactions: List[ReactionResult] = Field(default_factory=list)


class LoadCaseState(BaseModel):
    """State of a load case in the model.

    Attributes:
        id: Load case ID.
        name: Load case name.
        type: Load case type (permanent, variable, accidental, seismic).
        loads: List of loads in this case.
    """

    id: int
    name: str
    type: str = "permanent"
    loads: List[LoadState] = Field(default_factory=list)


class CargoConnectionState(BaseModel):
    """Connection from cargo to structure.

    Attributes:
        node_id: Node ID on structure.
        cargoOffset: Offset from cargo COG [x, y, z] in meters.
    """

    node_id: int
    cargoOffset: List[float]


class CargoState(BaseModel):
    """State of a cargo item in the model.

    Attributes:
        id: Cargo ID.
        name: Cargo name.
        cogPosition: Center of gravity [x, y, z] in meters.
        dimensions: Bounding box [length, width, height] in meters.
        mass: Mass in mT.
        connections: Connection points to structure.
    """

    id: int
    name: str
    cogPosition: List[float]
    dimensions: List[float]
    mass: float
    connections: List[CargoConnectionState] = Field(default_factory=list)


class ModelStateResponse(BaseModel):
    """Complete model state response for frontend.

    Attributes:
        exists: Whether a model exists.
        name: Model name.
        nodes: List of nodes.
        beams: List of beams.
        materials: List of materials.
        sections: List of sections.
        boundaryConditions: List of boundary conditions.
        loadCases: List of load cases.
        cargos: List of cargo items.
        springs: List of springs.
        isAnalyzed: Whether the model has been analyzed.
        results: Analysis results (if analyzed).
    """

    exists: bool = False
    name: Optional[str] = None
    nodes: List[NodeState] = Field(default_factory=list)
    beams: List[BeamState] = Field(default_factory=list)
    materials: List[MaterialState] = Field(default_factory=list)
    sections: List[SectionState] = Field(default_factory=list)
    boundaryConditions: List[BoundaryConditionState] = Field(default_factory=list)
    loadCases: List[LoadCaseState] = Field(default_factory=list)
    cargos: List[CargoState] = Field(default_factory=list)
    springs: List[SpringState] = Field(default_factory=list)
    isAnalyzed: bool = False
    results: Optional[ResultsState] = None
