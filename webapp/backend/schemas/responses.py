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
    """

    name: str


class SectionState(BaseModel):
    """State of a section in the model.

    Attributes:
        name: Section name.
    """

    name: str


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
        type: Load type ("point" or "line").
        position: Load position [x, y, z] in meters.
        dof: Degree of freedom (e.g., "UZ").
        value: Load magnitude in kN or kN/m.
    """

    type: str = "point"
    position: List[float]
    dof: str
    value: float


class BoundaryConditionState(BaseModel):
    """State of a boundary condition in the model.

    Attributes:
        position: BC position [x, y, z] in meters.
        type: BC type ("fixed", "pinned", "roller").
        dofs: List of fixed DOFs.
    """

    position: List[float]
    type: str
    dofs: List[str] = Field(default_factory=list)


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


class ModelStateResponse(BaseModel):
    """Complete model state response for frontend.

    Attributes:
        exists: Whether a model exists.
        name: Model name.
        nodes: List of nodes.
        beams: List of beams.
        materials: List of materials.
        sections: List of sections.
        loads: List of loads.
        boundaryConditions: List of boundary conditions.
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
    loads: List[LoadState] = Field(default_factory=list)
    boundaryConditions: List[BoundaryConditionState] = Field(default_factory=list)
    springs: List[SpringState] = Field(default_factory=list)
    isAnalyzed: bool = False
    results: Optional[ResultsState] = None
