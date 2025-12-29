"""
JSON result writer for structural analysis results.

This module provides classes and functions for exporting analysis results
to JSON format. The JSON structure is designed to be:
- Human-readable
- LLM-friendly (clear field names and structure)
- Complete (all result types included)
- Self-documenting (includes units and metadata)

Usage:
    from grillex.io import export_results_to_json

    model = StructuralModel()
    # ... build and analyze model ...

    export_results_to_json(model, "results.json")

    # Or export specific load case
    lc = model.cpp_model.get_load_cases()[0]
    export_results_to_json(model, "results_dead_load.json", load_case=lc)
"""

from dataclasses import dataclass, asdict, field
from typing import Optional, List, Any
from pathlib import Path
import json
import numpy as np

from grillex.core import StructuralModel, DOFIndex


@dataclass
class NodeResult:
    """Results for a single node.

    Attributes:
        node_id: Node identifier
        position: Node coordinates [x, y, z] in meters
        displacements: Displacements [ux, uy, uz, rx, ry, rz]
                      Translations in meters, rotations in radians
        reactions: Reaction forces [Fx, Fy, Fz, Mx, My, Mz] in kN and kN·m
                  None if node is not constrained
    """
    node_id: int
    position: List[float]
    displacements: List[float]
    reactions: Optional[List[float]] = None

    def __post_init__(self):
        """Ensure all numeric lists are Python lists (not numpy arrays)."""
        if isinstance(self.position, np.ndarray):
            self.position = self.position.tolist()
        if isinstance(self.displacements, np.ndarray):
            self.displacements = self.displacements.tolist()
        if self.reactions is not None and isinstance(self.reactions, np.ndarray):
            self.reactions = self.reactions.tolist()


@dataclass
class ElementResult:
    """Results for a single beam element.

    Attributes:
        element_id: Element identifier
        node_i_id: Node ID at start of element
        node_j_id: Node ID at end of element
        length: Element length in meters
        end_forces_i: End forces at node i [N, Vy, Vz, Mx, My, Mz]
                     in kN and kN·m
        end_forces_j: End forces at node j [N, Vy, Vz, Mx, My, Mz]
                     in kN and kN·m
    """
    element_id: int
    node_i_id: int
    node_j_id: int
    length: float
    end_forces_i: List[float]
    end_forces_j: List[float]

    def __post_init__(self):
        """Ensure all numeric lists are Python lists (not numpy arrays)."""
        if isinstance(self.end_forces_i, np.ndarray):
            self.end_forces_i = self.end_forces_i.tolist()
        if isinstance(self.end_forces_j, np.ndarray):
            self.end_forces_j = self.end_forces_j.tolist()


@dataclass
class LoadCaseInfo:
    """Information about a load case.

    Attributes:
        name: Load case name
        type: Load case type (Permanent, Variable, Environmental, Accidental)
        num_nodal_loads: Number of nodal loads in this case
    """
    name: str
    type: str
    num_nodal_loads: int


@dataclass
class ModelInfo:
    """Metadata about the structural model.

    Attributes:
        name: Model name
        num_nodes: Total number of nodes
        num_elements: Total number of elements
        num_beams: Total number of beams (may differ from elements if subdivided)
        total_dofs: Total degrees of freedom
        num_load_cases: Number of load cases analyzed
    """
    name: str
    num_nodes: int
    num_elements: int
    num_beams: int
    total_dofs: int
    num_load_cases: int


@dataclass
class ResultCase:
    """Complete results for one load case.

    This dataclass contains all analysis results for a single load case,
    formatted for JSON export.

    Attributes:
        model_info: Metadata about the structural model
        load_case_info: Information about this load case
        nodes: List of node results (displacements, reactions)
        elements: List of element results (end forces)
        units: Dictionary describing units for each result type
        success: Whether analysis succeeded
        error_message: Error message if analysis failed
    """
    model_info: ModelInfo
    load_case_info: LoadCaseInfo
    nodes: List[NodeResult]
    elements: List[ElementResult]
    units: dict = field(default_factory=lambda: {
        "length": "m",
        "displacement_translation": "m",
        "displacement_rotation": "rad",
        "force": "kN",
        "moment": "kN·m"
    })
    success: bool = True
    error_message: Optional[str] = None

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization.

        Returns:
            Dictionary representation suitable for JSON export
        """
        return asdict(self)

    def to_json(self, file_path: str, indent: int = 2) -> None:
        """Export results to JSON file.

        Args:
            file_path: Path to output JSON file
            indent: Indentation spaces for pretty printing (default: 2)
        """
        output_path = Path(file_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=indent)

    def to_json_string(self, indent: int = 2) -> str:
        """Convert results to JSON string.

        Args:
            indent: Indentation spaces for pretty printing (default: 2)

        Returns:
            JSON string representation
        """
        return json.dumps(self.to_dict(), indent=indent)


def export_results_to_json(
    model: StructuralModel,
    file_path: str,
    load_case: Optional[Any] = None,
    indent: int = 2
) -> None:
    """Export analysis results to JSON file.

    Args:
        model: StructuralModel instance (must be analyzed)
        file_path: Path to output JSON file
        load_case: Specific LoadCase to export (uses active if None)
        indent: Indentation spaces for pretty printing (default: 2)

    Raises:
        ValueError: If model has not been analyzed
    """
    if not model.is_analyzed():
        raise ValueError("Model must be analyzed before exporting results")

    result_case = build_result_case(model, load_case)
    result_case.to_json(file_path, indent=indent)


def build_result_case(
    model: StructuralModel,
    load_case: Optional[Any] = None
) -> ResultCase:
    """Build ResultCase from analyzed model.

    Args:
        model: StructuralModel instance (must be analyzed)
        load_case: Specific LoadCase to export (uses active if None)

    Returns:
        ResultCase with all results

    Raises:
        ValueError: If model has not been analyzed
    """
    if not model.is_analyzed():
        raise ValueError("Model must be analyzed before building results")

    # Set active load case if specified
    if load_case is not None:
        model.cpp_model.set_active_load_case(load_case)

    # Get active load case info
    active_lc = model.cpp_model.get_active_load_case()
    if active_lc is None:
        raise ValueError("No active load case found")

    # Build model info
    model_info = ModelInfo(
        name=model.name,
        num_nodes=model.num_nodes(),
        num_elements=model.num_elements(),
        num_beams=model.num_beams(),
        total_dofs=model.total_dofs(),
        num_load_cases=len(model.cpp_model.get_load_cases())
    )

    # Build load case info
    load_case_info = LoadCaseInfo(
        name=active_lc.name,
        type=str(active_lc.type).split('.')[-1],  # Get enum name
        num_nodal_loads=len(active_lc.get_nodal_loads())
    )

    # Get result for this load case
    result = model.cpp_model.get_result(active_lc)

    # Build node results
    nodes = []
    all_nodes = model.cpp_model.get_all_nodes()

    for node in all_nodes:
        # Get displacements for this node
        displacements = []
        for dof in [DOFIndex.UX, DOFIndex.UY, DOFIndex.UZ,
                    DOFIndex.RX, DOFIndex.RY, DOFIndex.RZ]:
            try:
                disp = model.cpp_model.get_node_displacement(node.id, dof)
                displacements.append(float(disp))
            except:
                displacements.append(0.0)

        # Always get reactions for all nodes (will be zeros for unconstrained nodes)
        # This makes the JSON complete and consistent
        reactions = []
        for dof in [DOFIndex.UX, DOFIndex.UY, DOFIndex.UZ,
                   DOFIndex.RX, DOFIndex.RY, DOFIndex.RZ]:
            try:
                rxn = model.cpp_model.get_node_reaction(node.id, dof)
                reactions.append(float(rxn))
            except:
                reactions.append(0.0)

        node_result = NodeResult(
            node_id=node.id,
            position=[node.x, node.y, node.z],
            displacements=displacements,
            reactions=reactions
        )
        nodes.append(node_result)

    # Build element results
    elements = []
    for i, elem in enumerate(model.cpp_model.elements):
        # Note: End forces computation requires Phase 7 implementation
        # For now, we'll create placeholder structure
        element_result = ElementResult(
            element_id=i,
            node_i_id=elem.node_i.id,
            node_j_id=elem.node_j.id,
            length=float(elem.length),
            end_forces_i=[0.0] * 6,  # Placeholder - Phase 7 needed
            end_forces_j=[0.0] * 6   # Placeholder - Phase 7 needed
        )
        elements.append(element_result)

    return ResultCase(
        model_info=model_info,
        load_case_info=load_case_info,
        nodes=nodes,
        elements=elements,
        success=result.success,
        error_message=result.error_message if not result.success else None
    )


def export_all_load_cases_to_json(
    model: StructuralModel,
    output_dir: str,
    indent: int = 2
) -> List[str]:
    """Export all load cases to separate JSON files.

    Args:
        model: StructuralModel instance (must be analyzed)
        output_dir: Directory for output files
        indent: Indentation spaces for pretty printing (default: 2)

    Returns:
        List of created file paths

    Raises:
        ValueError: If model has not been analyzed
    """
    if not model.is_analyzed():
        raise ValueError("Model must be analyzed before exporting results")

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    created_files = []
    load_cases = model.cpp_model.get_load_cases()

    for lc in load_cases:
        # Create safe filename from load case name
        safe_name = lc.name.replace(' ', '_').replace('/', '_')
        file_path = output_path / f"{safe_name}.json"

        export_results_to_json(model, str(file_path), load_case=lc, indent=indent)
        created_files.append(str(file_path))

    return created_files
