"""
Pythonic wrapper around C++ Model class.

This module provides a higher-level Python API for structural analysis,
wrapping the C++ Model class with convenience methods and preparing for
advanced result extraction (Phase 7).

The main classes are:
- Beam: Represents a structural beam (can span multiple elements)
- StructuralModel: Pythonic wrapper around C++ Model

Usage:
    from grillex.core import StructuralModel, DOFIndex, LoadCaseType

    # Create model
    model = StructuralModel(name="My Bridge")

    # Add beam by coordinates
    beam = model.add_beam_by_coords(
        [0, 0, 0], [6, 0, 0],
        section_name="IPE300",
        material_name="Steel"
    )

    # Apply boundary conditions
    model.fix_node_at([0, 0, 0])

    # Add loads
    model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)

    # Analyze
    results = model.analyze()

    # Get results
    disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
"""

from typing import List, Tuple, Optional, Union
import numpy as np

from grillex._grillex_cpp import (
    Model as _CppModel,
    Material as _CppMaterial,
    Section as _CppSection,
    BeamElement as _CppBeamElement,
    LoadCase as _CppLoadCase,
    LoadCaseType,
    DOFIndex,
    Node
)


class Beam:
    """
    Represents a structural beam in the model.

    A Beam is a Python-level abstraction that can consist of one or more
    BeamElement objects (C++ FE elements). This allows for:
    - Multi-element beams created by automatic subdivision
    - Continuous result queries along the entire beam length
    - Convenient plotting of internal actions (when Phase 7 is implemented)

    Attributes:
        start_pos: Starting position [x, y, z] in global coordinates
        end_pos: Ending position [x, y, z] in global coordinates
        section: Section object
        material: Material object
        elements: List of underlying C++ BeamElement objects
        length: Total length of the beam
    """

    def __init__(
        self,
        start_pos: List[float],
        end_pos: List[float],
        section: _CppSection,
        material: _CppMaterial,
        beam_id: Optional[int] = None
    ):
        """Initialize a Beam.

        Args:
            start_pos: Starting coordinates [x, y, z]
            end_pos: Ending coordinates [x, y, z]
            section: Cross-section properties
            material: Material properties
            beam_id: Optional unique identifier
        """
        self.beam_id = beam_id
        self.start_pos = np.array(start_pos)
        self.end_pos = np.array(end_pos)
        self.section = section
        self.material = material
        self.elements: List[_CppBeamElement] = []

        # Compute length
        self.length = float(np.linalg.norm(self.end_pos - self.start_pos))

    def add_element(self, element: _CppBeamElement) -> None:
        """Add a BeamElement to this beam."""
        self.elements.append(element)

    def get_midpoint(self) -> np.ndarray:
        """Get the midpoint coordinates of the beam."""
        return (self.start_pos + self.end_pos) / 2.0

    def get_direction(self) -> np.ndarray:
        """Get the unit direction vector of the beam."""
        direction = self.end_pos - self.start_pos
        return direction / np.linalg.norm(direction)

    def __repr__(self) -> str:
        return (f"Beam(id={self.beam_id}, start={self.start_pos.tolist()}, "
                f"end={self.end_pos.tolist()}, length={self.length:.3f}m, "
                f"n_elements={len(self.elements)})")

    # TODO: Phase 7 - Internal Actions
    # def get_internal_actions_at(self, x: float, model: 'StructuralModel'):
    #     """Query internal actions at position x along beam (Phase 7)"""
    #     pass
    #
    # def get_moment_diagram(self, axis: str, model: 'StructuralModel', num_points: int = 100):
    #     """Get moment diagram data for plotting (Phase 7)"""
    #     pass
    #
    # def plot_results(self, model: 'StructuralModel', components: List[str] = ['Mz', 'Vy', 'N']):
    #     """Plot internal action diagrams (Phase 7)"""
    #     pass


class StructuralModel:
    """
    High-level Python interface for structural analysis.

    This class wraps the C++ Model class to provide a more Pythonic API with:
    - Coordinate-based beam creation
    - Simplified load and BC application
    - Material/section library management
    - Convenient result querying

    Attributes:
        name: Model name for identification
        beams: List of Beam objects
        _cpp_model: Underlying C++ Model instance
        _materials: Dictionary of materials by name
        _sections: Dictionary of sections by name
        _node_map: Dictionary mapping (x,y,z) tuples to Node objects
    """

    def __init__(
        self,
        name: str = "Unnamed Model",
        node_tolerance: float = 1e-6
    ):
        """Initialize a StructuralModel.

        Args:
            name: Model name for identification
            node_tolerance: Tolerance for node merging (default: 1mm)
        """
        self.name = name
        self._cpp_model = _CppModel(node_tolerance=node_tolerance)
        self.beams: List[Beam] = []
        self._materials: dict[str, _CppMaterial] = {}
        self._sections: dict[str, _CppSection] = {}
        self._node_map: dict[Tuple[float, float, float], Node] = {}
        self._beam_id_counter = 1

    # ===== Material and Section Management =====

    def add_material(
        self,
        name: str,
        E: float,
        nu: float,
        rho: float
    ) -> _CppMaterial:
        """Add a material to the model library.

        Args:
            name: Material name
            E: Young's modulus [kN/m²]
            nu: Poisson's ratio [-]
            rho: Density [mT/m³ = kN/m³/g]

        Returns:
            Material object
        """
        if name in self._materials:
            return self._materials[name]

        mat = self._cpp_model.create_material(name, E, nu, rho)
        self._materials[name] = mat
        return mat

    def add_section(
        self,
        name: str,
        A: float,
        Iy: float,
        Iz: float,
        J: float,
        **kwargs
    ) -> _CppSection:
        """Add a section to the model library.

        Args:
            name: Section name
            A: Cross-sectional area [m²]
            Iy: Second moment of area about local y-axis [m⁴]
            Iz: Second moment of area about local z-axis [m⁴]
            J: Torsional constant [m⁴]
            **kwargs: Additional section properties

        Returns:
            Section object
        """
        if name in self._sections:
            return self._sections[name]

        sec = self._cpp_model.create_section(name, A, Iy, Iz, J)
        self._sections[name] = sec
        return sec

    def get_material(self, name: str) -> Optional[_CppMaterial]:
        """Get a material by name."""
        return self._materials.get(name)

    def get_section(self, name: str) -> Optional[_CppSection]:
        """Get a section by name."""
        return self._sections.get(name)

    # ===== Node Management =====

    def get_or_create_node(self, x: float, y: float, z: float) -> Node:
        """Get or create a node at the specified coordinates.

        Args:
            x, y, z: Coordinates in meters

        Returns:
            Node object (existing or newly created)
        """
        # Round to avoid floating point issues
        key = (round(x, 9), round(y, 9), round(z, 9))

        if key not in self._node_map:
            node = self._cpp_model.get_or_create_node(x, y, z)
            self._node_map[key] = node

        return self._node_map[key]

    def find_node_at(self, position: List[float], tolerance: Optional[float] = None) -> Optional[Node]:
        """Find a node at the specified position.

        Args:
            position: [x, y, z] coordinates
            tolerance: Search tolerance (uses model tolerance if None)

        Returns:
            Node if found, None otherwise
        """
        x, y, z = position
        key = (round(x, 9), round(y, 9), round(z, 9))
        return self._node_map.get(key)

    # ===== Beam Creation =====

    def add_beam_by_coords(
        self,
        start_pos: List[float],
        end_pos: List[float],
        section_name: str,
        material_name: str,
        **kwargs
    ) -> Beam:
        """Add a beam using endpoint coordinates.

        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            section_name: Name of section from library
            material_name: Name of material from library
            **kwargs: Additional beam properties

        Returns:
            Beam object

        Raises:
            ValueError: If material or section not found in library
        """
        # Get material and section
        material = self.get_material(material_name)
        if material is None:
            raise ValueError(f"Material '{material_name}' not found. Add it first with add_material().")

        section = self.get_section(section_name)
        if section is None:
            raise ValueError(f"Section '{section_name}' not found. Add it first with add_section().")

        # Create nodes
        node1 = self.get_or_create_node(*start_pos)
        node2 = self.get_or_create_node(*end_pos)

        # Create beam element in C++ model
        cpp_element = self._cpp_model.create_beam(node1, node2, material, section)

        # Create Python Beam object
        beam = Beam(start_pos, end_pos, section, material, beam_id=self._beam_id_counter)
        self._beam_id_counter += 1
        beam.add_element(cpp_element)
        self.beams.append(beam)

        return beam

    # ===== Boundary Conditions =====

    def fix_node_at(self, position: List[float], include_warping: bool = False) -> None:
        """Fix all DOFs at a node at the specified position.

        Args:
            position: [x, y, z] coordinates
            include_warping: Whether to also fix warping DOF (7th DOF)
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        if include_warping:
            self._cpp_model.boundary_conditions.fix_node_with_warping(node.id)
        else:
            self._cpp_model.boundary_conditions.fix_node(node.id)

    def pin_node_at(self, position: List[float]) -> None:
        """Pin a node (fix translations only) at the specified position.

        Args:
            position: [x, y, z] coordinates
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        self._cpp_model.boundary_conditions.pin_node(node.id)

    def fix_dof_at(self, position: List[float], dof: DOFIndex, value: float = 0.0) -> None:
        """Fix a specific DOF at a node.

        Args:
            position: [x, y, z] coordinates
            dof: DOF index (DOFIndex.UX, UY, UZ, RX, RY, RZ, WARP)
            value: Prescribed value (default: 0.0)
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        self._cpp_model.boundary_conditions.add_fixed_dof(node.id, dof, value)

    # ===== Load Application =====

    def create_load_case(
        self,
        name: str,
        load_type: LoadCaseType = LoadCaseType.Variable
    ) -> _CppLoadCase:
        """Create a new load case.

        Args:
            name: Load case name
            load_type: Type of load case (Permanent, Variable, Environmental, Accidental)

        Returns:
            LoadCase object
        """
        return self._cpp_model.create_load_case(name, load_type)

    def get_default_load_case(self) -> _CppLoadCase:
        """Get or create the default load case (for simple models)."""
        return self._cpp_model.get_default_load_case()

    def add_point_load(
        self,
        position: List[float],
        dof: DOFIndex,
        value: float,
        load_case: Optional[_CppLoadCase] = None
    ) -> None:
        """Add a point load at a node.

        Args:
            position: [x, y, z] coordinates
            dof: DOF index for load direction
            value: Load magnitude [kN] or [kN·m]
            load_case: LoadCase to add to (uses default if None)
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        if load_case is None:
            load_case = self.get_default_load_case()

        load_case.add_nodal_load(node.id, dof, value)

    # ===== Analysis =====

    def analyze(self) -> bool:
        """Run linear static analysis for all load cases.

        Returns:
            True if analysis succeeded for all load cases, False otherwise
        """
        return self._cpp_model.analyze()

    def is_analyzed(self) -> bool:
        """Check if the model has been analyzed."""
        return self._cpp_model.is_analyzed()

    # ===== Results Access =====

    def get_displacement_at(
        self,
        position: List[float],
        dof: DOFIndex,
        load_case: Optional[_CppLoadCase] = None
    ) -> float:
        """Get displacement at a node for a specific DOF.

        Args:
            position: [x, y, z] coordinates
            dof: DOF index
            load_case: LoadCase to query (uses active if None)

        Returns:
            Displacement value [m] or rotation [rad]
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        return self._cpp_model.get_node_displacement(node.id, dof)

    def get_all_displacements(self, load_case: Optional[_CppLoadCase] = None) -> np.ndarray:
        """Get global displacement vector.

        Args:
            load_case: LoadCase to query (uses active if None)

        Returns:
            Displacement vector as numpy array
        """
        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        return self._cpp_model.get_displacements()

    def get_reactions(self, load_case: Optional[_CppLoadCase] = None) -> np.ndarray:
        """Get reaction forces vector.

        Args:
            load_case: LoadCase to query (uses active if None)

        Returns:
            Reaction vector as numpy array
        """
        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        return self._cpp_model.get_reactions()

    # ===== Model Information =====

    def total_dofs(self) -> int:
        """Get total number of DOFs in the model."""
        return self._cpp_model.total_dofs()

    def num_nodes(self) -> int:
        """Get number of nodes."""
        return len(self._cpp_model.get_all_nodes())

    def num_elements(self) -> int:
        """Get number of elements."""
        return len(self._cpp_model.elements)

    def num_beams(self) -> int:
        """Get number of beams."""
        return len(self.beams)

    def __repr__(self) -> str:
        return (f"StructuralModel(name='{self.name}', "
                f"nodes={self.num_nodes()}, "
                f"beams={self.num_beams()}, "
                f"elements={self.num_elements()}, "
                f"dofs={self.total_dofs()}, "
                f"analyzed={self.is_analyzed()})")

    # ===== Access to underlying C++ model =====

    @property
    def cpp_model(self) -> _CppModel:
        """Get the underlying C++ Model object."""
        return self._cpp_model

    @property
    def boundary_conditions(self):
        """Direct access to boundary conditions handler."""
        return self._cpp_model.boundary_conditions
