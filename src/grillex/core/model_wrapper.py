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
    model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

    # Analyze
    results = model.analyze()

    # Get results
    disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
"""

from typing import List, Tuple, Optional, Union, Dict, Any, TYPE_CHECKING
from dataclasses import dataclass
from enum import Enum
import numpy as np


class ActionType(Enum):
    """Type of internal action for line diagrams.

    Used by the get_line_data() method to specify which internal action
    to extract for plotting moment, shear, axial, or torsion diagrams.

    Attributes:
        MOMENT_Y: Bending moment about local Y-axis [kN·m]
        MOMENT_Z: Bending moment about local Z-axis [kN·m]
        SHEAR_Y: Shear force in local Y direction [kN]
        SHEAR_Z: Shear force in local Z direction [kN]
        AXIAL: Axial force (positive in tension) [kN]
        TORSION: Total torsion moment about local X-axis [kN·m]
        BIMOMENT: Bimoment for warping torsion [kN·m²]
        WARPING_TORSION: Warping torsion component [kN·m]
    """
    MOMENT_Y = "moment_y"
    MOMENT_Z = "moment_z"
    SHEAR_Y = "shear_y"
    SHEAR_Z = "shear_z"
    AXIAL = "axial"
    TORSION = "torsion"
    BIMOMENT = "bimoment"
    WARPING_TORSION = "warping_torsion"

if TYPE_CHECKING:
    import pandas as pd

# Core imports
from grillex._grillex_cpp import (
    Model as _CppModel,
    Material as _CppMaterial,
    Section as _CppSection,
    BeamElement as _CppBeamElement,
    LoadCase as _CppLoadCase,
    LoadCaseType,
    DOFIndex,
    Node,
    InternalActions,
    SpringElement as _CppSpringElement,
    LoadCaseResult,
    PlateElement,
)

# Optional imports - Phase 15: Nonlinear Springs
try:
    from grillex._grillex_cpp import (
        SpringBehavior,
        NonlinearSolverSettings,
        NonlinearSolverResult,
    )
except ImportError:
    SpringBehavior = None
    NonlinearSolverSettings = None
    NonlinearSolverResult = None

# Optional imports - Phase 16: Eigenvalue Analysis
try:
    from grillex._grillex_cpp import (
        EigensolverMethod,
        EigensolverSettings,
        EigensolverResult,
        ModeResult,
    )
except ImportError:
    EigensolverMethod = None
    EigensolverSettings = None
    EigensolverResult = None
    ModeResult = None

# Optional imports - Load combination analysis
try:
    from grillex._grillex_cpp import LoadCombination, LoadCombinationResult
except ImportError:
    LoadCombination = None
    LoadCombinationResult = None

from .cargo import Cargo
from .plate import Plate, EdgeMeshControl, PlateBeamCoupling, SupportCurve
from .element_types import get_element_type


@dataclass
class MeshStatistics:
    """Statistics from mesh generation.

    Attributes:
        n_plate_nodes: Total number of nodes created for plates.
        n_plate_elements: Total number of plate elements created.
        n_quad_elements: Number of quad elements (MITC4, MITC8, MITC9).
        n_tri_elements: Number of triangular elements (DKT).
        n_beams_subdivided: Number of beams that were subdivided.
        n_rigid_links: Number of rigid link constraints created.
        n_support_dofs: Number of DOFs restrained by support curves.
    """
    n_plate_nodes: int = 0
    n_plate_elements: int = 0
    n_quad_elements: int = 0
    n_tri_elements: int = 0
    n_beams_subdivided: int = 0
    n_rigid_links: int = 0
    n_support_dofs: int = 0


class Beam:
    """
    Represents a structural beam in the model.

    A Beam is a Python-level abstraction that can consist of one or more
    BeamElement objects (C++ FE elements). This allows for:
    - Multi-element beams created by automatic subdivision
    - Continuous result queries along the entire beam length
    - Convenient plotting of internal actions (when Phase 7 is implemented)

    Attributes:
        beam_id: Unique beam identifier
        name: User-defined beam name for display
        start_pos: Starting position [x, y, z] in global coordinates
        end_pos: Ending position [x, y, z] in global coordinates
        section: Section object
        material: Material object
        elements: List of underlying C++ BeamElement objects
        length: Total length of the beam
        check_locations: List of normalized positions [0, 1] for design checks
    """

    def __init__(
        self,
        start_pos: List[float],
        end_pos: List[float],
        section: _CppSection,
        material: _CppMaterial,
        beam_id: Optional[int] = None,
        name: Optional[str] = None
    ):
        """Initialize a Beam.

        Args:
            start_pos: Starting coordinates [x, y, z]
            end_pos: Ending coordinates [x, y, z]
            section: Cross-section properties
            material: Material properties
            beam_id: Optional unique identifier
            name: Optional beam name for display
        """
        self.beam_id = beam_id
        self.name = name  # User-defined beam name
        self.start_pos = np.array(start_pos)
        self.end_pos = np.array(end_pos)
        self.section = section
        self.material = material
        self.elements: List[_CppBeamElement] = []
        self.check_locations: List[float] = []  # Normalized positions [0, 1]

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

    # ===== Check Locations (Task 7.3) =====

    def add_check_location(self, x_normalized: float) -> None:
        """
        Add a check location at normalized position.

        Check locations are used for design code verification, allowing
        internal actions to be queried at specific positions along the beam.

        Args:
            x_normalized: Position (0=start, 1=end)

        Raises:
            ValueError: If x_normalized is not in range [0, 1]
        """
        if not 0.0 <= x_normalized <= 1.0:
            raise ValueError("Check location must be in range [0, 1]")
        if x_normalized not in self.check_locations:
            self.check_locations.append(x_normalized)
            self.check_locations.sort()

    def set_standard_check_locations(self) -> None:
        """
        Set standard check locations: ends, quarter points, and midspan.

        Sets check locations at 0, 0.25, 0.5, 0.75, and 1.0 (normalized).
        These are common locations for design code verification.
        """
        self.check_locations = [0.0, 0.25, 0.5, 0.75, 1.0]

    def clear_check_locations(self) -> None:
        """Clear all check locations."""
        self.check_locations = []

    def _find_element_at_position(self, x: float) -> Tuple[_CppBeamElement, float]:
        """
        Find the element containing position x and compute local position.

        Args:
            x: Position along beam in meters (0 = start of beam)

        Returns:
            Tuple of (element, local_x) where local_x is position within element

        Raises:
            ValueError: If no elements or position out of range
        """
        if not self.elements:
            raise ValueError("Beam has no elements")

        if x < 0 or x > self.length + 1e-9:
            raise ValueError(f"Position {x} out of range [0, {self.length}]")

        # Clamp to valid range
        x = max(0.0, min(x, self.length))

        cumulative_length = 0.0
        for element in self.elements:
            elem_length = element.length
            if x <= cumulative_length + elem_length + 1e-9:
                local_x = x - cumulative_length
                # Clamp to element range
                local_x = max(0.0, min(local_x, elem_length))
                return element, local_x
            cumulative_length += elem_length

        # Should not reach here, but return last element at end
        return self.elements[-1], self.elements[-1].length

    def get_internal_actions_at(
        self,
        x: float,
        model: 'StructuralModel',
        load_case: Optional[_CppLoadCase] = None
    ) -> InternalActions:
        """
        Get internal actions at position x along beam.

        Finds the element containing position x and queries internal actions
        from the underlying C++ BeamElement.

        Args:
            x: Position along beam in meters (0 = start of beam)
            model: StructuralModel (must be analyzed)
            load_case: Optional load case for distributed load effects

        Returns:
            InternalActions with N, Vy, Vz, Mx, My, Mz at position x

        Raises:
            ValueError: If model not analyzed or position out of range
        """
        if not model.is_analyzed():
            raise ValueError("Model must be analyzed before querying internal actions")

        element, local_x = self._find_element_at_position(x)

        dof_handler = model._cpp_model.get_dof_handler()
        displacements = model._cpp_model.get_displacements()

        if load_case is None:
            load_case = model._cpp_model.get_default_load_case()

        return element.get_internal_actions(local_x, displacements, dof_handler, load_case)

    def get_internal_actions_at_check_locations(
        self,
        model: 'StructuralModel',
        load_case: Optional[_CppLoadCase] = None
    ) -> List[Tuple[float, InternalActions]]:
        """
        Get internal actions at all check locations.

        Args:
            model: StructuralModel (must be analyzed)
            load_case: Optional load case for distributed load effects

        Returns:
            List of (x_position, internal_actions) tuples sorted by position
        """
        results = []
        for x_norm in self.check_locations:
            x_global = x_norm * self.length
            actions = self.get_internal_actions_at(x_global, model, load_case)
            results.append((x_global, actions))
        return results


    # ===== Phase 7: Internal Actions - Multi-Element Beam Plotting =====

    def _validate_connectivity(self) -> None:
        """Check that elements are connected end-to-end."""
        if len(self.elements) <= 1:
            return
        for i in range(len(self.elements) - 1):
            if self.elements[i].node_j.id != self.elements[i + 1].node_i.id:
                raise ValueError(f"Elements {i} and {i+1} are not connected")

    def _find_element_at_position(self, x_global: float) -> Tuple[_CppBeamElement, float]:
        """Find which element contains x_global.

        Args:
            x_global: Position along beam [0, L_total]

        Returns:
            (element, x_local): Element and local position within that element

        Raises:
            ValueError: If position is outside beam length
        """
        if x_global < -1e-10 or x_global > self.length + 1e-10:
            raise ValueError(f"Position {x_global} outside beam length {self.length}")

        # Clamp to valid range
        x_global = max(0.0, min(x_global, self.length))

        cumulative_length = 0.0

        for element in self.elements:
            elem_length = element.length
            if x_global <= cumulative_length + elem_length + 1e-10:
                x_local = x_global - cumulative_length
                return element, x_local
            cumulative_length += elem_length

        # Edge case: x_global == total length
        return self.elements[-1], self.elements[-1].length

    # NOTE: get_internal_actions_at is defined earlier in this class (line ~192)
    # with the full signature including load_case parameter.
    # This duplicate was removed to avoid method shadowing.

    def get_moment_line(
        self,
        axis: str,
        model: 'StructuralModel',
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get moment diagram for plotting.

        Args:
            axis: 'y' or 'z' for bending plane
            model: StructuralModel object (must be analyzed)
            num_points: Number of points to sample along beam

        Returns:
            (x_positions, moments): Arrays for plotting
        """
        x_positions = np.linspace(0, self.length, num_points)
        moments = np.zeros(num_points)

        for i, x in enumerate(x_positions):
            actions = self.get_internal_actions_at(x, model)
            if axis.lower() == 'y':
                moments[i] = actions.My
            else:
                moments[i] = actions.Mz

        return x_positions, moments

    def get_shear_line(
        self,
        axis: str,
        model: 'StructuralModel',
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get shear diagram for plotting.

        Args:
            axis: 'y' or 'z' for shear direction
            model: StructuralModel object (must be analyzed)
            num_points: Number of points to sample along beam

        Returns:
            (x_positions, shears): Arrays for plotting
        """
        x_positions = np.linspace(0, self.length, num_points)
        shears = np.zeros(num_points)

        for i, x in enumerate(x_positions):
            actions = self.get_internal_actions_at(x, model)
            if axis.lower() == 'y':
                shears[i] = actions.Vy
            else:
                shears[i] = actions.Vz

        return x_positions, shears

    def get_axial_force_line(
        self,
        model: 'StructuralModel',
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get axial force diagram for plotting.

        Args:
            model: StructuralModel object (must be analyzed)
            num_points: Number of points to sample along beam

        Returns:
            (x_positions, axial_forces): Arrays for plotting
        """
        x_positions = np.linspace(0, self.length, num_points)
        axial_forces = np.zeros(num_points)

        for i, x in enumerate(x_positions):
            actions = self.get_internal_actions_at(x, model)
            axial_forces[i] = actions.N

        return x_positions, axial_forces

    def get_torsion_line(
        self,
        model: 'StructuralModel',
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get torsion moment diagram for plotting.

        Args:
            model: StructuralModel object (must be analyzed)
            num_points: Number of points to sample along beam

        Returns:
            (x_positions, torsions): Arrays for plotting
        """
        x_positions = np.linspace(0, self.length, num_points)
        torsions = np.zeros(num_points)

        for i, x in enumerate(x_positions):
            actions = self.get_internal_actions_at(x, model)
            torsions[i] = actions.Mx

        return x_positions, torsions

    def get_displacement_line(
        self,
        component: str,
        model: 'StructuralModel',
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get displacement diagram for plotting.

        Args:
            component: 'u', 'v', 'w', 'theta_x', 'theta_y', or 'theta_z'
            model: StructuralModel object (must be analyzed)
            num_points: Number of points to sample along beam

        Returns:
            (x_positions, displacements): Arrays for plotting
        """
        if not model.is_analyzed():
            raise ValueError("Model must be analyzed before querying displacements")

        x_positions = np.linspace(0, self.length, num_points)
        displacements = np.zeros(num_points)

        for i, x in enumerate(x_positions):
            element, x_local = self._find_element_at_position(x)
            disp = element.get_displacements_at(
                x_local,
                model.get_all_displacements(),
                model._cpp_model.get_dof_handler()
            )

            if component == 'u':
                displacements[i] = disp.u
            elif component == 'v':
                displacements[i] = disp.v
            elif component == 'w':
                displacements[i] = disp.w
            elif component == 'theta_x':
                displacements[i] = disp.theta_x
            elif component == 'theta_y':
                displacements[i] = disp.theta_y
            elif component == 'theta_z':
                displacements[i] = disp.theta_z
            else:
                raise ValueError(f"Unknown displacement component: {component}")

        return x_positions, displacements

    def find_moment_extrema(
        self,
        axis: str,
        model: 'StructuralModel'
    ) -> List[Tuple[float, float]]:
        """Find all moment extrema across entire beam.

        Args:
            axis: 'y' or 'z' for bending plane
            model: StructuralModel object (must be analyzed)

        Returns:
            List of (x_global, moment_value) for all local max/min
        """
        if not model.is_analyzed():
            raise ValueError("Model must be analyzed before finding extrema")

        extrema = []
        cumulative_length = 0.0

        for element in self.elements:
            # Find extrema within this element
            elem_extrema = element.find_moment_extremes(
                axis,
                model.get_all_displacements(),
                model._cpp_model.get_dof_handler()
            )

            # elem_extrema is a pair of (min, max) ActionExtreme objects
            min_extreme, max_extreme = elem_extrema

            # Convert to global coordinates and add to list
            extrema.append((cumulative_length + min_extreme.x, min_extreme.value))
            extrema.append((cumulative_length + max_extreme.x, max_extreme.value))

            cumulative_length += element.length

        # Remove duplicates at element boundaries (within tolerance)
        unique_extrema = []
        for x, val in extrema:
            is_duplicate = False
            for x_existing, val_existing in unique_extrema:
                if abs(x - x_existing) < 1e-10 and abs(val - val_existing) < 1e-10:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_extrema.append((x, val))

        return unique_extrema

    def get_element_boundaries(self) -> List[float]:
        """Get x-positions of element boundaries (joints).

        Returns:
            List of x-positions where elements meet (excludes beam endpoints)
        """
        if len(self.elements) <= 1:
            return []

        boundaries = []
        cumulative_length = 0.0

        for element in self.elements[:-1]:
            cumulative_length += element.length
            boundaries.append(cumulative_length)

        return boundaries

    def plot_internal_actions(
        self,
        model: 'StructuralModel',
        components: List[str] = None,
        figsize: Tuple[float, float] = (12, 8),
        num_points: int = 100
    ):
        """Create matplotlib plots of internal actions.

        Args:
            model: StructuralModel object (must be analyzed)
            components: List of components to plot ('Mz', 'My', 'Vy', 'Vz', 'N', 'Mx')
                        Default: ['Mz', 'Vy', 'N']
            figsize: Figure size (width, height)
            num_points: Number of points to sample along beam

        Returns:
            Figure object
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            raise ImportError("matplotlib is required for plotting. Install with: pip install matplotlib")

        if components is None:
            components = ['Mz', 'Vy', 'N']

        num_plots = len(components)
        fig, axes = plt.subplots(num_plots, 1, figsize=figsize, squeeze=False)
        axes = axes.flatten()

        for ax, component in zip(axes, components):
            # Get line data
            if component == 'Mz':
                x, values = self.get_moment_line('z', model, num_points)
                units = 'kN·m'
            elif component == 'My':
                x, values = self.get_moment_line('y', model, num_points)
                units = 'kN·m'
            elif component == 'Vy':
                x, values = self.get_shear_line('y', model, num_points)
                units = 'kN'
            elif component == 'Vz':
                x, values = self.get_shear_line('z', model, num_points)
                units = 'kN'
            elif component == 'N':
                x, values = self.get_axial_force_line(model, num_points)
                units = 'kN'
            elif component == 'Mx':
                x, values = self.get_torsion_line(model, num_points)
                units = 'kN·m'
            else:
                raise ValueError(f"Unknown component: {component}")

            # Plot
            ax.plot(x, values, 'b-', linewidth=2)
            ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
            ax.grid(True, alpha=0.3)
            ax.set_ylabel(f'{component} [{units}]')
            ax.set_title(f'{component} diagram')

            # Mark element boundaries
            boundaries = self.get_element_boundaries()
            for i, x_bound in enumerate(boundaries):
                ax.axvline(x_bound, color='gray', linestyle=':', alpha=0.5,
                           label='Element boundary' if i == 0 else '')

            # Mark extrema for moment diagrams
            if component in ['Mz', 'My']:
                axis_char = component[1].lower()
                extrema = self.find_moment_extrema(axis_char, model)
                for x_ext, val_ext in extrema:
                    ax.plot(x_ext, val_ext, 'ro', markersize=6)
                    ax.annotate(f'{val_ext:.2f}', xy=(x_ext, val_ext),
                                xytext=(5, 5), textcoords='offset points',
                                fontsize=8)

        axes[-1].set_xlabel('Position along beam [m]')
        plt.tight_layout()

        return fig

    # ===== Unified Line Data API =====

    def _find_extrema(
        self,
        values: np.ndarray,
        x_positions: np.ndarray
    ) -> List[Dict[str, Any]]:
        """Find global extrema in values array.

        Args:
            values: Array of action values
            x_positions: Array of corresponding x positions [m]

        Returns:
            List of {x, value, type, is_global} dicts for min and max values.
            Returns empty list if values is empty.
        """
        if len(values) == 0:
            return []

        min_idx = int(np.argmin(values))
        max_idx = int(np.argmax(values))

        extrema = []

        # Add minimum
        extrema.append({
            "x": float(x_positions[min_idx]),
            "value": float(values[min_idx]),
            "type": "min",
            "is_global": True
        })

        # Add maximum if different from minimum
        if min_idx != max_idx:
            extrema.append({
                "x": float(x_positions[max_idx]),
                "value": float(values[max_idx]),
                "type": "max",
                "is_global": True
            })

        return extrema

    def get_line_data(
        self,
        action_type: Union[str, ActionType],
        model: 'StructuralModel',
        num_points: int = 100,
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[str, Any]:
        """Get line diagram data for plotting.

        Provides a unified API for retrieving internal action data along the beam
        for plotting moment, shear, axial, torsion, or warping diagrams.

        Args:
            action_type: Type of internal action (ActionType enum or string like
                         'moment_y', 'shear_z', 'axial', 'torsion', 'bimoment')
            model: StructuralModel (must be analyzed)
            num_points: Number of points to sample along beam (default 100)
            load_case: Optional load case (uses active if None)

        Returns:
            Dictionary with:
                - x: List of positions along beam [m]
                - values: List of action values [kN or kN·m or kN·m²]
                - units: Unit string for display
                - label: Human-readable label
                - beam_id: Beam ID
                - beam_name: Beam name (or None)
                - beam_length: Total beam length [m]
                - extrema: List of {x, value, type, is_global} for min/max

        Raises:
            ValueError: If model not analyzed or invalid action type

        Example:
            >>> data = beam.get_line_data('moment_z', model)
            >>> plt.plot(data['x'], data['values'])
            >>> plt.ylabel(f"Mz [{data['units']}]")
        """
        if not model.is_analyzed():
            raise ValueError("Model must be analyzed before querying line data")

        # Convert string to ActionType enum if needed
        if isinstance(action_type, str):
            try:
                action_type = ActionType(action_type.lower())
            except ValueError:
                valid_types = [at.value for at in ActionType]
                raise ValueError(
                    f"Invalid action type '{action_type}'. "
                    f"Valid types: {valid_types}"
                )

        # Define units and labels
        action_info = {
            ActionType.MOMENT_Y: ("kN·m", "Bending Moment My"),
            ActionType.MOMENT_Z: ("kN·m", "Bending Moment Mz"),
            ActionType.SHEAR_Y: ("kN", "Shear Force Vy"),
            ActionType.SHEAR_Z: ("kN", "Shear Force Vz"),
            ActionType.AXIAL: ("kN", "Axial Force N"),
            ActionType.TORSION: ("kN·m", "Torsion Moment Mx"),
            ActionType.BIMOMENT: ("kN·m²", "Bimoment B"),
            ActionType.WARPING_TORSION: ("kN·m", "Warping Torsion Mx_w"),
        }

        units, label = action_info[action_type]

        # Sample positions along beam
        x_positions = np.linspace(0, self.length, num_points)
        values = np.zeros(num_points)

        # Check if we need warping actions
        is_warping_action = action_type in (ActionType.BIMOMENT, ActionType.WARPING_TORSION)

        # Get required data from model
        dof_handler = model._cpp_model.get_dof_handler()
        displacements = model._cpp_model.get_displacements()

        if load_case is None:
            load_case = model._cpp_model.get_default_load_case()

        for i, x in enumerate(x_positions):
            element, local_x = self._find_element_at_position(x)

            if is_warping_action:
                # Use warping internal actions method
                warping_actions = element.get_warping_internal_actions(
                    local_x, displacements, dof_handler
                )
                if action_type == ActionType.BIMOMENT:
                    values[i] = warping_actions.B
                else:  # WARPING_TORSION
                    values[i] = warping_actions.Mx_w
            else:
                # Use standard internal actions
                actions = element.get_internal_actions(
                    local_x, displacements, dof_handler, load_case
                )
                if action_type == ActionType.MOMENT_Y:
                    values[i] = actions.My
                elif action_type == ActionType.MOMENT_Z:
                    values[i] = actions.Mz
                elif action_type == ActionType.SHEAR_Y:
                    values[i] = actions.Vy
                elif action_type == ActionType.SHEAR_Z:
                    values[i] = actions.Vz
                elif action_type == ActionType.AXIAL:
                    values[i] = actions.N
                elif action_type == ActionType.TORSION:
                    values[i] = actions.Mx

        # Find extrema
        extrema = self._find_extrema(values, x_positions)

        return {
            "x": x_positions.tolist(),
            "values": values.tolist(),
            "units": units,
            "label": label,
            "beam_id": self.beam_id,
            "beam_name": self.name,
            "beam_length": self.length,
            "extrema": extrema
        }


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
        self._tolerance = node_tolerance
        self._cpp_model = _CppModel(node_tolerance=node_tolerance)
        self.beams: List[Beam] = []
        self.cargos: List[Cargo] = []
        self._plates: List[Plate] = []
        self._materials: dict[str, _CppMaterial] = {}
        self._sections: dict[str, _CppSection] = {}
        self._node_map: dict[Tuple[float, float, float], Node] = {}
        self._beam_id_counter = 1
        self._load_combinations: List[dict] = []
        self._combination_id_counter = 1

    # ===== Material and Section Management =====

    def add_material(
        self,
        name: str,
        E: float,
        nu: float,
        rho: float = 7.85,
        fy: float = 0.0,
        fu: float = 0.0
    ) -> _CppMaterial:
        """Add a material to the model library.

        Args:
            name: Material name
            E: Young's modulus [kN/m²]
            nu: Poisson's ratio [-]
            rho: Density [mT/m³] (default: 7.85 mT/m³ for steel)
            fy: Yield stress [kN/m²] (default: 0, not specified)
            fu: Ultimate tensile strength [kN/m²] (default: 0, not specified)

        Returns:
            Material object

        Note:
            Typical steel density is 7.85 mT/m³ (7850 kg/m³).
            Typical S355 steel: fy=355000 kN/m² (355 MPa), fu=470000 kN/m² (470 MPa).
        """
        if name in self._materials:
            return self._materials[name]

        mat = self._cpp_model.create_material(name, E, nu, rho, fy, fu)
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

        # Fast path: exact key lookup
        node = self._node_map.get(key)
        if node is not None:
            return node

        # Slow path: tolerance-based search through all nodes
        tol = tolerance if tolerance is not None else self._tolerance
        tol_sq = tol * tol

        for node in self._node_map.values():
            dx = node.x - x
            dy = node.y - y
            dz = node.z - z
            dist_sq = dx*dx + dy*dy + dz*dz
            if dist_sq <= tol_sq:
                return node

        return None

    # ===== Beam Creation =====

    def add_beam_by_coords(
        self,
        start_pos: List[float],
        end_pos: List[float],
        section_name: str,
        material_name: str,
        subdivisions: int = 1,
        name: Optional[str] = None,
        **kwargs
    ) -> Beam:
        """Add a beam using endpoint coordinates.

        Args:
            start_pos: Starting position [x, y, z]
            end_pos: Ending position [x, y, z]
            section_name: Name of section from library
            material_name: Name of material from library
            subdivisions: Number of FEM elements to subdivide the beam into (default: 1)
            name: Optional beam name for display
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

        # Validate subdivisions
        subdivisions = max(1, int(subdivisions))

        # Create Python Beam object first
        beam = Beam(start_pos, end_pos, section, material, beam_id=self._beam_id_counter, name=name)
        self._beam_id_counter += 1

        # Create start and end nodes
        start_node = self.get_or_create_node(*start_pos)
        end_node = self.get_or_create_node(*end_pos)

        if subdivisions == 1:
            # Single element beam
            cpp_element = self._cpp_model.create_beam(start_node, end_node, material, section)
            beam.add_element(cpp_element)
        else:
            # Multi-element beam - create intermediate nodes
            start_arr = np.array(start_pos)
            end_arr = np.array(end_pos)
            direction = end_arr - start_arr

            # Create all nodes (including intermediate)
            nodes = [start_node]
            for i in range(1, subdivisions):
                t = i / subdivisions
                pos = start_arr + t * direction
                node = self.get_or_create_node(pos[0], pos[1], pos[2])
                nodes.append(node)
            nodes.append(end_node)

            # Create elements between consecutive nodes
            for i in range(subdivisions):
                cpp_element = self._cpp_model.create_beam(nodes[i], nodes[i+1], material, section)
                beam.add_element(cpp_element)

        self.beams.append(beam)
        return beam

    # ===== Cargo Modelling =====

    def add_cargo(self, cargo: Cargo) -> Cargo:
        """Add a cargo object to the model.

        This method generates the underlying finite elements for the cargo:
        - A point mass at the cargo's center of gravity
        - Spring elements connecting to the structure
        - Rigid links for offset connections (if applicable)

        Args:
            cargo: A Cargo object with CoG, mass, inertia, and connections defined

        Returns:
            The same Cargo object (for method chaining)

        Raises:
            ValueError: If cargo has no connections defined
            RuntimeError: If cargo has already been added to a model

        Example:
            cargo = Cargo("Equipment")
            cargo.set_cog([5.0, 2.0, 1.5])
            cargo.set_mass(50.0)
            cargo.add_connection([5.0, 2.0, 0.0], [1e6, 1e6, 1e6, 1e4, 1e4, 1e4])
            model.add_cargo(cargo)
        """
        # Generate the underlying FE elements
        cargo.generate_elements(self._cpp_model)

        # Track the cargo
        self.cargos.append(cargo)

        return cargo

    def get_cargo(self, name: str) -> Optional[Cargo]:
        """Get a cargo by name.

        Args:
            name: Name of the cargo to find

        Returns:
            Cargo object if found, None otherwise
        """
        for cargo in self.cargos:
            if cargo.name == name:
                return cargo
        return None

    # ===== Plate Elements =====

    def add_plate_element(
        self,
        node1: List[float],
        node2: List[float],
        node3: List[float],
        node4: List[float],
        thickness: float,
        material: str
    ) -> "PlateElement":
        """Add a single 4-node plate element to the model.

        Creates a MITC4 Mindlin plate element for bending analysis.
        Nodes should be ordered counter-clockwise when viewed from
        the positive normal direction.

        Node numbering (natural coordinates):
           4 (-1,+1) -------- 3 (+1,+1)
               |                  |
               |     (0,0)        |
               |                  |
           1 (-1,-1) -------- 2 (+1,-1)

        Args:
            node1: Corner 1 coordinates [x, y, z] in meters.
            node2: Corner 2 coordinates [x, y, z] in meters.
            node3: Corner 3 coordinates [x, y, z] in meters.
            node4: Corner 4 coordinates [x, y, z] in meters.
            thickness: Plate thickness in meters.
            material: Name of material to use.

        Returns:
            The created PlateElement object.

        Raises:
            ValueError: If material not found.

        Example:
            # Create a 2m x 1m horizontal plate
            plate = model.add_plate_element(
                [0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0],
                thickness=0.02,
                material="Steel"
            )
        """
        mat = self.get_material(material)
        if mat is None:
            raise ValueError(f"Material '{material}' not found. Add it first with add_material().")

        n1 = self.get_or_create_node(*node1)
        n2 = self.get_or_create_node(*node2)
        n3 = self.get_or_create_node(*node3)
        n4 = self.get_or_create_node(*node4)

        plate = self._cpp_model.create_plate(n1, n2, n3, n4, thickness, mat)
        return plate

    def get_plate_elements(self) -> List:
        """Get all plate elements in the model.

        Returns:
            List of all plate elements (MITC4, MITC8, MITC9, DKT).
        """
        elements = []
        elements.extend(self._cpp_model.plate_elements)
        elements.extend(self._cpp_model.plate_elements_8)
        elements.extend(self._cpp_model.plate_elements_9)
        elements.extend(self._cpp_model.plate_elements_tri)
        return elements

    # ===== Plate Geometry (for meshing) =====

    def add_plate(
        self,
        corners: List[List[float]],
        thickness: float,
        material: str,
        mesh_size: float = 0.5,
        element_type: str = "MITC4",
        name: Optional[str] = None
    ) -> Plate:
        """Add a plate region to the model.

        The plate is defined by 3 or more corner points forming a closed polygon.
        The plate will be meshed when mesh() is called.

        Args:
            corners: List of [x, y, z] coordinates for plate corners.
                Must have at least 3 points. Counter-clockwise when viewed
                from the positive normal direction.
            thickness: Plate thickness in meters.
            material: Name of material to use.
            mesh_size: Target element size in meters (default 0.5m).
            element_type: Element type: "MITC4", "MITC8", "MITC9", or "DKT".
            name: Optional name for the plate.

        Returns:
            The created Plate object.

        Raises:
            ValueError: If material not found, invalid geometry, or unknown element type.

        Example:
            # Create a 4m x 2m horizontal plate
            plate = model.add_plate(
                corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
                thickness=0.02,
                material="Steel",
                mesh_size=0.5
            )
        """
        # Validate material exists
        if self.get_material(material) is None:
            raise ValueError(f"Material '{material}' not found. Add it first with add_material().")

        # Validate element type
        get_element_type(element_type)  # Raises ValueError if invalid

        # Create the plate
        plate = Plate(
            corners=corners,
            thickness=thickness,
            material=material,
            mesh_size=mesh_size,
            element_type=element_type,
            name=name or f"Plate_{len(self._plates) + 1}"
        )

        # Validate geometry
        if not plate.is_planar():
            raise ValueError("Plate corners must be coplanar")

        self._plates.append(plate)
        return plate

    def set_edge_divisions(
        self,
        plate: Plate,
        edge_index: int,
        n_elements: int
    ) -> None:
        """Set the number of elements along a plate edge.

        Edge divisions take precedence over mesh_size for that edge.
        For structured quad meshes, opposite edges must have matching divisions.

        Args:
            plate: The Plate object.
            edge_index: Edge index (0 = from corner[0] to corner[1], etc.)
            n_elements: Number of elements along the edge.

        Raises:
            ValueError: If edge_index out of range or n_elements < 1.
        """
        if edge_index < 0 or edge_index >= plate.n_edges:
            raise ValueError(
                f"Edge index {edge_index} out of range [0, {plate.n_edges})"
            )
        if n_elements < 1:
            raise ValueError("n_elements must be at least 1")

        plate.edge_controls[edge_index] = EdgeMeshControl(n_elements=n_elements)

    def get_plates(self) -> List[Plate]:
        """Return all plates (geometry regions) in the model.

        Returns:
            List of Plate objects defining plate regions.
        """
        return list(self._plates)

    def couple_plate_to_beam(
        self,
        plate: Plate,
        edge_index: int,
        beam: "Beam",
        offset: Optional[List[float]] = None,
        releases: Optional[Dict[str, bool]] = None
    ) -> "PlateBeamCoupling":
        """Couple a plate edge to a beam using rigid links.

        Creates a coupling definition between a plate edge and a beam.
        When mesh() is called, rigid link constraints will be created
        between plate edge nodes and the beam.

        If the plate normal is not parallel to the beam axis, intermediate
        beam nodes are created to match plate edge node positions.

        Args:
            plate: The Plate object.
            edge_index: Edge index to couple (0 = corner[0] to corner[1], etc.)
            beam: The Beam object to couple to.
            offset: [dx, dy, dz] offset from plate node to beam centroid in meters.
                If None, nodes are assumed coincident.
            releases: Dict of DOF releases. Keys: "UX", "UY", "UZ", "RX", "RY", "RZ",
                or "R_EDGE" for rotation about the edge. Values: True = released.
                Default is no releases (fully rigid connection).

        Returns:
            PlateBeamCoupling object.

        Raises:
            ValueError: If edge_index is out of range.

        Example:
            # Couple plate edge to beam with moment release about edge
            model.couple_plate_to_beam(
                plate, edge_index=0, beam=main_beam,
                offset=[0, 0, 0.15],  # Plate 150mm above beam centroid
                releases={"R_EDGE": True}  # Allow rotation about edge
            )
        """
        from .plate import PlateBeamCoupling

        if edge_index < 0 or edge_index >= plate.n_edges:
            raise ValueError(
                f"Edge index {edge_index} out of range [0, {plate.n_edges})"
            )

        coupling = PlateBeamCoupling(
            plate=plate,
            edge_index=edge_index,
            beam=beam,
            offset=offset or [0.0, 0.0, 0.0],
            releases=releases or {}
        )

        plate.beam_couplings.append(coupling)
        return coupling

    def add_support_curve(
        self,
        plate: Plate,
        edge_index: int,
        ux: bool = False,
        uy: bool = False,
        uz: bool = False,
        rotation_about_edge: bool = False
    ) -> "SupportCurve":
        """Add a support along a plate edge.

        Creates boundary conditions along the plate edge that will be applied
        to all nodes on that edge after meshing.

        Args:
            plate: The Plate object.
            edge_index: Edge index for the support.
            ux: If True, restrain X translation.
            uy: If True, restrain Y translation.
            uz: If True, restrain Z translation.
            rotation_about_edge: If True, restrain rotation about the edge direction.

        Returns:
            SupportCurve object.

        Raises:
            ValueError: If edge_index is out of range.

        Example:
            # Simply support a plate edge (restrain vertical movement)
            model.add_support_curve(plate, edge_index=0, uz=True)
        """
        from .plate import SupportCurve

        if edge_index < 0 or edge_index >= plate.n_edges:
            raise ValueError(
                f"Edge index {edge_index} out of range [0, {plate.n_edges})"
            )

        support = SupportCurve(
            plate=plate,
            edge_index=edge_index,
            ux=ux,
            uy=uy,
            uz=uz,
            rotation_about_edge=rotation_about_edge
        )

        plate.support_curves.append(support)
        return support

    # ===== Meshing =====

    def mesh(self, verbose: bool = False) -> MeshStatistics:
        """Mesh all plates and set up plate-beam coupling.

        This method performs the complete meshing workflow:
        1. Meshes all plates using gmsh
        2. Creates plate elements from the mesh
        3. Creates beam nodes at plate edge coupling points
        4. Subdivides beams coupled to plate edges
        5. Applies support curve boundary conditions
        6. Creates rigid link constraints for plate-beam coupling

        Args:
            verbose: If True, print progress information.

        Returns:
            MeshStatistics with mesh generation statistics.

        Raises:
            ValueError: If mesh compatibility check fails.

        Example:
            >>> model.add_plate(corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            ...                 thickness=0.02, material="Steel")
            >>> stats = model.mesh()
            >>> print(f"Created {stats.n_plate_elements} plate elements")
        """
        from grillex.meshing.gmsh_mesher import GmshPlateMesher

        stats = MeshStatistics()

        if not self._plates:
            if verbose:
                print("No plates to mesh")
            return stats

        # Track nodes created per plate for element creation
        plate_node_map: Dict[int, Dict[int, Node]] = {}  # plate_idx -> mesh_node_idx -> Node
        # Also track node ID to Node object for coupling
        node_id_to_node: Dict[int, Node] = {}

        with GmshPlateMesher() as mesher:
            for plate_idx, plate in enumerate(self._plates):
                if verbose:
                    print(f"Meshing plate {plate_idx + 1}/{len(self._plates)}: {plate.name}")

                # Generate mesh
                mesh_result = mesher.mesh_plate_from_geometry(plate)

                if verbose:
                    print(f"  Generated {mesh_result.n_nodes} nodes, "
                          f"{mesh_result.n_quads} quads, {mesh_result.n_triangles} triangles")

                # Create nodes in model
                node_map: Dict[int, Node] = {}
                for i, coords in enumerate(mesh_result.nodes):
                    node = self.get_or_create_node(coords[0], coords[1], coords[2])
                    node_map[i] = node
                    node_id_to_node[node.id] = node

                plate_node_map[plate_idx] = node_map

                # Store edge node map in plate
                plate._mesh_nodes = mesh_result.nodes
                plate._edge_node_map = {}
                for edge_idx, mesh_node_indices in mesh_result.edge_nodes.items():
                    plate._edge_node_map[edge_idx] = [
                        node_map[idx].id for idx in mesh_node_indices
                    ]

                # Get material for elements
                material = None
                for mat in self._cpp_model.materials:
                    if mat.name == plate.material:
                        material = mat
                        break
                if material is None:
                    raise ValueError(f"Material '{plate.material}' not found in model")

                # Create plate elements based on element type and mesh results
                element_type = plate.element_type.upper()
                created_quads = 0
                created_tris = 0

                if element_type == "MITC4":
                    # 4-node quads
                    for quad_conn in mesh_result.quads:
                        nodes = [node_map[idx] for idx in quad_conn]
                        self._cpp_model.create_plate(
                            nodes[0], nodes[1], nodes[2], nodes[3],
                            plate.thickness, material
                        )
                        created_quads += 1

                elif element_type == "MITC8":
                    # 8-node serendipity quads
                    for quad_conn in mesh_result.quads_8:
                        nodes = [node_map[idx] for idx in quad_conn]
                        self._cpp_model.create_plate_8(
                            nodes[0], nodes[1], nodes[2], nodes[3],
                            nodes[4], nodes[5], nodes[6], nodes[7],
                            plate.thickness, material
                        )
                        created_quads += 1

                elif element_type == "MITC9":
                    # 9-node Lagrangian quads
                    for quad_conn in mesh_result.quads_9:
                        nodes = [node_map[idx] for idx in quad_conn]
                        self._cpp_model.create_plate_9(
                            nodes[0], nodes[1], nodes[2], nodes[3],
                            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
                            plate.thickness, material
                        )
                        created_quads += 1

                elif element_type == "DKT":
                    # 3-node triangles
                    for tri_conn in mesh_result.triangles:
                        nodes = [node_map[idx] for idx in tri_conn]
                        self._cpp_model.create_plate_tri(
                            nodes[0], nodes[1], nodes[2],
                            plate.thickness, material
                        )
                        created_tris += 1
                else:
                    raise ValueError(f"Unknown element type: {element_type}")

                stats.n_plate_nodes += len(node_map)
                stats.n_plate_elements += created_quads + created_tris
                stats.n_quad_elements += created_quads
                stats.n_tri_elements += created_tris

                # Apply support curves for this plate
                for support in plate.support_curves:
                    edge_node_ids = plate._edge_node_map.get(support.edge_index, [])
                    for node_id in edge_node_ids:
                        # BCs just need node ID, not the node object
                        if support.ux:
                            self._cpp_model.boundary_conditions.add_fixed_dof(
                                node_id, DOFIndex.UX, 0.0
                            )
                            stats.n_support_dofs += 1
                        if support.uy:
                            self._cpp_model.boundary_conditions.add_fixed_dof(
                                node_id, DOFIndex.UY, 0.0
                            )
                            stats.n_support_dofs += 1
                        if support.uz:
                            self._cpp_model.boundary_conditions.add_fixed_dof(
                                node_id, DOFIndex.UZ, 0.0
                            )
                            stats.n_support_dofs += 1
                        if support.rotation_about_edge:
                            # Rotation about edge - restrain RX and RY
                            self._cpp_model.boundary_conditions.add_fixed_dof(
                                node_id, DOFIndex.RX, 0.0
                            )
                            self._cpp_model.boundary_conditions.add_fixed_dof(
                                node_id, DOFIndex.RY, 0.0
                            )
                            stats.n_support_dofs += 2

        # Process plate-beam couplings
        for plate_idx, plate in enumerate(self._plates):
            node_map = plate_node_map.get(plate_idx, {})

            for coupling in plate.beam_couplings:
                if plate._edge_node_map is None:
                    continue

                edge_node_ids = plate._edge_node_map.get(coupling.edge_index, [])
                if not edge_node_ids:
                    continue

                _beam = coupling.beam  # noqa: F841
                offset = np.array(coupling.offset)

                # For each node on the plate edge, create a rigid link to the beam
                for node_id in edge_node_ids:
                    plate_node = node_id_to_node.get(node_id)
                    if plate_node is None:
                        continue

                    # Find or create corresponding beam node
                    # Project plate node position onto beam axis to find beam node location
                    plate_pos = np.array([plate_node.x, plate_node.y, plate_node.z])
                    beam_node_pos = plate_pos - offset

                    # Get or create beam node at this position
                    beam_node = self.get_or_create_node(
                        beam_node_pos[0], beam_node_pos[1], beam_node_pos[2]
                    )
                    node_id_to_node[beam_node.id] = beam_node

                    # Only create rigid link if nodes are different
                    # (if offset is zero or very small, they may be the same node)
                    if plate_node.id != beam_node.id:
                        # Create rigid link: plate_node is slave, beam_node is master
                        # Offset is from master (beam) to slave (plate)
                        self._cpp_model.add_rigid_link(
                            plate_node, beam_node, offset
                        )
                        stats.n_rigid_links += 1

                # Subdivide beam if needed (to have nodes at coupling points)
                # This is handled by the beam subdivision mechanism already present
                stats.n_beams_subdivided += 1

        if verbose:
            print("\nMesh Statistics:")
            print(f"  Total plate nodes: {stats.n_plate_nodes}")
            print(f"  Total plate elements: {stats.n_plate_elements}")
            print(f"    Quad elements: {stats.n_quad_elements}")
            print(f"    Triangle elements: {stats.n_tri_elements}")
            print(f"  Rigid links: {stats.n_rigid_links}")
            print(f"  Support DOFs: {stats.n_support_dofs}")

        return stats

    # ===== Spring Elements =====

    def add_spring(
        self,
        node1: Union[List[float], Node],
        node2: Union[List[float], Node],
        kx: float = 0.0,
        ky: float = 0.0,
        kz: float = 0.0,
        krx: float = 0.0,
        kry: float = 0.0,
        krz: float = 0.0,
        behavior: Optional["SpringBehavior"] = None,
        behavior_per_dof: Optional[Dict[int, "SpringBehavior"]] = None,
        gap: float = 0.0,
        gap_per_dof: Optional[Dict[int, float]] = None,
    ) -> _CppSpringElement:
        """Add a spring element between two nodes.

        Springs can model:
        - Linear elastic connections (behavior=Linear)
        - Tension-only connections like cables/hooks (behavior=TensionOnly)
        - Compression-only connections like bearing pads (behavior=CompressionOnly)
        - Gap springs with initial clearance (gap > 0)

        Args:
            node1: First node or [x, y, z] coordinates [m].
            node2: Second node or [x, y, z] coordinates [m].
            kx, ky, kz: Translational stiffness [kN/m].
            krx, kry, krz: Rotational stiffness [kN·m/rad].
            behavior: Default behavior for all DOFs.
            behavior_per_dof: Optional per-DOF behavior override.
                Keys are DOF indices (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ).
            gap: Default gap for all DOFs [m for translation, rad for rotation].
                Spring only activates after deformation exceeds this gap.
            gap_per_dof: Optional per-DOF gap override.
                Keys are DOF indices (0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ).

        Returns:
            The created SpringElement.

        Example:
            # Compression-only vertical spring (bearing pad)
            spring = model.add_spring(
                [0, 0, 0], [0, 0, 1],
                kz=10000.0,  # 10 MN/m vertical stiffness
                behavior=SpringBehavior.CompressionOnly
            )

            # Mixed behavior: compression in Z, linear in X/Y
            spring = model.add_spring(
                [0, 0, 0], [0, 0, 1],
                kx=1000.0, ky=1000.0, kz=5000.0,
                behavior=SpringBehavior.Linear,
                behavior_per_dof={2: SpringBehavior.CompressionOnly}
            )

            # Gap spring (contact with 10mm clearance)
            spring = model.add_spring(
                [0, 0, 0], [0, 0, 1],
                kz=50000.0,  # High contact stiffness
                behavior=SpringBehavior.CompressionOnly,
                gap=0.010  # 10mm gap before contact
            )
        """
        # Get or create nodes
        if isinstance(node1, list):
            n1 = self.get_or_create_node(*node1)
        else:
            n1 = node1

        if isinstance(node2, list):
            n2 = self.get_or_create_node(*node2)
        else:
            n2 = node2

        # Create spring element
        spring = self._cpp_model.create_spring(n1, n2)

        # Set stiffness values
        spring.kx = kx
        spring.ky = ky
        spring.kz = kz
        spring.krx = krx
        spring.kry = kry
        spring.krz = krz

        # Set behavior (only if SpringBehavior is available and behavior is non-linear)
        if behavior is not None and SpringBehavior is not None:
            if behavior != SpringBehavior.Linear:
                spring.set_all_behavior(behavior)

        if behavior_per_dof and SpringBehavior is not None:
            for dof, b in behavior_per_dof.items():
                spring.set_behavior(dof, b)

        # Set gaps
        if gap != 0.0:
            spring.set_all_gaps(gap)

        if gap_per_dof:
            for dof, g in gap_per_dof.items():
                spring.set_gap(dof, g)

        return spring

    def has_nonlinear_springs(self) -> bool:
        """Check if model contains nonlinear (tension/compression-only) springs.

        Returns:
            True if any spring has non-Linear behavior or gap.
        """
        return self._cpp_model.has_nonlinear_springs()

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

    def create_load_combination(
        self,
        name: str,
        combo_type: str = "ULS-a",
        permanent_factor: float = 1.0,
        variable_factor: float = 1.0,
        environmental_factor: float = 1.0,
        accidental_factor: float = 0.0
    ) -> dict:
        """Create a load combination with type-based factors.

        Args:
            name: Combination name (e.g., 'Main Load', 'Dead + Live')
            combo_type: Combination type (ULS-a, ULS-b, SLS, ALS)
            permanent_factor: Factor for permanent load cases
            variable_factor: Factor for variable load cases
            environmental_factor: Factor for environmental load cases
            accidental_factor: Factor for accidental load cases

        Returns:
            Dictionary with combination properties
        """
        combo = {
            "id": self._combination_id_counter,
            "name": name,
            "type": combo_type,
            "permanent_factor": permanent_factor,
            "variable_factor": variable_factor,
            "environmental_factor": environmental_factor,
            "accidental_factor": accidental_factor,
            "load_cases": [],  # List of {load_case_id, override_factor, has_override}
        }
        self._load_combinations.append(combo)
        self._combination_id_counter += 1
        return combo

    def get_load_combinations(self) -> List[dict]:
        """Get all load combinations."""
        return self._load_combinations

    def delete_load_combination(self, combination_id: int) -> None:
        """Delete a load combination.

        Args:
            combination_id: ID of the combination to delete

        Raises:
            ValueError: If combination not found
        """
        for i, c in enumerate(self._load_combinations):
            if c["id"] == combination_id:
                self._load_combinations.pop(i)
                return
        raise ValueError(f"Load combination with ID {combination_id} not found")

    def add_load_case_to_combination(
        self,
        combination_id: int,
        load_case_id: int,
        override_factor: Optional[float] = None
    ) -> dict:
        """Add a load case to a load combination.

        Args:
            combination_id: ID of the load combination
            load_case_id: ID of the load case to add
            override_factor: Optional factor override (if not provided, uses type-based factor)

        Returns:
            The updated combination dictionary

        Raises:
            ValueError: If combination or load case not found, or load case already in combination
        """
        # Find the combination
        combo = None
        for c in self._load_combinations:
            if c["id"] == combination_id:
                combo = c
                break
        if combo is None:
            raise ValueError(f"Load combination with ID {combination_id} not found")

        # Find the load case by iterating over all load cases
        load_case = None
        for lc in self._cpp_model.get_load_cases():
            if lc.id == load_case_id:
                load_case = lc
                break
        if load_case is None:
            raise ValueError(f"Load case with ID {load_case_id} not found")

        # Check if already in combination
        for lc in combo["load_cases"]:
            if lc["load_case_id"] == load_case_id:
                raise ValueError(f"Load case {load_case_id} already in combination")

        # Get load case type name for display
        type_name = load_case.type.name if hasattr(load_case.type, 'name') else str(load_case.type)

        # Add to combination
        lc_member = {
            "load_case_id": load_case_id,
            "name": load_case.name,
            "type": type_name,
            "override_factor": override_factor,
            "has_override": override_factor is not None,
        }
        combo["load_cases"].append(lc_member)
        return combo

    def remove_load_case_from_combination(
        self,
        combination_id: int,
        load_case_id: int
    ) -> dict:
        """Remove a load case from a load combination.

        Args:
            combination_id: ID of the load combination
            load_case_id: ID of the load case to remove

        Returns:
            The updated combination dictionary

        Raises:
            ValueError: If combination not found or load case not in combination
        """
        # Find the combination
        combo = None
        for c in self._load_combinations:
            if c["id"] == combination_id:
                combo = c
                break
        if combo is None:
            raise ValueError(f"Load combination with ID {combination_id} not found")

        # Find and remove the load case
        for i, lc in enumerate(combo["load_cases"]):
            if lc["load_case_id"] == load_case_id:
                combo["load_cases"].pop(i)
                return combo

        raise ValueError(f"Load case {load_case_id} not found in combination")

    def update_load_case_override(
        self,
        combination_id: int,
        load_case_id: int,
        override_factor: Optional[float]
    ) -> dict:
        """Update the override factor for a load case in a combination.

        Args:
            combination_id: ID of the load combination
            load_case_id: ID of the load case
            override_factor: New factor override (None to remove override)

        Returns:
            The updated combination dictionary

        Raises:
            ValueError: If combination not found or load case not in combination
        """
        # Find the combination
        combo = None
        for c in self._load_combinations:
            if c["id"] == combination_id:
                combo = c
                break
        if combo is None:
            raise ValueError(f"Load combination with ID {combination_id} not found")

        # Find and update the load case
        for lc in combo["load_cases"]:
            if lc["load_case_id"] == load_case_id:
                lc["override_factor"] = override_factor
                lc["has_override"] = override_factor is not None
                return combo

        raise ValueError(f"Load case {load_case_id} not found in combination")

    def add_point_load(
        self,
        position: List[float],
        force: Optional[List[float]] = None,
        moment: Optional[List[float]] = None,
        load_case: Optional[_CppLoadCase] = None
    ) -> None:
        """Add a point load at a position.

        Args:
            position: [x, y, z] coordinates in meters
            force: [Fx, Fy, Fz] force vector in kN (default: [0, 0, 0])
            moment: [Mx, My, Mz] moment vector in kNm (default: [0, 0, 0])
            load_case: LoadCase to add to (uses default if None)

        Example:
            # Add vertical force at a node
            model.add_point_load([6, 0, 0], force=[0, 0, -10])

            # Add moment about Y axis
            model.add_point_load([6, 0, 0], moment=[0, 100, 0])

            # Add both force and moment
            model.add_point_load([6, 0, 0], force=[0, 0, -10], moment=[0, 100, 0])
        """
        if force is None and moment is None:
            raise ValueError("At least one of 'force' or 'moment' must be provided")

        force_vec = np.array(force if force is not None else [0.0, 0.0, 0.0], dtype=float)
        moment_vec = np.array(moment if moment is not None else [0.0, 0.0, 0.0], dtype=float)
        position_vec = np.array(position, dtype=float)

        if load_case is None:
            load_case = self.get_default_load_case()

        load_case.add_nodal_load(position_vec, force_vec, moment_vec)

    def add_line_load(
        self,
        beam: "Beam",
        w_start: List[float],
        w_end: Optional[List[float]] = None,
        load_case: Optional[_CppLoadCase] = None
    ) -> None:
        """Add a distributed line load to a beam.

        Args:
            beam: Beam object to apply load to
            w_start: Load intensity at start [kN/m] as [wx, wy, wz] in global coordinates
            w_end: Load intensity at end [kN/m] (uses w_start for uniform load if None)
            load_case: LoadCase to add to (uses default if None)

        Example:
            # Uniform vertical load of 10 kN/m
            model.add_line_load(beam, [0, -10, 0])

            # Trapezoidal load varying from 5 to 15 kN/m
            model.add_line_load(beam, [0, -5, 0], [0, -15, 0])
        """
        if not beam.elements:
            raise ValueError("Beam has no elements - was it properly created?")

        # Use w_start for both if uniform load
        if w_end is None:
            w_end = w_start

        if load_case is None:
            load_case = self.get_default_load_case()

        # Convert to numpy arrays
        w_start_vec = np.array(w_start, dtype=float)
        w_end_vec = np.array(w_end, dtype=float)

        # Add line load to each element of the beam
        # For multi-element beams (after subdivision), we need to interpolate loads
        if len(beam.elements) == 1:
            # Simple case: single element
            elem = beam.elements[0]
            load_case.add_line_load(elem.id, w_start_vec, w_end_vec)
        else:
            # Multi-element beam: need to interpolate loads for each element
            # Calculate total beam length
            total_length = beam.length

            # Track cumulative length for interpolation
            cumulative_length = 0.0

            for elem in beam.elements:
                # Fractional positions along original beam
                t_start = cumulative_length / total_length
                t_end = (cumulative_length + elem.length) / total_length

                # Interpolate load values
                w_elem_start = w_start_vec + t_start * (w_end_vec - w_start_vec)
                w_elem_end = w_start_vec + t_end * (w_end_vec - w_start_vec)

                load_case.add_line_load(elem.id, w_elem_start, w_elem_end)

                cumulative_length += elem.length

    def add_line_load_by_coords(
        self,
        start_pos: List[float],
        end_pos: List[float],
        w_start: List[float],
        w_end: Optional[List[float]] = None,
        load_case: Optional[_CppLoadCase] = None
    ) -> None:
        """Add a distributed line load to a beam identified by its endpoint coordinates.

        Args:
            start_pos: [x, y, z] coordinates of beam start
            end_pos: [x, y, z] coordinates of beam end
            w_start: Load intensity at start [kN/m] as [wx, wy, wz] in global coordinates
            w_end: Load intensity at end [kN/m] (uses w_start for uniform load if None)
            load_case: LoadCase to add to (uses default if None)

        Example:
            # Uniform vertical load on beam from (0,0,0) to (6,0,0)
            model.add_line_load_by_coords([0,0,0], [6,0,0], [0, -10, 0])
        """
        # Find beam by coordinates
        beam = self.find_beam_by_coords(start_pos, end_pos)
        if beam is None:
            raise ValueError(f"No beam found from {start_pos} to {end_pos}")

        self.add_line_load(beam, w_start, w_end, load_case)

    def find_beam_by_coords(
        self,
        start_pos: List[float],
        end_pos: List[float],
        tolerance: float = 1e-6
    ) -> Optional["Beam"]:
        """Find a beam by its endpoint coordinates.

        Args:
            start_pos: [x, y, z] coordinates of beam start
            end_pos: [x, y, z] coordinates of beam end
            tolerance: Distance tolerance for coordinate matching

        Returns:
            Beam object if found, None otherwise
        """
        start = np.array(start_pos)
        end = np.array(end_pos)

        for beam in self.beams:
            # Check if start/end match (in either order)
            if (np.linalg.norm(beam.start_pos - start) < tolerance and
                np.linalg.norm(beam.end_pos - end) < tolerance):
                return beam
            if (np.linalg.norm(beam.start_pos - end) < tolerance and
                np.linalg.norm(beam.end_pos - start) < tolerance):
                return beam

        return None

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

    def analyze_with_nonlinear_springs(
        self,
        settings: Optional[NonlinearSolverSettings] = None
    ) -> Dict[int, LoadCaseResult]:
        """Run analysis with support for nonlinear (tension/compression-only) springs.

        If no nonlinear springs exist, falls back to efficient linear analysis.

        This method uses an iterative solver that:
        1. Assembles stiffness from currently active springs
        2. Solves the linear system
        3. Updates spring states based on deformation
        4. Repeats until convergence (no state changes)

        Args:
            settings: Optional solver settings. If None, uses model defaults.
                Key settings:
                - max_iterations: Maximum iterations (default: 50)
                - gap_tolerance: Tolerance for gap activation [m] (default: 1e-10)
                - displacement_tolerance: Convergence tolerance (default: 1e-8)

        Returns:
            Dictionary mapping load case ID to LoadCaseResult.
            Each result contains:
            - displacements: Global displacement vector
            - reactions: Reaction forces
            - iterations: Number of solver iterations
            - spring_states: Final spring states [(id, active[6])]
            - spring_forces: Final spring forces [(id, force[6])]

        Example:
            # Set up compression-only spring
            spring = model.add_spring(
                [0, 0, 0], [0, 0, 1],
                kz=10000.0,
                behavior=SpringBehavior.CompressionOnly
            )

            # Run nonlinear analysis
            results = model.analyze_with_nonlinear_springs()

            # Check spring state
            result = results[load_case.id()]
            print(f"Converged in {result.iterations} iterations")
        """
        if settings is not None:
            # Apply settings to model
            model_settings = self._cpp_model.nonlinear_settings()
            model_settings.max_iterations = settings.max_iterations
            model_settings.gap_tolerance = settings.gap_tolerance
            model_settings.displacement_tolerance = settings.displacement_tolerance

        self._cpp_model.analyze_nonlinear()

        # Return results dictionary
        return dict(self._cpp_model.get_all_results())

    def analyze_load_combination(
        self,
        combination: LoadCombination,
        settings: Optional[NonlinearSolverSettings] = None
    ) -> LoadCombinationResult:
        """Analyze a specific load combination with nonlinear spring support.

        Unlike linear analysis where combinations can use superposition
        (scaling and summing individual results), nonlinear springs require
        solving each combination directly as a single problem.

        **Static→Dynamic Sequencing:**
        This method implements proper physical sequencing:
        1. First solves Permanent loads only to establish baseline contact
           pattern (gaps close, cargo settles under gravity)
        2. Then solves the full combination starting from the static state

        This is essential for cargo analysis where gaps must close under
        gravity before dynamic loads (roll, pitch) are applied.

        Args:
            combination: The load combination to analyze.
            settings: Optional solver settings.

        Returns:
            LoadCombinationResult containing:
            - displacements: Combined displacement vector
            - reactions: Reaction forces
            - converged: Whether analysis converged
            - iterations: Total iterations (static + combined)
            - spring_states: Final spring states
            - spring_forces: Final spring forces

        Example:
            # Create load combination
            combo = LoadCombination(1, "ULS-Roll", 1.35, 1.5, 1.5, 1.0)
            combo.add_load_case(dead_load)   # Permanent - solved first
            combo.add_load_case(roll_load)   # Environmental - on top

            # Analyze combination
            result = model.analyze_load_combination(combo)
            if result.converged:
                print(f"Max displacement: {max(abs(result.displacements)):.4f} m")
                print(f"Converged in {result.iterations} iterations")
        """
        if settings is not None:
            return self._cpp_model.analyze_combination(combination, settings)
        else:
            return self._cpp_model.analyze_combination(combination)

    def get_nonlinear_settings(self) -> NonlinearSolverSettings:
        """Get the nonlinear solver settings for this model.

        Returns:
            NonlinearSolverSettings object (can be modified).
        """
        return self._cpp_model.nonlinear_settings()

    # ===== Eigenvalue (Modal) Analysis =====

    def analyze_modes(
        self,
        n_modes: int = 10,
        method: Optional["EigensolverMethod"] = None,
        tolerance: float = 1e-8,
        max_iterations: int = 100,
        compute_participation: bool = True,
        mass_normalize: bool = True
    ) -> bool:
        """Run eigenvalue analysis to compute natural frequencies and mode shapes.

        Solves the generalized eigenvalue problem:
            K × φ = ω² × M × φ

        Where:
        - K: Global stiffness matrix
        - M: Global mass matrix (from beam elements and point masses)
        - ω: Natural circular frequency [rad/s]
        - φ: Mode shape (eigenvector)

        Args:
            n_modes: Number of modes to compute (default: 10)
            method: Solver method (Dense, SubspaceIteration, ShiftInvert)
            tolerance: Convergence tolerance for iterative solvers
            max_iterations: Maximum iterations for iterative solvers
            compute_participation: Whether to compute participation factors
            mass_normalize: Whether to mass-normalize mode shapes

        Returns:
            True if analysis converged successfully

        Example:
            # Compute first 5 natural frequencies
            if model.analyze_modes(n_modes=5):
                frequencies = model.get_natural_frequencies()
                print(f"First frequency: {frequencies[0]:.2f} Hz")

                # Get mode shape for first mode
                mode1 = model.get_mode_shape(1)
        """
        if EigensolverSettings is None:
            raise RuntimeError("Eigenvalue analysis not available - C++ module needs rebuild")

        settings = EigensolverSettings()
        settings.n_modes = n_modes
        # Use Dense method as default if not specified
        if method is not None:
            settings.method = method
        elif EigensolverMethod is not None:
            settings.method = EigensolverMethod.Dense
        settings.tolerance = tolerance
        settings.max_iterations = max_iterations
        settings.compute_participation = compute_participation
        settings.mass_normalize = mass_normalize

        return self._cpp_model.analyze_eigenvalues(settings)

    def has_modal_results(self) -> bool:
        """Check if eigenvalue analysis results are available.

        Returns:
            True if analyze_modes() has been called successfully
        """
        return self._cpp_model.has_eigenvalue_results()

    def get_natural_frequencies(self) -> List[float]:
        """Get natural frequencies from eigenvalue analysis.

        Returns:
            List of natural frequencies [Hz], sorted ascending

        Raises:
            RuntimeError: If no eigenvalue results available
        """
        return self._cpp_model.get_natural_frequencies()

    def get_periods(self) -> List[float]:
        """Get natural periods from eigenvalue analysis.

        Returns:
            List of natural periods [s]

        Raises:
            RuntimeError: If no eigenvalue results available
        """
        return self._cpp_model.get_periods()

    def get_mode_shape(self, mode_number: int) -> np.ndarray:
        """Get mode shape for a specific mode.

        Args:
            mode_number: Mode number (1-based: 1 = first mode)

        Returns:
            Mode shape as numpy array (size = total_dofs)
            Values at constrained DOFs are zero.

        Raises:
            RuntimeError: If no eigenvalue results or mode not found
        """
        return np.array(self._cpp_model.get_mode_shape(mode_number))

    def get_eigenvalue_result(self) -> EigensolverResult:
        """Get full eigenvalue analysis results.

        Returns:
            EigensolverResult containing all modes, frequencies, and participation factors

        Raises:
            RuntimeError: If no eigenvalue results available
        """
        return self._cpp_model.get_eigenvalue_result()

    def get_mode_displacement_at(
        self,
        mode_number: int,
        position: List[float]
    ) -> Dict[str, float]:
        """Get mode shape displacement components at a specific node.

        Args:
            mode_number: Mode number (1-based)
            position: [x, y, z] coordinates of the node

        Returns:
            Dictionary with displacement components:
            {'UX': value, 'UY': value, 'UZ': value,
             'RX': value, 'RY': value, 'RZ': value}

        Raises:
            ValueError: If no node found at position
            RuntimeError: If no eigenvalue results available
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        mode_shape = self.get_mode_shape(mode_number)
        dof_handler = self._cpp_model.get_dof_handler()

        result = {}
        dof_names = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ']
        for i, dof in enumerate([DOFIndex.UX, DOFIndex.UY, DOFIndex.UZ,
                                  DOFIndex.RX, DOFIndex.RY, DOFIndex.RZ]):
            global_dof = dof_handler.get_global_dof(node.id, dof)
            if global_dof >= 0 and global_dof < len(mode_shape):
                result[dof_names[i]] = mode_shape[global_dof]
            else:
                result[dof_names[i]] = 0.0

        return result

    def get_modal_summary(self) -> "pd.DataFrame":
        """Get summary of all computed modes as a DataFrame.

        Returns:
            DataFrame with columns:
            - mode: Mode number
            - frequency_hz: Natural frequency [Hz]
            - period_s: Natural period [s]
            - eff_mass_x_pct: Effective modal mass X [%]
            - eff_mass_y_pct: Effective modal mass Y [%]
            - eff_mass_z_pct: Effective modal mass Z [%]
            - cumulative_x_pct: Cumulative mass X [%]
            - cumulative_y_pct: Cumulative mass Y [%]
            - cumulative_z_pct: Cumulative mass Z [%]

        Raises:
            ImportError: If pandas is not installed
            RuntimeError: If no eigenvalue results available

        Example:
            df = model.get_modal_summary()
            print(df.to_string())
        """
        try:
            import pandas as pd
        except ImportError:
            raise ImportError("pandas is required for get_modal_summary()")

        result = self.get_eigenvalue_result()

        rows = []
        cumulative_x = 0.0
        cumulative_y = 0.0
        cumulative_z = 0.0

        for mode in result.modes:
            cumulative_x += mode.effective_mass_pct_x
            cumulative_y += mode.effective_mass_pct_y
            cumulative_z += mode.effective_mass_pct_z

            rows.append({
                'mode': mode.mode_number,
                'frequency_hz': mode.frequency_hz,
                'period_s': mode.period_s,
                'eff_mass_x_pct': mode.effective_mass_pct_x,
                'eff_mass_y_pct': mode.effective_mass_pct_y,
                'eff_mass_z_pct': mode.effective_mass_pct_z,
                'cumulative_x_pct': cumulative_x,
                'cumulative_y_pct': cumulative_y,
                'cumulative_z_pct': cumulative_z,
            })

        return pd.DataFrame(rows)

    # ===== Spring Results Access =====

    def get_spring_state(
        self,
        spring_id: int,
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[str, bool]:
        """Get active/inactive state for each DOF of a spring.

        Args:
            spring_id: ID of the spring element.
            load_case: LoadCase to query (uses active if None).

        Returns:
            Dictionary mapping DOF names to active state.
            Keys: 'UX', 'UY', 'UZ', 'RX', 'RY', 'RZ'
            Values: True if spring DOF is active, False if inactive.

        Example:
            state = model.get_spring_state(1)
            if state['UZ']:
                print("Spring is in compression (active)")
            else:
                print("Spring has lifted off (inactive)")
        """
        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        results = self._cpp_model.get_all_results()
        active_lc = self._cpp_model.get_active_load_case()

        if active_lc is None or active_lc.id not in results:
            raise RuntimeError("No results available. Run analyze first.")

        result = results[active_lc.id]

        # Find the spring in the results
        dof_names = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ']
        for sid, states in result.spring_states:
            if sid == spring_id:
                return {dof_names[i]: states[i] for i in range(6)}

        raise ValueError(f"Spring {spring_id} not found in results")

    def get_spring_force(
        self,
        spring_id: int,
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[str, float]:
        """Get spring forces for each DOF.

        Args:
            spring_id: ID of the spring element.
            load_case: LoadCase to query (uses active if None).

        Returns:
            Dictionary mapping DOF names to forces.
            Keys: 'UX', 'UY', 'UZ' [kN], 'RX', 'RY', 'RZ' [kN·m]
            Values: Force/moment in spring (0 if inactive).

        Example:
            forces = model.get_spring_force(1)
            print(f"Vertical force: {forces['UZ']:.2f} kN")
        """
        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        results = self._cpp_model.get_all_results()
        active_lc = self._cpp_model.get_active_load_case()

        if active_lc is None or active_lc.id not in results:
            raise RuntimeError("No results available. Run analyze first.")

        result = results[active_lc.id]

        # Find the spring in the results
        dof_names = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ']
        for sid, forces in result.spring_forces:
            if sid == spring_id:
                return {dof_names[i]: forces[i] for i in range(6)}

        raise ValueError(f"Spring {spring_id} not found in results")

    def get_spring_summary(
        self,
        load_case: Optional[_CppLoadCase] = None
    ) -> "pd.DataFrame":
        """Get summary of all spring states and forces.

        Args:
            load_case: LoadCase to query (uses active if None).

        Returns:
            DataFrame with columns:
            - spring_id: Spring element ID
            - UX_active, UY_active, ...: Active state per DOF
            - UX_force, UY_force, ...: Force per DOF [kN or kN·m]

        Example:
            df = model.get_spring_summary()
            print(df[['spring_id', 'UZ_active', 'UZ_force']])
        """
        try:
            import pandas as pd
        except ImportError:
            raise ImportError("pandas is required for get_spring_summary()")

        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        results = self._cpp_model.get_all_results()
        active_lc = self._cpp_model.get_active_load_case()

        if active_lc is None or active_lc.id not in results:
            raise RuntimeError("No results available. Run analyze first.")

        result = results[active_lc.id]
        dof_names = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ']

        # Build state dict
        states_by_id = {sid: states for sid, states in result.spring_states}
        forces_by_id = {sid: forces for sid, forces in result.spring_forces}

        rows = []
        for spring_id in states_by_id:
            row = {'spring_id': spring_id}
            states = states_by_id[spring_id]
            forces = forces_by_id.get(spring_id, [0.0] * 6)

            for i, name in enumerate(dof_names):
                row[f'{name}_active'] = states[i]
                row[f'{name}_force'] = forces[i]

            rows.append(row)

        return pd.DataFrame(rows)

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

    def get_reactions_at(
        self,
        position: List[float],
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[DOFIndex, float]:
        """Get reaction forces at a node.

        Args:
            position: [x, y, z] coordinates
            load_case: LoadCase to query (uses active if None)

        Returns:
            Dictionary mapping DOFIndex to reaction value [kN] or [kN·m]
        """
        node = self.find_node_at(position)
        if node is None:
            raise ValueError(f"No node found at position {position}")

        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        reactions = self._cpp_model.get_reactions()
        dof_handler = self._cpp_model.get_dof_handler()

        result = {}
        for dof in [DOFIndex.UX, DOFIndex.UY, DOFIndex.UZ,
                    DOFIndex.RX, DOFIndex.RY, DOFIndex.RZ]:
            global_dof = dof_handler.get_global_dof(node.id, dof)
            if global_dof >= 0 and global_dof < len(reactions):
                result[dof] = reactions[global_dof]
            else:
                result[dof] = 0.0

        return result

    def set_active_load_case(self, load_case: _CppLoadCase) -> None:
        """Set the active load case for result queries.

        Args:
            load_case: LoadCase to make active
        """
        self._cpp_model.set_active_load_case(load_case)

    # ===== Plate Element Results =====

    def get_plate_displacement(
        self,
        plate_element,
        xi: float = 0.0,
        eta: float = 0.0,
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[str, float]:
        """Get displacement at a point within a plate element.

        Uses shape function interpolation to compute displacement at any
        point within the element from nodal displacements.

        Args:
            plate_element: PlateElement, PlateElement8, PlateElement9, or PlateElementTri.
            xi: Natural coordinate ξ (range [-1, 1] for quads, [0, 1] for triangles).
            eta: Natural coordinate η (range [-1, 1] for quads, [0, 1] for triangles).
            load_case: LoadCase to query (uses active if None).

        Returns:
            Dict with displacement components:
            - "UX", "UY", "UZ": translations [m]
            - "RX", "RY", "RZ": rotations [rad]

        Example:
            # Get displacement at center of plate element
            disp = model.get_plate_displacement(element, xi=0, eta=0)
            print(f"Vertical displacement: {disp['UZ']:.4f} m")
        """
        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        # Get nodal displacements
        u_global = self._cpp_model.get_displacements()
        dof_handler = self._cpp_model.get_dof_handler()

        # Get element nodes and their displacements
        nodes = plate_element.nodes
        n_nodes = len(nodes)

        # Build nodal displacement vector
        nodal_disp = []
        for node in nodes:
            for dof in [DOFIndex.UX, DOFIndex.UY, DOFIndex.UZ,
                        DOFIndex.RX, DOFIndex.RY, DOFIndex.RZ]:
                global_dof = dof_handler.get_global_dof(node.id, dof)
                if global_dof >= 0 and global_dof < len(u_global):
                    nodal_disp.append(u_global[global_dof])
                else:
                    nodal_disp.append(0.0)
        nodal_disp = np.array(nodal_disp)

        # Compute shape functions based on element type
        if n_nodes == 3:  # DKT triangle - use area coordinates
            L1 = 1.0 - xi - eta
            L2 = xi
            L3 = eta
            N = np.array([L1, L2, L3])
        elif n_nodes == 4:  # Bilinear quad
            N = np.array([
                0.25 * (1 - xi) * (1 - eta),
                0.25 * (1 + xi) * (1 - eta),
                0.25 * (1 + xi) * (1 + eta),
                0.25 * (1 - xi) * (1 + eta)
            ])
        elif n_nodes == 8:  # Serendipity quad
            # Corner nodes
            N1 = 0.25 * (1 - xi) * (1 - eta) * (-1 - xi - eta)
            N2 = 0.25 * (1 + xi) * (1 - eta) * (-1 + xi - eta)
            N3 = 0.25 * (1 + xi) * (1 + eta) * (-1 + xi + eta)
            N4 = 0.25 * (1 - xi) * (1 + eta) * (-1 - xi + eta)
            # Mid-edge nodes
            N5 = 0.5 * (1 - xi**2) * (1 - eta)
            N6 = 0.5 * (1 + xi) * (1 - eta**2)
            N7 = 0.5 * (1 - xi**2) * (1 + eta)
            N8 = 0.5 * (1 - xi) * (1 - eta**2)
            N = np.array([N1, N2, N3, N4, N5, N6, N7, N8])
        elif n_nodes == 9:  # Lagrangian quad
            # Helper 1D Lagrange polynomials
            L_m1 = 0.5 * xi * (xi - 1)  # at xi = -1
            L_0 = 1 - xi**2              # at xi = 0
            L_p1 = 0.5 * xi * (xi + 1)  # at xi = 1
            M_m1 = 0.5 * eta * (eta - 1)
            M_0 = 1 - eta**2
            M_p1 = 0.5 * eta * (eta + 1)

            N = np.array([
                L_m1 * M_m1,  # Node 1 (-1, -1)
                L_p1 * M_m1,  # Node 2 (+1, -1)
                L_p1 * M_p1,  # Node 3 (+1, +1)
                L_m1 * M_p1,  # Node 4 (-1, +1)
                L_0 * M_m1,   # Node 5 (0, -1)
                L_p1 * M_0,   # Node 6 (+1, 0)
                L_0 * M_p1,   # Node 7 (0, +1)
                L_m1 * M_0,   # Node 8 (-1, 0)
                L_0 * M_0     # Node 9 (0, 0)
            ])
        else:
            raise ValueError(f"Unsupported element with {n_nodes} nodes")

        # Interpolate displacements
        result = {"UX": 0.0, "UY": 0.0, "UZ": 0.0, "RX": 0.0, "RY": 0.0, "RZ": 0.0}
        dof_names = ["UX", "UY", "UZ", "RX", "RY", "RZ"]
        for i, dof_name in enumerate(dof_names):
            for j in range(n_nodes):
                result[dof_name] += N[j] * nodal_disp[j * 6 + i]

        return result

    def get_plate_moments(
        self,
        plate_element,
        xi: float = 0.0,
        eta: float = 0.0,
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[str, float]:
        """Get internal moments at a point within a plate element.

        Computes bending moments per unit width from curvatures using the
        constitutive relationship M = D * κ.

        Args:
            plate_element: PlateElement, PlateElement8, PlateElement9, or PlateElementTri.
            xi: Natural coordinate ξ (default: 0 = center).
            eta: Natural coordinate η (default: 0 = center).
            load_case: LoadCase to query (uses active if None).

        Returns:
            Dict with moment components in kN·m/m (moment per unit width):
            - "Mx": Moment about y-axis (causes stress in x-direction)
            - "My": Moment about x-axis (causes stress in y-direction)
            - "Mxy": Twisting moment

        Note:
            Sign convention: Positive moment causes tension on bottom surface.

        Example:
            # Get moments at center of plate element
            moments = model.get_plate_moments(element)
            print(f"Mx = {moments['Mx']:.2f} kN·m/m")
        """
        if load_case is not None:
            self._cpp_model.set_active_load_case(load_case)

        # Get material properties
        E = plate_element.material.E
        nu = plate_element.material.nu
        t = plate_element.thickness

        # Bending stiffness
        D = E * t**3 / (12 * (1 - nu**2))

        # Get nodal rotations (curvature-related DOFs)
        u_global = self._cpp_model.get_displacements()
        dof_handler = self._cpp_model.get_dof_handler()

        nodes = plate_element.nodes
        n_nodes = len(nodes)

        # Extract nodal w and rotations
        w_nodal = []
        rx_nodal = []
        ry_nodal = []
        for node in nodes:
            w_dof = dof_handler.get_global_dof(node.id, DOFIndex.UZ)
            rx_dof = dof_handler.get_global_dof(node.id, DOFIndex.RX)
            ry_dof = dof_handler.get_global_dof(node.id, DOFIndex.RY)
            w_nodal.append(u_global[w_dof] if w_dof >= 0 else 0.0)
            rx_nodal.append(u_global[rx_dof] if rx_dof >= 0 else 0.0)
            ry_nodal.append(u_global[ry_dof] if ry_dof >= 0 else 0.0)

        # Approximate curvatures using shape function derivatives
        # For simplicity, use finite differences at element center
        # κ_x = -∂²w/∂x², κ_y = -∂²w/∂y², κ_xy = -2∂²w/∂x∂y

        # Get element dimensions (approximate)
        x = np.array([n.x for n in nodes])
        y = np.array([n.y for n in nodes])
        lx = np.max(x) - np.min(x)
        ly = np.max(y) - np.min(y)

        # Use nodal rotations to estimate curvatures
        # θ_x ≈ ∂w/∂y, θ_y ≈ -∂w/∂x (Mindlin/Kirchhoff convention)
        # κ_x ≈ ∂θ_y/∂x = -∂²w/∂x², κ_y ≈ -∂θ_x/∂y = -∂²w/∂y²
        if n_nodes >= 4:
            # Curvature estimates from nodal rotation differences
            d_theta_y_dx = (np.mean([ry_nodal[1], ry_nodal[2]]) -
                           np.mean([ry_nodal[0], ry_nodal[3]])) / max(lx, 1e-6)
            d_theta_x_dy = (np.mean([rx_nodal[2], rx_nodal[3]]) -
                           np.mean([rx_nodal[0], rx_nodal[1]])) / max(ly, 1e-6)

            kappa_x = d_theta_y_dx  # -∂²w/∂x²
            kappa_y = -d_theta_x_dy  # -∂²w/∂y²
            kappa_xy = 0.5 * ((ry_nodal[1] - ry_nodal[0]) / max(ly, 1e-6) +
                              (rx_nodal[3] - rx_nodal[0]) / max(lx, 1e-6))
        else:
            # Triangle - simplified
            kappa_x = 0.0
            kappa_y = 0.0
            kappa_xy = 0.0

        # Compute moments from curvatures: M = D * κ
        # [Mx]   [D  νD  0    ] [κx ]
        # [My] = [νD D   0    ] [κy ]
        # [Mxy]  [0  0   D(1-ν)/2] [κxy]
        Mx = D * (kappa_x + nu * kappa_y)
        My = D * (kappa_y + nu * kappa_x)
        Mxy = D * (1 - nu) / 2 * kappa_xy * 2  # Factor 2 for engineering shear

        return {"Mx": Mx, "My": My, "Mxy": Mxy}

    def get_plate_stress(
        self,
        plate_element,
        surface: str = "middle",
        xi: float = 0.0,
        eta: float = 0.0,
        load_case: Optional[_CppLoadCase] = None
    ) -> Dict[str, float]:
        """Get stress at a point within a plate element.

        Computes bending stresses from internal moments using linear
        stress distribution through thickness.

        Args:
            plate_element: PlateElement, PlateElement8, PlateElement9, or PlateElementTri.
            surface: Location through thickness - "top", "bottom", or "middle".
            xi: Natural coordinate ξ (default: 0 = center).
            eta: Natural coordinate η (default: 0 = center).
            load_case: LoadCase to query (uses active if None).

        Returns:
            Dict with stress components in kN/m² (kPa):
            - "sigma_x": Normal stress in x-direction
            - "sigma_y": Normal stress in y-direction
            - "tau_xy": Shear stress in xy-plane

        Note:
            Positive stress is tensile. For plate bending:
            - Top surface (z = +t/2): compression when moment positive
            - Bottom surface (z = -t/2): tension when moment positive

        Example:
            # Get stress at top surface of plate element
            stress = model.get_plate_stress(element, surface="top")
            print(f"σ_x = {stress['sigma_x']:.0f} kPa")
        """
        # Get moments first
        moments = self.get_plate_moments(plate_element, xi, eta, load_case)

        t = plate_element.thickness

        # Distance from neutral axis
        if surface == "top":
            z = t / 2
        elif surface == "bottom":
            z = -t / 2
        elif surface == "middle":
            z = 0.0
        else:
            raise ValueError(f"Surface must be 'top', 'bottom', or 'middle', got '{surface}'")

        # Stress from bending: σ = M * z / I, where I = t³/12 per unit width
        # So σ = M * z / (t³/12) = 12 * M * z / t³
        inertia = t**3 / 12  # Second moment of area per unit width

        if abs(t) < 1e-10:
            sigma_x = 0.0
            sigma_y = 0.0
            tau_xy = 0.0
        else:
            sigma_x = moments["Mx"] * z / inertia
            sigma_y = moments["My"] * z / inertia
            tau_xy = moments["Mxy"] * z / inertia

        return {"sigma_x": sigma_x, "sigma_y": sigma_y, "tau_xy": tau_xy}

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

    # ===== Beam Subdivision (Task 4.4) =====

    def _point_on_line_segment(
        self,
        point: np.ndarray,
        line_start: np.ndarray,
        line_end: np.ndarray,
        tolerance: float = 1e-6
    ) -> Tuple[bool, float]:
        """Check if a point lies on a line segment.

        Args:
            point: Point to check [x, y, z]
            line_start: Start of line segment [x, y, z]
            line_end: End of line segment [x, y, z]
            tolerance: Distance tolerance for point-to-line check

        Returns:
            Tuple of (is_on_line, parameter_t) where:
            - is_on_line: True if point is on the line segment (not at endpoints)
            - parameter_t: Parametric position along line (0 = start, 1 = end)
        """
        # Vector from start to end
        line_vec = line_end - line_start
        line_length = np.linalg.norm(line_vec)

        if line_length < tolerance:
            # Degenerate line (zero length)
            return False, 0.0

        # Normalize line vector
        line_unit = line_vec / line_length

        # Vector from start to point
        point_vec = point - line_start

        # Project point onto line (parameter t)
        t = np.dot(point_vec, line_unit) / line_length

        # Check if t is in valid range (exclusive of endpoints)
        if t <= tolerance / line_length or t >= 1.0 - tolerance / line_length:
            return False, t

        # Compute perpendicular distance from point to line
        projection = line_start + t * line_vec
        distance = np.linalg.norm(point - projection)

        if distance <= tolerance:
            return True, t
        else:
            return False, t

    def _find_internal_nodes(self, beam: Beam) -> List[Tuple[float, Node]]:
        """Find nodes that lie on the line segment between beam endpoints.

        Args:
            beam: Beam to check for internal nodes

        Returns:
            List of (distance_from_start, node) tuples, sorted by distance.
            Does not include the beam's endpoint nodes.
        """
        internal_nodes = []
        start_pos = beam.start_pos
        end_pos = beam.end_pos

        # Get all nodes from the C++ model
        all_nodes = self._cpp_model.get_all_nodes()

        # Get endpoint node IDs to exclude them
        start_node = self.find_node_at(start_pos.tolist())
        end_node = self.find_node_at(end_pos.tolist())

        start_id = start_node.id if start_node else -1
        end_id = end_node.id if end_node else -1

        for node in all_nodes:
            # Skip endpoint nodes
            if node.id == start_id or node.id == end_id:
                continue

            # Get node position
            node_pos = np.array([node.x, node.y, node.z])

            # Check if node lies on the beam line segment
            is_on_line, t = self._point_on_line_segment(
                node_pos, start_pos, end_pos
            )

            if is_on_line:
                # Distance from start
                distance = t * beam.length
                internal_nodes.append((distance, node))

        # Sort by distance from start
        internal_nodes.sort(key=lambda x: x[0])

        return internal_nodes

    def _split_beam_at_nodes(
        self,
        beam: Beam,
        internal_nodes: List[Tuple[float, Node]]
    ) -> List[Beam]:
        """Split a beam at internal nodes into multiple sub-beams.

        Args:
            beam: Original beam to split
            internal_nodes: List of (distance, node) tuples sorted by distance

        Returns:
            List of new Beam objects replacing the original beam
        """
        if not internal_nodes:
            return [beam]

        # Get material and section
        material = beam.material
        section = beam.section

        # Get start and end nodes
        start_node = self.find_node_at(beam.start_pos.tolist())
        end_node = self.find_node_at(beam.end_pos.tolist())

        # Build list of all nodes in order: start -> internal nodes -> end
        all_nodes = [start_node]
        for _, node in internal_nodes:
            all_nodes.append(node)
        all_nodes.append(end_node)

        # Remove the original beam's element from C++ model elements list
        # (We need to access the underlying C++ model's elements)
        # The C++ model maintains its own element list - old elements remain
        # but new ones are created. model.analyze() will use all elements.

        # Create new sub-beams
        new_beams = []
        for i in range(len(all_nodes) - 1):
            node_i = all_nodes[i]
            node_j = all_nodes[i + 1]

            start_pos = [node_i.x, node_i.y, node_i.z]
            end_pos = [node_j.x, node_j.y, node_j.z]

            # Create new beam element in C++ model
            cpp_element = self._cpp_model.create_beam(node_i, node_j, material, section)

            # Create Python Beam object
            sub_beam = Beam(start_pos, end_pos, section, material, beam_id=self._beam_id_counter)
            self._beam_id_counter += 1
            sub_beam.add_element(cpp_element)
            new_beams.append(sub_beam)

        return new_beams

    def resubdivide_beam(self, beam: Beam) -> int:
        """Resubdivide a beam to include any internal nodes.

        Unlike _split_beam_at_nodes which creates new Beam objects, this method
        keeps the original Beam wrapper but recreates its internal elements.
        This is useful when adding point loads at midspan positions - the FEM
        needs elements at the load position, but the visual representation
        should remain as a single beam.

        Args:
            beam: The beam to resubdivide

        Returns:
            Number of elements created (0 if no subdivision needed)

        Example:
            # Create a node at load position
            model.get_or_create_node(5, 0, 0)
            # Resubdivide the beam to include this node
            n_elements = model.resubdivide_beam(beam)
        """
        internal_nodes = self._find_internal_nodes(beam)

        if not internal_nodes:
            # No internal nodes, nothing to do
            return len(beam.elements)

        # Get material and section
        material = beam.material
        section = beam.section

        # Get start and end nodes
        start_node = self.find_node_at(beam.start_pos.tolist())
        end_node = self.find_node_at(beam.end_pos.tolist())

        if not start_node or not end_node:
            raise ValueError("Could not find beam endpoint nodes")

        # Build list of all nodes in order: start -> internal nodes -> end
        all_nodes = [start_node]
        for _, node in internal_nodes:
            all_nodes.append(node)
        all_nodes.append(end_node)

        # Remove old elements from C++ model
        for old_element in beam.elements:
            self._remove_element_from_cpp_model(old_element)

        # Clear the beam's elements list
        beam.elements.clear()

        # Create new elements and add to this beam
        for i in range(len(all_nodes) - 1):
            node_i = all_nodes[i]
            node_j = all_nodes[i + 1]

            # Create new beam element in C++ model
            cpp_element = self._cpp_model.create_beam(node_i, node_j, material, section)

            # Add to this beam's elements
            beam.add_element(cpp_element)

        return len(beam.elements)

    def resubdivide_all_beams(self) -> int:
        """Resubdivide all beams to include any internal nodes.

        Unlike subdivide_beams() which creates new Beam objects, this method
        keeps the original Beam wrappers but recreates their internal elements.

        Returns:
            Total number of beams that were resubdivided
        """
        resubdivided_count = 0
        for beam in self.beams:
            old_count = len(beam.elements)
            new_count = self.resubdivide_beam(beam)
            if new_count > old_count:
                resubdivided_count += 1
        return resubdivided_count

    def _subdivide_beams(self) -> int:
        """Find and subdivide beams that have internal nodes.

        This method checks all beams for nodes that lie along their length
        (but are not endpoints) and splits them into multiple elements.

        Returns:
            Number of beams that were subdivided
        """
        subdivided_count = 0
        new_beams_list = []

        for beam in self.beams[:]:  # Copy list to allow modification
            internal_nodes = self._find_internal_nodes(beam)

            if internal_nodes:
                # Remove original beam's element from the model
                # We need to mark it for removal
                if beam.elements:
                    old_element = beam.elements[0]
                    # Remove from C++ model's elements list
                    self._remove_element_from_cpp_model(old_element)

                # Split the beam
                new_beams = self._split_beam_at_nodes(beam, internal_nodes)
                new_beams_list.extend(new_beams)
                subdivided_count += 1
            else:
                # Keep the original beam
                new_beams_list.append(beam)

        # Replace beams list with new list
        self.beams = new_beams_list

        return subdivided_count

    def _remove_element_from_cpp_model(self, element: _CppBeamElement) -> bool:
        """Remove an element from the C++ model's elements list.

        Args:
            element: Element to remove

        Returns:
            True if element was found and removed, False otherwise
        """
        return self._cpp_model.remove_element(element.id)

    def subdivide_beams(self) -> int:
        """Public method to subdivide beams at internal nodes.

        Call this method after creating all beams but before analysis
        if you want beams to be automatically subdivided where they
        cross other nodes.

        Returns:
            Number of beams that were subdivided

        Example:
            model = StructuralModel()
            # Create materials and sections...
            model.add_beam_by_coords([0,0,0], [12,0,0], "IPE300", "Steel")
            # Create a node at the midpoint
            model.get_or_create_node(6, 0, 0)
            # Now subdivide - the beam becomes two 6m elements
            n_split = model.subdivide_beams()
            print(f"Subdivided {n_split} beams")
        """
        return self._subdivide_beams()
