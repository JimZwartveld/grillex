"""
Plate geometry and meshing data structures.

This module provides the Plate class for defining plate regions that
can be meshed into multiple plate elements.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Literal, TYPE_CHECKING
import numpy as np

if TYPE_CHECKING:
    from .model_wrapper import Beam


@dataclass
class EdgeMeshControl:
    """Mesh control for a plate edge.

    Attributes:
        n_elements: Number of elements along edge. If specified, takes
            precedence over the plate's mesh_size for this edge.
    """
    n_elements: Optional[int] = None
    # Future: bias, grading, etc.


@dataclass
class PlateBeamCoupling:
    """Coupling between a plate edge and a beam.

    Attributes:
        plate: The Plate object.
        edge_index: Edge index (0 = from corner[0] to corner[1], etc.)
        beam: The Beam object to couple to.
        offset: [dx, dy, dz] offset from plate node to beam centroid in meters.
        releases: Dict of DOF releases. Keys: "UX", "UY", "UZ", "RX", "RY", "RZ",
            or "R_EDGE" for rotation about the edge. Values: True = released.
    """
    plate: "Plate"
    edge_index: int
    beam: "Beam"
    offset: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    releases: Dict[str, bool] = field(default_factory=dict)


@dataclass
class SupportCurve:
    """Boundary condition along a plate edge.

    Attributes:
        plate: The Plate object.
        edge_index: Edge index for the support.
        ux: If True, restrain X translation.
        uy: If True, restrain Y translation.
        uz: If True, restrain Z translation.
        rotation_about_edge: If True, restrain rotation about the edge direction.
    """
    plate: "Plate"
    edge_index: int
    ux: bool = False
    uy: bool = False
    uz: bool = False
    rotation_about_edge: bool = False


@dataclass
class Plate:
    """
    Represents a plate region for meshing.

    A plate is defined by 3 or more corner points forming a closed polygon.
    The plate is meshed into quad (preferred) or triangular elements.

    Attributes:
        corners: List of [x, y, z] coordinates for plate corners.
            Must have at least 3 points. Counter-clockwise when viewed
            from the positive normal direction.
        thickness: Plate thickness in meters.
        material: Name of material to use.
        mesh_size: Target element size in meters (default 0.5m).
        element_type: Element type: "MITC4", "MITC8", "MITC9", or "DKT".
        name: Optional name for the plate.
        edge_controls: Per-edge mesh controls.
        beam_couplings: List of plate-beam couplings.
        support_curves: List of support curves.

    Example:
        >>> plate = Plate(  # doctest: +SKIP
        ...     corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
        ...     thickness=0.02,
        ...     material="Steel",
        ...     mesh_size=0.5
        ... )
        >>> plate.n_corners  # doctest: +SKIP
        4
        >>> plate.is_planar()  # doctest: +SKIP
        True
    """
    corners: List[List[float]]
    thickness: float
    material: str
    mesh_size: float = 0.5
    element_type: Literal["MITC4", "MITC8", "MITC9", "DKT"] = "MITC4"
    name: Optional[str] = None

    # Edge-specific mesh control (index corresponds to edge from corner[i] to corner[i+1])
    edge_controls: Dict[int, EdgeMeshControl] = field(default_factory=dict)

    # Coupling to beams (populated by couple_plate_to_beam)
    beam_couplings: List[PlateBeamCoupling] = field(default_factory=list)

    # Support curves (populated by add_support_curve)
    support_curves: List[SupportCurve] = field(default_factory=list)

    # Generated mesh data (populated by mesh())
    _mesh_nodes: Optional[np.ndarray] = field(default=None, repr=False)
    _mesh_elements: Optional[np.ndarray] = field(default=None, repr=False)
    _edge_node_map: Optional[Dict[int, List[int]]] = field(default=None, repr=False)

    def __post_init__(self):
        """Validate plate geometry after initialization."""
        if len(self.corners) < 3:
            raise ValueError("Plate requires at least 3 corners")

        # Ensure corners are lists (not tuples or numpy arrays)
        self.corners = [list(c) for c in self.corners]

        # Validate all corners have 3 coordinates
        for i, corner in enumerate(self.corners):
            if len(corner) != 3:
                raise ValueError(f"Corner {i} must have 3 coordinates, got {len(corner)}")

    @property
    def n_corners(self) -> int:
        """Number of corner points."""
        return len(self.corners)

    @property
    def n_edges(self) -> int:
        """Number of edges (same as corners for closed polygon)."""
        return len(self.corners)

    def get_edge(self, index: int) -> tuple:
        """Get edge as (start_point, end_point) tuple.

        Args:
            index: Edge index (0-based). Wraps around if negative or >= n_edges.

        Returns:
            Tuple of (start_point, end_point) as lists.
        """
        i = index % self.n_corners
        j = (index + 1) % self.n_corners
        return (self.corners[i], self.corners[j])

    def get_edge_length(self, index: int) -> float:
        """Get length of edge in meters.

        Args:
            index: Edge index (0-based).

        Returns:
            Edge length in meters.
        """
        p1, p2 = self.get_edge(index)
        return float(np.linalg.norm(np.array(p2) - np.array(p1)))

    def get_edge_direction(self, index: int) -> np.ndarray:
        """Get unit direction vector of edge.

        Args:
            index: Edge index (0-based).

        Returns:
            Unit direction vector as numpy array.
        """
        p1, p2 = self.get_edge(index)
        vec = np.array(p2) - np.array(p1)
        length = np.linalg.norm(vec)
        if length < 1e-10:
            raise ValueError(f"Edge {index} has zero length")
        return vec / length

    def get_edge_midpoint(self, index: int) -> np.ndarray:
        """Get midpoint of edge.

        Args:
            index: Edge index (0-based).

        Returns:
            Midpoint coordinates as numpy array.
        """
        p1, p2 = self.get_edge(index)
        return (np.array(p1) + np.array(p2)) / 2

    def get_normal(self) -> np.ndarray:
        """Get plate normal vector (assuming planar).

        Uses the first three corners to compute the normal via cross product.
        The normal direction follows the right-hand rule for counter-clockwise
        corner ordering.

        Returns:
            Unit normal vector as numpy array.

        Raises:
            ValueError: If corners are collinear (degenerate plate).
        """
        p0 = np.array(self.corners[0])
        p1 = np.array(self.corners[1])
        p2 = np.array(self.corners[2])

        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)

        length = np.linalg.norm(normal)
        if length < 1e-10:
            raise ValueError("Plate corners are collinear (degenerate plate)")

        return normal / length

    def is_planar(self, tolerance: float = 1e-6) -> bool:
        """Check if all corners lie in the same plane.

        Args:
            tolerance: Maximum distance from plane in meters.

        Returns:
            True if all corners are coplanar within tolerance.
        """
        if self.n_corners <= 3:
            return True

        try:
            normal = self.get_normal()
        except ValueError:
            return False  # Degenerate plate

        p0 = np.array(self.corners[0])

        for corner in self.corners[3:]:
            dist = abs(np.dot(np.array(corner) - p0, normal))
            if dist > tolerance:
                return False

        return True

    def get_area(self) -> float:
        """Get approximate plate area using shoelace formula projected to plane.

        For non-planar plates, this is an approximation.

        Returns:
            Plate area in square meters.
        """
        if self.n_corners < 3:
            return 0.0

        # Project to local 2D coordinates
        corners_3d = np.array(self.corners)

        # Use first edge as local x-axis
        origin = corners_3d[0]
        x_axis = self.get_edge_direction(0)
        normal = self.get_normal()
        y_axis = np.cross(normal, x_axis)

        # Project to 2D
        corners_2d = []
        for corner in corners_3d:
            rel = corner - origin
            x = np.dot(rel, x_axis)
            y = np.dot(rel, y_axis)
            corners_2d.append([x, y])

        # Shoelace formula
        n = len(corners_2d)
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += corners_2d[i][0] * corners_2d[j][1]
            area -= corners_2d[j][0] * corners_2d[i][1]

        return abs(area) / 2.0

    def get_centroid(self) -> np.ndarray:
        """Get plate centroid (average of corners).

        Returns:
            Centroid coordinates as numpy array.
        """
        return np.mean(self.corners, axis=0)

    def get_bounding_box(self) -> tuple:
        """Get axis-aligned bounding box.

        Returns:
            Tuple of (min_coords, max_coords) as numpy arrays.
        """
        corners = np.array(self.corners)
        return (corners.min(axis=0), corners.max(axis=0))

    def is_convex(self) -> bool:
        """Check if the plate polygon is convex.

        Returns:
            True if all interior angles are less than 180 degrees.
        """
        if self.n_corners < 3:
            return False
        if self.n_corners == 3:
            return True

        # Project to 2D for convexity check
        corners_3d = np.array(self.corners)
        origin = corners_3d[0]
        x_axis = self.get_edge_direction(0)
        normal = self.get_normal()
        y_axis = np.cross(normal, x_axis)

        corners_2d = []
        for corner in corners_3d:
            rel = corner - origin
            x = np.dot(rel, x_axis)
            y = np.dot(rel, y_axis)
            corners_2d.append([x, y])

        # Check cross product sign consistency
        n = len(corners_2d)
        sign = None

        for i in range(n):
            p0 = corners_2d[i]
            p1 = corners_2d[(i + 1) % n]
            p2 = corners_2d[(i + 2) % n]

            dx1 = p1[0] - p0[0]
            dy1 = p1[1] - p0[1]
            dx2 = p2[0] - p1[0]
            dy2 = p2[1] - p1[1]

            cross = dx1 * dy2 - dy1 * dx2

            if abs(cross) < 1e-10:
                continue  # Collinear points

            current_sign = cross > 0

            if sign is None:
                sign = current_sign
            elif sign != current_sign:
                return False

        return True

    def __repr__(self) -> str:
        return (
            f"Plate(name={self.name!r}, n_corners={self.n_corners}, "
            f"thickness={self.thickness}, material={self.material!r}, "
            f"mesh_size={self.mesh_size}, element_type={self.element_type!r})"
        )
