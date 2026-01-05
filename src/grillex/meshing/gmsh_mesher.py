"""
Mesh generation using gmsh.

This module provides the GmshPlateMesher class for generating structured
and unstructured meshes for plate regions.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple


class MeshingError(Exception):
    """Exception raised for meshing errors."""
    pass


@dataclass
class MeshResult:
    """Result of mesh generation.

    Attributes:
        nodes: Node coordinates, shape (n_nodes, 3).
        node_tags: Gmsh node tags, shape (n_nodes,).
        quads: Quad element connectivity (node indices), shape (n_quads, 4).
        triangles: Triangle element connectivity (node indices), shape (n_tris, 3).
        quads_8: 8-node quad connectivity for quadratic elements, shape (n_quads8, 8).
        quads_9: 9-node quad connectivity for quadratic elements, shape (n_quads9, 9).
        triangles_6: 6-node triangle connectivity for quadratic elements, shape (n_tris6, 6).
        edge_nodes: Dict mapping edge index to list of node indices on that edge.
    """
    nodes: np.ndarray
    node_tags: np.ndarray
    quads: np.ndarray = field(default_factory=lambda: np.zeros((0, 4), dtype=int))
    triangles: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=int))
    quads_8: np.ndarray = field(default_factory=lambda: np.zeros((0, 8), dtype=int))
    quads_9: np.ndarray = field(default_factory=lambda: np.zeros((0, 9), dtype=int))
    triangles_6: np.ndarray = field(default_factory=lambda: np.zeros((0, 6), dtype=int))
    edge_nodes: Dict[int, List[int]] = field(default_factory=dict)

    @property
    def n_nodes(self) -> int:
        """Total number of nodes."""
        return len(self.nodes)

    @property
    def n_elements(self) -> int:
        """Total number of elements (quads + triangles)."""
        return (
            self.quads.shape[0] +
            self.triangles.shape[0] +
            self.quads_8.shape[0] +
            self.quads_9.shape[0] +
            self.triangles_6.shape[0]
        )

    @property
    def n_quads(self) -> int:
        """Number of quad elements (all types)."""
        return self.quads.shape[0] + self.quads_8.shape[0] + self.quads_9.shape[0]

    @property
    def n_triangles(self) -> int:
        """Number of triangle elements (all types)."""
        return self.triangles.shape[0] + self.triangles_6.shape[0]


def _check_gmsh_available():
    """Check if gmsh is available and provide helpful error if not."""
    try:
        import gmsh
        return gmsh
    except ImportError:
        raise MeshingError(
            "gmsh is required for plate meshing but is not installed.\n"
            "Install with: pip install gmsh\n"
            "Or install grillex with meshing support: pip install grillex[meshing]"
        )


class GmshPlateMesher:
    """Mesh generator for plate regions using gmsh.

    This class provides methods to generate structured and unstructured
    meshes for polygonal plate regions using the gmsh library.

    Features:
        - Quad-dominant meshing with triangle fallback
        - Structured meshing for 4-sided regions with matching edge divisions
        - Support for general polygons (3+ corners)
        - Per-edge element count control
        - Linear and quadratic element support

    Example:
        >>> mesher = GmshPlateMesher()  # doctest: +SKIP
        >>> result = mesher.mesh_plate(  # doctest: +SKIP
        ...     corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
        ...     mesh_size=0.5,
        ...     prefer_quads=True
        ... )
        >>> print(f"Generated {result.n_quads} quads, {result.n_triangles} triangles")  # doctest: +SKIP
        >>> mesher.finalize()  # doctest: +SKIP

    Note:
        Always call finalize() when done to release gmsh resources,
        or use as a context manager:

        >>> with GmshPlateMesher() as mesher:  # doctest: +SKIP
        ...     result = mesher.mesh_plate(...)  # doctest: +SKIP
    """

    def __init__(self):
        """Initialize the mesher (lazy initialization of gmsh)."""
        self._gmsh = None
        self._initialized = False

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - finalize gmsh."""
        self.finalize()
        return False

    def _ensure_initialized(self):
        """Ensure gmsh is initialized."""
        if not self._initialized:
            self._gmsh = _check_gmsh_available()
            self._gmsh.initialize()
            self._gmsh.option.setNumber("General.Terminal", 0)  # Suppress output
            self._initialized = True

    def mesh_plate(
        self,
        corners: List[List[float]],
        mesh_size: float,
        edge_divisions: Optional[Dict[int, int]] = None,
        prefer_quads: bool = True,
        element_order: int = 1
    ) -> MeshResult:
        """Generate mesh for a plate defined by corner points.

        Args:
            corners: List of [x, y, z] corner coordinates. At least 3 corners
                required. Corners should be ordered counter-clockwise when
                viewed from the positive normal direction.
            mesh_size: Target element size in meters.
            edge_divisions: Optional dict mapping edge index to number of
                divisions along that edge. Edge i goes from corners[i] to
                corners[(i+1) % n]. When specified, takes precedence over
                mesh_size for that edge.
            prefer_quads: If True, attempt to generate quad-dominant mesh.
                Default True.
            element_order: Polynomial order of elements. 1 for linear (4-node
                quads, 3-node triangles), 2 for quadratic (8/9-node quads,
                6-node triangles). Default 1.

        Returns:
            MeshResult containing nodes and element connectivity.

        Raises:
            MeshingError: If meshing fails.
            ValueError: If corners are invalid.
        """
        if len(corners) < 3:
            raise ValueError("At least 3 corners are required for a plate")

        # Validate corner coordinates before trying to mesh
        for i, corner in enumerate(corners):
            if len(corner) != 3:
                raise ValueError(f"Each corner must have 3 coordinates, got {len(corner)}")

        self._ensure_initialized()
        gmsh = self._gmsh

        # Clear any previous geometry
        gmsh.clear()
        gmsh.model.add("plate")

        try:
            # Create points
            point_tags = []
            for corner in corners:
                tag = gmsh.model.geo.addPoint(corner[0], corner[1], corner[2], mesh_size)
                point_tags.append(tag)

            # Create lines (edges)
            line_tags = []
            n = len(corners)
            for i in range(n):
                p1 = point_tags[i]
                p2 = point_tags[(i + 1) % n]
                line_tag = gmsh.model.geo.addLine(p1, p2)
                line_tags.append(line_tag)

                # Apply edge divisions if specified
                if edge_divisions and i in edge_divisions:
                    n_div = edge_divisions[i]
                    if n_div < 1:
                        raise ValueError(f"Edge {i} must have at least 1 division, got {n_div}")
                    # setTransfiniteCurve takes number of nodes = n_div + 1
                    gmsh.model.geo.mesh.setTransfiniteCurve(line_tag, n_div + 1)

            # Create curve loop and surface
            loop_tag = gmsh.model.geo.addCurveLoop(line_tags)
            surface_tag = gmsh.model.geo.addPlaneSurface([loop_tag])

            # For structured quad mesh: need 4 corners with matching opposite edge divisions
            can_use_transfinite = (
                prefer_quads and
                len(corners) == 4 and
                edge_divisions is not None and
                all(i in edge_divisions for i in range(4)) and
                edge_divisions.get(0) == edge_divisions.get(2) and
                edge_divisions.get(1) == edge_divisions.get(3)
            )

            if can_use_transfinite:
                # Use structured (transfinite) meshing for pure quads
                gmsh.model.geo.mesh.setTransfiniteSurface(surface_tag)
                gmsh.model.geo.mesh.setRecombine(2, surface_tag)

            gmsh.model.geo.synchronize()

            # Mesh algorithm settings for quad preference
            if prefer_quads:
                # Algorithm 8 = Frontal-Delaunay for quads
                gmsh.option.setNumber("Mesh.Algorithm", 8)
                gmsh.option.setNumber("Mesh.RecombineAll", 1)
                # Algorithm 2 = Blossom recombination (best for quads)
                gmsh.option.setNumber("Mesh.RecombinationAlgorithm", 2)
            else:
                # Default Delaunay for triangles
                gmsh.option.setNumber("Mesh.Algorithm", 6)
                gmsh.option.setNumber("Mesh.RecombineAll", 0)

            # Element order
            gmsh.option.setNumber("Mesh.ElementOrder", element_order)

            # Generate 2D mesh
            gmsh.model.mesh.generate(2)

            # Extract mesh data
            return self._extract_mesh_data(line_tags, element_order)

        except Exception as e:
            raise MeshingError(f"Meshing failed: {e}") from e

    def _extract_mesh_data(
        self,
        edge_line_tags: List[int],
        element_order: int
    ) -> MeshResult:
        """Extract nodes and elements from gmsh.

        Args:
            edge_line_tags: List of gmsh line tags for edges.
            element_order: Element polynomial order (1 or 2).

        Returns:
            MeshResult with extracted mesh data.
        """
        gmsh = self._gmsh

        # Get all nodes
        node_tags, node_coords, _ = gmsh.model.mesh.getNodes()
        n_nodes = len(node_tags)

        if n_nodes == 0:
            raise MeshingError("Mesh generation produced no nodes")

        nodes = np.array(node_coords).reshape(n_nodes, 3)

        # Create tag to index mapping
        tag_to_idx = {int(tag): i for i, tag in enumerate(node_tags)}

        # Initialize element arrays
        quads = np.zeros((0, 4), dtype=int)
        triangles = np.zeros((0, 3), dtype=int)
        quads_8 = np.zeros((0, 8), dtype=int)
        quads_9 = np.zeros((0, 9), dtype=int)
        triangles_6 = np.zeros((0, 6), dtype=int)

        # Element type codes in gmsh:
        # 2 = 3-node triangle
        # 3 = 4-node quad
        # 9 = 6-node triangle (quadratic)
        # 10 = 9-node quad (quadratic)
        # 16 = 8-node quad (serendipity)

        # Get linear elements
        try:
            tri_tags, tri_conn = gmsh.model.mesh.getElementsByType(2)
            if len(tri_tags) > 0:
                n_tris = len(tri_tags)
                tri_conn = np.array(tri_conn, dtype=int).reshape(n_tris, 3)
                triangles = np.array([[tag_to_idx[t] for t in elem] for elem in tri_conn])
        except Exception:
            pass

        try:
            quad_tags, quad_conn = gmsh.model.mesh.getElementsByType(3)
            if len(quad_tags) > 0:
                n_quads = len(quad_tags)
                quad_conn = np.array(quad_conn, dtype=int).reshape(n_quads, 4)
                quads = np.array([[tag_to_idx[t] for t in elem] for elem in quad_conn])
        except Exception:
            pass

        # Get quadratic elements if element_order == 2
        if element_order == 2:
            try:
                tri6_tags, tri6_conn = gmsh.model.mesh.getElementsByType(9)
                if len(tri6_tags) > 0:
                    n_tris6 = len(tri6_tags)
                    tri6_conn = np.array(tri6_conn, dtype=int).reshape(n_tris6, 6)
                    triangles_6 = np.array([[tag_to_idx[t] for t in elem] for elem in tri6_conn])
            except Exception:
                pass

            try:
                quad9_tags, quad9_conn = gmsh.model.mesh.getElementsByType(10)
                if len(quad9_tags) > 0:
                    n_quads9 = len(quad9_tags)
                    quad9_conn = np.array(quad9_conn, dtype=int).reshape(n_quads9, 9)
                    quads_9 = np.array([[tag_to_idx[t] for t in elem] for elem in quad9_conn])
            except Exception:
                pass

            try:
                quad8_tags, quad8_conn = gmsh.model.mesh.getElementsByType(16)
                if len(quad8_tags) > 0:
                    n_quads8 = len(quad8_tags)
                    quad8_conn = np.array(quad8_conn, dtype=int).reshape(n_quads8, 8)
                    quads_8 = np.array([[tag_to_idx[t] for t in elem] for elem in quad8_conn])
            except Exception:
                pass

        # Get nodes on each edge (including corner nodes)
        edge_nodes: Dict[int, List[int]] = {}
        for i, line_tag in enumerate(edge_line_tags):
            try:
                # Get nodes on the line (dimension 1) - includes interior and boundary nodes
                # Use includeBoundary=True to include corner nodes
                edge_node_tags, _, _ = gmsh.model.mesh.getNodes(1, line_tag, includeBoundary=True)
                edge_nodes[i] = [tag_to_idx[int(t)] for t in edge_node_tags]
            except Exception:
                edge_nodes[i] = []

        return MeshResult(
            nodes=nodes,
            node_tags=np.array(node_tags),
            quads=quads,
            triangles=triangles,
            quads_8=quads_8,
            quads_9=quads_9,
            triangles_6=triangles_6,
            edge_nodes=edge_nodes
        )

    def mesh_plate_from_geometry(
        self,
        plate: "Plate"
    ) -> MeshResult:
        """Generate mesh from a Plate geometry object.

        Convenience method that extracts parameters from a Plate object.

        Args:
            plate: Plate geometry object with corners, mesh_size, and
                edge_controls.

        Returns:
            MeshResult with generated mesh.
        """
        # Extract edge divisions from plate edge controls
        edge_divisions = {}
        for edge_idx, control in plate.edge_controls.items():
            if control.n_elements is not None:
                edge_divisions[edge_idx] = control.n_elements

        # Determine element order from element type
        element_order = 1
        if plate.element_type in ("MITC8", "MITC9"):
            element_order = 2

        return self.mesh_plate(
            corners=plate.corners,
            mesh_size=plate.mesh_size,
            edge_divisions=edge_divisions if edge_divisions else None,
            prefer_quads=(plate.element_type != "DKT"),
            element_order=element_order
        )

    def finalize(self):
        """Clean up gmsh resources.

        Should be called when done with meshing to release resources.
        Alternatively, use the mesher as a context manager.
        """
        if self._initialized and self._gmsh is not None:
            try:
                self._gmsh.finalize()
            except Exception:
                pass  # Ignore errors during finalization
            self._initialized = False
            self._gmsh = None
