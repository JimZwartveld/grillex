## Phase 19: Plate Meshing & Plate-Beam Coupling

### Overview

This phase implements comprehensive plate meshing capabilities using gmsh, including support for general polygon plates, mesh control, plate-to-beam coupling, and support curves. The existing MITC4 plate element is exposed in the Python API and extended with higher-order element options.

**Key Features:**
1. **General Polygon Plates** - Define plates with 3+ corners, automatically decomposed and meshed
2. **Gmsh Integration** - Industry-standard mesh generation with quad preference
3. **Higher-Order Elements** - 8-node and 9-node quadrilateral plate elements
4. **Mesh Control** - Element size and per-edge element count control
5. **Plate-Beam Coupling** - Rigid links with DOF release control and eccentric offsets
6. **Support Curves** - Boundary conditions along plate edges
7. **Unified Meshing** - Single `mesh()` function handling both beams and plates

**Architecture:**
```
Python StructuralModel
    │
    ├── add_plate(corners, thickness, material, mesh_size, element_type)
    │       │
    │       └── Plate object (stores geometry, mesh properties)
    │
    ├── set_edge_divisions(plate, edge_index, n_elements)
    │
    ├── couple_plate_to_beam(plate, edge_index, beam, releases, offset)
    │       │
    │       └── Creates rigid links with offset transformation
    │
    ├── add_support_curve(plate, edge_index, ux, uy, uz, rotation_about_edge)
    │
    └── mesh()  # Unified meshing for plates and beams
            │
            ├── Generate plate meshes via gmsh
            ├── Create PlateElement objects for each quad/tri
            ├── Create beam nodes at plate edge nodes (if coupled)
            └── Subdivide beams at internal nodes
```

**Dependencies:**
- Phase 8 complete (PlateElement exists in C++)
- gmsh Python package (`pip install gmsh`)

---

### Task 19.1: Python API for Existing Plate Element ✓

**Requirements:** R-ELEM-004
**Dependencies:** Phase 8 (PlateElement in C++)
**Difficulty:** Low
**Status:** Complete

**Description:**
Expose the existing MITC4 plate element in the Python StructuralModel API with a method to create individual plate elements.

**Steps:**

1. Add `add_plate_element()` method to StructuralModel:
   ```python
   def add_plate_element(
       self,
       node1: List[float],
       node2: List[float],
       node3: List[float],
       node4: List[float],
       thickness: float,
       material: str
   ) -> PlateElement:
       """
       Add a single 4-node plate element to the model.

       Args:
           node1-node4: Corner coordinates [x, y, z] in meters (counter-clockwise).
           thickness: Plate thickness in meters.
           material: Name of material to use.

       Returns:
           The created PlateElement object.

       Raises:
           ValueError: If material not found.
       """
   ```

2. Implement the method using the C++ `model.create_plate()`:
   ```python
   def add_plate_element(self, node1, node2, node3, node4, thickness, material):
       mat = self._get_material(material)
       if mat is None:
           raise ValueError(f"Material '{material}' not found")

       n1 = self._cpp_model.get_or_create_node(*node1)
       n2 = self._cpp_model.get_or_create_node(*node2)
       n3 = self._cpp_model.get_or_create_node(*node3)
       n4 = self._cpp_model.get_or_create_node(*node4)

       plate = self._cpp_model.create_plate(n1, n2, n3, n4, thickness, mat)
       return plate
   ```

3. Add plate element tracking in StructuralModel:
   ```python
   def get_plate_elements(self) -> List[PlateElement]:
       """Return all plate elements in the model."""
       return list(self._cpp_model.plate_elements)
   ```

4. Write unit tests:
   ```python
   def test_add_plate_element():
       model = StructuralModel(name="PlateTest")
       model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

       plate = model.add_plate_element(
           [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
           thickness=0.01, material="Steel"
       )

       assert plate is not None
       assert len(model.get_plate_elements()) == 1
   ```

**Acceptance Criteria:**
- [ ] `add_plate_element()` creates MITC4 plate via C++ API
- [ ] Plate element has correct thickness and material
- [ ] Nodes automatically created/merged at corners
- [ ] `get_plate_elements()` returns all plates
- [ ] Error raised if material not found
- [ ] Unit tests pass

---

### Task 19.2: Plate Geometry Class ✓

**Requirements:** R-MOD-007
**Dependencies:** None
**Difficulty:** Medium
**Status:** Complete

**Description:**
Create a `Plate` class to represent a plate region defined by corner points, storing mesh properties and providing geometry utilities.

**Steps:**

1. Create `src/grillex/core/plate.py`:
   ```python
   from dataclasses import dataclass, field
   from typing import List, Optional, Dict, Literal
   import numpy as np

   @dataclass
   class EdgeMeshControl:
       """Mesh control for a plate edge."""
       n_elements: Optional[int] = None  # Number of elements along edge
       # Future: bias, grading, etc.

   @dataclass
   class Plate:
       """
       Represents a plate region for meshing.

       A plate is defined by 3 or more corner points forming a closed polygon.
       The plate is meshed into quad (preferred) or triangular elements.
       """
       corners: List[List[float]]  # List of [x, y, z] coordinates
       thickness: float  # Plate thickness in meters
       material: str  # Material name
       mesh_size: float = 0.5  # Target element size in meters
       element_type: Literal["MITC4", "MITC8", "MITC9"] = "MITC4"
       name: Optional[str] = None

       # Edge-specific mesh control (index corresponds to edge from corner[i] to corner[i+1])
       edge_controls: Dict[int, EdgeMeshControl] = field(default_factory=dict)

       # Coupling to beams (populated by couple_plate_to_beam)
       beam_couplings: List["PlateBeamCoupling"] = field(default_factory=list)

       # Support curves (populated by add_support_curve)
       support_curves: List["SupportCurve"] = field(default_factory=list)

       # Generated mesh data (populated by mesh())
       _mesh_nodes: Optional[np.ndarray] = field(default=None, repr=False)
       _mesh_elements: Optional[np.ndarray] = field(default=None, repr=False)

       def __post_init__(self):
           if len(self.corners) < 3:
               raise ValueError("Plate requires at least 3 corners")
           self.corners = [list(c) for c in self.corners]

       @property
       def n_corners(self) -> int:
           """Number of corner points."""
           return len(self.corners)

       @property
       def n_edges(self) -> int:
           """Number of edges (same as corners for closed polygon)."""
           return len(self.corners)

       def get_edge(self, index: int) -> tuple:
           """Get edge as (start_point, end_point) tuple."""
           i = index % self.n_corners
           j = (index + 1) % self.n_corners
           return (self.corners[i], self.corners[j])

       def get_edge_length(self, index: int) -> float:
           """Get length of edge in meters."""
           p1, p2 = self.get_edge(index)
           return np.linalg.norm(np.array(p2) - np.array(p1))

       def get_edge_direction(self, index: int) -> np.ndarray:
           """Get unit direction vector of edge."""
           p1, p2 = self.get_edge(index)
           vec = np.array(p2) - np.array(p1)
           return vec / np.linalg.norm(vec)

       def get_normal(self) -> np.ndarray:
           """Get plate normal vector (assuming planar)."""
           p0 = np.array(self.corners[0])
           p1 = np.array(self.corners[1])
           p2 = np.array(self.corners[2])
           v1 = p1 - p0
           v2 = p2 - p0
           normal = np.cross(v1, v2)
           return normal / np.linalg.norm(normal)

       def is_planar(self, tolerance: float = 1e-6) -> bool:
           """Check if all corners lie in the same plane."""
           if self.n_corners <= 3:
               return True
           normal = self.get_normal()
           p0 = np.array(self.corners[0])
           for corner in self.corners[3:]:
               dist = abs(np.dot(np.array(corner) - p0, normal))
               if dist > tolerance:
                   return False
           return True
   ```

2. Add Plate to exports in `__init__.py`:
   ```python
   from .plate import Plate, EdgeMeshControl
   ```

3. Write unit tests for Plate geometry:
   ```python
   def test_plate_geometry():
       plate = Plate(
           corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
           thickness=0.01,
           material="Steel"
       )
       assert plate.n_corners == 4
       assert plate.n_edges == 4
       assert np.allclose(plate.get_edge_length(0), 2.0)
       assert np.allclose(plate.get_normal(), [0, 0, 1])
       assert plate.is_planar()

   def test_plate_pentagon():
       # Pentagon plate
       import math
       corners = [[math.cos(2*math.pi*i/5), math.sin(2*math.pi*i/5), 0] for i in range(5)]
       plate = Plate(corners=corners, thickness=0.01, material="Steel")
       assert plate.n_corners == 5
       assert plate.is_planar()
   ```

**Acceptance Criteria:**
- [ ] Plate class stores corners, thickness, material, mesh properties
- [ ] Supports 3+ corners (triangles, quads, pentagons, etc.)
- [ ] Edge geometry methods work correctly
- [ ] Normal vector calculation correct
- [ ] Planarity check works
- [ ] Edge mesh controls stored per-edge
- [ ] Unit tests pass

---

### Task 19.3: Gmsh Integration ✓

**Requirements:** R-MESH-001
**Dependencies:** Task 19.2
**Difficulty:** High
**Status:** Complete

**Description:**
Integrate gmsh for mesh generation with quad preference and element size control.

**Steps:**

1. Create `src/grillex/meshing/__init__.py` and `src/grillex/meshing/gmsh_mesher.py`:
   ```python
   """Mesh generation using gmsh."""

   import numpy as np
   from typing import List, Tuple, Optional, Dict
   from dataclasses import dataclass

   try:
       import gmsh
   except ImportError:
       raise ImportError(
           "gmsh is required for plate meshing. Install with: pip install gmsh"
       )

   @dataclass
   class MeshResult:
       """Result of mesh generation."""
       nodes: np.ndarray  # Shape (n_nodes, 3) - node coordinates
       node_tags: np.ndarray  # Shape (n_nodes,) - gmsh node tags
       quads: np.ndarray  # Shape (n_quads, 4) - quad element connectivity (node indices)
       triangles: np.ndarray  # Shape (n_tris, 3) - triangle element connectivity
       edge_nodes: Dict[int, List[int]]  # edge_index -> list of node indices on edge


   class GmshPlateMesher:
       """Mesh generator for plate regions using gmsh."""

       def __init__(self):
           self._initialized = False

       def _ensure_initialized(self):
           if not self._initialized:
               gmsh.initialize()
               gmsh.option.setNumber("General.Terminal", 0)  # Suppress output
               self._initialized = True

       def mesh_plate(
           self,
           corners: List[List[float]],
           mesh_size: float,
           edge_divisions: Optional[Dict[int, int]] = None,
           prefer_quads: bool = True,
           element_order: int = 1
       ) -> MeshResult:
           """
           Generate mesh for a plate defined by corner points.

           Args:
               corners: List of [x, y, z] corner coordinates.
               mesh_size: Target element size in meters.
               edge_divisions: Dict mapping edge index to number of divisions.
               prefer_quads: If True, generate quads where possible.
               element_order: 1 for linear, 2 for quadratic elements.

           Returns:
               MeshResult with nodes and elements.
           """
           self._ensure_initialized()

           gmsh.clear()
           gmsh.model.add("plate")

           # Create points
           point_tags = []
           for i, corner in enumerate(corners):
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
                   gmsh.model.geo.mesh.setTransfiniteCurve(
                       line_tag, edge_divisions[i] + 1
                   )

           # Create curve loop and surface
           loop_tag = gmsh.model.geo.addCurveLoop(line_tags)
           surface_tag = gmsh.model.geo.addPlaneSurface([loop_tag])

           # For quads: transfinite surface if 4 corners with edge divisions
           if prefer_quads and len(corners) == 4 and edge_divisions:
               # Check if all edges have divisions specified
               if all(i in edge_divisions for i in range(4)):
                   # Verify opposite edges have matching divisions
                   if (edge_divisions[0] == edge_divisions[2] and
                       edge_divisions[1] == edge_divisions[3]):
                       gmsh.model.geo.mesh.setTransfiniteSurface(surface_tag)
                       gmsh.model.geo.mesh.setRecombine(2, surface_tag)

           gmsh.model.geo.synchronize()

           # Mesh settings for quad preference
           if prefer_quads:
               gmsh.option.setNumber("Mesh.Algorithm", 8)  # Frontal-Delaunay for quads
               gmsh.option.setNumber("Mesh.RecombineAll", 1)
               gmsh.option.setNumber("Mesh.RecombinationAlgorithm", 2)  # Blossom

           # Element order
           if element_order == 2:
               gmsh.option.setNumber("Mesh.ElementOrder", 2)

           # Generate mesh
           gmsh.model.mesh.generate(2)

           # Extract mesh data
           result = self._extract_mesh_data(line_tags)

           return result

       def _extract_mesh_data(self, edge_line_tags: List[int]) -> MeshResult:
           """Extract nodes and elements from gmsh."""
           # Get all nodes
           node_tags, node_coords, _ = gmsh.model.mesh.getNodes()
           n_nodes = len(node_tags)
           nodes = np.array(node_coords).reshape(n_nodes, 3)

           # Create tag to index mapping
           tag_to_idx = {tag: i for i, tag in enumerate(node_tags)}

           # Get quad elements (type 3 = 4-node quad)
           quad_tags, quad_connectivity = gmsh.model.mesh.getElementsByType(3)
           n_quads = len(quad_tags)
           if n_quads > 0:
               quads = np.array(quad_connectivity).reshape(n_quads, 4)
               quads = np.array([[tag_to_idx[t] for t in elem] for elem in quads])
           else:
               quads = np.zeros((0, 4), dtype=int)

           # Get triangle elements (type 2 = 3-node triangle)
           tri_tags, tri_connectivity = gmsh.model.mesh.getElementsByType(2)
           n_tris = len(tri_tags)
           if n_tris > 0:
               triangles = np.array(tri_connectivity).reshape(n_tris, 3)
               triangles = np.array([[tag_to_idx[t] for t in elem] for elem in triangles])
           else:
               triangles = np.zeros((0, 3), dtype=int)

           # Get nodes on each edge
           edge_nodes = {}
           for i, line_tag in enumerate(edge_line_tags):
               edge_node_tags, _, _ = gmsh.model.mesh.getNodes(1, line_tag)
               edge_nodes[i] = [tag_to_idx[t] for t in edge_node_tags]

           return MeshResult(
               nodes=nodes,
               node_tags=np.array(node_tags),
               quads=quads,
               triangles=triangles,
               edge_nodes=edge_nodes
           )

       def finalize(self):
           """Clean up gmsh resources."""
           if self._initialized:
               gmsh.finalize()
               self._initialized = False
   ```

2. Add dependency to pyproject.toml:
   ```toml
   [project.optional-dependencies]
   meshing = ["gmsh>=4.11.0"]
   ```

3. Write integration tests:
   ```python
   def test_gmsh_quad_mesh():
       mesher = GmshPlateMesher()
       try:
           result = mesher.mesh_plate(
               corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
               mesh_size=0.5,
               prefer_quads=True
           )
           assert result.nodes.shape[1] == 3
           assert result.quads.shape[0] > 0  # Should have quads
           assert result.quads.shape[1] == 4
       finally:
           mesher.finalize()

   def test_gmsh_edge_divisions():
       mesher = GmshPlateMesher()
       try:
           result = mesher.mesh_plate(
               corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
               mesh_size=1.0,
               edge_divisions={0: 4, 1: 2, 2: 4, 3: 2},  # Matching opposite edges
               prefer_quads=True
           )
           # Should have 4x2 = 8 quads
           assert result.quads.shape[0] == 8
       finally:
           mesher.finalize()

   def test_gmsh_pentagon():
       mesher = GmshPlateMesher()
       try:
           import math
           corners = [[math.cos(2*math.pi*i/5), math.sin(2*math.pi*i/5), 0]
                      for i in range(5)]
           result = mesher.mesh_plate(
               corners=corners,
               mesh_size=0.3,
               prefer_quads=True
           )
           # Pentagon will have mix of quads and triangles
           assert result.nodes.shape[0] > 5
           total_elements = result.quads.shape[0] + result.triangles.shape[0]
           assert total_elements > 0
       finally:
           mesher.finalize()
   ```

**Acceptance Criteria:**
- [ ] Gmsh integration works with `pip install gmsh`
- [ ] Rectangular plates mesh into pure quads when edge divisions match
- [ ] General polygons (5+ corners) mesh with quad preference
- [ ] Triangles used as fallback for irregular regions
- [ ] Edge division control overrides global mesh size
- [ ] Nodes on edges tracked for coupling
- [ ] Quadratic elements supported (element_order=2)
- [ ] Unit tests pass

### Execution Notes (Tasks 19.1-19.3, Completed 2025-12-28)

**Steps Taken:**
1. Added `add_plate_element()` and `get_plate_elements()` methods to StructuralModel
2. Created `src/grillex/core/plate.py` with Plate, EdgeMeshControl, PlateBeamCoupling, SupportCurve classes
3. Created `src/grillex/meshing/` module with GmshPlateMesher class
4. Added `meshing` optional dependency to setup.cfg
5. Updated exports in `core/__init__.py`
6. Wrote comprehensive tests in `tests/python/test_phase19_plate_meshing.py`

**Problems Encountered:**
- **Issue**: Used `_get_material()` instead of `get_material()` in add_plate_element
  - **Solution**: Fixed to use correct method name
- **Issue**: Gmsh requires OpenGL system libraries (libGLU) not available in test environment
  - **Solution**: Tests properly skip when gmsh cannot be loaded

**Verification:**
- 31 tests passing (7 for Task 19.1, 21 for Task 19.2, 3 basic tests for Task 19.3)
- 9 gmsh-specific tests skipped (require system OpenGL libraries)

**Files Created/Modified:**
- `src/grillex/core/model_wrapper.py` - Added add_plate_element(), get_plate_elements()
- `src/grillex/core/plate.py` - New file with Plate geometry class
- `src/grillex/core/__init__.py` - Updated exports
- `src/grillex/meshing/__init__.py` - New module
- `src/grillex/meshing/gmsh_mesher.py` - GmshPlateMesher implementation
- `setup.cfg` - Added `meshing` optional dependency
- `tests/python/test_phase19_plate_meshing.py` - New test file

---

### Task 19.4: Higher-Order Plate Elements (MITC8, MITC9)

**Requirements:** R-ELEM-004 (extended)
**Dependencies:** Phase 8 (MITC4), Task 19.3
**Difficulty:** High

**Description:**
Implement 8-node and 9-node quadrilateral plate elements for improved accuracy.

**Steps:**

1. Create `cpp/include/grillex/plate_element_8.hpp`:
   ```cpp
   #pragma once

   #include "plate_element.hpp"

   namespace grillex {

   /**
    * 8-node serendipity plate element (MITC8).
    *
    * Node numbering:
    *   4---7---3
    *   |       |
    *   8       6
    *   |       |
    *   1---5---2
    *
    * DOFs per node: 6 (UX, UY, UZ, RX, RY, RZ)
    * Total DOFs: 48
    */
   class PlateElement8 : public PlateElementBase {
   public:
       PlateElement8(
           int id,
           std::array<Node*, 8> nodes,
           double thickness,
           const Material& material
       );

       size_t num_nodes() const override { return 8; }
       size_t num_dofs() const override { return 48; }

       Eigen::MatrixXd stiffness_matrix() const override;
       Eigen::MatrixXd mass_matrix() const override;

   private:
       std::array<Node*, 8> nodes_;

       // 8-node serendipity shape functions
       static Eigen::VectorXd shape_functions(double xi, double eta);
       static Eigen::MatrixXd shape_derivatives(double xi, double eta);

       // 3x3 Gauss quadrature for 8-node elements
       static const std::vector<std::pair<double, double>>& gauss_points_3x3();
       static const std::vector<double>& gauss_weights_3x3();
   };

   }  // namespace grillex
   ```

2. Create `cpp/include/grillex/plate_element_9.hpp`:
   ```cpp
   #pragma once

   #include "plate_element.hpp"

   namespace grillex {

   /**
    * 9-node Lagrangian plate element (MITC9).
    *
    * Node numbering:
    *   4---7---3
    *   |   9   |
    *   8       6
    *   |       |
    *   1---5---2
    *
    * DOFs per node: 6 (UX, UY, UZ, RX, RY, RZ)
    * Total DOFs: 54
    */
   class PlateElement9 : public PlateElementBase {
   public:
       PlateElement9(
           int id,
           std::array<Node*, 9> nodes,
           double thickness,
           const Material& material
       );

       size_t num_nodes() const override { return 9; }
       size_t num_dofs() const override { return 54; }

       Eigen::MatrixXd stiffness_matrix() const override;
       Eigen::MatrixXd mass_matrix() const override;

   private:
       std::array<Node*, 9> nodes_;

       // 9-node Lagrangian shape functions
       static Eigen::VectorXd shape_functions(double xi, double eta);
       static Eigen::MatrixXd shape_derivatives(double xi, double eta);

       // 3x3 Gauss quadrature
       static const std::vector<std::pair<double, double>>& gauss_points_3x3();
       static const std::vector<double>& gauss_weights_3x3();
   };

   }  // namespace grillex
   ```

3. Implement shape functions for 8-node element:
   ```cpp
   Eigen::VectorXd PlateElement8::shape_functions(double xi, double eta) {
       Eigen::VectorXd N(8);
       // Corner nodes (1, 2, 3, 4)
       N(0) = 0.25 * (1 - xi) * (1 - eta) * (-xi - eta - 1);
       N(1) = 0.25 * (1 + xi) * (1 - eta) * (xi - eta - 1);
       N(2) = 0.25 * (1 + xi) * (1 + eta) * (xi + eta - 1);
       N(3) = 0.25 * (1 - xi) * (1 + eta) * (-xi + eta - 1);
       // Mid-side nodes (5, 6, 7, 8)
       N(4) = 0.5 * (1 - xi * xi) * (1 - eta);
       N(5) = 0.5 * (1 + xi) * (1 - eta * eta);
       N(6) = 0.5 * (1 - xi * xi) * (1 + eta);
       N(7) = 0.5 * (1 - xi) * (1 - eta * eta);
       return N;
   }
   ```

4. Implement shape functions for 9-node element:
   ```cpp
   Eigen::VectorXd PlateElement9::shape_functions(double xi, double eta) {
       Eigen::VectorXd N(9);
       // Lagrange polynomials in xi
       double L1_xi = 0.5 * xi * (xi - 1);
       double L2_xi = 1 - xi * xi;
       double L3_xi = 0.5 * xi * (xi + 1);
       // Lagrange polynomials in eta
       double L1_eta = 0.5 * eta * (eta - 1);
       double L2_eta = 1 - eta * eta;
       double L3_eta = 0.5 * eta * (eta + 1);

       // Tensor product
       N(0) = L1_xi * L1_eta;  // Node 1
       N(1) = L3_xi * L1_eta;  // Node 2
       N(2) = L3_xi * L3_eta;  // Node 3
       N(3) = L1_xi * L3_eta;  // Node 4
       N(4) = L2_xi * L1_eta;  // Node 5
       N(5) = L3_xi * L2_eta;  // Node 6
       N(6) = L2_xi * L3_eta;  // Node 7
       N(7) = L1_xi * L2_eta;  // Node 8
       N(8) = L2_xi * L2_eta;  // Node 9 (center)
       return N;
   }
   ```

5. Add pybind11 bindings for new element types
6. Add factory function for element type selection
7. Write unit tests comparing accuracy with MITC4

**Acceptance Criteria:**
- [x] PlateElement8 class implemented with correct shape functions
- [x] PlateElement9 class implemented with correct shape functions
- [x] Stiffness matrices are symmetric and positive semi-definite
- [x] Mass matrices are consistent
- [x] 3x3 Gauss quadrature used for integration
- [x] pybind11 bindings expose new element types
- [ ] Factory function selects element type by name (deferred to Task 19.6)
- [ ] Patch test passes for both element types (deferred to Phase 13)
- [ ] Convergence better than MITC4 for same mesh density (deferred to Phase 13)

---

### Task 19.5: Triangular Plate Element (DKT)

**Requirements:** R-ELEM-004
**Dependencies:** Phase 8
**Difficulty:** High

**Description:**
Implement a 3-node triangular plate element (Discrete Kirchhoff Triangle) as fallback for irregular geometries.

**Steps:**

1. Create `cpp/include/grillex/plate_element_tri.hpp`:
   ```cpp
   #pragma once

   #include "element_base.hpp"
   #include "node.hpp"
   #include "material.hpp"

   namespace grillex {

   /**
    * 3-node Discrete Kirchhoff Triangle (DKT) plate element.
    *
    * DOFs per node: 6 (UX, UY, UZ, RX, RY, RZ)
    * Active bending DOFs: W, θx, θy (3 per node = 9 total)
    * Total DOFs: 18
    */
   class PlateElementTri : public ElementBase {
   public:
       PlateElementTri(
           int id,
           Node* n1, Node* n2, Node* n3,
           double thickness,
           const Material& material
       );

       size_t num_nodes() const override { return 3; }
       size_t num_dofs() const override { return 18; }

       Eigen::MatrixXd stiffness_matrix() const override;
       Eigen::MatrixXd mass_matrix() const override;

       double area() const;
       Eigen::Vector3d centroid() const;
       Eigen::Vector3d normal() const;

   private:
       std::array<Node*, 3> nodes_;
       double thickness_;
       const Material& material_;

       // DKT shape functions and derivatives
       Eigen::MatrixXd bending_strain_matrix(double L1, double L2, double L3) const;
   };

   }  // namespace grillex
   ```

2. Implement DKT bending formulation with area coordinates
3. Add to pybind11 bindings
4. Add to element factory
5. Write unit tests

**Acceptance Criteria:**
- [x] PlateElementTri class implemented
- [x] Area coordinate shape functions correct
- [x] Stiffness matrix symmetric and positive semi-definite
- [ ] Constant strain patch test passes (deferred to Phase 13)
- [ ] Can mesh any polygon using triangles (deferred to Task 19.10)
- [x] Unit tests pass

---

### Task 19.6: Element Type Infrastructure

**Requirements:** R-ELEM-004
**Dependencies:** Tasks 19.4, 19.5
**Difficulty:** Medium

**Description:**
Create infrastructure for selecting between plate element types, with extensibility for future element types (Kirchhoff, membrane, shell).

**Steps:**

1. Create `src/grillex/core/element_types.py`:
   ```python
   from enum import Enum
   from typing import Dict, Any

   class PlateElementType(Enum):
       """Available plate element formulations."""
       MITC4 = "MITC4"        # 4-node Mindlin (default)
       MITC8 = "MITC8"        # 8-node serendipity
       MITC9 = "MITC9"        # 9-node Lagrangian
       DKT = "DKT"            # 3-node Discrete Kirchhoff Triangle

       # Future element types (not yet implemented)
       # KIRCHHOFF4 = "KIRCHHOFF4"  # 4-node thin plate (no shear)
       # MEMBRANE4 = "MEMBRANE4"     # 4-node membrane (in-plane only)
       # SHELL4 = "SHELL4"           # 4-node shell (bending + membrane)

   ELEMENT_TYPE_INFO: Dict[PlateElementType, Dict[str, Any]] = {
       PlateElementType.MITC4: {
           "n_nodes": 4,
           "description": "4-node Mindlin plate with MITC shear locking treatment",
           "supports_shear": True,
           "polynomial_order": 1,
       },
       PlateElementType.MITC8: {
           "n_nodes": 8,
           "description": "8-node serendipity Mindlin plate",
           "supports_shear": True,
           "polynomial_order": 2,
       },
       PlateElementType.MITC9: {
           "n_nodes": 9,
           "description": "9-node Lagrangian Mindlin plate",
           "supports_shear": True,
           "polynomial_order": 2,
       },
       PlateElementType.DKT: {
           "n_nodes": 3,
           "description": "3-node Discrete Kirchhoff Triangle (thin plate)",
           "supports_shear": False,
           "polynomial_order": 2,
       },
   }

   def get_element_type(name: str) -> PlateElementType:
       """Get element type by name (case-insensitive)."""
       try:
           return PlateElementType(name.upper())
       except ValueError:
           valid = [e.value for e in PlateElementType]
           raise ValueError(f"Unknown element type '{name}'. Valid types: {valid}")
   ```

2. Create element factory in C++:
   ```cpp
   // cpp/include/grillex/plate_factory.hpp
   std::unique_ptr<PlateElementBase> create_plate_element(
       const std::string& type,
       int id,
       const std::vector<Node*>& nodes,
       double thickness,
       const Material& material
   );
   ```

3. Update `Plate` class to use element type enum
4. Write tests for element type selection

**Acceptance Criteria:**
- [x] PlateElementType enum defines all available types
- [x] Element type info provides metadata (n_nodes, description)
- [x] Factory function creates correct element type
- [x] Invalid element type raises clear error
- [x] Infrastructure ready for future element types

---

### Task 19.7: Add Plate to StructuralModel

**Requirements:** R-MOD-007
**Dependencies:** Tasks 19.2, 19.3
**Difficulty:** Medium

**Description:**
Add methods to StructuralModel for creating plates (geometry) and controlling mesh properties.

**Steps:**

1. Add plate storage and methods to StructuralModel:
   ```python
   class StructuralModel:
       def __init__(self, ...):
           ...
           self._plates: List[Plate] = []

       def add_plate(
           self,
           corners: List[List[float]],
           thickness: float,
           material: str,
           mesh_size: float = 0.5,
           element_type: str = "MITC4",
           name: Optional[str] = None
       ) -> Plate:
           """
           Add a plate region to the model.

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
               ValueError: If material not found or invalid geometry.
           """
           if self._get_material(material) is None:
               raise ValueError(f"Material '{material}' not found")

           plate = Plate(
               corners=corners,
               thickness=thickness,
               material=material,
               mesh_size=mesh_size,
               element_type=element_type,
               name=name or f"Plate_{len(self._plates) + 1}"
           )

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
           """
           Set the number of elements along a plate edge.

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
               raise ValueError(f"Edge index {edge_index} out of range [0, {plate.n_edges})")
           if n_elements < 1:
               raise ValueError("n_elements must be at least 1")

           plate.edge_controls[edge_index] = EdgeMeshControl(n_elements=n_elements)

       def get_plates(self) -> List[Plate]:
           """Return all plates in the model."""
           return list(self._plates)
   ```

2. Write unit tests:
   ```python
   def test_add_plate():
       model = StructuralModel(name="PlateTest")
       model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

       plate = model.add_plate(
           corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
           thickness=0.02,
           material="Steel",
           mesh_size=0.5
       )

       assert plate.n_corners == 4
       assert len(model.get_plates()) == 1

   def test_set_edge_divisions():
       model = StructuralModel(name="PlateTest")
       model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

       plate = model.add_plate(
           corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
           thickness=0.02,
           material="Steel"
       )

       model.set_edge_divisions(plate, 0, 8)  # 8 elements along first edge
       model.set_edge_divisions(plate, 2, 8)  # 8 elements along opposite edge

       assert plate.edge_controls[0].n_elements == 8
       assert plate.edge_controls[2].n_elements == 8
   ```

**Acceptance Criteria:**
- [x] `add_plate()` creates Plate object and stores in model
- [x] Validates material exists
- [x] Validates plate is planar
- [x] `set_edge_divisions()` sets per-edge element count
- [x] Edge division validates edge index and n_elements
- [x] `get_plates()` returns all plates
- [x] Unit tests pass

---

### Task 19.8: Plate-Beam Coupling

**Requirements:** R-MPC-003
**Dependencies:** Phase 6 (rigid links), Task 19.7
**Difficulty:** High

**Description:**
Implement coupling between plate edges and beam elements using rigid links with DOF release control and eccentric offset support.

**Steps:**

1. Create coupling data structure:
   ```python
   @dataclass
   class PlateBeamCoupling:
       """Coupling between a plate edge and a beam."""
       plate: "Plate"
       edge_index: int
       beam: "Beam"
       offset: List[float]  # [dx, dy, dz] from plate node to beam centroid
       releases: Dict[str, bool] = field(default_factory=dict)  # DOF -> released

       # DOF release options:
       # "UX", "UY", "UZ" - translational releases
       # "RX", "RY", "RZ" - rotational releases
       # "R_EDGE" - rotation about the edge direction (computed from edge geometry)
   ```

2. Add coupling method to StructuralModel:
   ```python
   def couple_plate_to_beam(
       self,
       plate: Plate,
       edge_index: int,
       beam: "Beam",
       offset: Optional[List[float]] = None,
       releases: Optional[Dict[str, bool]] = None
   ) -> PlateBeamCoupling:
       """
       Couple a plate edge to a beam using rigid links.

       Creates rigid link constraints between plate edge nodes and the beam.
       If the plate normal is not parallel to the beam local x-axis,
       intermediate beam nodes are created to match plate edge node positions.

       Args:
           plate: The Plate object.
           edge_index: Edge index to couple.
           beam: The Beam object to couple to.
           offset: [dx, dy, dz] offset from plate node to beam centroid.
               If None, nodes are assumed coincident.
           releases: Dict of DOF releases. Keys: "UX", "UY", "UZ", "RX", "RY", "RZ",
               or "R_EDGE" for rotation about the edge. Values: True = released.

       Returns:
           PlateBeamCoupling object.

       Example:
           # Couple plate edge to beam with moment release about edge
           model.couple_plate_to_beam(
               plate, edge_index=0, beam=main_beam,
               offset=[0, 0, 0.15],  # Plate 150mm above beam centroid
               releases={"R_EDGE": True}  # Allow rotation about edge
           )
       """
       if edge_index < 0 or edge_index >= plate.n_edges:
           raise ValueError(f"Edge index {edge_index} out of range")

       coupling = PlateBeamCoupling(
           plate=plate,
           edge_index=edge_index,
           beam=beam,
           offset=offset or [0, 0, 0],
           releases=releases or {}
       )

       plate.beam_couplings.append(coupling)
       return coupling
   ```

3. Implement coupling creation in mesh():
   ```python
   def _create_plate_beam_couplings(self, plate: Plate, edge_node_indices: Dict[int, List[int]]):
       """Create rigid links for plate-beam couplings after meshing."""
       for coupling in plate.beam_couplings:
           edge_nodes = edge_node_indices[coupling.edge_index]
           edge_coords = [plate._mesh_nodes[i] for i in edge_nodes]

           # Get plate edge direction and normal
           edge_dir = plate.get_edge_direction(coupling.edge_index)
           plate_normal = plate.get_normal()

           # Check if beam needs subdivision (normal not parallel to beam x-axis)
           beam_dir = coupling.beam.direction
           needs_subdivision = not np.allclose(abs(np.dot(plate_normal, beam_dir)), 1.0)

           if needs_subdivision:
               # Create beam nodes at plate edge node positions (projected onto beam)
               for coord in edge_coords:
                   projected = self._project_point_to_beam(coord, coupling.beam)
                   self.get_or_create_node(*projected)

           # Create rigid links from plate nodes to beam
           for node_idx in edge_nodes:
               plate_node_coord = plate._mesh_nodes[node_idx]
               plate_node = self.get_or_create_node(*plate_node_coord)

               # Find corresponding beam node
               if needs_subdivision:
                   projected = self._project_point_to_beam(plate_node_coord, coupling.beam)
                   beam_node = self.get_or_create_node(*projected)
               else:
                   # Use beam end node
                   beam_node = coupling.beam.start_node  # or end_node

               # Create rigid link with offset and releases
               self._create_rigid_link_with_releases(
                   master=beam_node,
                   slave=plate_node,
                   offset=coupling.offset,
                   releases=coupling.releases,
                   edge_direction=edge_dir
               )
   ```

4. Implement rigid link with releases:
   ```python
   def _create_rigid_link_with_releases(
       self,
       master: Node,
       slave: Node,
       offset: List[float],
       releases: Dict[str, bool],
       edge_direction: np.ndarray
   ):
       """Create rigid link with DOF releases."""
       # Map "R_EDGE" to actual rotation DOF based on edge direction
       if releases.get("R_EDGE"):
           # Determine which rotation DOF corresponds to edge direction
           edge_rot_dof = self._edge_to_rotation_dof(edge_direction)
           releases[edge_rot_dof] = True
           del releases["R_EDGE"]

       # Get active (non-released) DOFs
       active_dofs = []
       for dof in ["UX", "UY", "UZ", "RX", "RY", "RZ"]:
           if not releases.get(dof, False):
               active_dofs.append(DOFIndex[dof])

       # Create rigid link constraint with offset
       if len(offset) > 0 and any(o != 0 for o in offset):
           # Eccentric connection: u_slave = u_master + θ_master × r
           self._add_eccentric_rigid_link(master, slave, offset, active_dofs)
       else:
           # Concentric: direct coupling
           for dof in active_dofs:
               self._add_equality_constraint(master, slave, dof)
   ```

5. Write comprehensive tests

**Acceptance Criteria:**
- [x] `couple_plate_to_beam()` stores coupling information
- [ ] Coupling creates rigid links after meshing (deferred to Task 19.10)
- [ ] Offset (eccentric connection) correctly transforms DOFs (deferred to Task 19.10)
- [ ] DOF releases work for individual DOFs (deferred to Task 19.10)
- [ ] "R_EDGE" release maps to correct rotation DOF based on edge direction (deferred to Task 19.10)
- [ ] Beam nodes created at plate edge positions when needed (deferred to Task 19.10)
- [ ] Works for horizontal and vertical plates connecting to beams (deferred to Task 19.10)
- [x] Unit tests pass

---

### Task 19.9: Support Curves

**Requirements:** R-BC-002
**Dependencies:** Task 19.7
**Difficulty:** Medium

**Description:**
Implement boundary condition support along plate edges (support curves).

**Steps:**

1. Create support curve data structure:
   ```python
   @dataclass
   class SupportCurve:
       """Boundary condition along a plate edge."""
       plate: "Plate"
       edge_index: int
       ux: bool = False  # Restrain X translation
       uy: bool = False  # Restrain Y translation
       uz: bool = False  # Restrain Z translation
       rotation_about_edge: bool = False  # Restrain rotation about edge direction
   ```

2. Add method to StructuralModel:
   ```python
   def add_support_curve(
       self,
       plate: Plate,
       edge_index: int,
       ux: bool = False,
       uy: bool = False,
       uz: bool = False,
       rotation_about_edge: bool = False
   ) -> SupportCurve:
       """
       Add boundary condition support along a plate edge.

       After meshing, the specified DOFs are restrained for all nodes
       along the edge.

       Args:
           plate: The Plate object.
           edge_index: Edge index for the support.
           ux: If True, restrain X translation.
           uy: If True, restrain Y translation.
           uz: If True, restrain Z translation.
           rotation_about_edge: If True, restrain rotation about the edge direction.

       Returns:
           SupportCurve object.

       Example:
           # Simply supported edge (restrain UZ only)
           model.add_support_curve(plate, edge_index=0, uz=True)

           # Fixed edge
           model.add_support_curve(plate, edge_index=0,
               ux=True, uy=True, uz=True, rotation_about_edge=True)
       """
       if edge_index < 0 or edge_index >= plate.n_edges:
           raise ValueError(f"Edge index {edge_index} out of range")

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
   ```

3. Apply support curves after meshing:
   ```python
   def _apply_support_curves(self, plate: Plate, edge_node_indices: Dict[int, List[int]]):
       """Apply support curve boundary conditions after meshing."""
       for support in plate.support_curves:
           edge_nodes = edge_node_indices[support.edge_index]

           for node_idx in edge_nodes:
               node_coord = plate._mesh_nodes[node_idx]
               node = self.get_or_create_node(*node_coord)

               if support.ux:
                   self.fix_dof_at(node_coord, DOFIndex.UX)
               if support.uy:
                   self.fix_dof_at(node_coord, DOFIndex.UY)
               if support.uz:
                   self.fix_dof_at(node_coord, DOFIndex.UZ)

               if support.rotation_about_edge:
                   # Map edge direction to rotation DOF
                   edge_dir = plate.get_edge_direction(support.edge_index)
                   rot_dof = self._edge_to_rotation_dof(edge_dir)
                   self.fix_dof_at(node_coord, rot_dof)
   ```

4. Write unit tests

**Acceptance Criteria:**
- [x] `add_support_curve()` stores support information on plate
- [ ] Support applied to all nodes along edge after meshing (deferred to Task 19.10)
- [x] Individual DOFs (UX, UY, UZ) can be restrained independently
- [ ] Rotation about edge direction correctly mapped to RX/RY/RZ (deferred to Task 19.10)
- [x] Multiple support curves can be applied to different edges
- [x] Unit tests pass

---

### Task 19.10: Unified Mesh Function

**Requirements:** R-MESH-002
**Dependencies:** Tasks 19.3, 19.7, 19.8, 19.9
**Difficulty:** High

**Description:**
Create a unified `mesh()` function that handles both plate meshing and beam subdivision, considering their interactions.

**Steps:**

1. Implement unified mesh function:
   ```python
   def mesh(self) -> MeshStatistics:
       """
       Generate mesh for all plates and subdivide beams.

       This unified meshing function:
       1. Meshes all plates using gmsh with specified element sizes and types
       2. Creates plate elements from mesh
       3. Creates beam nodes at plate-beam coupling points
       4. Subdivides beams at internal nodes (including coupling points)
       5. Applies support curves
       6. Creates plate-beam coupling constraints

       Returns:
           MeshStatistics with counts of created elements.

       Raises:
           ValueError: If mesh validation fails (e.g., non-matching edge divisions).
       """
       stats = MeshStatistics()

       # Step 1: Validate mesh compatibility
       self._validate_plate_mesh_compatibility()

       # Step 2: Mesh all plates
       mesher = GmshPlateMesher()
       try:
           for plate in self._plates:
               mesh_result = self._mesh_single_plate(mesher, plate)
               stats.n_plate_nodes += len(mesh_result.nodes)
               stats.n_plate_elements += mesh_result.quads.shape[0] + mesh_result.triangles.shape[0]

               # Create plate elements in C++ model
               self._create_plate_elements(plate, mesh_result)

               # Store edge nodes for coupling and supports
               plate._edge_node_map = mesh_result.edge_nodes
       finally:
           mesher.finalize()

       # Step 3: Create beam nodes at plate coupling points
       for plate in self._plates:
           for coupling in plate.beam_couplings:
               self._create_coupling_beam_nodes(plate, coupling)

       # Step 4: Subdivide all beams (including at coupling points)
       stats.n_beams_subdivided = self.subdivide_beams()

       # Step 5: Apply support curves
       for plate in self._plates:
           self._apply_support_curves(plate, plate._edge_node_map)

       # Step 6: Create plate-beam coupling constraints
       for plate in self._plates:
           self._create_plate_beam_couplings(plate, plate._edge_node_map)

       return stats

   def _validate_plate_mesh_compatibility(self):
       """Validate that adjacent plates have matching edge divisions."""
       # Check for shared edges between plates
       for i, plate1 in enumerate(self._plates):
           for j, plate2 in enumerate(self._plates[i+1:], i+1):
               shared_edge = self._find_shared_edge(plate1, plate2)
               if shared_edge:
                   edge1, edge2 = shared_edge
                   ctrl1 = plate1.edge_controls.get(edge1)
                   ctrl2 = plate2.edge_controls.get(edge2)

                   if ctrl1 and ctrl2:
                       if ctrl1.n_elements != ctrl2.n_elements:
                           raise ValueError(
                               f"Adjacent plates {plate1.name} and {plate2.name} "
                               f"have non-matching edge divisions: "
                               f"{ctrl1.n_elements} vs {ctrl2.n_elements}"
                           )

   @dataclass
   class MeshStatistics:
       """Statistics from mesh generation."""
       n_plate_nodes: int = 0
       n_plate_elements: int = 0
       n_beams_subdivided: int = 0
       n_quad_elements: int = 0
       n_tri_elements: int = 0
       n_rigid_links: int = 0
   ```

2. Implement helper methods for mesh creation
3. Write comprehensive integration tests

**Acceptance Criteria:**
- [x] `mesh()` generates mesh for all plates
- [x] Creates PlateElement objects from mesh (MITC4/8/9 or DKT)
- [x] Creates beam nodes at plate-beam coupling points
- [x] Subdivides beams after plate meshing (via get_or_create_node for coupling points)
- [x] Applies support curves after meshing
- [x] Creates rigid link constraints for plate-beam coupling
- [ ] Validates matching edge divisions for adjacent plates (deferred - validation not yet implemented)
- [x] Returns MeshStatistics with element counts
- [x] Integration tests pass

### Execution Notes (Completed 2025-12-28)

**Steps Taken:**
1. Added `create_plate_8()`, `create_plate_9()`, `create_plate_tri()` methods to C++ Model class
2. Added pybind11 bindings for new plate creation methods
3. Added separate vectors in Model for 8-node, 9-node, and triangular elements
4. Implemented `mesh()` method in StructuralModel with:
   - Plate meshing via GmshPlateMesher
   - Element creation based on element type
   - Support curve BC application
   - Plate-beam coupling via rigid links
5. Created comprehensive test suite with 15 tests
6. Fixed material lookup (iterate through model.materials)
7. Fixed node lookup (track node_id_to_node mapping)
8. Fixed rigid link creation (skip when nodes are colocated)

**Problems Encountered:**
- **Issue**: C++ Model didn't have `get_material_by_name()` method
  - **Solution**: Iterate through `self._cpp_model.materials` to find by name

- **Issue**: C++ Model didn't have `get_node_by_id()` method
  - **Solution**: Maintain `node_id_to_node` dictionary during mesh generation

- **Issue**: Rigid link fails when slave and master are the same node
  - **Solution**: Skip rigid link creation when plate and beam nodes are colocated (offset is zero)

**Verification:**
- 15/15 mesh tests passing ✓
- 65/65 Phase 19 tests passing ✓
- MITC4, MITC8, MITC9, and DKT elements all work correctly

---

### Task 19.11: Results for Plate Elements

**Requirements:** R-RES-002
**Dependencies:** Tasks 19.1, 19.10
**Difficulty:** Medium

**Description:**
Implement result queries for plate elements including displacements, stresses, and moments.

**Steps:**

1. Add result methods to StructuralModel:
   ```python
   def get_plate_displacement(
       self,
       plate: Plate,
       xi: float,
       eta: float
   ) -> Dict[str, float]:
       """
       Get displacement at a point within a plate.

       Args:
           plate: The Plate object (must be a 4-corner plate for now).
           xi, eta: Natural coordinates in range [-1, 1].

       Returns:
           Dict with "UX", "UY", "UZ", "RX", "RY", "RZ" values in m and rad.
       """

   def get_plate_stress(
       self,
       plate_element: PlateElement,
       surface: Literal["top", "bottom", "middle"] = "middle"
   ) -> Dict[str, float]:
       """
       Get stress at plate element center.

       Args:
           plate_element: The PlateElement object.
           surface: Which surface ("top", "bottom", or "middle").

       Returns:
           Dict with "sigma_x", "sigma_y", "tau_xy" in kN/m².
       """

   def get_plate_moments(
       self,
       plate_element: PlateElement
   ) -> Dict[str, float]:
       """
       Get internal moments at plate element center.

       Returns:
           Dict with "Mx", "My", "Mxy" in kN*m/m (moment per unit width).
       """
   ```

2. Implement stress and moment calculations in C++
3. Add pybind11 bindings for result queries
4. Write validation tests against analytical solutions

**Acceptance Criteria:**
- [ ] `get_plate_displacement()` returns displacement at point
- [ ] `get_plate_stress()` returns stress at top/bottom/middle surface
- [ ] `get_plate_moments()` returns internal moments
- [ ] Results verified against analytical solutions for simple cases
- [ ] Unit tests pass

---

### Task 19.12: LLM Tool Integration

**Requirements:** R-LLM-001
**Dependencies:** Task 19.10
**Difficulty:** Low

**Description:**
Add LLM tool schemas for plate meshing functionality.

**Steps:**

1. Add tool schemas to `src/grillex/llm/tools.py`:
   ```python
   {
       "name": "add_plate",
       "description": "Add a plate region to the model defined by corner points. The plate will be meshed when mesh() is called.",
       "input_schema": {
           "type": "object",
           "properties": {
               "corners": {
                   "type": "array",
                   "items": {"type": "array", "items": {"type": "number"}},
                   "description": "List of [x, y, z] corner coordinates in meters"
               },
               "thickness": {
                   "type": "number",
                   "description": "Plate thickness in meters"
               },
               "material": {
                   "type": "string",
                   "description": "Name of material to use"
               },
               "mesh_size": {
                   "type": "number",
                   "description": "Target element size in meters (default 0.5)"
               },
               "element_type": {
                   "type": "string",
                   "enum": ["MITC4", "MITC8", "MITC9", "DKT"],
                   "description": "Plate element type (default MITC4)"
               }
           },
           "required": ["corners", "thickness", "material"]
       }
   },
   {
       "name": "set_edge_divisions",
       "description": "Set number of elements along a plate edge. Takes precedence over mesh_size.",
       "input_schema": {
           "type": "object",
           "properties": {
               "plate_name": {"type": "string", "description": "Name of plate"},
               "edge_index": {"type": "integer", "description": "Edge index (0-based)"},
               "n_elements": {"type": "integer", "description": "Number of elements"}
           },
           "required": ["plate_name", "edge_index", "n_elements"]
       }
   },
   {
       "name": "couple_plate_to_beam",
       "description": "Couple a plate edge to a beam using rigid links with optional releases and offset.",
       "input_schema": {
           "type": "object",
           "properties": {
               "plate_name": {"type": "string"},
               "edge_index": {"type": "integer"},
               "beam_id": {"type": "string"},
               "offset": {
                   "type": "array",
                   "items": {"type": "number"},
                   "description": "[dx, dy, dz] offset from plate to beam centroid in meters"
               },
               "releases": {
                   "type": "object",
                   "description": "DOF releases, e.g., {\"R_EDGE\": true} for moment release"
               }
           },
           "required": ["plate_name", "edge_index", "beam_id"]
       }
   },
   {
       "name": "add_support_curve",
       "description": "Add boundary condition support along a plate edge.",
       "input_schema": {
           "type": "object",
           "properties": {
               "plate_name": {"type": "string"},
               "edge_index": {"type": "integer"},
               "ux": {"type": "boolean", "description": "Restrain X translation"},
               "uy": {"type": "boolean", "description": "Restrain Y translation"},
               "uz": {"type": "boolean", "description": "Restrain Z translation"},
               "rotation_about_edge": {"type": "boolean", "description": "Restrain rotation about edge"}
           },
           "required": ["plate_name", "edge_index"]
       }
   },
   {
       "name": "mesh",
       "description": "Generate mesh for all plates and subdivide beams. Call after defining plates and before analysis.",
       "input_schema": {
           "type": "object",
           "properties": {},
           "required": []
       }
   }
   ```

2. Implement tool handlers in ToolExecutor
3. Add diagnostics for mesh-related errors

**Acceptance Criteria:**
- [x] All plate meshing tools have schemas
- [x] Tool handlers execute correctly
- [x] Error messages are actionable
- [x] Diagnostics provide fix suggestions for common errors

### Execution Notes (Completed 2025-12-28)

**Steps Taken:**
1. Added 8 tool schemas to `src/grillex/llm/tools.py`:
   - add_plate, set_edge_divisions, couple_plate_to_beam, add_support_curve
   - mesh_model, get_plate_displacement, get_plate_moments, get_plate_stress
2. Implemented tool handlers in ToolExecutor class for all 8 tools
3. Added plate-related error diagnostics in `_get_suggestion_for_error`:
   - Plate not found, edge index out of range, non-coplanar points
   - Mesh failures, gmsh installation issues
4. Added 7 tests in `test_phase12_llm_tooling.py`

**Verification:**
- All 47 LLM tooling tests passing ✓
- All 17 plate result tests passing ✓
- All 15 mesh tests passing ✓

**Key Learnings:**
- Tool handlers need to find objects by name/ID since LLM tools use identifiers
- get_plates() returns Plate objects for lookup by name

---

### Task 19.13: Documentation and Examples

**Requirements:** R-DOC-001
**Dependencies:** All previous tasks
**Difficulty:** Low

**Description:**
Document the plate meshing API and provide examples.

**Steps:**

1. Add docstrings to all new methods (done in previous tasks)
2. Create example in `docs/user/plate_meshing.rst`:
   ```rst
   Plate Meshing
   =============

   Grillex supports plate elements for modeling deck plates, bulkheads,
   and other planar structures.

   Basic Plate Example
   -------------------

   .. doctest::

       >>> from grillex.core import StructuralModel
       >>> model = StructuralModel(name="Plate Example")
       >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

       # Add a 4m x 2m plate
       >>> plate = model.add_plate(
       ...     corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
       ...     thickness=0.02,
       ...     material="Steel",
       ...     mesh_size=0.5
       ... )

       # Set edge divisions for structured mesh
       >>> model.set_edge_divisions(plate, 0, 8)  # 8 elements along bottom
       >>> model.set_edge_divisions(plate, 2, 8)  # 8 elements along top

       # Add support along one edge
       >>> _ = model.add_support_curve(plate, edge_index=0, ux=True, uy=True, uz=True)

       # Generate mesh
       >>> stats = model.mesh()
       >>> print(f"Created {stats.n_plate_elements} plate elements")
       Created 16 plate elements

   Plate-Beam Coupling
   -------------------

   Plates can be coupled to beams using rigid links with optional
   moment releases and eccentric offsets...
   ```

3. Update CLAUDE.md with plate meshing section

**Acceptance Criteria:**
- [x] All new methods have complete docstrings with units
- [x] User documentation with working examples
- [x] Doctests pass
- [x] CLAUDE.md updated with plate meshing guidance

### Execution Notes (Completed 2025-12-28)

**Steps Taken:**
1. Created `docs/user/plate_meshing.rst` with comprehensive documentation:
   - Overview and two-step meshing workflow
   - Basic plate example with doctest
   - Mesh control and edge divisions
   - Element types table (MITC4, MITC8, MITC9, DKT)
   - Boundary conditions with support curves
   - Plate-beam coupling with offsets and releases
   - Mesh generation and statistics
   - Result querying (displacement, moments, stress)
   - Units reference table
   - Troubleshooting section
2. Added `plate_meshing` to `docs/user/index.rst` toctree
3. Added "Creating a Plate Model" section to CLAUDE.md

**Verification:**
- All 37 doctests in plate_meshing.rst passing ✓
- Documentation builds correctly ✓

---

## Summary

Phase 19 introduces comprehensive plate meshing capabilities:

| Feature | Description |
|---------|-------------|
| General Polygons | Support for 3+ corner plates |
| Gmsh Integration | Industry-standard mesh generation |
| Quad Preference | Quads preferred, triangles as fallback |
| Higher-Order Elements | MITC8 (8-node), MITC9 (9-node) |
| Element Type Selection | Infrastructure for future element types |
| Mesh Control | Element size and per-edge divisions |
| Plate-Beam Coupling | Rigid links with releases and offsets |
| Support Curves | Boundary conditions along edges |
| Unified Meshing | Single mesh() for plates and beams |

**Key Design Decisions:**
1. **Gmsh for meshing** - Proven library with quad recombination
2. **Edge divisions take precedence** - For structured mesh control
3. **Matching edges required** - No transition elements (simplicity)
4. **Rigid links for coupling** - Consistent with existing constraint system
5. **Unified mesh()** - Single entry point for all meshing

**Dependencies:**
- gmsh Python package (GPL with linking exception)
- Phase 8 complete (PlateElement in C++)
- Phase 6 complete (rigid links)

**Estimated Effort:** 15-20 developer days
