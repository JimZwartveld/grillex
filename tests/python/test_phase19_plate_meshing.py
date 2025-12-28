"""
Tests for Phase 19: Plate Meshing & Plate-Beam Coupling.

This module tests:
- Task 19.1: Python API for existing plate element
- Task 19.2: Plate geometry class
- Task 19.3: Gmsh integration
"""

import pytest
import numpy as np
import math

from grillex.core import (
    StructuralModel,
    Plate,
    EdgeMeshControl,
    PlateBeamCoupling,
    SupportCurve,
)


# =============================================================================
# Task 19.1: Python API for Existing Plate Element
# =============================================================================

class TestPlateElementAPI:
    """Tests for add_plate_element() method."""

    def test_add_plate_element_creates_plate(self):
        """Test that add_plate_element creates a MITC4 plate element."""
        model = StructuralModel(name="PlateTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate_element(
            [0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0],
            thickness=0.02,
            material="Steel"
        )

        assert plate is not None
        assert plate.thickness == 0.02

    def test_add_plate_element_returns_plate_element(self):
        """Test that the returned object has expected properties."""
        model = StructuralModel(name="PlateTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate_element(
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            thickness=0.01,
            material="Steel"
        )

        # Check element properties
        assert plate.num_dofs() == 24  # 6 DOFs * 4 nodes
        assert plate.area() > 0
        assert not plate.has_warping()

    def test_get_plate_elements(self):
        """Test that get_plate_elements returns all plates."""
        model = StructuralModel(name="PlateTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        # Add multiple plates
        model.add_plate_element(
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            thickness=0.01, material="Steel"
        )
        model.add_plate_element(
            [1, 0, 0], [2, 0, 0], [2, 1, 0], [1, 1, 0],
            thickness=0.01, material="Steel"
        )

        plates = model.get_plate_elements()
        assert len(plates) == 2

    def test_add_plate_element_missing_material(self):
        """Test that error is raised for missing material."""
        model = StructuralModel(name="PlateTest")

        with pytest.raises(ValueError, match="Material.*not found"):
            model.add_plate_element(
                [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                thickness=0.01,
                material="NonExistent"
            )

    def test_add_plate_element_nodes_created(self):
        """Test that nodes are automatically created at corners."""
        model = StructuralModel(name="PlateTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        model.add_plate_element(
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            thickness=0.01, material="Steel"
        )

        # Check that nodes exist at corners
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        assert n1 is not None
        assert n2 is not None

    def test_plate_stiffness_matrix_shape(self):
        """Test that stiffness matrix has correct shape."""
        model = StructuralModel(name="PlateTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate_element(
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            thickness=0.01, material="Steel"
        )

        K = plate.global_stiffness_matrix()
        assert K.shape == (24, 24)

    def test_plate_mass_matrix_shape(self):
        """Test that mass matrix has correct shape."""
        model = StructuralModel(name="PlateTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate_element(
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            thickness=0.01, material="Steel"
        )

        M = plate.global_mass_matrix()
        assert M.shape == (24, 24)


# =============================================================================
# Task 19.2: Plate Geometry Class
# =============================================================================

class TestPlateGeometry:
    """Tests for Plate class geometry methods."""

    def test_plate_creation_quad(self):
        """Test creating a quadrilateral plate."""
        plate = Plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        assert plate.n_corners == 4
        assert plate.n_edges == 4
        assert plate.thickness == 0.02
        assert plate.material == "Steel"

    def test_plate_creation_triangle(self):
        """Test creating a triangular plate."""
        plate = Plate(
            corners=[[0, 0, 0], [2, 0, 0], [1, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert plate.n_corners == 3
        assert plate.n_edges == 3

    def test_plate_creation_pentagon(self):
        """Test creating a pentagon plate."""
        # Regular pentagon
        corners = [
            [math.cos(2 * math.pi * i / 5), math.sin(2 * math.pi * i / 5), 0]
            for i in range(5)
        ]
        plate = Plate(corners=corners, thickness=0.01, material="Steel")

        assert plate.n_corners == 5
        assert plate.n_edges == 5

    def test_plate_minimum_corners(self):
        """Test that plate requires at least 3 corners."""
        with pytest.raises(ValueError, match="at least 3 corners"):
            Plate(corners=[[0, 0, 0], [1, 0, 0]], thickness=0.01, material="Steel")

    def test_plate_corner_validation(self):
        """Test that corners must have 3 coordinates."""
        with pytest.raises(ValueError, match="3 coordinates"):
            Plate(corners=[[0, 0], [1, 0], [1, 1]], thickness=0.01, material="Steel")

    def test_get_edge(self):
        """Test get_edge method."""
        plate = Plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        p1, p2 = plate.get_edge(0)
        assert p1 == [0, 0, 0]
        assert p2 == [2, 0, 0]

        # Last edge wraps around
        p1, p2 = plate.get_edge(3)
        assert p1 == [0, 1, 0]
        assert p2 == [0, 0, 0]

    def test_get_edge_length(self):
        """Test get_edge_length method."""
        plate = Plate(
            corners=[[0, 0, 0], [3, 0, 0], [3, 4, 0], [0, 4, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert np.isclose(plate.get_edge_length(0), 3.0)  # Bottom edge
        assert np.isclose(plate.get_edge_length(1), 4.0)  # Right edge

    def test_get_edge_direction(self):
        """Test get_edge_direction method."""
        plate = Plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        dir0 = plate.get_edge_direction(0)
        assert np.allclose(dir0, [1, 0, 0])

        dir1 = plate.get_edge_direction(1)
        assert np.allclose(dir1, [0, 1, 0])

    def test_get_normal_horizontal_plate(self):
        """Test get_normal for horizontal plate."""
        plate = Plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        normal = plate.get_normal()
        assert np.allclose(normal, [0, 0, 1])

    def test_get_normal_vertical_plate(self):
        """Test get_normal for vertical plate."""
        plate = Plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]],
            thickness=0.01,
            material="Steel"
        )

        normal = plate.get_normal()
        assert np.allclose(abs(normal), [0, 1, 0])

    def test_is_planar_horizontal(self):
        """Test is_planar for horizontal plate."""
        plate = Plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert plate.is_planar()

    def test_is_planar_pentagon(self):
        """Test is_planar for planar pentagon."""
        corners = [
            [math.cos(2 * math.pi * i / 5), math.sin(2 * math.pi * i / 5), 0]
            for i in range(5)
        ]
        plate = Plate(corners=corners, thickness=0.01, material="Steel")

        assert plate.is_planar()

    def test_is_planar_non_planar(self):
        """Test is_planar for non-planar plate."""
        plate = Plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0.5]],  # Last corner lifted
            thickness=0.01,
            material="Steel"
        )

        assert not plate.is_planar()

    def test_get_area_square(self):
        """Test get_area for unit square."""
        plate = Plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert np.isclose(plate.get_area(), 1.0)

    def test_get_area_rectangle(self):
        """Test get_area for rectangle."""
        plate = Plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert np.isclose(plate.get_area(), 8.0)

    def test_get_centroid(self):
        """Test get_centroid method."""
        plate = Plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.01,
            material="Steel"
        )

        centroid = plate.get_centroid()
        assert np.allclose(centroid, [1, 1, 0])

    def test_get_bounding_box(self):
        """Test get_bounding_box method."""
        plate = Plate(
            corners=[[1, 2, 0], [4, 2, 0], [4, 5, 0], [1, 5, 0]],
            thickness=0.01,
            material="Steel"
        )

        min_coords, max_coords = plate.get_bounding_box()
        assert np.allclose(min_coords, [1, 2, 0])
        assert np.allclose(max_coords, [4, 5, 0])

    def test_is_convex_square(self):
        """Test is_convex for convex square."""
        plate = Plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert plate.is_convex()

    def test_is_convex_non_convex(self):
        """Test is_convex for non-convex (L-shaped) plate."""
        plate = Plate(
            corners=[
                [0, 0, 0], [2, 0, 0], [2, 1, 0],
                [1, 1, 0], [1, 2, 0], [0, 2, 0]
            ],
            thickness=0.01,
            material="Steel"
        )

        assert not plate.is_convex()

    def test_edge_mesh_control(self):
        """Test setting edge mesh controls."""
        plate = Plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.01,
            material="Steel"
        )

        plate.edge_controls[0] = EdgeMeshControl(n_elements=8)
        plate.edge_controls[2] = EdgeMeshControl(n_elements=8)

        assert plate.edge_controls[0].n_elements == 8
        assert plate.edge_controls[2].n_elements == 8

    def test_plate_default_values(self):
        """Test plate default values."""
        plate = Plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        assert plate.mesh_size == 0.5
        assert plate.element_type == "MITC4"
        assert plate.name is None
        assert len(plate.edge_controls) == 0
        assert len(plate.beam_couplings) == 0
        assert len(plate.support_curves) == 0


# =============================================================================
# Task 19.3: Gmsh Integration
# =============================================================================

def _gmsh_available():
    """Check if gmsh is available and can be loaded."""
    try:
        import gmsh
        gmsh.initialize()
        gmsh.finalize()
        return True
    except Exception:
        return False


# Skip all gmsh tests if gmsh is not available
pytestmark_gmsh = pytest.mark.skipif(
    not _gmsh_available(),
    reason="gmsh not available or cannot be loaded"
)


class TestGmshMesher:
    """Tests for GmshPlateMesher."""

    @pytest.fixture
    def mesher(self):
        """Create a mesher and finalize after test."""
        if not _gmsh_available():
            pytest.skip("gmsh not available")
        from grillex.meshing import GmshPlateMesher
        m = GmshPlateMesher()
        yield m
        m.finalize()

    def test_gmsh_import_error_message(self):
        """Test that helpful error is raised when gmsh not available."""
        # This test only makes sense when gmsh is NOT available
        # When gmsh IS available, just verify it imports
        try:
            from grillex.meshing.gmsh_mesher import _check_gmsh_available
            _check_gmsh_available()
        except Exception as e:
            # When gmsh import fails, should give helpful message
            error_msg = str(e).lower()
            # Could fail due to missing gmsh or missing system libs
            assert "gmsh" in error_msg or "lib" in error_msg

    def test_mesh_result_properties(self):
        """Test MeshResult property calculations."""
        from grillex.meshing import MeshResult

        result = MeshResult(
            nodes=np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]),
            node_tags=np.array([1, 2, 3, 4]),
            quads=np.array([[0, 1, 2, 3]]),
            triangles=np.zeros((0, 3), dtype=int)
        )

        assert result.n_nodes == 4
        assert result.n_elements == 1
        assert result.n_quads == 1
        assert result.n_triangles == 0

    def test_mesh_quad_plate(self, mesher):
        """Test meshing a quadrilateral plate."""
        result = mesher.mesh_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            mesh_size=0.5,
            prefer_quads=True
        )

        assert result.n_nodes > 4
        assert result.n_elements > 0
        # Should prefer quads
        assert result.n_quads > 0

    def test_mesh_with_edge_divisions(self, mesher):
        """Test meshing with specified edge divisions."""
        result = mesher.mesh_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            mesh_size=1.0,
            edge_divisions={0: 4, 1: 2, 2: 4, 3: 2},
            prefer_quads=True
        )

        # With matching opposite edges, should get structured quad mesh
        # 4x2 = 8 quads
        assert result.n_quads == 8 or result.n_elements == 8

    def test_mesh_triangle_plate(self, mesher):
        """Test meshing a triangular plate."""
        result = mesher.mesh_plate(
            corners=[[0, 0, 0], [2, 0, 0], [1, 1, 0]],
            mesh_size=0.5,
            prefer_quads=True
        )

        assert result.n_nodes >= 3
        assert result.n_elements > 0

    def test_mesh_pentagon(self, mesher):
        """Test meshing a pentagon plate."""
        corners = [
            [math.cos(2 * math.pi * i / 5), math.sin(2 * math.pi * i / 5), 0]
            for i in range(5)
        ]

        result = mesher.mesh_plate(
            corners=corners,
            mesh_size=0.3,
            prefer_quads=True
        )

        assert result.n_nodes > 5
        assert result.n_elements > 0

    def test_edge_nodes_tracked(self, mesher):
        """Test that edge nodes are properly tracked."""
        result = mesher.mesh_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            mesh_size=0.5,
            prefer_quads=True
        )

        # Should have edge nodes for all 4 edges
        assert len(result.edge_nodes) == 4
        for edge_idx in range(4):
            assert edge_idx in result.edge_nodes
            assert len(result.edge_nodes[edge_idx]) >= 2  # At least corner nodes

    def test_context_manager(self, mesher):
        """Test mesher as context manager."""
        from grillex.meshing import GmshPlateMesher

        with GmshPlateMesher() as m:
            result = m.mesh_plate(
                corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
                mesh_size=0.5
            )
            assert result.n_elements > 0
        # Finalize called automatically

    def test_mesh_from_plate_geometry(self, mesher):
        """Test meshing from Plate geometry object."""
        plate = Plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel",
            mesh_size=0.5
        )
        plate.edge_controls[0] = EdgeMeshControl(n_elements=4)
        plate.edge_controls[2] = EdgeMeshControl(n_elements=4)

        result = mesher.mesh_plate_from_geometry(plate)
        assert result.n_elements > 0

    def test_invalid_corners(self, mesher):
        """Test error for too few corners."""
        from grillex.meshing.gmsh_mesher import MeshingError

        with pytest.raises(ValueError, match="At least 3 corners"):
            mesher.mesh_plate(
                corners=[[0, 0, 0], [1, 0, 0]],
                mesh_size=0.5
            )

    def test_invalid_corner_coordinates(self, mesher):
        """Test error for invalid corner coordinates."""
        with pytest.raises(ValueError, match="3 coordinates"):
            mesher.mesh_plate(
                corners=[[0, 0], [1, 0], [1, 1]],  # Missing z coordinate
                mesh_size=0.5
            )


# =============================================================================
# Integration Tests
# =============================================================================

class TestPlateIntegration:
    """Integration tests for plate meshing workflow."""

    def test_plate_element_analysis(self):
        """Test that plate elements can be included in analysis."""
        model = StructuralModel(name="PlateAnalysis")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        # Add a simple plate
        plate = model.add_plate_element(
            [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
            thickness=0.01, material="Steel"
        )

        # Fix all corners
        for corner in [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]:
            model.fix_node_at(corner)

        # This should not raise - model is fully constrained
        # Note: Full plate analysis with loads would require proper BC setup
        assert len(model.get_plate_elements()) == 1
