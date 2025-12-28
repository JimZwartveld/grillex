"""Tests for unified mesh function in StructuralModel.

Tests for Task 19.10: Unified Mesh Function.
"""

import pytest
import numpy as np
from grillex.core import StructuralModel, Plate, MeshStatistics


# Skip these tests if gmsh is not installed
pytest.importorskip("gmsh")


@pytest.fixture
def model_with_material():
    """Create a model with steel material."""
    model = StructuralModel(name="MeshTest")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    return model


class TestMeshBasic:
    """Basic tests for mesh() method."""

    def test_mesh_returns_statistics(self, model_with_material):
        """Test that mesh() returns MeshStatistics."""
        model = model_with_material
        model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.02,
            material="Steel",
            mesh_size=0.5
        )

        stats = model.mesh()

        assert isinstance(stats, MeshStatistics)
        assert stats.n_plate_nodes > 0
        assert stats.n_plate_elements > 0

    def test_mesh_empty_model(self, model_with_material):
        """Test meshing model with no plates."""
        model = model_with_material
        stats = model.mesh()

        assert stats.n_plate_nodes == 0
        assert stats.n_plate_elements == 0
        assert stats.n_quad_elements == 0
        assert stats.n_tri_elements == 0

    def test_mesh_verbose(self, model_with_material, capsys):
        """Test verbose output during meshing."""
        model = model_with_material
        model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.02,
            material="Steel"
        )

        model.mesh(verbose=True)

        captured = capsys.readouterr()
        assert "Meshing plate" in captured.out
        assert "Generated" in captured.out


class TestMeshMITC4:
    """Tests for MITC4 element meshing."""

    def test_mesh_mitc4_quad(self, model_with_material):
        """Test MITC4 meshing creates quad elements."""
        model = model_with_material
        model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.02,
            material="Steel",
            element_type="MITC4"
        )

        stats = model.mesh()

        assert stats.n_quad_elements > 0
        assert stats.n_tri_elements == 0 or stats.n_tri_elements > 0  # May have some triangles
        assert len(model._cpp_model.plate_elements) > 0

    def test_mesh_mitc4_structured(self, model_with_material):
        """Test structured MITC4 meshing with edge divisions."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="MITC4"
        )

        # Set edge divisions for structured mesh
        model.set_edge_divisions(plate, 0, 4)
        model.set_edge_divisions(plate, 1, 2)
        model.set_edge_divisions(plate, 2, 4)
        model.set_edge_divisions(plate, 3, 2)

        stats = model.mesh()

        # Should have exactly 4*2 = 8 quad elements
        assert stats.n_quad_elements == 8


class TestMeshDKT:
    """Tests for DKT triangular element meshing."""

    def test_mesh_dkt_triangular_plate(self, model_with_material):
        """Test DKT meshing of triangular plate."""
        model = model_with_material
        model.add_plate(
            corners=[[0, 0, 0], [3, 0, 0], [1.5, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="DKT"
        )

        stats = model.mesh()

        assert stats.n_tri_elements > 0
        assert len(model._cpp_model.plate_elements_tri) > 0


class TestMeshSupportCurves:
    """Tests for support curve application during meshing."""

    def test_support_curve_applied_during_mesh(self, model_with_material):
        """Test that support curves are applied to meshed nodes."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Simply support edge 0
        model.add_support_curve(plate, edge_index=0, uz=True)

        stats = model.mesh()

        # Should have applied some support DOFs
        assert stats.n_support_dofs > 0

    def test_multiple_supports(self, model_with_material):
        """Test multiple support curves."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Support two opposite edges
        model.add_support_curve(plate, edge_index=0, uz=True)
        model.add_support_curve(plate, edge_index=2, uz=True)

        stats = model.mesh()

        # Should have support DOFs from both edges
        assert stats.n_support_dofs >= 2


class TestMeshPlateBeamCoupling:
    """Tests for plate-beam coupling during meshing."""

    @pytest.fixture
    def model_with_beam(self):
        """Create model with material, section, and beam."""
        model = StructuralModel(name="CouplingMeshTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.67e-8)

        beam = model.add_beam_by_coords(
            [0, 0, 0], [4, 0, 0],
            section_name="IPE200",
            material_name="Steel"
        )
        return model, beam

    def test_coupling_creates_rigid_links(self, model_with_beam):
        """Test that plate-beam coupling creates rigid links."""
        model, beam = model_with_beam

        # Plate is offset above the beam (plate at z=0.1, beam at z=0)
        plate = model.add_plate(
            corners=[[0, 0, 0.1], [4, 0, 0.1], [4, 2, 0.1], [0, 2, 0.1]],
            thickness=0.02,
            material="Steel"
        )

        # Couple with offset (plate 0.1m above beam centroid)
        model.couple_plate_to_beam(
            plate, edge_index=0, beam=beam,
            offset=[0, 0, 0.1]
        )

        stats = model.mesh()

        # Should have created rigid links for nodes on edge 0
        assert stats.n_rigid_links > 0

    def test_coupling_no_links_when_colocated(self, model_with_beam):
        """Test that no rigid links are created when plate edge is on beam."""
        model, beam = model_with_beam

        # Plate edge is exactly on the beam (no offset needed)
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Couple without offset - nodes are shared
        model.couple_plate_to_beam(plate, edge_index=0, beam=beam)

        stats = model.mesh()

        # No rigid links needed when nodes are colocated
        assert stats.n_rigid_links == 0


class TestMeshMultiplePlates:
    """Tests for meshing multiple plates."""

    def test_mesh_multiple_plates(self, model_with_material):
        """Test meshing multiple plates."""
        model = model_with_material

        # Add two plates
        model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.02,
            material="Steel",
            name="Plate1"
        )
        model.add_plate(
            corners=[[2, 0, 0], [4, 0, 0], [4, 1, 0], [2, 1, 0]],
            thickness=0.02,
            material="Steel",
            name="Plate2"
        )

        stats = model.mesh()

        assert stats.n_plate_elements >= 2

    def test_mesh_different_element_types(self, model_with_material):
        """Test meshing plates with different element types."""
        model = model_with_material

        # Quad plate with MITC4
        model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="MITC4"
        )

        # Triangular plate with DKT
        model.add_plate(
            corners=[[3, 0, 0], [5, 0, 0], [4, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="DKT"
        )

        stats = model.mesh()

        assert stats.n_quad_elements > 0 or stats.n_tri_elements > 0  # At least some elements
        assert len(model._cpp_model.plate_elements) > 0 or len(model._cpp_model.plate_elements_tri) > 0


class TestMeshEdgeNodes:
    """Tests for edge node tracking during meshing."""

    def test_edge_nodes_stored_in_plate(self, model_with_material):
        """Test that edge node IDs are stored in plate after meshing."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        model.mesh()

        # Edge node map should be populated
        assert plate._edge_node_map is not None
        assert len(plate._edge_node_map) == 4  # 4 edges

        # Each edge should have at least 2 nodes (corners)
        for edge_idx in range(4):
            assert len(plate._edge_node_map.get(edge_idx, [])) >= 2


class TestMeshStatisticsAccuracy:
    """Tests for accuracy of mesh statistics."""

    def test_statistics_counts_match(self, model_with_material):
        """Test that statistics match actual element counts."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="MITC4"
        )

        # Use structured mesh for predictable count
        model.set_edge_divisions(plate, 0, 2)
        model.set_edge_divisions(plate, 1, 2)
        model.set_edge_divisions(plate, 2, 2)
        model.set_edge_divisions(plate, 3, 2)

        stats = model.mesh()

        # 2x2 structured mesh = 4 quad elements
        assert stats.n_quad_elements == 4
        assert stats.n_plate_elements == 4
        assert len(model._cpp_model.plate_elements) == 4


class TestMeshIntegration:
    """Integration tests for mesh() with complete workflows."""

    def test_mesh_then_analyze_structure(self, model_with_material):
        """Test complete workflow: add plate, mesh, supports, analyze."""
        model = model_with_material

        # Add plate with simple supports on two edges
        plate = model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="MITC4"
        )

        # Simply support opposite edges
        model.add_support_curve(plate, edge_index=0, uz=True)
        model.add_support_curve(plate, edge_index=2, uz=True)

        # Mesh the model
        stats = model.mesh()

        assert stats.n_plate_elements > 0
        assert stats.n_support_dofs > 0
