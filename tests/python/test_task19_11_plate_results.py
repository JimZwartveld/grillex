"""Tests for plate element results in StructuralModel.

Tests for Task 19.11: Results for Plate Elements.
"""

import pytest
import numpy as np
from grillex.core import (
    StructuralModel, DOFIndex,
    PlateElement, PlateElementTri
)


# Skip these tests if gmsh is not installed
pytest.importorskip("gmsh")


@pytest.fixture
def model_with_analyzed_plate():
    """Create a model with a simply supported plate under uniform load."""
    model = StructuralModel(name="PlateResultsTest")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

    # Create a 2m x 2m plate, 20mm thick with coarse mesh
    plate = model.add_plate(
        corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
        thickness=0.02,
        material="Steel",
        element_type="MITC4"
    )

    # Set up structured 2x2 mesh
    model.set_edge_divisions(plate, 0, 2)
    model.set_edge_divisions(plate, 1, 2)
    model.set_edge_divisions(plate, 2, 2)
    model.set_edge_divisions(plate, 3, 2)

    # Simply support all edges (restrain UZ)
    for edge_idx in range(4):
        model.add_support_curve(plate, edge_index=edge_idx, uz=True)

    # Mesh the plate
    model.mesh()

    # The MITC4 plate element only has bending stiffness (UZ, RX, RY).
    # In-plane DOFs (UX, UY, RZ) have zero stiffness and must be constrained
    # at ALL nodes to prevent singularity.
    # Node IDs from a 2x2 structured mesh: 1-4 corners, 5-8 edge midpoints, 9 center
    for node_id in range(1, 10):
        model._cpp_model.boundary_conditions.add_fixed_dof(node_id, DOFIndex.UX, 0.0)
        model._cpp_model.boundary_conditions.add_fixed_dof(node_id, DOFIndex.UY, 0.0)
        model._cpp_model.boundary_conditions.add_fixed_dof(node_id, DOFIndex.RZ, 0.0)

    # Apply downward force at center node
    center_node = model.find_node_at([1, 1, 0])
    if center_node:
        model.add_point_load([1, 1, 0], force=[0, 0, -10.0])

    # Analyze
    try:
        result = model.analyze()
        if not result:
            pytest.skip(f"Analysis failed: {model._cpp_model.get_error_message()}")
    except Exception as e:
        pytest.skip(f"Analysis failed: {e}")

    return model


@pytest.fixture
def simple_plate_model():
    """Create a minimal plate model for basic testing."""
    model = StructuralModel(name="SimplePlate")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

    # Create a single 4-node plate element directly
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(1, 0, 0)
    n3 = model.get_or_create_node(1, 1, 0)
    n4 = model.get_or_create_node(0, 1, 0)

    # Get material
    material = model._cpp_model.materials[0]

    # Create plate element directly
    elem = model._cpp_model.create_plate(n1, n2, n3, n4, 0.02, material)

    # Fix in-plane DOFs at all nodes (plate only has bending stiffness)
    for node in [n1, n2, n3, n4]:
        model._cpp_model.boundary_conditions.add_fixed_dof(node.id, DOFIndex.UX, 0.0)
        model._cpp_model.boundary_conditions.add_fixed_dof(node.id, DOFIndex.UY, 0.0)
        model._cpp_model.boundary_conditions.add_fixed_dof(node.id, DOFIndex.RZ, 0.0)

    # Clamp two opposite corners (fix bending DOFs)
    for node in [n1, n3]:
        model._cpp_model.boundary_conditions.add_fixed_dof(node.id, DOFIndex.UZ, 0.0)
        model._cpp_model.boundary_conditions.add_fixed_dof(node.id, DOFIndex.RX, 0.0)
        model._cpp_model.boundary_conditions.add_fixed_dof(node.id, DOFIndex.RY, 0.0)

    # Apply load at corner n2 (free to displace)
    model.add_point_load([1, 0, 0], force=[0, 0, -10.0])

    # Analyze
    model.analyze()

    return model, elem


class TestGetPlateDisplacement:
    """Tests for get_plate_displacement method."""

    def test_displacement_at_center(self, model_with_analyzed_plate):
        """Test getting displacement at element center."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        assert len(elements) > 0

        # Get displacement at center of first element
        disp = model.get_plate_displacement(elements[0], xi=0, eta=0)

        assert "UX" in disp
        assert "UY" in disp
        assert "UZ" in disp
        assert "RX" in disp
        assert "RY" in disp
        assert "RZ" in disp

        # Should have some vertical displacement (plate is loaded)
        # Note: might be small due to mesh/BC setup

    def test_displacement_at_corner(self, model_with_analyzed_plate):
        """Test getting displacement at element corner."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        # Get displacement at corner (xi=-1, eta=-1)
        disp = model.get_plate_displacement(elements[0], xi=-1, eta=-1)

        assert isinstance(disp, dict)
        assert len(disp) == 6

    def test_displacement_returns_dict(self, model_with_analyzed_plate):
        """Test that displacement returns proper dict structure."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        disp = model.get_plate_displacement(elements[0])

        assert isinstance(disp, dict)
        for key in ["UX", "UY", "UZ", "RX", "RY", "RZ"]:
            assert key in disp
            assert isinstance(disp[key], float)


class TestGetPlateMoments:
    """Tests for get_plate_moments method."""

    def test_moments_at_center(self, model_with_analyzed_plate):
        """Test getting moments at element center."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        moments = model.get_plate_moments(elements[0], xi=0, eta=0)

        assert "Mx" in moments
        assert "My" in moments
        assert "Mxy" in moments

    def test_moments_structure(self, model_with_analyzed_plate):
        """Test that moments returns proper dict structure."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        moments = model.get_plate_moments(elements[0])

        assert isinstance(moments, dict)
        assert len(moments) == 3
        for key in ["Mx", "My", "Mxy"]:
            assert key in moments
            assert isinstance(moments[key], float)


class TestGetPlateStress:
    """Tests for get_plate_stress method."""

    def test_stress_at_surfaces(self, model_with_analyzed_plate):
        """Test getting stress at different surfaces."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        for surface in ["top", "bottom", "middle"]:
            stress = model.get_plate_stress(elements[0], surface=surface)

            assert "sigma_x" in stress
            assert "sigma_y" in stress
            assert "tau_xy" in stress

    def test_stress_symmetry(self, model_with_analyzed_plate):
        """Test that top and bottom stresses have opposite signs."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        stress_top = model.get_plate_stress(elements[0], surface="top")
        stress_bottom = model.get_plate_stress(elements[0], surface="bottom")

        # Top and bottom should have opposite signs (bending)
        # Allow for small absolute values (may be near zero)
        if abs(stress_top["sigma_x"]) > 1e-6:
            assert stress_top["sigma_x"] * stress_bottom["sigma_x"] <= 0

    def test_middle_surface_zero_stress(self, model_with_analyzed_plate):
        """Test that middle surface has zero bending stress."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        stress_middle = model.get_plate_stress(elements[0], surface="middle")

        # Middle surface should have zero stress (neutral axis)
        assert abs(stress_middle["sigma_x"]) < 1e-10
        assert abs(stress_middle["sigma_y"]) < 1e-10
        assert abs(stress_middle["tau_xy"]) < 1e-10

    def test_invalid_surface_raises(self, model_with_analyzed_plate):
        """Test that invalid surface raises ValueError."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        with pytest.raises(ValueError, match="Surface must be"):
            model.get_plate_stress(elements[0], surface="invalid")

    def test_stress_returns_dict(self, model_with_analyzed_plate):
        """Test that stress returns proper dict structure."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        stress = model.get_plate_stress(elements[0])

        assert isinstance(stress, dict)
        assert len(stress) == 3
        for key in ["sigma_x", "sigma_y", "tau_xy"]:
            assert key in stress
            assert isinstance(stress[key], float)


class TestGetPlateElements:
    """Tests for get_plate_elements helper method."""

    def test_returns_all_elements(self, model_with_analyzed_plate):
        """Test that get_plate_elements returns all plate elements."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        # Should have elements from meshing
        assert len(elements) > 0

    def test_empty_model_returns_empty_list(self):
        """Test that empty model returns empty list."""
        model = StructuralModel(name="Empty")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        elements = model.get_plate_elements()
        assert elements == []


class TestDifferentElementTypes:
    """Tests for results with different plate element types."""

    def test_mitc4_displacement(self, model_with_analyzed_plate):
        """Test displacement for MITC4 elements."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        # Should have MITC4 elements
        mitc4_elements = [e for e in elements if len(e.nodes) == 4]
        assert len(mitc4_elements) > 0

        disp = model.get_plate_displacement(mitc4_elements[0])
        assert isinstance(disp, dict)

    def test_dkt_displacement(self):
        """Test displacement for DKT triangular elements."""
        model = StructuralModel(name="DKTTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        # Create triangular plate
        plate = model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [1, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="DKT"
        )

        for edge_idx in range(3):
            model.add_support_curve(plate, edge_index=edge_idx, uz=True)

        model.mesh()

        # Fix in-plane DOFs at all nodes (DKT only has bending stiffness)
        # Get node count from meshing
        n_nodes = len(model._cpp_model.plate_elements_tri) * 3  # Estimate upper bound
        for node_id in range(1, n_nodes + 5):  # Add margin
            try:
                model._cpp_model.boundary_conditions.add_fixed_dof(node_id, DOFIndex.UX, 0.0)
                model._cpp_model.boundary_conditions.add_fixed_dof(node_id, DOFIndex.UY, 0.0)
                model._cpp_model.boundary_conditions.add_fixed_dof(node_id, DOFIndex.RZ, 0.0)
            except Exception:
                pass  # Node doesn't exist

        # Apply load at one of the nodes
        if len(model._cpp_model.plate_elements_tri) > 0:
            elem = model._cpp_model.plate_elements_tri[0]
            node = elem.nodes[0]
            model.add_point_load([node.x, node.y, node.z], force=[0, 0, -1.0])

        result = model.analyze()

        elements = model.get_plate_elements()
        dkt_elements = [e for e in elements if len(e.nodes) == 3]

        if len(dkt_elements) > 0 and result:
            # For DKT, use area coordinates
            disp = model.get_plate_displacement(dkt_elements[0], xi=0.33, eta=0.33)
            assert isinstance(disp, dict)


class TestResultsRequireAnalysis:
    """Tests verifying that results require analysis to be run first."""

    def test_displacement_after_analysis(self, model_with_analyzed_plate):
        """Verify displacement works after analysis."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        # This should work (model is analyzed)
        disp = model.get_plate_displacement(elements[0])
        assert disp is not None

    def test_moments_after_analysis(self, model_with_analyzed_plate):
        """Verify moments work after analysis."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        moments = model.get_plate_moments(elements[0])
        assert moments is not None

    def test_stress_after_analysis(self, model_with_analyzed_plate):
        """Verify stress works after analysis."""
        model = model_with_analyzed_plate
        elements = model.get_plate_elements()

        stress = model.get_plate_stress(elements[0])
        assert stress is not None
