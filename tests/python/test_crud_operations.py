"""
Tests for CRUD operations on model entities.

This module tests create, read, update, and delete operations for:
- Load Cases
- Point Loads
- Line Loads
- Plates
- Boundary Conditions
"""

import pytest
import numpy as np
from grillex.core import StructuralModel, LoadCaseType, DOFIndex


@pytest.fixture
def simple_model():
    """Create a simple beam model for testing."""
    model = StructuralModel(name="Test Model")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85)
    model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel", name="Beam1")
    return model


class TestLoadCaseCRUD:
    """Tests for load case CRUD operations."""

    def test_create_load_case(self, simple_model):
        """Test creating a load case."""
        lc = simple_model.create_load_case("Dead Load", LoadCaseType.Permanent)
        assert lc.name == "Dead Load"
        assert lc.type == LoadCaseType.Permanent

    def test_get_load_cases(self, simple_model):
        """Test getting all load cases."""
        simple_model.create_load_case("LC1", LoadCaseType.Permanent)
        simple_model.create_load_case("LC2", LoadCaseType.Variable)

        load_cases = simple_model.get_load_cases()
        assert len(load_cases) >= 2  # May include default load case
        names = [lc.name for lc in load_cases]
        assert "LC1" in names
        assert "LC2" in names

    def test_get_load_case_by_name(self, simple_model):
        """Test getting a load case by name."""
        simple_model.create_load_case("TestLC", LoadCaseType.Variable)

        lc = simple_model.get_load_case("TestLC")
        assert lc is not None
        assert lc.name == "TestLC"

        # Non-existent load case
        assert simple_model.get_load_case("NonExistent") is None

    def test_delete_load_case(self, simple_model):
        """Test deleting a load case."""
        lc = simple_model.create_load_case("ToDelete", LoadCaseType.Variable)

        initial_count = len(simple_model.get_load_cases())
        result = simple_model.delete_load_case(lc)

        assert result is True
        assert len(simple_model.get_load_cases()) == initial_count - 1
        assert simple_model.get_load_case("ToDelete") is None

    def test_delete_load_case_removes_from_combinations(self, simple_model):
        """Test that deleting a load case removes it from combinations."""
        lc = simple_model.create_load_case("LC1", LoadCaseType.Permanent)
        combo = simple_model.create_load_combination("Combo1", "ULS-a")
        simple_model.add_load_case_to_combination(combo["id"], lc.id)

        # Verify it's in the combination
        combos = simple_model.get_load_combinations()
        assert len(combos[0]["load_cases"]) == 1

        # Delete and verify removal from combination
        simple_model.delete_load_case(lc)
        combos = simple_model.get_load_combinations()
        assert len(combos[0]["load_cases"]) == 0


class TestPointLoadCRUD:
    """Tests for point load CRUD operations."""

    def test_add_and_get_point_loads(self, simple_model):
        """Test adding and retrieving point loads."""
        simple_model.add_point_load([3, 0, 0], force=[0, 0, -10])
        simple_model.add_point_load([6, 0, 0], force=[0, -5, 0], moment=[0, 100, 0])

        loads = simple_model.get_point_loads()
        assert len(loads) == 2

        # Check first load
        assert loads[0]["index"] == 0
        np.testing.assert_array_almost_equal(loads[0]["force"], [0, 0, -10])

    def test_update_point_load(self, simple_model):
        """Test updating a point load."""
        simple_model.add_point_load([3, 0, 0], force=[0, 0, -10])

        # Update force
        result = simple_model.update_point_load(0, force=[0, 0, -20])
        assert result is True

        loads = simple_model.get_point_loads()
        np.testing.assert_array_almost_equal(loads[0]["force"], [0, 0, -20])

    def test_update_point_load_moment(self, simple_model):
        """Test updating only the moment of a point load."""
        simple_model.add_point_load([3, 0, 0], force=[0, 0, -10], moment=[0, 0, 0])

        result = simple_model.update_point_load(0, moment=[0, 50, 0])
        assert result is True

        loads = simple_model.get_point_loads()
        np.testing.assert_array_almost_equal(loads[0]["moment"], [0, 50, 0])
        # Force should remain unchanged
        np.testing.assert_array_almost_equal(loads[0]["force"], [0, 0, -10])

    def test_delete_point_load(self, simple_model):
        """Test deleting a point load."""
        simple_model.add_point_load([3, 0, 0], force=[0, 0, -10])
        simple_model.add_point_load([6, 0, 0], force=[0, 0, -5])

        assert len(simple_model.get_point_loads()) == 2

        result = simple_model.delete_point_load(0)
        assert result is True
        assert len(simple_model.get_point_loads()) == 1

    def test_delete_point_load_invalid_index(self, simple_model):
        """Test deleting a point load with invalid index."""
        result = simple_model.delete_point_load(999)
        assert result is False


class TestLineLoadCRUD:
    """Tests for line load CRUD operations."""

    def test_add_and_get_line_loads(self, simple_model):
        """Test adding and retrieving line loads."""
        beam = simple_model.beams[0]
        simple_model.add_line_load(beam, [0, 0, -5])

        loads = simple_model.get_beam_line_loads()
        assert len(loads) == 1
        np.testing.assert_array_almost_equal(loads[0].w_start, [0, 0, -5])

    def test_update_line_load(self, simple_model):
        """Test updating a line load."""
        beam = simple_model.beams[0]
        simple_model.add_line_load(beam, [0, 0, -5])

        result = simple_model.update_line_load(0, w_start=[0, 0, -10])
        assert result is True

        loads = simple_model.get_beam_line_loads()
        np.testing.assert_array_almost_equal(loads[0].w_start, [0, 0, -10])

    def test_delete_line_load(self, simple_model):
        """Test deleting a line load."""
        beam = simple_model.beams[0]
        simple_model.add_line_load(beam, [0, 0, -5])

        assert len(simple_model.get_beam_line_loads()) == 1

        result = simple_model.delete_line_load(0)
        assert result is True
        assert len(simple_model.get_beam_line_loads()) == 0


class TestPlateCRUD:
    """Tests for plate CRUD operations."""

    def test_add_and_get_plates(self, simple_model):
        """Test adding and retrieving plates."""
        plate = simple_model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            name="TestPlate"
        )

        plates = simple_model.get_plates()
        assert len(plates) == 1
        assert plates[0].name == "TestPlate"

    def test_get_plate_by_name(self, simple_model):
        """Test getting a plate by name."""
        simple_model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            name="MyPlate"
        )

        plate = simple_model.get_plate("MyPlate")
        assert plate is not None
        assert plate.name == "MyPlate"

        # Non-existent plate
        assert simple_model.get_plate("NonExistent") is None

    def test_delete_plate(self, simple_model):
        """Test deleting a plate."""
        plate = simple_model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            name="ToDelete"
        )

        assert len(simple_model.get_plates()) == 1

        result = simple_model.delete_plate(plate)
        assert result is True
        assert len(simple_model.get_plates()) == 0


class TestBoundaryConditionCRUD:
    """Tests for boundary condition CRUD operations."""

    def test_fix_node_and_get_bcs(self, simple_model):
        """Test fixing a node and retrieving boundary conditions."""
        simple_model.fix_node_at([0, 0, 0])

        bcs = simple_model.get_boundary_conditions()
        # Should have 6 DOFs fixed (UX, UY, UZ, RX, RY, RZ)
        assert len(bcs) == 6

        # All should be at same position
        positions = [bc["position"] for bc in bcs]
        for pos in positions:
            np.testing.assert_array_almost_equal(pos, [0, 0, 0])

    def test_update_boundary_condition(self, simple_model):
        """Test updating a boundary condition's prescribed value."""
        simple_model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)

        # Update to prescribed displacement
        result = simple_model.update_boundary_condition([0, 0, 0], DOFIndex.UY, 0.001)
        assert result is True

        bcs = simple_model.get_boundary_conditions()
        uy_bcs = [bc for bc in bcs if bc["dof_name"] == "UY"]
        assert len(uy_bcs) == 1
        assert abs(uy_bcs[0]["value"] - 0.001) < 1e-10

    def test_remove_boundary_condition_single(self, simple_model):
        """Test removing a single boundary condition."""
        simple_model.fix_node_at([0, 0, 0])

        initial_count = len(simple_model.get_boundary_conditions())

        # Remove just UZ
        count = simple_model.remove_boundary_condition([0, 0, 0], DOFIndex.UZ)
        assert count == 1

        bcs = simple_model.get_boundary_conditions()
        assert len(bcs) == initial_count - 1

        # Verify UZ is removed
        dof_names = [bc["dof_name"] for bc in bcs]
        assert "UZ" not in dof_names

    def test_remove_boundary_condition_all_at_node(self, simple_model):
        """Test removing all boundary conditions at a node."""
        simple_model.fix_node_at([0, 0, 0])

        count = simple_model.remove_boundary_condition([0, 0, 0])
        assert count == 6  # All 6 DOFs

        bcs = simple_model.get_boundary_conditions()
        assert len(bcs) == 0


class TestLoadCaseSpecificLoads:
    """Test CRUD operations with specific load cases."""

    def test_point_loads_per_load_case(self, simple_model):
        """Test point loads are correctly associated with load cases."""
        lc1 = simple_model.create_load_case("LC1", LoadCaseType.Permanent)
        lc2 = simple_model.create_load_case("LC2", LoadCaseType.Variable)

        simple_model.add_point_load([3, 0, 0], force=[0, 0, -10], load_case=lc1)
        simple_model.add_point_load([6, 0, 0], force=[0, 0, -5], load_case=lc2)

        loads_lc1 = simple_model.get_point_loads(lc1)
        loads_lc2 = simple_model.get_point_loads(lc2)

        assert len(loads_lc1) == 1
        assert len(loads_lc2) == 1
        np.testing.assert_array_almost_equal(loads_lc1[0]["force"], [0, 0, -10])
        np.testing.assert_array_almost_equal(loads_lc2[0]["force"], [0, 0, -5])

    def test_line_loads_per_load_case(self, simple_model):
        """Test line loads are correctly associated with load cases."""
        beam = simple_model.beams[0]
        lc1 = simple_model.create_load_case("LC1", LoadCaseType.Permanent)
        lc2 = simple_model.create_load_case("LC2", LoadCaseType.Variable)

        simple_model.add_line_load(beam, [0, 0, -5], load_case=lc1)
        simple_model.add_line_load(beam, [0, 0, -10], load_case=lc2)

        loads_lc1 = simple_model.get_beam_line_loads(lc1)
        loads_lc2 = simple_model.get_beam_line_loads(lc2)

        assert len(loads_lc1) == 1
        assert len(loads_lc2) == 1
        np.testing.assert_array_almost_equal(loads_lc1[0].w_start, [0, 0, -5])
        np.testing.assert_array_almost_equal(loads_lc2[0].w_start, [0, 0, -10])


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
