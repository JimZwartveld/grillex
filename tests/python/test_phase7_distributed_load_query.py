"""
Test suite for Phase 7: Task 7.0 - Distributed Load Query Methods

Tests the BeamElement methods for querying distributed loads from LoadCase:
1. get_distributed_load_y() - local y-direction loads
2. get_distributed_load_z() - local z-direction loads
3. get_distributed_load_axial() - local x-direction (axial) loads

These methods are essential for Phase 7's differential equation approach
to computing accurate internal actions with distributed loads.

Acceptance Criteria (Task 7.0):
- AC1: get_distributed_load_y returns correct local y-component
- AC2: get_distributed_load_z returns correct local z-component
- AC3: get_distributed_load_axial returns correct local x-component
- AC4: Global to local transformation is correct
- AC5: Multiple line loads on same element are summed
- AC6: Returns zero load when no line loads on element
- AC7: Works with trapezoidal (linearly varying) loads
"""

import pytest
import numpy as np

from grillex.core import (
    Model,
    BeamConfig,
    LoadCaseType,
    DistributedLoad,
)


class TestDistributedLoadQueryBasics:
    """Basic tests for distributed load query methods"""

    @pytest.fixture
    def model_with_horizontal_beam_x(self):
        """Create a model with a 6m horizontal beam along global X axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(6.0, 0.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_no_loads_returns_zero(self, model_with_horizontal_beam_x):
        """Test that empty load case returns zero loads"""
        model, beam = model_with_horizontal_beam_x
        lc = model.create_load_case("Empty", LoadCaseType.Permanent)

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qy.is_zero()
        assert qz.is_zero()
        assert qx.is_zero()

    def test_load_on_different_element_returns_zero(self, model_with_horizontal_beam_x):
        """Test that load on a different element returns zero for this element"""
        model, beam = model_with_horizontal_beam_x
        lc = model.create_load_case("Other Element", LoadCaseType.Permanent)
        # Add load to element ID 99 (not our beam which has ID 1)
        lc.add_line_load(99, np.array([0.0, -10.0, 0.0]), np.array([0.0, -10.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qy.is_zero()
        assert qz.is_zero()
        assert qx.is_zero()


class TestHorizontalBeamAlongX:
    """Tests for horizontal beam along global X axis

    Local axes for beam along X:
    - local x: along beam (global X direction)
    - local y: perpendicular in XY plane (global Y direction for roll=0)
    - local z: perpendicular, completing right-hand system (global Z direction)
    """

    @pytest.fixture
    def model_beam_x(self):
        """Create a model with a 6m horizontal beam along global X axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(6.0, 0.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_global_y_load_becomes_local_y(self, model_beam_x):
        """Test that global Y load maps to local y for X-aligned beam"""
        model, beam = model_beam_x
        lc = model.create_load_case("Y Load", LoadCaseType.Permanent)
        # Uniform load of 10 kN/m in global Y direction
        lc.add_line_load(1, np.array([0.0, 10.0, 0.0]), np.array([0.0, 10.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qy.q_start == pytest.approx(10.0, rel=1e-6)
        assert qy.q_end == pytest.approx(10.0, rel=1e-6)
        assert qy.is_uniform()
        assert qz.is_zero()
        assert qx.is_zero()

    def test_global_z_load_becomes_local_z(self, model_beam_x):
        """Test that global Z load maps to local z for X-aligned beam"""
        model, beam = model_beam_x
        lc = model.create_load_case("Z Load", LoadCaseType.Permanent)
        # Uniform load of 15 kN/m in global Z direction (typically gravity direction)
        lc.add_line_load(1, np.array([0.0, 0.0, -15.0]), np.array([0.0, 0.0, -15.0]))

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qy.is_zero()
        assert qz.q_start == pytest.approx(-15.0, rel=1e-6)
        assert qz.q_end == pytest.approx(-15.0, rel=1e-6)
        assert qz.is_uniform()
        assert qx.is_zero()

    def test_global_x_load_becomes_axial(self, model_beam_x):
        """Test that global X load maps to axial for X-aligned beam"""
        model, beam = model_beam_x
        lc = model.create_load_case("Axial Load", LoadCaseType.Permanent)
        # Axial load of 5 kN/m in global X direction
        lc.add_line_load(1, np.array([5.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qy.is_zero()
        assert qz.is_zero()
        assert qx.q_start == pytest.approx(5.0, rel=1e-6)
        assert qx.q_end == pytest.approx(5.0, rel=1e-6)

    def test_combined_load(self, model_beam_x):
        """Test load with components in all directions"""
        model, beam = model_beam_x
        lc = model.create_load_case("Combined", LoadCaseType.Permanent)
        # Load with components in all global directions
        lc.add_line_load(1, np.array([1.0, 2.0, 3.0]), np.array([1.0, 2.0, 3.0]))

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qx.q_start == pytest.approx(1.0, rel=1e-6)
        assert qy.q_start == pytest.approx(2.0, rel=1e-6)
        assert qz.q_start == pytest.approx(3.0, rel=1e-6)


class TestHorizontalBeamAlongY:
    """Tests for horizontal beam along global Y axis

    Local axes for beam along Y:
    - local x: along beam (global Y direction)
    - local y: perpendicular (global -X direction for roll=0, due to right-hand rule)
    - local z: perpendicular (global Z direction)
    """

    @pytest.fixture
    def model_beam_y(self):
        """Create a model with a 6m horizontal beam along global Y axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(0.0, 6.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_global_z_load_becomes_local_z(self, model_beam_y):
        """Test that global Z load maps to local z for Y-aligned beam"""
        model, beam = model_beam_y
        lc = model.create_load_case("Z Load", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 0.0, -10.0]), np.array([0.0, 0.0, -10.0]))

        qz = beam.get_distributed_load_z(lc)

        assert qz.q_start == pytest.approx(-10.0, rel=1e-6)
        assert qz.is_uniform()

    def test_global_y_load_becomes_axial(self, model_beam_y):
        """Test that global Y load maps to axial for Y-aligned beam"""
        model, beam = model_beam_y
        lc = model.create_load_case("Axial", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 8.0, 0.0]), np.array([0.0, 8.0, 0.0]))

        qx = beam.get_distributed_load_axial(lc)

        assert qx.q_start == pytest.approx(8.0, rel=1e-6)
        assert qx.is_uniform()


class TestVerticalBeam:
    """Tests for vertical beam along global Z axis

    Local axes for vertical beam (along Z):
    - local x: along beam (global Z direction)
    - local y: perpendicular (global X direction for roll=0)
    - local z: perpendicular (global Y direction)
    """

    @pytest.fixture
    def model_beam_z(self):
        """Create a model with a 4m vertical beam along global Z axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(0.0, 0.0, 4.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_global_z_load_becomes_axial(self, model_beam_z):
        """Test that global Z load maps to axial for Z-aligned beam"""
        model, beam = model_beam_z
        lc = model.create_load_case("Axial", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 0.0, -5.0]), np.array([0.0, 0.0, -5.0]))

        qx = beam.get_distributed_load_axial(lc)
        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)

        # Global Z maps to local x (axial) for vertical beam
        assert qx.q_start == pytest.approx(-5.0, rel=1e-6)
        assert qy.is_zero()
        assert qz.is_zero()

    def test_global_x_load_becomes_local_z(self, model_beam_z):
        """Test that global X load maps to local z for vertical beam

        For vertical beam along Z:
        - local x: [0, 0, 1] (global Z direction)
        - local y: [0, -1, 0] (global -Y direction)
        - local z: [1, 0, 0] (global X direction)

        Therefore global X -> local z
        """
        model, beam = model_beam_z
        lc = model.create_load_case("Lateral", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([7.0, 0.0, 0.0]), np.array([7.0, 0.0, 0.0]))

        qz = beam.get_distributed_load_z(lc)

        # Global X maps to local z for vertical beam (with roll=0)
        assert qz.q_start == pytest.approx(7.0, rel=1e-6)


class TestTrapezoidalLoads:
    """Tests for linearly varying (trapezoidal) distributed loads"""

    @pytest.fixture
    def model_beam(self):
        """Create a model with a 6m horizontal beam along global X axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(6.0, 0.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_trapezoidal_load_y(self, model_beam):
        """Test trapezoidal load in local y direction"""
        model, beam = model_beam
        lc = model.create_load_case("Trapezoidal", LoadCaseType.Permanent)
        # Load varying from 5 to 15 kN/m in global Y direction
        lc.add_line_load(1, np.array([0.0, 5.0, 0.0]), np.array([0.0, 15.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)

        assert qy.q_start == pytest.approx(5.0, rel=1e-6)
        assert qy.q_end == pytest.approx(15.0, rel=1e-6)
        assert not qy.is_uniform()
        assert not qy.is_zero()

        # Check interpolation using DistributedLoad.at()
        L = beam.length
        assert qy.at(0.0, L) == pytest.approx(5.0, rel=1e-6)
        assert qy.at(L / 2, L) == pytest.approx(10.0, rel=1e-6)
        assert qy.at(L, L) == pytest.approx(15.0, rel=1e-6)

    def test_trapezoidal_load_z(self, model_beam):
        """Test trapezoidal load in local z direction"""
        model, beam = model_beam
        lc = model.create_load_case("Trapezoidal Z", LoadCaseType.Permanent)
        # Load varying from -20 to -5 kN/m in global Z direction
        lc.add_line_load(1, np.array([0.0, 0.0, -20.0]), np.array([0.0, 0.0, -5.0]))

        qz = beam.get_distributed_load_z(lc)

        assert qz.q_start == pytest.approx(-20.0, rel=1e-6)
        assert qz.q_end == pytest.approx(-5.0, rel=1e-6)
        assert not qz.is_uniform()

    def test_trapezoidal_load_axial(self, model_beam):
        """Test trapezoidal axial load"""
        model, beam = model_beam
        lc = model.create_load_case("Trapezoidal Axial", LoadCaseType.Permanent)
        # Axial load varying from 2 to 8 kN/m in global X direction
        lc.add_line_load(1, np.array([2.0, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]))

        qx = beam.get_distributed_load_axial(lc)

        assert qx.q_start == pytest.approx(2.0, rel=1e-6)
        assert qx.q_end == pytest.approx(8.0, rel=1e-6)
        assert not qx.is_uniform()


class TestMultipleLoads:
    """Tests for multiple line loads on the same element"""

    @pytest.fixture
    def model_beam(self):
        """Create a model with a 6m horizontal beam along global X axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(6.0, 0.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_multiple_loads_accumulate(self, model_beam):
        """Test that multiple line loads on same element are summed"""
        model, beam = model_beam
        lc = model.create_load_case("Multiple", LoadCaseType.Permanent)
        # Add two loads in global Y direction
        lc.add_line_load(1, np.array([0.0, 5.0, 0.0]), np.array([0.0, 5.0, 0.0]))
        lc.add_line_load(1, np.array([0.0, 3.0, 0.0]), np.array([0.0, 3.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)

        # Total should be 5 + 3 = 8 kN/m
        assert qy.q_start == pytest.approx(8.0, rel=1e-6)
        assert qy.q_end == pytest.approx(8.0, rel=1e-6)

    def test_multiple_loads_different_directions(self, model_beam):
        """Test multiple loads in different global directions"""
        model, beam = model_beam
        lc = model.create_load_case("Multi-Direction", LoadCaseType.Permanent)
        # Add loads in different global directions
        lc.add_line_load(1, np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))  # Axial
        lc.add_line_load(1, np.array([0.0, 2.0, 0.0]), np.array([0.0, 2.0, 0.0]))  # Y
        lc.add_line_load(1, np.array([0.0, 0.0, 3.0]), np.array([0.0, 0.0, 3.0]))  # Z

        qx = beam.get_distributed_load_axial(lc)
        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)

        assert qx.q_start == pytest.approx(1.0, rel=1e-6)
        assert qy.q_start == pytest.approx(2.0, rel=1e-6)
        assert qz.q_start == pytest.approx(3.0, rel=1e-6)

    def test_mixed_elements_in_load_case(self, model_beam):
        """Test that loads on other elements don't affect this element"""
        model, beam = model_beam
        lc = model.create_load_case("Mixed", LoadCaseType.Permanent)
        # Add load to this element (ID 1)
        lc.add_line_load(1, np.array([0.0, 10.0, 0.0]), np.array([0.0, 10.0, 0.0]))
        # Add load to different element (ID 2)
        lc.add_line_load(2, np.array([0.0, 100.0, 0.0]), np.array([0.0, 100.0, 0.0]))
        # Add load to another different element (ID 3)
        lc.add_line_load(3, np.array([0.0, 50.0, 0.0]), np.array([0.0, 50.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)

        # Should only include load from element ID 1
        assert qy.q_start == pytest.approx(10.0, rel=1e-6)
        assert qy.q_end == pytest.approx(10.0, rel=1e-6)


class TestDiagonalBeam:
    """Tests for beams at diagonal orientations to verify coordinate transform"""

    def test_diagonal_beam_xy_plane(self):
        """Test beam at 45 degrees in XY plane"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        # Beam from (0,0,0) to (4,4,0) - 45 degrees in XY plane
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(4.0, 4.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)

        lc = model.create_load_case("Test", LoadCaseType.Permanent)
        # Apply load in global Z direction
        lc.add_line_load(beam.id, np.array([0.0, 0.0, -10.0]), np.array([0.0, 0.0, -10.0]))

        qz = beam.get_distributed_load_z(lc)
        qy = beam.get_distributed_load_y(lc)
        qx = beam.get_distributed_load_axial(lc)

        # For beam in XY plane with local z ~ global Z:
        # Global Z load should map primarily to local z
        assert qz.q_start == pytest.approx(-10.0, rel=1e-6)
        assert qy.is_zero()
        assert qx.is_zero()


class TestAcceptanceCriteria:
    """Tests matching the acceptance criteria from Task 7.0"""

    @pytest.fixture
    def model_beam(self):
        """Create a model with a 6m horizontal beam along global X axis"""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)
        n1 = model.get_or_create_node(0.0, 0.0, 0.0)
        n2 = model.get_or_create_node(6.0, 0.0, 0.0)
        beam = model.create_beam(n1, n2, mat, sec)
        return model, beam

    def test_ac1_get_distributed_load_y_returns_correct_local_y(self, model_beam):
        """AC1: get_distributed_load_y returns correct local y-component"""
        model, beam = model_beam
        lc = model.create_load_case("Test", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 12.5, 0.0]), np.array([0.0, 12.5, 0.0]))

        qy = beam.get_distributed_load_y(lc)

        assert qy.q_start == pytest.approx(12.5, rel=1e-6)
        assert qy.q_end == pytest.approx(12.5, rel=1e-6)

    def test_ac2_get_distributed_load_z_returns_correct_local_z(self, model_beam):
        """AC2: get_distributed_load_z returns correct local z-component"""
        model, beam = model_beam
        lc = model.create_load_case("Test", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 0.0, -7.5]), np.array([0.0, 0.0, -7.5]))

        qz = beam.get_distributed_load_z(lc)

        assert qz.q_start == pytest.approx(-7.5, rel=1e-6)
        assert qz.q_end == pytest.approx(-7.5, rel=1e-6)

    def test_ac3_get_distributed_load_axial_returns_correct_local_x(self, model_beam):
        """AC3: get_distributed_load_axial returns correct local x-component"""
        model, beam = model_beam
        lc = model.create_load_case("Test", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([3.0, 0.0, 0.0]), np.array([3.0, 0.0, 0.0]))

        qx = beam.get_distributed_load_axial(lc)

        assert qx.q_start == pytest.approx(3.0, rel=1e-6)
        assert qx.q_end == pytest.approx(3.0, rel=1e-6)

    def test_ac4_global_to_local_transformation_correct(self, model_beam):
        """AC4: Global to local transformation is correct"""
        model, beam = model_beam
        lc = model.create_load_case("Test", LoadCaseType.Permanent)
        # Apply combined load in all global directions
        lc.add_line_load(1, np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0]))

        qx = beam.get_distributed_load_axial(lc)
        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)

        # For X-aligned beam: global X->local x, global Y->local y, global Z->local z
        assert qx.q_start == pytest.approx(1.0, rel=1e-6)
        assert qx.q_end == pytest.approx(4.0, rel=1e-6)
        assert qy.q_start == pytest.approx(2.0, rel=1e-6)
        assert qy.q_end == pytest.approx(5.0, rel=1e-6)
        assert qz.q_start == pytest.approx(3.0, rel=1e-6)
        assert qz.q_end == pytest.approx(6.0, rel=1e-6)

    def test_ac5_multiple_line_loads_summed(self, model_beam):
        """AC5: Multiple line loads on same element are summed"""
        model, beam = model_beam
        lc = model.create_load_case("Test", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 5.0, 0.0]), np.array([0.0, 5.0, 0.0]))
        lc.add_line_load(1, np.array([0.0, 7.0, 0.0]), np.array([0.0, 7.0, 0.0]))
        lc.add_line_load(1, np.array([0.0, 3.0, 0.0]), np.array([0.0, 3.0, 0.0]))

        qy = beam.get_distributed_load_y(lc)

        # Total: 5 + 7 + 3 = 15
        assert qy.q_start == pytest.approx(15.0, rel=1e-6)
        assert qy.q_end == pytest.approx(15.0, rel=1e-6)

    def test_ac6_returns_zero_when_no_loads(self, model_beam):
        """AC6: Returns zero load when no line loads on element"""
        model, beam = model_beam
        lc = model.create_load_case("Empty", LoadCaseType.Permanent)

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)
        qx = beam.get_distributed_load_axial(lc)

        assert qy.is_zero()
        assert qz.is_zero()
        assert qx.is_zero()

    def test_ac7_trapezoidal_loads_work(self, model_beam):
        """AC7: Works with trapezoidal (linearly varying) loads"""
        model, beam = model_beam
        lc = model.create_load_case("Trapezoidal", LoadCaseType.Permanent)
        lc.add_line_load(1, np.array([0.0, 5.0, -10.0]), np.array([0.0, 15.0, -20.0]))

        qy = beam.get_distributed_load_y(lc)
        qz = beam.get_distributed_load_z(lc)

        assert qy.q_start == pytest.approx(5.0, rel=1e-6)
        assert qy.q_end == pytest.approx(15.0, rel=1e-6)
        assert not qy.is_uniform()

        assert qz.q_start == pytest.approx(-10.0, rel=1e-6)
        assert qz.q_end == pytest.approx(-20.0, rel=1e-6)
        assert not qz.is_uniform()
