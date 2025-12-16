"""
Test suite for Phase 5: Line Load Equivalent Nodal Forces (Task 5.2)

Tests the distributed line load functionality:
1. Equivalent nodal forces computation
2. Uniform and trapezoidal loads
3. Fixed-end moment verification
4. Reactions match analytical solutions

Acceptance Criteria (Task 5.2):
- AC1: Uniform load produces correct reactions
- AC2: Fixed-end moments match theory
- AC3: Trapezoidal loads work correctly
- AC4: BeamElement can query distributed loads (for Phase 7)
- AC5: DistributedLoad structure compatible with Phase 7

Requirements: R-LOAD-001
"""

import pytest
import numpy as np

from grillex.core import (
    StructuralModel,
    DOFIndex,
    DistributedLoad,
)


class TestDistributedLoadStruct:
    """Test the DistributedLoad struct for Phase 7 compatibility"""

    def test_zero_load(self):
        """Test zero distributed load"""
        dl = DistributedLoad()
        assert dl.q_start == 0.0
        assert dl.q_end == 0.0
        assert dl.is_zero() is True
        assert dl.is_uniform() is True

    def test_uniform_load(self):
        """Test uniform distributed load"""
        dl = DistributedLoad()
        dl.q_start = 10.0
        dl.q_end = 10.0
        assert dl.is_uniform() is True
        assert dl.is_zero() is False

        # Check load at any position
        assert dl.at(0.0, 6.0) == pytest.approx(10.0)
        assert dl.at(3.0, 6.0) == pytest.approx(10.0)
        assert dl.at(6.0, 6.0) == pytest.approx(10.0)

    def test_trapezoidal_load(self):
        """Test linearly varying distributed load"""
        dl = DistributedLoad()
        dl.q_start = 5.0
        dl.q_end = 15.0
        assert dl.is_uniform() is False
        assert dl.is_zero() is False

        # Check linear interpolation
        L = 6.0
        assert dl.at(0.0, L) == pytest.approx(5.0)
        assert dl.at(3.0, L) == pytest.approx(10.0)  # Midpoint
        assert dl.at(6.0, L) == pytest.approx(15.0)


class TestEquivalentNodalForces:
    """Test the BeamElement.equivalent_nodal_forces method directly"""

    def test_uniform_vertical_load_simple(self):
        """Test uniform vertical load produces correct equivalent forces"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create a 6m horizontal beam along X axis
        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        elem = beam.elements[0]

        # Uniform load of 10 kN/m in -Y direction (global)
        w_start = np.array([0.0, -10.0, 0.0])
        w_end = np.array([0.0, -10.0, 0.0])

        f_equiv = elem.equivalent_nodal_forces(w_start, w_end)

        # For uniform load w over length L:
        # Total force = w * L = 10 * 6 = 60 kN
        # Force at each end = 30 kN

        L = 6.0
        w = 10.0

        # Check vertical reactions (should be wL/2 each = 30 kN)
        # Note: load is -Y, so reactions should be positive Y at both ends
        # Actually, equivalent nodal forces are the LOADS, not reactions
        # So they should be negative (same direction as applied load)
        assert f_equiv[1] == pytest.approx(-w * L / 2, rel=1e-6)  # Fy at node i
        assert f_equiv[7] == pytest.approx(-w * L / 2, rel=1e-6)  # Fy at node j

        # Check moments (fixed-end moments = wL²/12)
        # For beam along X, bending about Z (Mz)
        # At node i: +wL²/12 (counterclockwise from above)
        # At node j: -wL²/12
        fem = w * L**2 / 12
        assert f_equiv[5] == pytest.approx(-fem, rel=1e-6)   # Mz at node i
        assert f_equiv[11] == pytest.approx(fem, rel=1e-6)  # Mz at node j


class TestCantileverWithUniformLoad:
    """Test cantilever beam with uniform distributed load"""

    def test_cantilever_uniform_load_reactions(self):
        """AC1: Uniform load produces correct reactions for cantilever"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # 6m cantilever along X axis
        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        # Fix left end (all DOFs)
        model.fix_node_at([0, 0, 0])

        # Apply uniform vertical load of 10 kN/m
        model.add_line_load(beam, [0, -10, 0])

        # Analyze
        success = model.analyze()
        assert success is True

        # Get reactions at fixed support
        reactions = model.get_reactions_at([0, 0, 0])

        # For cantilever with UDL:
        # Vertical reaction R = w * L = 10 * 6 = 60 kN (upward)
        L = 6.0
        w = 10.0
        expected_fy = w * L  # 60 kN

        assert reactions[DOFIndex.UY] == pytest.approx(expected_fy, rel=0.01)

        # Moment reaction M = w * L² / 2 = 10 * 36 / 2 = 180 kN·m
        # Sign convention: for downward load, moment at fixed end is clockwise
        # Looking from +Z, clockwise is negative Mz
        expected_mz = w * L**2 / 2  # 180 kN·m
        assert abs(reactions[DOFIndex.RZ]) == pytest.approx(expected_mz, rel=0.01)

    def test_cantilever_uniform_load_tip_deflection(self):
        """Test tip deflection of cantilever with uniform load"""
        model = StructuralModel()
        E = 210e9  # Pa = N/m²
        I = 1e-4   # m⁴ (moment of inertia)
        model.add_material("Steel", E / 1e3, 0.3, 7.85e-6)  # E in kN/m² = kPa
        model.add_section("Test", 0.01, I, I, 1.5e-5)

        L = 6.0  # m
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel")

        model.fix_node_at([0, 0, 0])

        w = 10.0  # kN/m
        model.add_line_load(beam, [0, -w, 0])

        success = model.analyze()
        assert success is True

        # Tip deflection for cantilever with UDL: δ = wL⁴/(8EI)
        # Note: w in kN/m, E in kN/m², I in m⁴, L in m
        E_kn = E / 1e3  # Convert to kN/m²
        expected_deflection = w * L**4 / (8 * E_kn * I)

        tip_disp = model.get_displacement_at([L, 0, 0], DOFIndex.UY)

        # Should be negative (downward)
        assert tip_disp < 0
        assert abs(tip_disp) == pytest.approx(expected_deflection, rel=0.05)


class TestSimplySupportedBeam:
    """Test simply supported beam with uniform load"""

    def test_simply_supported_uniform_load(self):
        """AC2: Fixed-end moments match theory for simply supported beam"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        L = 8.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")

        # Pin at left (fix translations, free rotations)
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)

        # Roller at right (fix UY only)
        model.fix_dof_at([L, 0, 0], DOFIndex.UY, 0.0)

        # Also restrain out-of-plane to prevent instability
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([L, 0, 0], DOFIndex.UZ, 0.0)

        w = 12.0  # kN/m
        model.add_line_load(beam, [0, -w, 0])

        success = model.analyze()
        assert success is True

        # Reactions should be wL/2 at each support
        reactions_left = model.get_reactions_at([0, 0, 0])
        reactions_right = model.get_reactions_at([L, 0, 0])

        expected_reaction = w * L / 2  # = 12 * 8 / 2 = 48 kN

        assert reactions_left[DOFIndex.UY] == pytest.approx(expected_reaction, rel=0.01)
        assert reactions_right[DOFIndex.UY] == pytest.approx(expected_reaction, rel=0.01)


class TestTrapezoidalLoad:
    """Test trapezoidal (linearly varying) distributed loads"""

    def test_trapezoidal_load_reactions(self):
        """AC3: Trapezoidal loads work correctly"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")

        # Fix left end
        model.fix_node_at([0, 0, 0])

        # Trapezoidal load: 5 kN/m at start, 15 kN/m at end
        w1 = 5.0
        w2 = 15.0
        model.add_line_load(beam, [0, -w1, 0], [0, -w2, 0])

        success = model.analyze()
        assert success is True

        # Total load = average * L = ((w1 + w2) / 2) * L = 10 * 6 = 60 kN
        # Reactions for cantilever with trapezoidal load
        reactions = model.get_reactions_at([0, 0, 0])

        total_load = (w1 + w2) / 2 * L
        assert reactions[DOFIndex.UY] == pytest.approx(total_load, rel=0.02)


class TestLineLoadAPI:
    """Test the Python API for adding line loads"""

    def test_add_line_load_uniform(self):
        """Test adding uniform line load via API"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        # Should work with just w_start (uniform load)
        model.add_line_load(beam, [0, -10, 0])

        model.fix_node_at([0, 0, 0])
        success = model.analyze()
        assert success is True

    def test_add_line_load_by_coords(self):
        """Test adding line load by beam coordinates"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

        # Add load by coordinates
        model.add_line_load_by_coords([0, 0, 0], [6, 0, 0], [0, -10, 0])

        model.fix_node_at([0, 0, 0])
        success = model.analyze()
        assert success is True

    def test_find_beam_by_coords(self):
        """Test finding beam by coordinates"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        beam1 = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        beam2 = model.add_beam_by_coords([6, 0, 0], [12, 0, 0], "IPE300", "Steel")

        # Find first beam
        found = model.find_beam_by_coords([0, 0, 0], [6, 0, 0])
        assert found is not None
        assert found.beam_id == beam1.beam_id

        # Find second beam (reversed coordinates)
        found = model.find_beam_by_coords([12, 0, 0], [6, 0, 0])
        assert found is not None
        assert found.beam_id == beam2.beam_id

        # Should not find non-existent beam
        not_found = model.find_beam_by_coords([0, 0, 0], [100, 0, 0])
        assert not_found is None


class TestLineLoadWithSubdividedBeam:
    """Test line loads on beams that have been subdivided"""

    def test_uniform_load_on_subdivided_beam(self):
        """Test uniform load on beam with internal nodes"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        # Create beam and subdivide
        beam = model.add_beam_by_coords([0, 0, 0], [12, 0, 0], "IPE300", "Steel")
        model.get_or_create_node(6, 0, 0)  # Internal node
        n_split = model.subdivide_beams()
        assert n_split == 1
        assert len(model.beams) == 2

        # Apply uniform load to one of the sub-beams
        sub_beam = model.beams[0]  # First sub-beam (0-6m)
        model.add_line_load(sub_beam, [0, -10, 0])

        model.fix_node_at([0, 0, 0])
        success = model.analyze()
        assert success is True


class TestAcceptanceCriteria:
    """Test the acceptance criteria from Task 5.2"""

    def test_ac1_uniform_load_correct_reactions(self):
        """AC1: Uniform load produces correct reactions"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        w = 10.0
        model.add_line_load(beam, [0, -w, 0])

        success = model.analyze()
        assert success is True

        reactions = model.get_reactions_at([0, 0, 0])
        expected_fy = w * L
        assert reactions[DOFIndex.UY] == pytest.approx(expected_fy, rel=0.01)

    def test_ac2_fixed_end_moments_match_theory(self):
        """AC2: Fixed-end moments match theory (wL²/12 for fixed-fixed)"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")

        # Fixed-fixed beam
        model.fix_node_at([0, 0, 0])
        model.fix_node_at([L, 0, 0])

        w = 10.0
        model.add_line_load(beam, [0, -w, 0])

        success = model.analyze()
        assert success is True

        # For fixed-fixed beam with UDL:
        # Moment at supports = wL²/12
        expected_fem = w * L**2 / 12

        reactions_left = model.get_reactions_at([0, 0, 0])
        reactions_right = model.get_reactions_at([L, 0, 0])

        # Both ends should have same magnitude of moment
        assert abs(reactions_left[DOFIndex.RZ]) == pytest.approx(expected_fem, rel=0.01)
        assert abs(reactions_right[DOFIndex.RZ]) == pytest.approx(expected_fem, rel=0.01)

    def test_ac3_trapezoidal_loads_correct(self):
        """AC3: Trapezoidal loads work correctly"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        w1 = 5.0
        w2 = 15.0
        model.add_line_load(beam, [0, -w1, 0], [0, -w2, 0])

        success = model.analyze()
        assert success is True

        reactions = model.get_reactions_at([0, 0, 0])
        total_load = (w1 + w2) / 2 * L
        assert reactions[DOFIndex.UY] == pytest.approx(total_load, rel=0.02)

    def test_ac4_distributed_load_struct_phase7_compatible(self):
        """AC4/AC5: DistributedLoad structure compatible with Phase 7"""
        # Test that DistributedLoad can represent beam loads
        dl = DistributedLoad()
        dl.q_start = 10.0
        dl.q_end = 10.0

        # Should be able to query load at any position
        L = 6.0
        for x in [0.0, 1.5, 3.0, 4.5, 6.0]:
            q = dl.at(x, L)
            assert q == pytest.approx(10.0)

        # Trapezoidal load
        dl.q_start = 5.0
        dl.q_end = 15.0
        assert dl.at(0.0, L) == pytest.approx(5.0)
        assert dl.at(L/2, L) == pytest.approx(10.0)
        assert dl.at(L, L) == pytest.approx(15.0)


class TestMultipleLoadCases:
    """Test line loads with multiple load cases"""

    def test_different_loads_different_cases(self):
        """Test applying different line loads to different load cases"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.01, 1e-5, 2e-5, 1.5e-5)

        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create two load cases with different line loads
        lc1 = model.create_load_case("Dead Load")
        lc2 = model.create_load_case("Live Load")

        model.add_line_load(beam, [0, -10, 0], load_case=lc1)  # 10 kN/m
        model.add_line_load(beam, [0, -5, 0], load_case=lc2)   # 5 kN/m

        success = model.analyze()
        assert success is True

        # Check reactions for each case
        model.set_active_load_case(lc1)
        r1 = model.get_reactions_at([0, 0, 0])

        model.set_active_load_case(lc2)
        r2 = model.get_reactions_at([0, 0, 0])

        assert r1[DOFIndex.UY] == pytest.approx(60.0, rel=0.01)  # 10 * 6
        assert r2[DOFIndex.UY] == pytest.approx(30.0, rel=0.01)  # 5 * 6
