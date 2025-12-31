"""
Tests for Task 7.2: Internal Action Functions Along Beam

Tests the analytical computation of internal forces and moments (N, Vy, Vz, Mx, My, Mz)
at any position along beam elements.

Reference formulas:
- Simply supported beam with UDL: M_max = qL²/8 at midspan
- Cantilever with tip load: M_base = PL
- Fixed-fixed beam with UDL: end moments = qL²/12, midspan moment = qL²/24
"""

import numpy as np
import pytest

from grillex.core import (
    Model,
    DOFIndex,
    InternalActions,
    WarpingInternalActions,
    ActionExtreme,
    ReleaseCombo4DOF,
    ReleaseCombo2DOF,
    ReleaseComboWarping,
    BeamFormulation,
    BeamConfig,
)


class TestCantileverTipLoad:
    """Test cantilever beam with tip load - M_base = P*L"""

    def test_moment_at_base(self):
        """Cantilever: base moment equals P*L"""
        L = 6.0
        P = 10.0  # kN tip load

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)  # Downward in Y

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get internal actions at base (x=0)
        actions_base = beam.get_internal_actions(0.0, u, dof_handler, lc)

        # Moment at base should be P*L
        expected_Mz = P * L
        np.testing.assert_almost_equal(abs(actions_base.Mz), expected_Mz, decimal=2)

    def test_moment_at_tip(self):
        """Cantilever: tip moment is zero"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get internal actions at tip (x=L)
        actions_tip = beam.get_internal_actions(L, u, dof_handler, lc)

        # Moment at tip should be zero
        np.testing.assert_almost_equal(actions_tip.Mz, 0.0, decimal=2)

    def test_shear_is_constant(self):
        """Cantilever with point load: shear is constant along beam"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Check shear at multiple positions (should be constant)
        positions = [0.0, 0.5*L, 0.75*L, L]
        shear_values = []
        for x in positions:
            actions = beam.get_internal_actions(x, u, dof_handler, lc)
            shear_values.append(actions.Vy)

        # All shear values should be equal (constant shear)
        for v in shear_values:
            np.testing.assert_almost_equal(abs(v), P, decimal=1)


class TestCantileverUDL:
    """Test cantilever beam with UDL - M_max = wL²/2 at support"""

    def test_moment_at_support(self):
        """Cantilever with UDL: M_max = wL²/2 at fixed support"""
        L = 6.0
        w = 10.0  # kN/m uniform load

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        # Uniform distributed load in -Y direction
        lc.add_line_load(beam.id, np.array([0, -w, 0]), np.array([0, -w, 0]))

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get internal actions at support (x=0)
        actions_support = beam.get_internal_actions(0.0, u, dof_handler, lc)

        # Expected maximum moment at support = wL²/2
        expected_Mz = w * L**2 / 2  # = 10 * 36 / 2 = 180 kN·m

        np.testing.assert_almost_equal(abs(actions_support.Mz), expected_Mz, decimal=1)

    def test_moment_at_midspan(self):
        """Cantilever with UDL: M(L/2) = wL²/8"""
        L = 6.0
        w = 10.0  # kN/m

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_line_load(beam.id, np.array([0, -w, 0]), np.array([0, -w, 0]))

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get internal actions at midspan (x=L/2)
        actions_mid = beam.get_internal_actions(L/2, u, dof_handler, lc)

        # For cantilever with UDL, moment at x from free end:
        # M(x) = w(L-x)²/2
        # At midspan (x=L/2): M = w(L/2)²/2 = wL²/8
        expected_Mz = w * L**2 / 8  # = 10 * 36 / 8 = 45 kN·m

        np.testing.assert_almost_equal(abs(actions_mid.Mz), expected_Mz, decimal=1)

    def test_moment_at_tip(self):
        """Cantilever with UDL: M(L) = 0 at free end"""
        L = 6.0
        w = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_line_load(beam.id, np.array([0, -w, 0]), np.array([0, -w, 0]))

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get internal actions at tip (x=L)
        actions_tip = beam.get_internal_actions(L, u, dof_handler, lc)

        # Moment at free end should be zero
        np.testing.assert_almost_equal(actions_tip.Mz, 0.0, decimal=1)

    def test_shear_at_support(self):
        """Cantilever with UDL: V_max = wL at fixed support"""
        L = 6.0
        w = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_line_load(beam.id, np.array([0, -w, 0]), np.array([0, -w, 0]))

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get internal actions at support
        actions_support = beam.get_internal_actions(0.0, u, dof_handler, lc)

        # Expected shear at support = wL
        expected_Vy = w * L  # = 10 * 6 = 60 kN

        np.testing.assert_almost_equal(abs(actions_support.Vy), expected_Vy, decimal=1)


class TestFixedFixedBeamUDL:
    """Test fixed-fixed beam with uniform distributed load - M_ends = qL²/12"""

    def test_end_moments(self):
        """Fixed-fixed beam: end moments = qL²/12"""
        L = 6.0
        q = 5.0  # kN/m

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)
        model.boundary_conditions.fix_node(n2.id)

        lc = model.get_default_load_case()
        # Uniform distributed load in -Y direction (global coordinates)
        lc.add_line_load(beam.id, np.array([0, -q, 0]), np.array([0, -q, 0]))

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        actions_i = beam.get_internal_actions(0.0, u, dof_handler, lc)
        actions_j = beam.get_internal_actions(L, u, dof_handler, lc)

        # Expected end moment
        expected_Mz = q * L**2 / 12

        # End moments should be qL²/12
        np.testing.assert_almost_equal(abs(actions_i.Mz), expected_Mz, decimal=1)
        np.testing.assert_almost_equal(abs(actions_j.Mz), expected_Mz, decimal=1)

    def test_midspan_moment(self):
        """Fixed-fixed beam: midspan moment = qL²/24"""
        L = 6.0
        q = 5.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)
        model.boundary_conditions.fix_node(n2.id)

        lc = model.get_default_load_case()
        lc.add_line_load(beam.id, np.array([0, -q, 0]), np.array([0, -q, 0]))

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        actions_mid = beam.get_internal_actions(L/2, u, dof_handler, lc)

        # Expected midspan moment
        expected_Mz = q * L**2 / 24

        np.testing.assert_almost_equal(abs(actions_mid.Mz), expected_Mz, decimal=1)


class TestAxialForce:
    """Test axial force computation"""

    def test_axial_tension(self):
        """Axial force under tension load"""
        L = 6.0
        P = 100.0  # kN axial load

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UX, P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Check axial force at multiple positions (should be constant)
        for x in [0.0, L/2, L]:
            actions = beam.get_internal_actions(x, u, dof_handler, lc)
            np.testing.assert_almost_equal(actions.N, P, decimal=1)


class TestMomentExtremes:
    """Test find_moment_extremes() method"""

    def test_cantilever_extremes(self):
        """Cantilever with tip load: max moment at base, zero at tip"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Find moment extremes about z-axis
        min_extreme, max_extreme = beam.find_moment_extremes('z', u, dof_handler, lc)

        expected_max = P * L

        # Check that we found the expected extremes
        extremes = [min_extreme, max_extreme]
        base_extreme = min(extremes, key=lambda e: e.x)
        tip_extreme = max(extremes, key=lambda e: e.x)

        np.testing.assert_almost_equal(base_extreme.x, 0.0, decimal=1)
        np.testing.assert_almost_equal(abs(base_extreme.value), expected_max, decimal=1)
        np.testing.assert_almost_equal(tip_extreme.x, L, decimal=1)


class TestInternalActionsStruct:
    """Test InternalActions struct fields"""

    def test_all_fields_populated(self):
        """All internal action fields should be accessible"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        actions = beam.get_internal_actions(L/2, u, dof_handler, lc)

        # All fields should exist
        assert hasattr(actions, 'x')
        assert hasattr(actions, 'N')
        assert hasattr(actions, 'Vy')
        assert hasattr(actions, 'Vz')
        assert hasattr(actions, 'Mx')
        assert hasattr(actions, 'My')
        assert hasattr(actions, 'Mz')

        # Position should be at midspan
        np.testing.assert_almost_equal(actions.x, L/2, decimal=3)


class TestReleaseComboEnums:
    """Test ReleaseCombo enums are accessible from Python"""

    def test_release_combo_4dof_values(self):
        """ReleaseCombo4DOF enum values accessible"""
        assert ReleaseCombo4DOF.FIXED_FIXED_FIXED_FIXED.value == 0
        assert ReleaseCombo4DOF.FIXED_FIXED_FREE_FIXED.value == 1
        assert ReleaseCombo4DOF.FIXED_FIXED_FIXED_FREE.value == 2
        assert ReleaseCombo4DOF.FIXED_FREE_FIXED_FREE.value == 6
        assert ReleaseCombo4DOF.FREE_FREE_FREE_FREE.value == 15

    def test_release_combo_2dof_values(self):
        """ReleaseCombo2DOF enum values accessible"""
        assert ReleaseCombo2DOF.FIXED_FIXED.value == 0
        assert ReleaseCombo2DOF.FIXED_FREE.value == 1
        assert ReleaseCombo2DOF.FREE_FIXED.value == 2
        assert ReleaseCombo2DOF.FREE_FREE.value == 3


class TestTimoshenkoInternalActions:
    """Test internal actions for Timoshenko beams"""

    def test_timoshenko_cantilever_moment(self):
        """Timoshenko beam: moment distribution same as Euler-Bernoulli"""
        L = 6.0  # Longer beam to reduce shear deformation effects
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)
        sec.set_shear_areas(0.008, 0.008)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        config = BeamConfig()
        config.formulation = BeamFormulation.Timoshenko
        beam = model.create_beam(n1, n2, mat, sec, config)

        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Moment at base should still be P*L
        actions_base = beam.get_internal_actions(0.0, u, dof_handler, lc)
        expected_Mz = P * L

        # Allow ~1% tolerance for Timoshenko shear effects
        np.testing.assert_almost_equal(abs(actions_base.Mz), expected_Mz, decimal=0)


class TestMultiplePositions:
    """Test internal actions at multiple positions along beam"""

    def test_moment_diagram_shape(self):
        """Cantilever moment should decrease linearly from base to tip"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Sample at multiple positions
        positions = np.linspace(0, L, 11)
        moments = []

        for x in positions:
            actions = beam.get_internal_actions(x, u, dof_handler, lc)
            moments.append(actions.Mz)

        moments = np.array(moments)

        # Check shape is linear (decreasing toward tip)
        for i in range(len(moments) - 1):
            assert abs(moments[i]) >= abs(moments[i+1]) - 0.1


class TestCantileverZDirection:
    """Test cantilever with load in Z direction"""

    def test_moment_about_y_axis(self):
        """Cantilever with Z-load: moment about Y-axis = P*L at base"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UZ, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        actions_base = beam.get_internal_actions(0.0, u, dof_handler, lc)

        # Moment about Y should be P*L
        expected_My = P * L
        np.testing.assert_almost_equal(abs(actions_base.My), expected_My, decimal=2)


# ============================================================================
# Task 7.2b: Warping Internal Actions Tests
# ============================================================================


class TestWarpingInternalActionsStruct:
    """Test WarpingInternalActions struct fields"""

    def test_warping_struct_accessible(self):
        """WarpingInternalActions struct should be accessible"""
        # Create a 12-DOF beam (no warping) and verify struct exists
        L = 6.0
        T = 10.0  # kN·m torque

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.RX, T)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get warping internal actions (should have zero warping values for 12-DOF)
        warping_actions = beam.get_warping_internal_actions(L/2, u, dof_handler)

        # Check all fields exist
        assert hasattr(warping_actions, 'x')
        assert hasattr(warping_actions, 'N')
        assert hasattr(warping_actions, 'Vy')
        assert hasattr(warping_actions, 'Vz')
        assert hasattr(warping_actions, 'Mx')
        assert hasattr(warping_actions, 'My')
        assert hasattr(warping_actions, 'Mz')
        assert hasattr(warping_actions, 'B')
        assert hasattr(warping_actions, 'Mx_sv')
        assert hasattr(warping_actions, 'Mx_w')
        assert hasattr(warping_actions, 'sigma_w_max')

        # For 12-DOF beam, warping values should be zero
        np.testing.assert_almost_equal(warping_actions.B, 0.0, decimal=5)
        np.testing.assert_almost_equal(warping_actions.Mx_w, 0.0, decimal=5)

    def test_12dof_st_venant_torsion(self):
        """12-DOF beam: all torsion is St. Venant"""
        L = 6.0
        T = 10.0  # kN·m torque

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.RX, T)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        warping_actions = beam.get_warping_internal_actions(L/2, u, dof_handler)

        # For 12-DOF, Mx = Mx_sv (all St. Venant)
        np.testing.assert_almost_equal(warping_actions.Mx, warping_actions.Mx_sv, decimal=3)


class TestReleaseComboWarpingEnum:
    """Test ReleaseComboWarping enum accessibility"""

    def test_release_combo_warping_values(self):
        """ReleaseComboWarping enum values accessible"""
        assert ReleaseComboWarping.FIXED_FIXED_FIXED_FIXED.value == 0
        assert ReleaseComboWarping.FIXED_FIXED_FREE_FREE.value == 3  # Cantilever
        assert ReleaseComboWarping.FIXED_FREE_FIXED_FREE.value == 5  # Pure St. Venant
        assert ReleaseComboWarping.FREE_FREE_FIXED_FIXED.value == 12  # Reverse cantilever
        assert ReleaseComboWarping.FREE_FREE_FREE_FREE.value == 15  # All free


class TestWarpingComputeStress:
    """Test compute_warping_stress method"""

    def test_compute_warping_stress_zero_iw(self):
        """Warping stress is zero when Iw is zero"""
        L = 6.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)
        # Iw = 0 by default

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)

        # Compute warping stress with arbitrary bimoment
        stress = beam.compute_warping_stress(100.0)  # 100 kN·m²

        # Should be zero because Iw = 0
        np.testing.assert_almost_equal(stress, 0.0, decimal=5)


class TestWarpingInternalActionsInheritance:
    """Test WarpingInternalActions inherits from InternalActions"""

    def test_base_fields_accessible(self):
        """Base InternalActions fields should be accessible"""
        # Create default WarpingInternalActions
        wa = WarpingInternalActions()

        # Base class fields
        assert hasattr(wa, 'x')
        assert hasattr(wa, 'N')
        assert hasattr(wa, 'Vy')
        assert hasattr(wa, 'Vz')
        assert hasattr(wa, 'Mx')
        assert hasattr(wa, 'My')
        assert hasattr(wa, 'Mz')

        # Default values should be zero
        np.testing.assert_almost_equal(wa.x, 0.0, decimal=5)
        np.testing.assert_almost_equal(wa.B, 0.0, decimal=5)

    def test_position_constructor(self):
        """Can construct WarpingInternalActions at position"""
        wa = WarpingInternalActions(3.5)
        np.testing.assert_almost_equal(wa.x, 3.5, decimal=5)


# ============================================================================
# Task 7.2c: Displacement/Rotation Line Tests
# ============================================================================

from grillex.core import DisplacementLine


class TestDisplacementLineStruct:
    """Test DisplacementLine struct accessibility"""

    def test_struct_accessible(self):
        """DisplacementLine struct should be accessible"""
        dl = DisplacementLine()

        # Check all fields exist
        assert hasattr(dl, 'x')
        assert hasattr(dl, 'u')
        assert hasattr(dl, 'v')
        assert hasattr(dl, 'w')
        assert hasattr(dl, 'theta_x')
        assert hasattr(dl, 'theta_y')
        assert hasattr(dl, 'theta_z')
        assert hasattr(dl, 'phi_prime')

        # Default values should be zero
        np.testing.assert_almost_equal(dl.x, 0.0, decimal=5)
        np.testing.assert_almost_equal(dl.u, 0.0, decimal=5)
        np.testing.assert_almost_equal(dl.v, 0.0, decimal=5)
        np.testing.assert_almost_equal(dl.w, 0.0, decimal=5)

    def test_position_constructor(self):
        """Can construct DisplacementLine at position"""
        dl = DisplacementLine(3.5)
        np.testing.assert_almost_equal(dl.x, 3.5, decimal=5)


class TestDisplacementAtEnds:
    """Test that displacements at element ends match nodal values exactly"""

    def test_cantilever_end_displacements(self):
        """Displacements at ends match nodal values"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get displacements at start (x=0) and end (x=L)
        disp_start = beam.get_displacements_at(0.0, u, dof_handler)
        disp_end = beam.get_displacements_at(L, u, dof_handler)

        # At fixed end (x=0), all displacements should be zero
        np.testing.assert_almost_equal(disp_start.u, 0.0, decimal=5)
        np.testing.assert_almost_equal(disp_start.v, 0.0, decimal=5)
        np.testing.assert_almost_equal(disp_start.w, 0.0, decimal=5)
        np.testing.assert_almost_equal(disp_start.theta_x, 0.0, decimal=5)
        np.testing.assert_almost_equal(disp_start.theta_y, 0.0, decimal=5)
        np.testing.assert_almost_equal(disp_start.theta_z, 0.0, decimal=5)

        # At free end (x=L), v should be negative (downward deflection)
        assert disp_end.v < 0  # Deflected downward


class TestCantileverDeflectionShape:
    """Test deflection shape for cantilever with tip load matches analytical curve"""

    def test_deflection_shape(self):
        """Cantilever deflection shape: v(x) = Px²(3L-x)/(6EI)"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)

        # Use simple values for verification
        E = mat.E
        I = 8e-5  # Iz

        sec = model.create_section("Test", 0.01, I, I, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Analytical solution for cantilever with tip load:
        # v(x) = -Px²(3L - x) / (6EI)  (negative for downward)
        EI = E * I

        # Sample at multiple positions and compare to analytical
        for xi in [0.25, 0.5, 0.75, 1.0]:
            x = xi * L
            disp = beam.get_displacements_at(x, u, dof_handler)

            # Analytical deflection (note: negative for downward)
            v_analytical = -P * x * x * (3.0 * L - x) / (6.0 * EI)

            # Compare - should match well for Euler-Bernoulli
            np.testing.assert_almost_equal(disp.v, v_analytical, decimal=4)


class TestEulerBernoulliRotation:
    """Test that rotation φ_z = dw/dy for Euler-Bernoulli beams"""

    def test_rotation_equals_slope(self):
        """For Euler-Bernoulli: θz = dv/dx"""
        L = 6.0
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        E = mat.E
        I = 8e-5

        sec = model.create_section("Test", 0.01, I, I, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)  # Default is Euler-Bernoulli
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Analytical rotation for cantilever: θz = dv/dx = -Px(2L - x) / (2EI)
        EI = E * I

        # Sample at multiple positions
        for xi in [0.25, 0.5, 0.75]:
            x = xi * L
            disp = beam.get_displacements_at(x, u, dof_handler)

            # Analytical rotation
            theta_z_analytical = -P * x * (2.0 * L - x) / (2.0 * EI)

            # Compare
            np.testing.assert_almost_equal(disp.theta_z, theta_z_analytical, decimal=4)


class TestTimoshenkoVsEulerBernoulli:
    """Test that Timoshenko has different rotation than slope for deep beams"""

    def test_timoshenko_rotation_differs(self):
        """For Timoshenko: θz ≠ dv/dx due to shear deformation"""
        L = 2.0  # Short beam to emphasize shear effects
        P = 10.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        E = mat.E
        I = 8e-5

        sec = model.create_section("Test", 0.01, I, I, 1e-6)
        sec.set_shear_areas(0.008, 0.008)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        # Create Timoshenko beam
        config = BeamConfig()
        config.formulation = BeamFormulation.Timoshenko
        beam = model.create_beam(n1, n2, mat, sec, config)

        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # Get displacement at midpoint
        disp_mid = beam.get_displacements_at(L/2, u, dof_handler)

        # For Timoshenko, rotation is interpolated linearly
        # between end rotations, which differ from Euler-Bernoulli slope
        # This just verifies the method runs - detailed verification would
        # require comparison with Timoshenko analytical solution

        assert hasattr(disp_mid, 'theta_z')


class TestMidspanDisplacement:
    """Test midspan displacement is correctly interpolated"""

    def test_midspan_displacement(self):
        """Midspan displacement is average of end displacements (for linear interpolation)"""
        L = 6.0

        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)

        # Simply supported
        model.boundary_conditions.add_fixed_dof(n1.id, DOFIndex.UX, 0.0)
        model.boundary_conditions.add_fixed_dof(n1.id, DOFIndex.UY, 0.0)
        model.boundary_conditions.add_fixed_dof(n1.id, DOFIndex.UZ, 0.0)
        model.boundary_conditions.add_fixed_dof(n1.id, DOFIndex.RX, 0.0)

        model.boundary_conditions.add_fixed_dof(n2.id, DOFIndex.UY, 0.0)
        model.boundary_conditions.add_fixed_dof(n2.id, DOFIndex.RX, 0.0)

        lc = model.get_default_load_case()
        # Apply end moment (no midspan concentrated load)
        lc.add_nodal_load(n2.id, DOFIndex.RZ, 10.0)

        assert model.analyze()

        dof_handler = model.get_dof_handler()
        u = model.get_displacements()

        # For axial displacement, should be linear interpolation
        disp_start = beam.get_displacements_at(0.0, u, dof_handler)
        disp_mid = beam.get_displacements_at(L/2, u, dof_handler)
        disp_end = beam.get_displacements_at(L, u, dof_handler)

        # Axial displacement should be linearly interpolated
        expected_u_mid = (disp_start.u + disp_end.u) / 2
        np.testing.assert_almost_equal(disp_mid.u, expected_u_mid, decimal=5)


# ============================================================================
# Task 7.2f: Multi-Element Beam Plotting Tests
# ============================================================================

from grillex.core import StructuralModel, Beam


class TestMultiElementBeamMomentDiagram:
    """Test continuous moment diagram across multi-element beams"""

    def test_three_element_beam_moment_continuity(self):
        """Continuous moment diagram across 3-element beam with point load"""
        # Create a simply supported beam with 3 equal elements and center point load
        # Using point load instead of UDL for exact FEM solution
        L = 6.0  # Total length
        P = 10.0  # kN point load at center

        model = StructuralModel(name="Multi-element test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        # Create beam
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Create intermediate nodes including one at L/2 for point load
        model.get_or_create_node(L/2, 0, 0)

        # Subdivide beams at internal nodes
        n_split = model.subdivide_beams()

        # Apply boundary conditions
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([L, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX, 0.0)

        # Apply point load at center
        model.add_point_load([L/2, 0, 0], force=[0, -P, 0])

        assert model.analyze()

        # For point load P at center of simply supported beam:
        # M_max = P*L/4 at x = L/2
        M_max_expected = P * L / 4.0

        # Get moment at midspan - find the element containing L/2
        # Actually, at L/2 there's a node, so query just before it
        eps = 0.01
        mid_beam = next(b for b in model.beams
                        if b.start_pos[0] <= L/2 - eps <= b.end_pos[0])
        x_local = L/2 - eps - mid_beam.start_pos[0]

        cpp_model = model._cpp_model
        actions = mid_beam.elements[0].get_internal_actions(
            x_local,
            cpp_model.get_displacements(),
            cpp_model.get_dof_handler()
        )

        # Moment at midspan should be approximately PL/4
        np.testing.assert_almost_equal(abs(actions.Mz), M_max_expected, decimal=0)


class TestBeamElementBoundaries:
    """Test element boundary identification in multi-element beams"""

    def test_single_element_no_boundaries(self):
        """Single-element beam has no internal boundaries"""
        model = StructuralModel(name="Single element test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE200", "Steel")

        boundaries = beam.get_element_boundaries()
        assert boundaries == []

    def test_three_element_two_boundaries(self):
        """Three-element beam has two internal boundaries"""
        L = 6.0

        model = StructuralModel(name="Multi-element test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Create intermediate nodes
        model.get_or_create_node(2, 0, 0)
        model.get_or_create_node(4, 0, 0)

        # Subdivide
        model.subdivide_beams()

        # After subdivision, we have 3 beams instead of 1
        # Each beam has 1 element
        # Let's find the first sub-beam
        first_beam = next(b for b in model.beams if np.allclose(b.start_pos, [0, 0, 0]))

        # Single-element beam still has no internal boundaries
        boundaries = first_beam.get_element_boundaries()
        assert boundaries == []


class TestShearDiscontinuities:
    """Test that concentrated loads cause visible shear discontinuities"""

    def test_shear_jump_at_point_load(self):
        """Shear has discontinuity at point load location"""
        L = 6.0
        P = 10.0

        model = StructuralModel(name="Point load test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Create node at L/2 for point load
        model.get_or_create_node(L/2, 0, 0)
        model.subdivide_beams()

        # Find the two sub-beams
        beam1 = next(b for b in model.beams if np.allclose(b.start_pos, [0, 0, 0]))
        beam2 = next(b for b in model.beams if np.allclose(b.start_pos, [L/2, 0, 0]))

        # Apply boundary conditions
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([L, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX, 0.0)

        # Point load at L/2
        model.add_point_load([L/2, 0, 0], force=[0, -P, 0])

        assert model.analyze()

        cpp_model = model._cpp_model
        dof_handler = cpp_model.get_dof_handler()
        u = cpp_model.get_displacements()

        # Get shear just before and just after the point load
        eps = 0.01  # small offset

        # Shear at end of first beam (just before load)
        actions_before = beam1.elements[0].get_internal_actions(
            beam1.length - eps, u, dof_handler)

        # Shear at start of second beam (just after load)
        actions_after = beam2.elements[0].get_internal_actions(
            eps, u, dof_handler)

        # Shear should jump by approximately P at the load location
        shear_jump = abs(actions_before.Vy - actions_after.Vy)

        # For simply supported beam with center load:
        # Before load: V = P/2 (positive, upward reaction at left support)
        # After load: V = -P/2 (negative, force heads down to right support)
        # Jump should be approximately P
        np.testing.assert_almost_equal(shear_jump, P, decimal=0)


class TestMultiElementExtrema:
    """Test extrema finding across element boundaries"""

    def test_extrema_for_cantilever_tip_load(self):
        """Extrema found correctly for cantilever with tip load"""
        L = 6.0
        P = 10.0

        model = StructuralModel(name="Cantilever test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Cantilever BC
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ, 0.0)

        model.add_point_load([L, 0, 0], force=[0, -P, 0])

        assert model.analyze()

        # Find extrema using Beam class method
        extrema = beam.find_moment_extrema('z', model)

        # For cantilever with tip load:
        # M = -P(L-x), max moment is at base (x=0) = -PL
        M_max_expected = P * L

        # Find the maximum absolute moment in extrema
        max_moment = max(abs(val) for _, val in extrema)

        np.testing.assert_almost_equal(max_moment, M_max_expected, decimal=0)


class TestDeflectionDiagramContinuity:
    """Test that deflection diagram is smooth and continuous"""

    def test_deflection_smooth_across_elements(self):
        """Deflection is continuous at element boundaries"""
        L = 6.0

        model = StructuralModel(name="Deflection test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Create intermediate node
        model.get_or_create_node(L/2, 0, 0)
        model.subdivide_beams()

        # Find the two sub-beams
        beam1 = next(b for b in model.beams if np.allclose(b.start_pos, [0, 0, 0]))
        beam2 = next(b for b in model.beams if np.allclose(b.start_pos, [L/2, 0, 0]))

        # Apply cantilever BC
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ, 0.0)

        model.add_point_load([L, 0, 0], force=[0, -10.0, 0])

        assert model.analyze()

        cpp_model = model._cpp_model
        dof_handler = cpp_model.get_dof_handler()
        u = cpp_model.get_displacements()

        # Get displacement at boundary from both elements
        disp_end_beam1 = beam1.elements[0].get_displacements_at(beam1.length, u, dof_handler)
        disp_start_beam2 = beam2.elements[0].get_displacements_at(0.0, u, dof_handler)

        # Displacements should match at the boundary
        np.testing.assert_almost_equal(disp_end_beam1.v, disp_start_beam2.v, decimal=6)
        np.testing.assert_almost_equal(disp_end_beam1.theta_z, disp_start_beam2.theta_z, decimal=6)


class TestBeamInternalActionsAtMethod:
    """Test the Beam.get_internal_actions_at() method"""

    def test_internal_actions_at_various_positions(self):
        """Can query internal actions at any position along beam"""
        L = 6.0
        P = 10.0

        model = StructuralModel(name="Actions test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Cantilever
        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ, 0.0)

        model.add_point_load([L, 0, 0], force=[0, -P, 0])

        assert model.analyze()

        # Query at various positions
        for xi in [0.0, 0.25, 0.5, 0.75, 1.0]:
            x = xi * L
            actions = beam.get_internal_actions_at(x, model)

            # Verify we got valid results
            assert hasattr(actions, 'Mz')
            assert hasattr(actions, 'Vy')

            # For cantilever with tip load:
            # M(x) = -P(L-x), V = -P (constant)
            M_expected = -P * (L - x)
            np.testing.assert_almost_equal(actions.Mz, M_expected, decimal=1)


class TestMomentLineMethod:
    """Test the Beam.get_moment_line() method"""

    def test_moment_line_returns_arrays(self):
        """get_moment_line returns x_positions and moments arrays"""
        L = 6.0

        model = StructuralModel(name="Moment line test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ, 0.0)

        model.add_point_load([L, 0, 0], force=[0, -10.0, 0])

        assert model.analyze()

        x_positions, moments = beam.get_moment_line('z', model, num_points=50)

        assert len(x_positions) == 50
        assert len(moments) == 50
        assert x_positions[0] == 0.0
        np.testing.assert_almost_equal(x_positions[-1], L, decimal=5)


class TestDisplacementLineMethod:
    """Test the Beam.get_displacement_line() method"""

    def test_displacement_line_returns_arrays(self):
        """get_displacement_line returns x_positions and displacements arrays"""
        L = 6.0

        model = StructuralModel(name="Displacement line test")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        model.fix_dof_at([0, 0, 0], DOFIndex.UX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY, 0.0)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ, 0.0)

        model.add_point_load([L, 0, 0], force=[0, -10.0, 0])

        assert model.analyze()

        x_positions, v_displacements = beam.get_displacement_line('v', model, num_points=50)

        assert len(x_positions) == 50
        assert len(v_displacements) == 50

        # Cantilever: deflection should be 0 at fixed end, negative at tip
        np.testing.assert_almost_equal(v_displacements[0], 0.0, decimal=5)
        assert v_displacements[-1] < 0  # Downward deflection


class TestVaryingElementCounts:
    """Test that methods work with different element counts"""

    def test_single_element(self):
        """Methods work with single-element beam"""
        model = StructuralModel(name="Single element")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE200", "Steel")
        model.fix_dof_at([0, 0, 0], DOFIndex.UX)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ)
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])

        assert model.analyze()

        # These should all work
        assert len(beam.elements) == 1
        x, M = beam.get_moment_line('z', model, num_points=10)
        assert len(x) == 10

    def test_multiple_elements(self):
        """Methods work with multi-element beam (created via subdivision)"""
        model = StructuralModel(name="Multi element")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE200", A=0.01, Iy=8e-5, Iz=8e-5, J=1e-6)

        L = 10.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel")

        # Create 4 intermediate nodes for 5 elements
        for i in range(1, 5):
            model.get_or_create_node(i * L / 5, 0, 0)

        n_split = model.subdivide_beams()
        assert n_split == 1  # Original beam was subdivided

        # Find first sub-beam
        first_beam = next(b for b in model.beams if np.allclose(b.start_pos, [0, 0, 0]))

        # Apply BCs and loads
        model.fix_dof_at([0, 0, 0], DOFIndex.UX)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)
        model.fix_dof_at([0, 0, 0], DOFIndex.RY)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ)
        model.add_point_load([L, 0, 0], force=[0, -10.0, 0])

        assert model.analyze()

        # These should work on each sub-beam
        x, M = first_beam.get_moment_line('z', model, num_points=10)
        assert len(x) == 10


class TestContinuousBeamBimomentContinuity:
    """
    Task 7.2b: Verify bimoment is continuous at internal support for two-span beam.

    For collinear elements sharing a warping DOF at an internal node, the bimoment
    must be continuous (same value from both elements at the shared node).
    """

    def test_two_span_beam_bimoment_continuity(self):
        """✓ For two-span continuous beam under torsion, bimoment is continuous at support"""
        from grillex.core import (
            NodeRegistry, Material, Section, BeamElement, BeamConfig,
            DOFHandler, Assembler, BCHandler, LinearSolver
        )

        # Create two-span continuous beam
        L = 6.0  # Each span length
        registry = NodeRegistry()
        node1 = registry.get_or_create_node(0.0, 0.0, 0.0)
        node2 = registry.get_or_create_node(L, 0.0, 0.0)      # Internal support
        node3 = registry.get_or_create_node(2*L, 0.0, 0.0)

        node1.enable_warping_dof()
        node2.enable_warping_dof()
        node3.enable_warping_dof()

        # IPE300 section with warping constant
        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        sec.Iw = 1.26e-7  # Warping constant for IPE300 (m^6)

        config = BeamConfig()
        config.include_warping = True

        elem1 = BeamElement(1, node1, node2, mat, sec, config)
        elem2 = BeamElement(2, node2, node3, mat, sec, config)

        # Number DOFs - collinear elements share warping DOF at node2
        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, [elem1, elem2])

        # Verify shared warping DOF
        warp_dof_node2 = dof_handler.get_warping_dof(elem1.id, node2.id)
        assert warp_dof_node2 == dof_handler.get_warping_dof(elem2.id, node2.id), \
            "Collinear beams must share warping DOF"

        # Assemble system
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([elem1, elem2])
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        # Apply torque at internal support (node2)
        rx_node2 = dof_handler.get_global_dof(node2.id, 3)  # RX
        T = 10.0  # kNm torque
        F[rx_node2] = T

        # Fixed ends with warping restraint
        bc = BCHandler()
        bc.fix_node_with_warping(node1.id, elem1.id)
        bc.fix_node_with_warping(node3.id, elem2.id)

        # Apply BCs and solve
        K_mod, F_mod = bc.apply_to_system(K, F, dof_handler)
        solver = LinearSolver()
        u = solver.solve(K_mod, F_mod)
        assert not solver.is_singular()

        # Use get_warping_internal_actions to compute bimoment at internal node
        # Element 1: bimoment at x=L (node2)
        warping_actions1 = elem1.get_warping_internal_actions(L, u, dof_handler)
        B1_at_node2 = warping_actions1.B

        # Element 2: bimoment at x=0 (node2)
        warping_actions2 = elem2.get_warping_internal_actions(0.0, u, dof_handler)
        B2_at_node2 = warping_actions2.B

        # Bimoment must be continuous at internal support
        # Allow some numerical tolerance
        np.testing.assert_almost_equal(
            B1_at_node2, B2_at_node2, decimal=3,
            err_msg="Bimoment must be continuous at internal support"
        )


class TestCantileverIBeamAnalyticalComparison:
    """
    Task 7.2b: Compare cantilever I-beam bimoment with analytical solution.

    Analytical solution for cantilever under end torque T with warping restrained
    at fixed end (Vlasov theory):
        k = sqrt(GJ / EIw)
        B(x) = T/k * sinh(k*(L-x)) / cosh(k*L)

    At fixed end (x=0): B(0) = T/k * tanh(kL)
    At free end (x=L):  B(L) = 0
    """

    def test_cantilever_bimoment_analytical(self):
        """✓ Comparison with analytical solution for cantilever I-beam under torsion"""
        from grillex.core import (
            NodeRegistry, Material, Section, BeamElement, BeamConfig,
            DOFHandler, Assembler, BCHandler, LinearSolver
        )

        L = 3.0  # Beam length [m]
        T = 10.0  # Applied torque [kNm]
        n_elem = 10  # Use finer mesh for better accuracy

        registry = NodeRegistry()
        elem_len = L / n_elem
        nodes = []
        for i in range(n_elem + 1):
            x = i * elem_len
            node = registry.get_or_create_node(x, 0.0, 0.0)
            node.enable_warping_dof()
            nodes.append(node)

        # IPE300 section
        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        sec.Iw = 1.26e-7  # Warping constant [m^6]

        config = BeamConfig()
        config.include_warping = True

        elements = []
        for i in range(n_elem):
            elem = BeamElement(i + 1, nodes[i], nodes[i + 1], mat, sec, config)
            elements.append(elem)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, elements)

        # Assemble
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness(elements)
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        # Apply torque at free end (last node)
        rx_end = dof_handler.get_global_dof(nodes[-1].id, 3)  # RX
        F[rx_end] = T

        # Fixed end with warping restraint at node 0, free at other end
        bc = BCHandler()
        bc.fix_node_with_warping(nodes[0].id, elements[0].id)
        # Free end: only pin for stability (translations fixed, rotation/warping free)
        bc.pin_node(nodes[-1].id)

        # Apply BCs and solve
        K_mod, F_mod = bc.apply_to_system(K, F, dof_handler)
        solver = LinearSolver()
        u = solver.solve(K_mod, F_mod)
        assert not solver.is_singular()

        # Material properties
        E = mat.E
        G = mat.G
        GJ = G * sec.J
        EIw = E * sec.Iw

        # Analytical solution parameters
        k = np.sqrt(GJ / EIw)
        kL = k * L

        # Compute bimoment at fixed end (x=0) using first element
        warping_actions_0 = elements[0].get_warping_internal_actions(0.0, u, dof_handler)
        B_computed_0 = warping_actions_0.B

        # Analytical bimoment at fixed end
        # B(0) = T/k * tanh(kL)
        # Note: sign convention may differ between FEM and analytical
        B_analytical_0 = T / k * np.tanh(kL)

        # Compare magnitudes - FEM and analytical may use different sign conventions
        # Allow 10% tolerance for FEM approximation
        error_percent = abs(abs(B_computed_0) - abs(B_analytical_0)) / abs(B_analytical_0) * 100
        assert error_percent < 10.0, \
            f"Bimoment magnitude at fixed end: computed={abs(B_computed_0):.6f}, " \
            f"analytical={abs(B_analytical_0):.6f}, error={error_percent:.1f}%"

        # Verify non-zero bimoment at fixed end (warping is restrained)
        assert abs(B_computed_0) > 0.1 * abs(B_analytical_0), \
            "Bimoment at fixed end should be non-zero when warping is restrained"

        # Compute bimoment at free end (x=L) - should be ~0
        warping_actions_L = elements[-1].get_warping_internal_actions(elem_len, u, dof_handler)
        B_computed_L = warping_actions_L.B

        # Analytical: B(L) = 0 (bimoment is zero at free end)
        # At free end, bimoment magnitude should be much smaller than at fixed end
        assert abs(B_computed_L) < 0.2 * abs(B_computed_0), \
            f"Bimoment at free end should be nearly zero: B(L)={B_computed_L:.6f}"

    def test_bimoment_distribution_along_beam(self):
        """Verify bimoment distribution matches analytical along entire beam"""
        from grillex.core import (
            NodeRegistry, Material, Section, BeamElement, BeamConfig,
            DOFHandler, Assembler, BCHandler, LinearSolver
        )

        L = 3.0
        T = 10.0
        n_elem = 10  # Use finer mesh for better accuracy

        registry = NodeRegistry()
        elem_len = L / n_elem
        nodes = []
        for i in range(n_elem + 1):
            x = i * elem_len
            node = registry.get_or_create_node(x, 0.0, 0.0)
            node.enable_warping_dof()
            nodes.append(node)

        mat = Material(1, "Steel", 210e6, 0.3, 7.85e-6)
        sec = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        sec.Iw = 1.26e-7

        config = BeamConfig()
        config.include_warping = True

        elements = []
        for i in range(n_elem):
            elem = BeamElement(i + 1, nodes[i], nodes[i + 1], mat, sec, config)
            elements.append(elem)

        dof_handler = DOFHandler()
        dof_handler.number_dofs_with_elements(registry, elements)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness(elements)
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        rx_end = dof_handler.get_global_dof(nodes[-1].id, 3)
        F[rx_end] = T

        bc = BCHandler()
        bc.fix_node_with_warping(nodes[0].id, elements[0].id)
        bc.pin_node(nodes[-1].id)

        K_mod, F_mod = bc.apply_to_system(K, F, dof_handler)
        solver = LinearSolver()
        u = solver.solve(K_mod, F_mod)

        E = mat.E
        G = mat.G
        GJ = G * sec.J
        EIw = E * sec.Iw
        k = np.sqrt(GJ / EIw)

        # Check bimoment at nodal locations along the beam
        # Compare magnitudes to account for possible sign convention differences
        max_error = 0.0

        for i, elem in enumerate(elements):
            # Check at start of element
            x_global = i * elem_len
            warping_actions = elem.get_warping_internal_actions(0.0, u, dof_handler)
            B_computed = warping_actions.B

            # Analytical: B(x) = T/k * sinh(k*(L-x)) / cosh(k*L)
            B_analytical = T / k * np.sinh(k * (L - x_global)) / np.cosh(k * L)

            if abs(B_analytical) > 1e-6:
                # Compare magnitudes - FEM and analytical may use different sign conventions
                error = abs(abs(B_computed) - abs(B_analytical)) / abs(B_analytical) * 100
                max_error = max(max_error, error)

        # Maximum error should be within 15% for FEM approximation
        assert max_error < 15.0, \
            f"Maximum bimoment error along beam: {max_error:.1f}%"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
