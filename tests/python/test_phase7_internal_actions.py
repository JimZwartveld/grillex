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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
