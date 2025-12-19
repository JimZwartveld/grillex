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
    ActionExtreme,
    ReleaseCombo4DOF,
    ReleaseCombo2DOF,
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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
