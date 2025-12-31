"""
Tests for Task 7.3: Python Bindings for Internal Actions

Verifies that all Phase 7 C++ types are correctly bound and accessible from Python.
This is a comprehensive verification test for the pybind11 bindings.

Acceptance Criteria:
- [x] InternalActions, EndForces, and DisplacementLine structs are accessible from Python
- [x] BeamElement methods are callable from Python
- [x] Methods accept Eigen arrays and return appropriate types
"""

import numpy as np
import pytest

from grillex.core import (
    # Phase 7 structs
    InternalActions,
    EndForces,
    DisplacementLine,
    WarpingInternalActions,
    ActionExtreme,
    DistributedLoad,
    # Release combo enums
    ReleaseCombo4DOF,
    ReleaseCombo2DOF,
    ReleaseComboWarping,
    # Core types for testing
    Model,
    DOFIndex,
    BeamConfig,
    BeamFormulation,
)


class TestPhase7StructsAccessible:
    """Verify all Phase 7 structs are accessible from Python."""

    def test_internal_actions_accessible(self):
        """InternalActions struct is accessible and constructible."""
        # Default constructor
        ia = InternalActions()
        assert hasattr(ia, 'x')
        assert hasattr(ia, 'N')
        assert hasattr(ia, 'Vy')
        assert hasattr(ia, 'Vz')
        assert hasattr(ia, 'Mx')
        assert hasattr(ia, 'My')
        assert hasattr(ia, 'Mz')

        # Position constructor
        ia2 = InternalActions(3.5)
        np.testing.assert_almost_equal(ia2.x, 3.5)

        # Full constructor
        ia3 = InternalActions(1.0, 10.0, 5.0, 2.0, 1.0, 20.0, 15.0)
        np.testing.assert_almost_equal(ia3.N, 10.0)
        np.testing.assert_almost_equal(ia3.Mz, 15.0)

    def test_end_forces_accessible(self):
        """EndForces struct is accessible and constructible."""
        # Default constructor
        ef = EndForces()
        assert hasattr(ef, 'N')
        assert hasattr(ef, 'Vy')
        assert hasattr(ef, 'Vz')
        assert hasattr(ef, 'Mx')
        assert hasattr(ef, 'My')
        assert hasattr(ef, 'Mz')
        assert hasattr(ef, 'B')

        # Full constructor
        ef2 = EndForces(100.0, 10.0, 5.0, 2.0, 30.0, 25.0, 1.5)
        np.testing.assert_almost_equal(ef2.N, 100.0)
        np.testing.assert_almost_equal(ef2.B, 1.5)

        # to_vector methods
        vec6 = ef2.to_vector6()
        assert len(vec6) == 6
        np.testing.assert_almost_equal(vec6[0], 100.0)  # N

        vec7 = ef2.to_vector7()
        assert len(vec7) == 7
        np.testing.assert_almost_equal(vec7[6], 1.5)  # B

    def test_displacement_line_accessible(self):
        """DisplacementLine struct is accessible and constructible."""
        # Default constructor
        dl = DisplacementLine()
        assert hasattr(dl, 'x')
        assert hasattr(dl, 'u')
        assert hasattr(dl, 'v')
        assert hasattr(dl, 'w')
        assert hasattr(dl, 'theta_x')
        assert hasattr(dl, 'theta_y')
        assert hasattr(dl, 'theta_z')
        assert hasattr(dl, 'phi_prime')

        # Position constructor
        dl2 = DisplacementLine(2.5)
        np.testing.assert_almost_equal(dl2.x, 2.5)

    def test_warping_internal_actions_accessible(self):
        """WarpingInternalActions struct is accessible."""
        wa = WarpingInternalActions()
        # Base class fields
        assert hasattr(wa, 'x')
        assert hasattr(wa, 'N')
        assert hasattr(wa, 'Mx')
        # Warping-specific fields
        assert hasattr(wa, 'B')
        assert hasattr(wa, 'Mx_sv')
        assert hasattr(wa, 'Mx_w')
        assert hasattr(wa, 'sigma_w_max')

    def test_action_extreme_accessible(self):
        """ActionExtreme struct is accessible."""
        ae = ActionExtreme(3.0, 150.0)
        np.testing.assert_almost_equal(ae.x, 3.0)
        np.testing.assert_almost_equal(ae.value, 150.0)

    def test_distributed_load_accessible(self):
        """DistributedLoad struct is accessible."""
        dl = DistributedLoad()
        assert hasattr(dl, 'q_start')
        assert hasattr(dl, 'q_end')


class TestReleaseComboEnumsAccessible:
    """Verify all release combo enums are accessible."""

    def test_release_combo_4dof(self):
        """ReleaseCombo4DOF enum is accessible."""
        assert ReleaseCombo4DOF.FIXED_FIXED_FIXED_FIXED.value == 0
        assert ReleaseCombo4DOF.FREE_FREE_FREE_FREE.value == 15

    def test_release_combo_2dof(self):
        """ReleaseCombo2DOF enum is accessible."""
        assert ReleaseCombo2DOF.FIXED_FIXED.value == 0
        assert ReleaseCombo2DOF.FREE_FREE.value == 3

    def test_release_combo_warping(self):
        """ReleaseComboWarping enum is accessible."""
        assert ReleaseComboWarping.FIXED_FIXED_FIXED_FIXED.value == 0
        assert ReleaseComboWarping.FREE_FREE_FREE_FREE.value == 15


class TestBeamElementMethodsCallable:
    """Verify BeamElement internal action methods are callable from Python."""

    @pytest.fixture
    def cantilever_model(self):
        """Create a simple cantilever beam model."""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, -10.0, 0])

        model.analyze()

        return {
            'model': model,
            'beam': beam,
            'lc': lc,
            'dof_handler': model.get_dof_handler(),
            'u': model.get_displacements()
        }

    def test_compute_end_forces_callable(self, cantilever_model):
        """compute_end_forces method is callable and returns EndForces tuple."""
        beam = cantilever_model['beam']
        u = cantilever_model['u']
        dof_handler = cantilever_model['dof_handler']

        result = beam.compute_end_forces(u, dof_handler)

        # Returns tuple of (EndForces_i, EndForces_j)
        assert isinstance(result, tuple)
        assert len(result) == 2
        assert hasattr(result[0], 'N')
        assert hasattr(result[0], 'Mz')

    def test_get_internal_actions_callable(self, cantilever_model):
        """get_internal_actions method is callable and returns InternalActions."""
        beam = cantilever_model['beam']
        u = cantilever_model['u']
        dof_handler = cantilever_model['dof_handler']
        lc = cantilever_model['lc']

        result = beam.get_internal_actions(3.0, u, dof_handler, lc)

        assert hasattr(result, 'x')
        assert hasattr(result, 'N')
        assert hasattr(result, 'Mz')
        np.testing.assert_almost_equal(result.x, 3.0)

    def test_find_moment_extremes_callable(self, cantilever_model):
        """find_moment_extremes method is callable and returns ActionExtreme tuple."""
        beam = cantilever_model['beam']
        u = cantilever_model['u']
        dof_handler = cantilever_model['dof_handler']
        lc = cantilever_model['lc']

        result = beam.find_moment_extremes('z', u, dof_handler, lc)

        assert isinstance(result, tuple)
        assert len(result) == 2
        assert hasattr(result[0], 'x')
        assert hasattr(result[0], 'value')

    def test_get_warping_internal_actions_callable(self, cantilever_model):
        """get_warping_internal_actions method is callable and returns WarpingInternalActions."""
        beam = cantilever_model['beam']
        u = cantilever_model['u']
        dof_handler = cantilever_model['dof_handler']

        result = beam.get_warping_internal_actions(3.0, u, dof_handler)

        assert hasattr(result, 'B')
        assert hasattr(result, 'Mx_sv')
        assert hasattr(result, 'Mx_w')

    def test_compute_warping_stress_callable(self, cantilever_model):
        """compute_warping_stress method is callable and returns float."""
        beam = cantilever_model['beam']

        result = beam.compute_warping_stress(100.0)

        assert isinstance(result, float)

    def test_get_distributed_load_methods_callable(self, cantilever_model):
        """get_distributed_load_* methods are callable and return DistributedLoad."""
        beam = cantilever_model['beam']
        lc = cantilever_model['lc']

        # Add a line load for testing
        lc.add_line_load(beam.id, np.array([0, -10, 0]), np.array([0, -10, 0]))

        load_y = beam.get_distributed_load_y(lc)
        load_z = beam.get_distributed_load_z(lc)
        load_axial = beam.get_distributed_load_axial(lc)

        assert hasattr(load_y, 'q_start')
        assert hasattr(load_y, 'q_end')
        assert hasattr(load_z, 'q_start')
        assert hasattr(load_axial, 'q_start')

    def test_get_displacements_at_callable(self, cantilever_model):
        """get_displacements_at method is callable and returns DisplacementLine."""
        beam = cantilever_model['beam']
        u = cantilever_model['u']
        dof_handler = cantilever_model['dof_handler']

        result = beam.get_displacements_at(3.0, u, dof_handler)

        assert hasattr(result, 'x')
        assert hasattr(result, 'u')
        assert hasattr(result, 'v')
        assert hasattr(result, 'theta_z')


class TestMethodsAcceptEigenArrays:
    """Verify methods accept numpy arrays (Eigen bindings)."""

    def test_get_internal_actions_accepts_numpy_array(self):
        """get_internal_actions accepts numpy array for global_displacements."""
        model = Model()
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        lc = model.get_default_load_case()
        lc.add_nodal_load([6, 0, 0], [0, -10.0, 0])

        model.analyze()

        dof_handler = model.get_dof_handler()

        # Get displacements as numpy array
        u = model.get_displacements()
        assert isinstance(u, np.ndarray)

        # Method should accept numpy array
        result = beam.get_internal_actions(3.0, u, dof_handler, lc)
        assert result is not None

    def test_end_forces_to_vector_returns_numpy(self):
        """EndForces.to_vector* methods return numpy arrays."""
        ef = EndForces(100.0, 10.0, 5.0, 2.0, 30.0, 25.0, 1.5)

        vec6 = ef.to_vector6()
        vec7 = ef.to_vector7()

        assert isinstance(vec6, np.ndarray)
        assert isinstance(vec7, np.ndarray)
        assert vec6.shape == (6,)
        assert vec7.shape == (7,)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
