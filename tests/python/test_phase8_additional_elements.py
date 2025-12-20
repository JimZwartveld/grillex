"""
Tests for Phase 8: Additional Element Types

Task 8.1: Spring Element
- [x] Spring provides correct stiffness between nodes
- [x] Uncoupled DOFs are independent
- [ ] Eccentricity can be handled via rigid links (deferred - use existing RigidLink)

Task 8.2: Point Mass Element
- [x] Point mass contributes to global mass matrix
- [x] Inertia tensor is correctly represented
- [x] Off-diagonal terms work for asymmetric masses
"""

import numpy as np
import pytest

from grillex.core import (
    Model,
    DOFIndex,
    SpringElement,
    PointMass,
    BeamConfig,
    BeamFormulation,
)


class TestSpringElementBasics:
    """Basic tests for SpringElement class."""

    def test_spring_element_creation(self):
        """SpringElement can be created between two nodes."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)

        assert spring is not None
        assert spring.id == 1
        assert spring.node_i == n1
        assert spring.node_j == n2

    def test_spring_default_stiffness_is_zero(self):
        """Spring stiffnesses default to zero."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)

        assert spring.kx == 0.0
        assert spring.ky == 0.0
        assert spring.kz == 0.0
        assert spring.krx == 0.0
        assert spring.kry == 0.0
        assert spring.krz == 0.0

    def test_spring_set_stiffness(self):
        """Spring stiffnesses can be set individually."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        spring.kx = 1000.0
        spring.ky = 500.0
        spring.kz = 2000.0
        spring.krx = 100.0
        spring.kry = 50.0
        spring.krz = 200.0

        assert spring.kx == 1000.0
        assert spring.ky == 500.0
        assert spring.kz == 2000.0
        assert spring.krx == 100.0
        assert spring.kry == 50.0
        assert spring.krz == 200.0

    def test_spring_set_translational_stiffness(self):
        """set_translational_stiffness sets all translational stiffnesses."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        spring.set_translational_stiffness(100.0, 200.0, 300.0)

        assert spring.kx == 100.0
        assert spring.ky == 200.0
        assert spring.kz == 300.0

    def test_spring_set_rotational_stiffness(self):
        """set_rotational_stiffness sets all rotational stiffnesses."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        spring.set_rotational_stiffness(10.0, 20.0, 30.0)

        assert spring.krx == 10.0
        assert spring.kry == 20.0
        assert spring.krz == 30.0

    def test_spring_has_stiffness(self):
        """has_stiffness returns True only when any stiffness is non-zero."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        assert not spring.has_stiffness()

        spring.kz = 100.0
        assert spring.has_stiffness()


class TestSpringElementStiffnessMatrix:
    """Tests for spring element stiffness matrix."""

    def test_spring_stiffness_matrix_shape(self):
        """Spring stiffness matrix is 12x12."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        spring.kz = 1000.0

        K = spring.global_stiffness_matrix()

        assert K.shape == (12, 12)

    def test_spring_stiffness_matrix_symmetry(self):
        """Spring stiffness matrix is symmetric."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        spring.set_translational_stiffness(100.0, 200.0, 300.0)

        K = spring.global_stiffness_matrix()

        np.testing.assert_array_almost_equal(K, K.T)

    def test_spring_stiffness_matrix_structure(self):
        """Spring stiffness matrix has correct +k/-k structure.

        For a spring with stiffness k connecting DOF i at node_i to DOF i at node_j:
        K[i,i] = +k
        K[i,i+6] = -k
        K[i+6,i] = -k
        K[i+6,i+6] = +k
        """
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        k_val = 1234.0
        spring.kz = k_val  # UZ direction (index 2)

        K = spring.global_stiffness_matrix()

        # Check UZ stiffness (index 2)
        assert K[2, 2] == pytest.approx(k_val)      # +k at node_i UZ
        assert K[2, 8] == pytest.approx(-k_val)     # -k coupling
        assert K[8, 2] == pytest.approx(-k_val)     # -k coupling (symmetric)
        assert K[8, 8] == pytest.approx(k_val)      # +k at node_j UZ

        # Other DOFs should be zero
        assert K[0, 0] == pytest.approx(0.0)  # UX
        assert K[1, 1] == pytest.approx(0.0)  # UY

    def test_uncoupled_dofs_are_independent(self):
        """Different spring stiffnesses create independent DOF couplings.

        Task 8.1 criterion: Uncoupled DOFs are independent
        """
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        spring = model.create_spring(n1, n2)
        spring.kx = 100.0
        spring.ky = 200.0
        spring.kz = 300.0

        K = spring.global_stiffness_matrix()

        # Check that each stiffness only affects its own DOF
        # UX (index 0) has kx
        assert K[0, 0] == pytest.approx(100.0)
        assert K[0, 6] == pytest.approx(-100.0)

        # UY (index 1) has ky
        assert K[1, 1] == pytest.approx(200.0)
        assert K[1, 7] == pytest.approx(-200.0)

        # UZ (index 2) has kz
        assert K[2, 2] == pytest.approx(300.0)
        assert K[2, 8] == pytest.approx(-300.0)

        # No cross-coupling between different DOFs
        assert K[0, 1] == pytest.approx(0.0)
        assert K[0, 2] == pytest.approx(0.0)
        assert K[1, 2] == pytest.approx(0.0)


class TestSpringElementInModel:
    """Tests for spring elements integrated into Model analysis."""

    def test_spring_provides_correct_stiffness(self):
        """Spring provides correct stiffness between nodes.

        Task 8.1 criterion: Spring provides correct stiffness between nodes

        Test: Vertical spring supporting a load should give deflection = F/k
        """
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)  # Fixed node
        n2 = model.get_or_create_node(0, 0, 1)  # Free node with spring support

        # Create spring with stiffness in all 6 DOFs
        # (avoids singular system from unrestricted DOFs)
        spring = model.create_spring(n1, n2)
        k = 1000.0  # kN/m
        spring.set_translational_stiffness(k, k, k)
        spring.set_rotational_stiffness(k, k, k)

        # Fix node 1 completely
        model.boundary_conditions.fix_node(n1.id)

        # Apply vertical load at node 2
        F = 100.0  # kN
        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UZ, F)

        # Analyze
        assert model.analyze(), f"Analysis failed: {model.get_error_message()}"

        # Expected deflection: δ = F/k = 100/1000 = 0.1 m
        u_z = model.get_node_displacement(n2.id, int(DOFIndex.UZ))
        expected = F / k

        np.testing.assert_almost_equal(u_z, expected, decimal=8)

    def test_spring_with_beam_element(self):
        """Spring element works together with beam element."""
        model = Model()

        # Create beam
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(3, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("IPE200", 0.00285, 1.94e-5, 1.42e-6, 7.0e-8)

        config = BeamConfig()
        config.formulation = BeamFormulation.EulerBernoulli
        model.create_beam(n1, n2, mat, sec, config)

        # Fix node 1
        model.boundary_conditions.fix_node(n1.id)

        # Add vertical spring support at node 2
        n3 = model.get_or_create_node(3, 0, -1)  # Ground node below n2
        model.boundary_conditions.fix_node(n3.id)

        spring = model.create_spring(n3, n2)
        spring.kz = 500.0  # Moderate spring stiffness

        # Apply load
        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UZ, -10.0)

        # Should analyze successfully
        assert model.analyze()

        # Deflection should be less than cantilever without spring
        u_z = model.get_node_displacement(n2.id, int(DOFIndex.UZ))
        assert u_z < 0  # Downward deflection

    def test_model_spring_elements_property(self):
        """Model.spring_elements property returns list of springs."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        n3 = model.get_or_create_node(2, 0, 0)

        spring1 = model.create_spring(n1, n2)
        spring2 = model.create_spring(n2, n3)

        springs = model.spring_elements
        assert len(springs) == 2
        assert springs[0] == spring1
        assert springs[1] == spring2


class TestPointMassBasics:
    """Basic tests for PointMass class."""

    def test_point_mass_creation(self):
        """PointMass can be created at a node."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)

        assert pm is not None
        assert pm.id == 1
        assert pm.node == node

    def test_point_mass_default_values(self):
        """Point mass properties default to zero."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)

        assert pm.mass == 0.0
        assert pm.Ixx == 0.0
        assert pm.Iyy == 0.0
        assert pm.Izz == 0.0
        assert pm.Ixy == 0.0
        assert pm.Ixz == 0.0
        assert pm.Iyz == 0.0

    def test_point_mass_set_mass(self):
        """Point mass can be set directly."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.mass = 10.0

        assert pm.mass == 10.0

    def test_point_mass_set_inertia(self):
        """set_inertia sets diagonal moments of inertia."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.set_inertia(5.0, 6.0, 7.0)

        assert pm.Ixx == 5.0
        assert pm.Iyy == 6.0
        assert pm.Izz == 7.0

    def test_point_mass_set_products_of_inertia(self):
        """set_products_of_inertia sets off-diagonal terms."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.set_products_of_inertia(1.0, 2.0, 3.0)

        assert pm.Ixy == 1.0
        assert pm.Ixz == 2.0
        assert pm.Iyz == 3.0

    def test_point_mass_set_full_inertia(self):
        """set_full_inertia sets all inertia components."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.set_full_inertia(10.0, 20.0, 30.0, 1.0, 2.0, 3.0)

        assert pm.Ixx == 10.0
        assert pm.Iyy == 20.0
        assert pm.Izz == 30.0
        assert pm.Ixy == 1.0
        assert pm.Ixz == 2.0
        assert pm.Iyz == 3.0


class TestPointMassMassMatrix:
    """Tests for point mass mass matrix."""

    def test_mass_matrix_shape(self):
        """Point mass matrix is 6x6."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.mass = 10.0

        M = pm.mass_matrix()

        assert M.shape == (6, 6)

    def test_mass_matrix_symmetry(self):
        """Point mass matrix is symmetric.

        Task 8.2 criterion: Inertia tensor is correctly represented
        """
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.mass = 10.0
        pm.set_full_inertia(5.0, 6.0, 7.0, 1.0, 0.5, 0.3)

        M = pm.mass_matrix()

        np.testing.assert_array_almost_equal(M, M.T)

    def test_mass_matrix_translational_terms(self):
        """Translational mass appears on first 3 diagonal entries."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        m = 12.5
        pm.mass = m

        M = pm.mass_matrix()

        assert M[0, 0] == pytest.approx(m)
        assert M[1, 1] == pytest.approx(m)
        assert M[2, 2] == pytest.approx(m)

    def test_mass_matrix_inertia_terms(self):
        """Rotational inertia appears in 3x3 block (3:6, 3:6)."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.set_inertia(10.0, 20.0, 30.0)

        M = pm.mass_matrix()

        assert M[3, 3] == pytest.approx(10.0)  # Ixx
        assert M[4, 4] == pytest.approx(20.0)  # Iyy
        assert M[5, 5] == pytest.approx(30.0)  # Izz

    def test_mass_matrix_off_diagonal_inertia(self):
        """Off-diagonal inertia terms work for asymmetric masses.

        Task 8.2 criterion: Off-diagonal terms work for asymmetric masses
        """
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.set_products_of_inertia(1.5, 2.5, 3.5)

        M = pm.mass_matrix()

        # Check off-diagonal terms are correctly placed
        assert M[3, 4] == pytest.approx(1.5)  # Ixy
        assert M[4, 3] == pytest.approx(1.5)  # Symmetric

        assert M[3, 5] == pytest.approx(2.5)  # Ixz
        assert M[5, 3] == pytest.approx(2.5)  # Symmetric

        assert M[4, 5] == pytest.approx(3.5)  # Iyz
        assert M[5, 4] == pytest.approx(3.5)  # Symmetric


class TestPointMassValidation:
    """Tests for point mass validation."""

    def test_is_valid_positive_mass(self):
        """Valid when mass is positive."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.mass = 10.0
        pm.set_inertia(5.0, 6.0, 7.0)

        assert pm.is_valid()

    def test_is_valid_zero_mass(self):
        """Valid when mass is zero."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.mass = 0.0

        assert pm.is_valid()

    def test_is_valid_negative_mass_invalid(self):
        """Invalid when mass is negative."""
        model = Model()
        node = model.get_or_create_node(0, 0, 0)

        pm = model.create_point_mass(node)
        pm.mass = -1.0

        assert not pm.is_valid()


class TestPointMassInModel:
    """Tests for point mass elements in Model."""

    def test_model_point_masses_property(self):
        """Model.point_masses property returns list of point masses."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        pm1 = model.create_point_mass(n1)
        pm2 = model.create_point_mass(n2)

        point_masses = model.point_masses
        assert len(point_masses) == 2
        assert point_masses[0] == pm1
        assert point_masses[1] == pm2


class TestSpringOnlyModel:
    """Tests for models containing only spring elements (no beams)."""

    def test_spring_only_model_analyzes(self):
        """Model with only spring elements can be analyzed."""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        # Create spring with stiffness in all DOFs to avoid singular system
        spring = model.create_spring(n1, n2)
        k = 1000.0
        spring.set_translational_stiffness(k, k, k)
        spring.set_rotational_stiffness(k, k, k)

        # Fix one node
        model.boundary_conditions.fix_node(n1.id)

        # Apply load
        F = 50.0
        lc = model.get_default_load_case()
        lc.add_nodal_load(n2.id, DOFIndex.UX, F)

        # Should analyze successfully
        assert model.analyze(), f"Analysis failed: {model.get_error_message()}"

        # Check displacement
        u_x = model.get_node_displacement(n2.id, int(DOFIndex.UX))
        expected = F / k
        np.testing.assert_almost_equal(u_x, expected, decimal=8)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
