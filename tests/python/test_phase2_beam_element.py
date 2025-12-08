"""
Tests for Phase 2: Beam Element Foundation

This test suite verifies acceptance criteria for Phase 2 tasks:
- Task 2.1: Local Coordinate System
- Task 2.2: Beam Element Stiffness Matrix
- Task 2.3: Beam Element Mass Matrix

Tests verify correct implementation of:
- Local axes computation for horizontal, vertical, and arbitrary beams
- Roll angle rotation
- 12x12 stiffness matrix assembly
- 12x12 mass matrix assembly
- Transformation matrices
- Simple validation against known beam deflections
"""

import pytest
import numpy as np
from grillex.core import Node, NodeRegistry, Material, Section, LocalAxes, BeamElement


class TestLocalAxes:
    """Tests for Task 2.1: Local Coordinate System acceptance criteria"""

    def test_horizontal_beam_along_x(self):
        """Horizontal beam along global X has local z pointing up (global Z)"""
        end_a = np.array([0.0, 0.0, 0.0])
        end_b = np.array([6.0, 0.0, 0.0])

        axes = LocalAxes(end_a, end_b)

        # Local x should be along global X
        assert np.allclose(axes.x_axis, [1.0, 0.0, 0.0])

        # Local z should point up (global Z direction)
        assert np.allclose(axes.z_axis, [0.0, 0.0, 1.0], atol=1e-10)

        # Local y should be perpendicular to both x and z (completing right-handed system)
        # y = z × x = (0,0,1) × (1,0,0) = (0,1,0)
        assert np.allclose(axes.y_axis, [0.0, 1.0, 0.0], atol=1e-10)

    def test_vertical_beam_no_nan(self):
        """Vertical beam has well-defined local axes (no NaN)"""
        end_a = np.array([0.0, 0.0, 0.0])
        end_b = np.array([0.0, 0.0, 5.0])

        axes = LocalAxes(end_a, end_b)

        # Local x should be along global Z
        assert np.allclose(axes.x_axis, [0.0, 0.0, 1.0])

        # No NaN values in any axis
        assert not np.isnan(axes.x_axis).any()
        assert not np.isnan(axes.y_axis).any()
        assert not np.isnan(axes.z_axis).any()

        # Axes should be orthonormal
        assert np.allclose(np.dot(axes.x_axis, axes.y_axis), 0.0, atol=1e-10)
        assert np.allclose(np.dot(axes.y_axis, axes.z_axis), 0.0, atol=1e-10)
        assert np.allclose(np.dot(axes.z_axis, axes.x_axis), 0.0, atol=1e-10)

    def test_roll_angle_rotates_y_and_z(self):
        """Roll angle rotates y and z about x"""
        end_a = np.array([0.0, 0.0, 0.0])
        end_b = np.array([6.0, 0.0, 0.0])
        roll = np.pi / 2  # 90 degrees

        axes = LocalAxes(end_a, end_b, roll)

        # Local x should still be along global X (unchanged by roll)
        assert np.allclose(axes.x_axis, [1.0, 0.0, 0.0])

        # After 90-degree roll about x:
        # Rotation matrix R_x(90°) rotates y and z
        # Original y = (0,1,0), z = (0,0,1)
        # After rotation: y' = cos(90)*y - sin(90)*z = 0*y - 1*z = (0,0,-1)
        #                 z' = sin(90)*y + cos(90)*z = 1*y + 0*z = (0,1,0)
        assert np.allclose(axes.y_axis, [0.0, 0.0, -1.0], atol=1e-10)
        assert np.allclose(axes.z_axis, [0.0, 1.0, 0.0], atol=1e-10)

    def test_rotation_matrix_orthonormal(self):
        """Rotation matrix is orthonormal (R^T * R = I)"""
        end_a = np.array([1.0, 2.0, 3.0])
        end_b = np.array([4.0, 5.0, 6.0])

        axes = LocalAxes(end_a, end_b)

        R = np.array(axes.rotation_matrix)

        # R^T * R should equal identity
        I = R.T @ R
        assert np.allclose(I, np.eye(3), atol=1e-10)

        # Determinant should be +1 (proper rotation, not reflection)
        assert np.allclose(np.linalg.det(R), 1.0, atol=1e-10)

    def test_to_local_to_global_roundtrip(self):
        """Transforming global->local->global returns original vector"""
        end_a = np.array([0.0, 0.0, 0.0])
        end_b = np.array([3.0, 4.0, 0.0])

        axes = LocalAxes(end_a, end_b)

        global_vec = np.array([1.0, 2.0, 3.0])

        # Transform to local and back to global
        local_vec = axes.to_local(global_vec)
        global_vec_back = axes.to_global(local_vec)

        assert np.allclose(global_vec_back, global_vec, atol=1e-10)


class TestBeamElementStiffness:
    """Tests for Task 2.2: Beam Element Stiffness Matrix acceptance criteria"""

    def setup_method(self):
        """Setup common test data"""
        # Steel material
        self.steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)

        # IPE300 section
        self.section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)

        # Nodes
        self.node_a = Node(1, 0.0, 0.0, 0.0)
        self.node_b = Node(2, 6.0, 0.0, 0.0)

    def test_stiffness_matrix_is_symmetric(self):
        """Stiffness matrix is symmetric"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        K_local = beam.local_stiffness_matrix()
        K_global = beam.global_stiffness_matrix()

        # Check symmetry
        assert np.allclose(K_local, K_local.T, atol=1e-10)
        assert np.allclose(K_global, K_global.T, atol=1e-10)

    def test_stiffness_matrix_positive_semidefinite(self):
        """Stiffness matrix is positive semi-definite with 6 zero eigenvalues"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        K_local = beam.local_stiffness_matrix()

        # Compute eigenvalues
        eigenvalues = np.linalg.eigvalsh(K_local)

        # Should have 6 near-zero eigenvalues (rigid body modes)
        zero_eigenvalues = np.sum(np.abs(eigenvalues) < 1e-3)
        assert zero_eigenvalues == 6

        # Remaining eigenvalues should be positive
        non_zero_eigenvalues = eigenvalues[np.abs(eigenvalues) > 1e-3]
        assert np.all(non_zero_eigenvalues > 0)

    def test_cantilever_deflection(self):
        """Simple cantilever deflection matches: δ = PL³/(3EI)"""
        # Create a 6m long beam
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        beam = BeamElement(1, node_i, node_j, self.steel, self.section)

        # Get local stiffness matrix
        K = beam.local_stiffness_matrix()

        # For a cantilever with tip load P in negative z-direction:
        # Fixed end: u = [0, 0, 0, 0, 0, 0]
        # Loaded end: F = [0, 0, -P, 0, 0, 0]

        # Extract the 6x6 stiffness relating free end DOFs
        K_free = K[6:12, 6:12]

        # Apply load P = 10 kN in negative z direction
        P = 10.0  # kN
        F_free = np.array([0.0, 0.0, -P, 0.0, 0.0, 0.0])

        # Solve for displacements
        u_free = np.linalg.solve(K_free, F_free)

        # Tip deflection in z (w_j, index 2)
        delta_z = u_free[2]

        # Theoretical deflection: δ = -PL³/(3EI) (negative because load is downward)
        L = 6.0
        E = self.steel.E
        I = self.section.Iy  # Bending about y-axis (deflection in z)
        delta_theory = -P * L**3 / (3.0 * E * I)  # Negative for downward deflection

        # Check agreement (should be very close for Euler-Bernoulli beam)
        assert np.abs(delta_z - delta_theory) / np.abs(delta_theory) < 0.01  # Within 1%

    def test_axial_stiffness(self):
        """Axial stiffness k = EA/L is correct"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        K = beam.local_stiffness_matrix()

        # Axial stiffness is at K(0,0) and K(6,6)
        k_axial_expected = self.steel.E * self.section.A / beam.length

        assert np.allclose(K[0, 0], k_axial_expected)
        assert np.allclose(K[6, 6], k_axial_expected)
        assert np.allclose(K[0, 6], -k_axial_expected)

    def test_transformation_matrix_structure(self):
        """Transformation matrix has block diagonal structure"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        T = beam.transformation_matrix()

        # Should be block diagonal: 4 blocks of 3x3
        R = beam.local_axes.rotation_matrix

        # Check each 3x3 block
        assert np.allclose(T[0:3, 0:3], R)
        assert np.allclose(T[3:6, 3:6], R)
        assert np.allclose(T[6:9, 6:9], R)
        assert np.allclose(T[9:12, 9:12], R)

        # Off-diagonal blocks should be zero
        assert np.allclose(T[0:3, 3:6], 0.0)
        assert np.allclose(T[3:6, 6:9], 0.0)


class TestBeamElementMass:
    """Tests for Task 2.3: Beam Element Mass Matrix acceptance criteria"""

    def setup_method(self):
        """Setup common test data"""
        self.steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        self.section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        self.node_a = Node(1, 0.0, 0.0, 0.0)
        self.node_b = Node(2, 6.0, 0.0, 0.0)

    def test_mass_matrix_is_symmetric(self):
        """Mass matrix is symmetric"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        M_local = beam.local_mass_matrix()
        M_global = beam.global_mass_matrix()

        assert np.allclose(M_local, M_local.T, atol=1e-10)
        assert np.allclose(M_global, M_global.T, atol=1e-10)

    def test_mass_matrix_positive_semidefinite(self):
        """Mass matrix is positive semi-definite"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        M_local = beam.local_mass_matrix()

        # Compute eigenvalues
        eigenvalues = np.linalg.eigvalsh(M_local)

        # All eigenvalues should be non-negative
        assert np.all(eigenvalues >= -1e-10)

    def test_total_mass_equals_rho_A_L(self):
        """Total mass equals ρAL when integrated"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        M = beam.local_mass_matrix()

        # For consistent mass matrix, sum of all translational mass terms
        # should equal total mass
        expected_mass = self.steel.rho * self.section.A * beam.length

        # Extract translational DOFs (u, v, w at both ends)
        # Translational masses are in rows/cols: 0, 1, 2 (node i) and 6, 7, 8 (node j)
        trans_indices = [0, 1, 2, 6, 7, 8]

        # Sum the entire translational mass submatrix
        M_trans = M[np.ix_(trans_indices, trans_indices)]
        total_mass = np.sum(M_trans) / 3.0  # Divide by 3 because of 3 directions

        # Should be close to expected mass (within a few percent for consistent mass)
        assert np.abs(total_mass - expected_mass) / expected_mass < 0.05


class TestPythonBindings:
    """Tests for Python bindings of Phase 2 classes"""

    def test_import_beam_classes(self):
        """Can import LocalAxes and BeamElement from grillex.core"""
        from grillex.core import LocalAxes, BeamElement

        assert LocalAxes is not None
        assert BeamElement is not None

    def test_create_beam_element_from_python(self):
        """Can create BeamElement instance from Python"""
        steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        node_a = Node(1, 0.0, 0.0, 0.0)
        node_b = Node(2, 6.0, 0.0, 0.0)

        beam = BeamElement(1, node_a, node_b, steel, section)

        assert isinstance(beam, BeamElement)
        assert beam.id == 1
        assert beam.length > 0
        assert beam.node_i.id == 1
        assert beam.node_j.id == 2

    def test_beam_element_matrices_accessible(self):
        """Can access stiffness and mass matrices from Python"""
        steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        node_a = Node(1, 0.0, 0.0, 0.0)
        node_b = Node(2, 6.0, 0.0, 0.0)

        beam = BeamElement(1, node_a, node_b, steel, section)

        K_local = beam.local_stiffness_matrix()
        K_global = beam.global_stiffness_matrix()
        M_local = beam.local_mass_matrix()
        M_global = beam.global_mass_matrix()
        T = beam.transformation_matrix()

        # All should be 12x12 matrices
        assert K_local.shape == (12, 12)
        assert K_global.shape == (12, 12)
        assert M_local.shape == (12, 12)
        assert M_global.shape == (12, 12)
        assert T.shape == (12, 12)

    def test_local_axes_accessible(self):
        """Can access local axes from Python"""
        steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        node_a = Node(1, 0.0, 0.0, 0.0)
        node_b = Node(2, 6.0, 0.0, 0.0)

        beam = BeamElement(1, node_a, node_b, steel, section)

        axes = beam.local_axes
        assert hasattr(axes, 'x_axis')
        assert hasattr(axes, 'y_axis')
        assert hasattr(axes, 'z_axis')
        assert hasattr(axes, 'rotation_matrix')


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
