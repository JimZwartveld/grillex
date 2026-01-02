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
from grillex.core import Node, NodeRegistry, Material, Section, LocalAxes, BeamElement, BeamFormulation, BeamConfig


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


class TestBeamElementOffsets:
    """Tests for Task 2.4: End Offsets acceptance criteria"""

    def setup_method(self):
        """Setup common test data"""
        self.steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)
        self.section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        self.node_a = Node(1, 0.0, 0.0, 0.0)
        self.node_b = Node(2, 6.0, 0.0, 0.0)

    def test_beam_without_offsets_has_no_offsets(self):
        """Beam without offsets reports has_offsets() as False"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        assert not beam.has_offsets()
        assert beam.effective_length() == beam.length

    def test_beam_with_offsets_reports_correctly(self):
        """Beam with offsets reports has_offsets() as True"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        # Set offsets in local coordinates (along local axes)
        offset_i = np.array([0.0, 0.0, 0.5])  # 0.5m in local z direction at node i
        offset_j = np.array([0.0, 0.0, -0.5])  # -0.5m in local z direction at node j
        beam.set_offsets(offset_i, offset_j)

        assert beam.has_offsets()

    def test_effective_length_with_axial_offsets(self):
        """Effective length changes correctly with axial offsets"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        # Original length is 6.0m (from x=0 to x=6)
        assert np.isclose(beam.length, 6.0)

        # Add axial offsets: move end i by +0.5m along beam, end j by -0.3m
        # Effective length should be reduced by 0.5 + 0.3 = 0.8m
        offset_i = np.array([0.5, 0.0, 0.0])  # +0.5m along local x (away from node i)
        offset_j = np.array([-0.3, 0.0, 0.0])  # -0.3m along local x (toward node i)
        beam.set_offsets(offset_i, offset_j)

        expected_length = 6.0 - 0.5 - 0.3
        assert np.isclose(beam.effective_length(), expected_length, rtol=1e-6)

    def test_effective_length_with_transverse_offsets(self):
        """Effective length increases with transverse offsets"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        # Add transverse offsets in local z direction
        # This creates a diagonal connection
        offset_i = np.array([0.0, 0.0, 0.5])  # 0.5m in +z at node i
        offset_j = np.array([0.0, 0.0, 0.5])  # 0.5m in +z at node j
        beam.set_offsets(offset_i, offset_j)

        # Length should remain unchanged (offsets are parallel to each other)
        assert np.isclose(beam.effective_length(), beam.length, rtol=1e-6)

        # Now set opposite offsets
        offset_j = np.array([0.0, 0.0, -0.5])  # -0.5m in -z at node j
        beam.set_offsets(offset_i, offset_j)

        # Effective length should increase: sqrt(6² + 1²) = sqrt(37)
        expected_length = np.sqrt(6.0**2 + 1.0**2)
        assert np.isclose(beam.effective_length(), expected_length, rtol=1e-6)

    def test_offset_affects_stiffness_matrix(self):
        """Stiffness matrix changes when offsets are applied"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        # Get stiffness without offsets
        K_no_offset = beam.local_stiffness_matrix()

        # Add offsets
        offset_i = np.array([0.0, 0.0, 0.3])
        offset_j = np.array([0.0, 0.0, 0.3])
        beam.set_offsets(offset_i, offset_j)

        # Get stiffness with offsets
        K_with_offset = beam.local_stiffness_matrix()

        # Matrices should be different
        assert not np.allclose(K_no_offset, K_with_offset)

        # But should still be symmetric
        assert np.allclose(K_with_offset, K_with_offset.T, atol=1e-10)

    def test_offset_affects_mass_matrix(self):
        """Mass matrix changes when offsets are applied"""
        beam = BeamElement(1, self.node_a, self.node_b, self.steel, self.section)

        # Get mass without offsets
        M_no_offset = beam.local_mass_matrix()

        # Add offsets
        offset_i = np.array([0.0, 0.0, 0.3])
        offset_j = np.array([0.0, 0.0, 0.3])
        beam.set_offsets(offset_i, offset_j)

        # Get mass with offsets
        M_with_offset = beam.local_mass_matrix()

        # Matrices should be different (offset couples translation and rotation)
        assert not np.allclose(M_no_offset, M_with_offset)

        # But should still be symmetric
        assert np.allclose(M_with_offset, M_with_offset.T, atol=1e-10)

    def test_cantilever_with_offset_tip_load(self):
        """Cantilever with offset at tip has increased deflection due to moment arm"""
        # Create a 6m cantilever beam
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)
        beam = BeamElement(1, node_i, node_j, self.steel, self.section)

        # Add vertical offset at tip (local z direction)
        # This means the load point is offset from the beam axis
        offset_z = 0.5  # 0.5m offset in z direction
        offset_j = np.array([0.0, 0.0, offset_z])
        beam.set_offsets(np.array([0.0, 0.0, 0.0]), offset_j)

        # Get stiffness matrix with offset
        K = beam.local_stiffness_matrix()
        K_free = K[6:12, 6:12]

        # Apply vertical load at the offset tip
        P = 10.0  # kN downward
        F_free = np.array([0.0, 0.0, -P, 0.0, 0.0, 0.0])

        # Solve for displacements
        u_free = np.linalg.solve(K_free, F_free)

        # The vertical displacement should be present
        delta_z = u_free[2]

        # Due to the offset, we also get rotation which causes additional deflection
        # The deflection should be non-zero (we don't have an exact analytical solution
        # for this complex case, so just verify it's physically reasonable)
        assert delta_z < 0  # Downward deflection
        assert np.abs(delta_z) > 1e-6  # Non-trivial deflection


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


class TestTimoshenkoBeam:
    """Tests for Task 2.6: Timoshenko Beam Element acceptance criteria"""

    def setup_method(self):
        """Setup common test data"""
        # Steel material
        self.steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)

        # Rectangular section 0.2m x 0.4m for testing
        # A = 0.08 m², Iz = 0.2*0.4³/12 = 0.001067 m⁴, Iy = 0.4*0.2³/12 = 0.000267 m⁴
        self.rect_section = Section(1, "Rect200x400", 0.08, 0.000267, 0.001067, 0.000569)
        # Set shear areas (5/6 * A for rectangular section)
        self.rect_section.set_shear_areas(0.08 * 5.0 / 6.0, 0.08 * 5.0 / 6.0)

        # Slender beam (L/d = 30, should behave like Euler-Bernoulli)
        self.node_slender_i = Node(1, 0.0, 0.0, 0.0)
        self.node_slender_j = Node(2, 12.0, 0.0, 0.0)  # L = 12m, d = 0.4m

        # Deep beam (L/d = 3, significant shear deformation)
        self.node_deep_i = Node(3, 0.0, 0.0, 0.0)
        self.node_deep_j = Node(4, 1.2, 0.0, 0.0)  # L = 1.2m, d = 0.4m

    def test_slender_beam_timoshenko_matches_euler_bernoulli(self):
        """For slender beams (L/d > 20), Timoshenko should match Euler-Bernoulli"""
        beam = BeamElement(1, self.node_slender_i, self.node_slender_j,
                          self.steel, self.rect_section)

        # Get stiffness matrices
        K_euler = beam.local_stiffness_matrix(BeamFormulation.EulerBernoulli)
        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        # For slender beams, should be very close (within 1%)
        relative_diff = np.max(np.abs((K_timo - K_euler) / (K_euler + 1e-10)))
        assert relative_diff < 0.01, f"Slender beam: Timoshenko should match Euler-Bernoulli, diff={relative_diff}"

    def test_deep_beam_timoshenko_softer_than_euler_bernoulli(self):
        """For deep beams (L/d < 5), Timoshenko should be softer (smaller stiffness)"""
        beam = BeamElement(1, self.node_deep_i, self.node_deep_j,
                          self.steel, self.rect_section)

        # Get stiffness matrices
        K_euler = beam.local_stiffness_matrix(BeamFormulation.EulerBernoulli)
        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        # For deep beams, Timoshenko bending stiffness should be notably smaller
        # Check bending stiffness in z-direction (DOF 1,1 - transverse displacement)
        k_euler_bend = K_euler[1, 1]
        k_timo_bend = K_timo[1, 1]

        # Timoshenko should be softer (smaller stiffness)
        assert k_timo_bend < k_euler_bend
        # Should be at least 5% different for L/d = 3
        relative_diff = (k_euler_bend - k_timo_bend) / k_euler_bend
        assert relative_diff > 0.05, f"Deep beam: expected >5% difference, got {relative_diff*100:.1f}%"

    def test_timoshenko_stiffness_symmetric(self):
        """Timoshenko stiffness matrix should be symmetric"""
        beam = BeamElement(1, self.node_slender_i, self.node_slender_j,
                          self.steel, self.rect_section)

        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        assert np.allclose(K_timo, K_timo.T, atol=1e-10)

    def test_timoshenko_stiffness_positive_semidefinite(self):
        """Timoshenko stiffness matrix should be positive semi-definite"""
        beam = BeamElement(1, self.node_slender_i, self.node_slender_j,
                          self.steel, self.rect_section)

        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        # Compute eigenvalues
        eigenvalues = np.linalg.eigvalsh(K_timo)

        # Should have 6 near-zero eigenvalues (rigid body modes)
        zero_eigenvalues = np.sum(np.abs(eigenvalues) < 1e-3)
        assert zero_eigenvalues == 6

        # Remaining eigenvalues should be positive
        non_zero_eigenvalues = eigenvalues[np.abs(eigenvalues) > 1e-3]
        assert np.all(non_zero_eigenvalues > 0)

    def test_timoshenko_mass_matrix_symmetric(self):
        """Timoshenko mass matrix should be symmetric"""
        beam = BeamElement(1, self.node_slender_i, self.node_slender_j,
                          self.steel, self.rect_section)

        M_timo = beam.local_mass_matrix(BeamFormulation.Timoshenko)

        assert np.allclose(M_timo, M_timo.T, atol=1e-10)

    def test_timoshenko_cantilever_deflection(self):
        """Timoshenko cantilever should have larger deflection than Euler-Bernoulli"""
        # Use deep beam for significant shear deformation
        beam = BeamElement(1, self.node_deep_i, self.node_deep_j,
                          self.steel, self.rect_section)

        # Get stiffness matrices
        K_euler = beam.local_stiffness_matrix(BeamFormulation.EulerBernoulli)
        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        # Extract the 6x6 stiffness relating free end DOFs
        K_euler_free = K_euler[6:12, 6:12]
        K_timo_free = K_timo[6:12, 6:12]

        # Apply load P = 10 kN in negative z direction
        P = 10.0
        F_free = np.array([0.0, 0.0, -P, 0.0, 0.0, 0.0])

        # Solve for displacements
        u_euler = np.linalg.solve(K_euler_free, F_free)
        u_timo = np.linalg.solve(K_timo_free, F_free)

        # Tip deflection in z
        delta_euler = u_euler[2]
        delta_timo = u_timo[2]

        # Timoshenko should have larger (more negative) deflection
        assert delta_timo < delta_euler
        # Should be noticeably different for deep beam
        relative_diff = abs((delta_timo - delta_euler) / delta_euler)
        assert relative_diff > 0.02, f"Expected >2% difference, got {relative_diff*100:.1f}%"

    def test_shear_deformation_factor_calculation(self):
        """Verify shear deformation factor φ is computed correctly"""
        # For a rectangular section beam
        # φ = 12EI / (κAsG L²)
        # With κ = 5/6, As = (5/6)A for rectangular section

        beam = BeamElement(1, self.node_deep_i, self.node_deep_j,
                          self.steel, self.rect_section)

        E = self.steel.E
        G = self.steel.G
        L = beam.length
        Iz = self.rect_section.Iz
        Asz = self.rect_section.Asz

        # Calculate expected phi_z
        phi_z_expected = 12.0 * E * Iz / (Asz * G * L * L)

        # Get Timoshenko stiffness
        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)
        K_euler = beam.local_stiffness_matrix(BeamFormulation.EulerBernoulli)

        # The bending stiffness should be reduced by factor (1 + phi_z)
        # k_timo = k_euler / (1 + phi_z)
        # So: (1 + phi_z) = k_euler / k_timo
        k_euler_bend = K_euler[1, 1]  # Bending stiffness about z-axis
        k_timo_bend = K_timo[1, 1]

        phi_z_computed = (k_euler_bend / k_timo_bend) - 1.0

        # Should match expected value
        assert np.isclose(phi_z_computed, phi_z_expected, rtol=0.01)

    def test_axial_and_torsion_unchanged(self):
        """Axial and torsional stiffness should be same for both formulations"""
        beam = BeamElement(1, self.node_deep_i, self.node_deep_j,
                          self.steel, self.rect_section)

        K_euler = beam.local_stiffness_matrix(BeamFormulation.EulerBernoulli)
        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        # Axial stiffness (DOF 0 and 6)
        assert np.allclose(K_euler[0, 0], K_timo[0, 0])
        assert np.allclose(K_euler[6, 6], K_timo[6, 6])

        # Torsional stiffness (DOF 3 and 9)
        assert np.allclose(K_euler[3, 3], K_timo[3, 3])
        assert np.allclose(K_euler[9, 9], K_timo[9, 9])


class TestWarpingBeam:
    """Tests for Task 2.7: Warping Beam Element (7th DOF) acceptance criteria"""

    def setup_method(self):
        """Setup common test data"""
        # Steel material
        self.steel = Material(1, "Steel", 210000000.0, 0.3, 7.85)

        # I-section with warping properties (IPE300)
        # A = 0.00538 m², Iy = 8.36e-5 m⁴, Iz = 6.04e-6 m⁴, J = 2.01e-7 m⁴
        # Iw ≈ 1.26e-5 m⁶ (typical for IPE300)
        self.i_section = Section(1, "IPE300", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        self.i_section.enable_warping(1.26e-5)  # Enable warping with Iw

        # Nodes for beam
        self.node_i = Node(1, 0.0, 0.0, 0.0)
        self.node_j = Node(2, 6.0, 0.0, 0.0)

    def test_node_warping_dof_control(self):
        """Node warping DOF can be enabled and queried"""
        node = Node(1, 0.0, 0.0, 0.0)

        # By default, warping DOF is disabled
        assert not node.has_warping_dof()
        assert node.num_active_dofs() == 6

        # Enable warping DOF
        node.enable_warping_dof()
        assert node.has_warping_dof()
        assert node.num_active_dofs() == 7

    def test_section_warping_configuration(self):
        """Section can be configured for warping analysis"""
        section = Section(1, "Test", 0.01, 0.0001, 0.0001, 0.0001)

        # By default, warping is disabled
        assert not section.requires_warping
        assert section.Iw == 0.0

        # Enable warping
        section.enable_warping(1.0e-5, 0.001)
        assert section.requires_warping
        assert section.Iw == 1.0e-5
        assert section.omega_max == 0.001

    def test_warping_stiffness_matrix_is_symmetric(self):
        """14×14 warping stiffness matrix is symmetric"""
        beam = BeamElement(1, self.node_i, self.node_j, self.steel, self.i_section)

        K_warp = beam.local_stiffness_matrix_warping()

        # Check shape
        assert K_warp.shape == (14, 14)

        # Check symmetry
        assert np.allclose(K_warp, K_warp.T, atol=1e-10)

    def test_warping_mass_matrix_is_symmetric(self):
        """14×14 warping mass matrix is symmetric"""
        beam = BeamElement(1, self.node_i, self.node_j, self.steel, self.i_section)

        M_warp = beam.local_mass_matrix_warping()

        # Check shape
        assert M_warp.shape == (14, 14)

        # Check symmetry
        assert np.allclose(M_warp, M_warp.T, atol=1e-10)

    def test_warping_transformation_matrix_structure(self):
        """14×14 transformation matrix has correct block diagonal structure"""
        beam = BeamElement(1, self.node_i, self.node_j, self.steel, self.i_section)

        T = beam.transformation_matrix_warping()

        # Check shape
        assert T.shape == (14, 14)

        # Get rotation matrix
        R = beam.local_axes.rotation_matrix

        # Check blocks
        assert np.allclose(T[0:3, 0:3], R)    # Node i translations
        assert np.allclose(T[3:6, 3:6], R)    # Node i rotations
        assert T[6, 6] == 1.0                 # Node i warping (scalar)
        assert np.allclose(T[7:10, 7:10], R)  # Node j translations
        assert np.allclose(T[10:13, 10:13], R)  # Node j rotations
        assert T[13, 13] == 1.0               # Node j warping (scalar)

    def test_zero_warping_constant_behaves_like_12dof(self):
        """For Iw = 0, warping matrix should behave like standard beam"""
        # Create section with zero warping constant
        section_no_warp = Section(1, "NoWarp", 0.00538, 0.0000836, 0.00000604, 0.000000201)
        section_no_warp.set_warping_constant(0.0)

        beam = BeamElement(1, self.node_i, self.node_j, self.steel, section_no_warp)

        # Get both matrices
        K12 = beam.local_stiffness_matrix()
        K14 = beam.local_stiffness_matrix_warping()

        # Extract the 12×12 portion from 14×14 (excluding warping DOFs at indices 6 and 13)
        indices_12 = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
        K12_from_14 = K14[np.ix_(indices_12, indices_12)]

        # Should match closely (St. Venant torsion only)
        assert np.allclose(K12_from_14, K12, rtol=1e-10)

    def test_warping_increases_torsional_stiffness(self):
        """Warping stiffness increases overall torsional rigidity"""
        beam = BeamElement(1, self.node_i, self.node_j, self.steel, self.i_section)

        # Get warping and non-warping stiffness
        K12 = beam.local_stiffness_matrix()
        K14 = beam.local_stiffness_matrix_warping()

        # Torsional stiffness (θx_i, θx_i)
        # In 12×12: index 3
        # In 14×14: index 3 (but now coupled with warping)
        k_torsion_12 = K12[3, 3]
        k_torsion_14 = K14[3, 3]

        # With warping, torsional stiffness should be higher
        assert k_torsion_14 > k_torsion_12

    def test_warping_stiffness_positive_semidefinite(self):
        """14×14 warping stiffness matrix is positive semi-definite"""
        beam = BeamElement(1, self.node_i, self.node_j, self.steel, self.i_section)

        K_warp = beam.local_stiffness_matrix_warping()

        # Compute eigenvalues
        eigenvalues = np.linalg.eigvalsh(K_warp)

        # Should have 7 near-zero eigenvalues (rigid body modes + warping mode)
        # Actually, warping adds coupling, so we still expect 6 rigid body modes
        zero_eigenvalues = np.sum(np.abs(eigenvalues) < 1e-3)
        assert zero_eigenvalues == 6

        # Remaining eigenvalues should be positive
        non_zero_eigenvalues = eigenvalues[np.abs(eigenvalues) > 1e-3]
        assert np.all(non_zero_eigenvalues > 0)

    def test_cantilever_with_warping_restrained(self):
        """Cantilever with warping restrained at fixed end has higher torsional stiffness"""
        beam = BeamElement(1, self.node_i, self.node_j, self.steel, self.i_section)

        K_warp = beam.local_stiffness_matrix_warping()

        # For a cantilever: fix all DOFs at node i (indices 0-6)
        # Apply torque at node j (index 10 for θx_j)

        # Extract free DOFs at node j (indices 7-13)
        K_free = K_warp[7:14, 7:14]

        # Apply unit torque
        T = 1.0  # kN·m
        F_free = np.array([0.0, 0.0, 0.0, T, 0.0, 0.0, 0.0])  # Torque in θx direction

        # Solve for displacements
        u_free = np.linalg.solve(K_free, F_free)

        # Twist at free end (θx_j)
        theta_x = u_free[3]

        # Should have some twist (positive value)
        assert theta_x > 0

        # Warping displacement at free end (index 6 in u_free)
        phi_prime = u_free[6]

        # Should also have non-zero warping displacement
        # (sign depends on direction, just check non-zero)
        assert abs(phi_prime) > 1e-10

    def test_warping_dof_arrays_size_seven(self):
        """Node DOF arrays have size 7 with warping support"""
        node = Node(1, 0.0, 0.0, 0.0)

        assert len(node.dof_active) == 7
        assert len(node.global_dof_numbers) == 7


class TestEndReleases:
    """Tests for Task 2.5: End Releases acceptance criteria"""

    def test_end_release_struct_creation(self):
        """EndRelease struct can be created and has all release flags"""
        from grillex.core import EndRelease

        release = EndRelease()

        # All flags should default to False (fully fixed)
        assert release.release_ux_i == False
        assert release.release_uy_i == False
        assert release.release_uz_i == False
        assert release.release_rx_i == False
        assert release.release_ry_i == False
        assert release.release_rz_i == False
        assert release.release_warp_i == False

        assert release.release_ux_j == False
        assert release.release_uy_j == False
        assert release.release_uz_j == False
        assert release.release_rx_j == False
        assert release.release_ry_j == False
        assert release.release_rz_j == False
        assert release.release_warp_j == False

        # Should have no releases
        assert release.has_any_release() == False

    def test_release_moment_convenience_methods(self):
        """Convenience methods release_moment_i/j release both bending moments"""
        from grillex.core import EndRelease

        release = EndRelease()
        release.release_moment_i()

        assert release.release_ry_i == True
        assert release.release_rz_i == True
        assert release.has_any_release() == True

        release2 = EndRelease()
        release2.release_moment_j()

        assert release2.release_ry_j == True
        assert release2.release_rz_j == True

    def test_release_all_rotations_convenience_methods(self):
        """Convenience methods release_all_rotations_i/j release all rotations"""
        from grillex.core import EndRelease

        release = EndRelease()
        release.release_all_rotations_i()

        assert release.release_rx_i == True
        assert release.release_ry_i == True
        assert release.release_rz_i == True

        release2 = EndRelease()
        release2.release_all_rotations_j()

        assert release2.release_rx_j == True
        assert release2.release_ry_j == True
        assert release2.release_rz_j == True

    def test_get_released_indices_12dof(self):
        """get_released_indices returns correct DOF indices for 12-DOF elements"""
        from grillex.core import EndRelease

        release = EndRelease()
        release.release_moment_i()  # Releases RY_i and RZ_i (indices 4, 5)

        indices = release.get_released_indices(False)  # 12-DOF

        assert 4 in indices  # RY_i
        assert 5 in indices  # RZ_i
        assert len(indices) == 2

    def test_get_released_indices_14dof(self):
        """get_released_indices excludes warping releases (they use force-free BC, not condensation)

        Warping releases represent bimoment = 0 (force-free condition), not structural
        elimination of the DOF. The warping DOF remains in the system with full stiffness
        coupling to torsion. Therefore, warping releases are NOT included in the static
        condensation indices.
        """
        from grillex.core import EndRelease

        release = EndRelease()
        release.release_warp_i = True  # Warp_i is index 6 in 14-DOF

        indices = release.get_released_indices(True)  # 14-DOF

        # Warping releases should NOT be included (force-free, not condensation)
        assert 6 not in indices  # WARP_i is NOT condensed out
        assert len(indices) == 0

        # But standard moment releases still work
        release.release_ry_i = True  # Moment about y
        indices = release.get_released_indices(True)
        assert 4 in indices  # RY_i is condensed out
        assert len(indices) == 1

    def test_simply_supported_beam_stiffness(self):
        """Simply supported beam (pinned-pinned) has correct reduced stiffness"""
        # Create a 4m horizontal beam
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 4.0, 0.0, 0.0)

        # Material: Steel
        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)  # kN/m², mT/m³

        # Section: Rectangular 0.2m x 0.3m
        b, h = 0.2, 0.3
        A = b * h
        Iy = b * h**3 / 12  # Bending about y (weak axis)
        Iz = h * b**3 / 12  # Bending about z (strong axis)
        J = 0.0001  # Small torsional constant
        sec = Section(1, "Rect200x300", A=A, Iy=Iy, Iz=Iz, J=J)

        # Create beam with releases at both ends (moment releases)
        beam = BeamElement(1, node_i, node_j, mat, sec, roll=0.0)
        beam.releases.release_moment_i()  # Pin at i
        beam.releases.release_moment_j()  # Pin at j

        # Get local stiffness matrix
        K = beam.local_stiffness_matrix()

        # For a simply supported beam, moment DOFs should be released
        # Check that moment DOFs (RY_i=4, RZ_i=5, RY_j=10, RZ_j=11) have very small diagonal terms
        assert abs(K[4, 4]) < 1e-6, "RY_i should be released"
        assert abs(K[5, 5]) < 1e-6, "RZ_i should be released"
        assert abs(K[10, 10]) < 1e-6, "RY_j should be released"
        assert abs(K[11, 11]) < 1e-6, "RZ_j should be released"

        # Axial stiffness should remain (not coupled to moments)
        assert K[0, 0] > 1e6, "Axial stiffness should remain"

        # NOTE: Transverse stiffnesses become zero when both moment ends are released
        # because the beam becomes a mechanism (no bending stiffness)

    def test_pinned_fixed_beam_stiffness(self):
        """Pinned-fixed beam has correct stiffness pattern"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 5.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0001)

        beam = BeamElement(1, node_i, node_j, mat, sec)
        beam.releases.release_moment_i()  # Pin at i (release moments)
        # Node j is fixed (no releases)

        K = beam.local_stiffness_matrix()

        # Node i moments should be released
        assert abs(K[4, 4]) < 1e-6, "RY_i should be released"
        assert abs(K[5, 5]) < 1e-6, "RZ_i should be released"

        # Node j moments should be stiff
        assert K[10, 10] > 1e3, "RY_j should be stiff"
        assert K[11, 11] > 1e3, "RZ_j should be stiff"

    def test_axial_release_sliding_joint(self):
        """Axial release creates sliding joint (no axial force)"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 3.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0001)

        beam = BeamElement(1, node_i, node_j, mat, sec)
        beam.releases.release_ux_i = True  # Axial release at i

        K = beam.local_stiffness_matrix()

        # Axial DOF at i should be released
        assert abs(K[0, 0]) < 1e-6, "UX_i should be released"

        # Bending DOFs should remain
        assert K[1, 1] > 1e3, "UY_i should be stiff"

        # NOTE: Axial release at i creates a mechanism, so UX_j also becomes zero
        # This is correct: if one end can slide freely, the whole element has no axial stiffness

    def test_torsion_release(self):
        """Torsion release allows free rotation about beam axis"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 4.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0002)

        beam = BeamElement(1, node_i, node_j, mat, sec)
        beam.releases.release_rx_i = True  # Torsion release at i

        K = beam.local_stiffness_matrix()

        # Torsion DOF at i should be released
        assert abs(K[3, 3]) < 1e-6, "RX_i should be released"

        # Bending should remain
        assert K[4, 4] > 1e3, "RY_i should be stiff"

    def test_mass_matrix_with_releases(self):
        """Mass matrix is also modified by releases"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 4.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0001)

        beam = BeamElement(1, node_i, node_j, mat, sec)
        beam.releases.release_moment_i()

        M = beam.local_mass_matrix()

        # Released moment DOFs should have near-zero diagonal terms
        assert abs(M[4, 4]) < 1e-6, "RY_i mass should be released"
        assert abs(M[5, 5]) < 1e-6, "RZ_i mass should be released"

        # Translational mass should remain (use smaller threshold due to condensation effects)
        assert M[0, 0] > 1e-8, "Axial mass should remain"
        assert M[1, 1] > 1e-8, "Transverse mass should remain"

    def test_warping_release_14dof(self):
        """Warping release represents force-free condition, not structural elimination.

        Warping releases represent bimoment = 0 (force-free condition) at the element end,
        NOT structural elimination of the DOF via static condensation. The warping DOF
        remains in the system with full torsion-warping coupling stiffness.

        This is physically correct because:
        1. Torsion and warping are coupled in thin-walled beams (Vlasov theory)
        2. A force-free warping end (B=0) still contributes to overall torsional stiffness
        3. Similar to a simply-supported beam: moment=0 at ends, but bending stiffness exists
        """
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 5.0, 0.0, 0.0)

        # Enable warping DOF
        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)

        # I-beam section with warping
        sec = Section(1, "I-beam", A=0.015, Iy=0.0001, Iz=0.0002, J=0.00001)
        sec.enable_warping(Iw=0.00005)

        beam = BeamElement(1, node_i, node_j, mat, sec)
        beam.releases.release_warp_i = True  # Release warping at i

        K = beam.local_stiffness_matrix_warping()

        # Warping DOF at i (index 6) retains stiffness (force-free, not eliminated)
        # The stiffness comes from torsion-warping coupling in Vlasov beam theory
        assert K[6, 6] > 1e-6, "WARP_i should retain stiffness (force-free BC, not condensed)"

        # Warping DOF at j (index 13) should also have stiffness
        assert K[13, 13] > 1e-6, "WARP_j should be stiff"

        # Verify the release flag is set
        assert beam.releases.release_warp_i == True
        assert beam.releases.release_warp_j == False

    def test_multiple_releases_combined(self):
        """Multiple releases can be combined"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 4.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0001)

        beam = BeamElement(1, node_i, node_j, mat, sec)

        # Release multiple DOFs at end i
        beam.releases.release_ux_i = True   # Axial
        beam.releases.release_moment_i()    # Both moments

        K = beam.local_stiffness_matrix()

        # All released DOFs should have near-zero diagonal
        assert abs(K[0, 0]) < 1e-6, "UX_i should be released"
        assert abs(K[4, 4]) < 1e-6, "RY_i should be released"
        assert abs(K[5, 5]) < 1e-6, "RZ_i should be released"

        # Non-released DOFs should remain
        assert K[1, 1] > 1e3, "UY_i should be stiff"
        assert K[2, 2] > 1e3, "UZ_i should be stiff"
        assert K[3, 3] > 1e-6, "RX_i should be stiff"

    def test_timoshenko_beam_with_releases(self):
        """End releases work with Timoshenko formulation"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 3.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0001)

        beam = BeamElement(1, node_i, node_j, mat, sec)
        beam.releases.release_moment_i()

        # Get Timoshenko stiffness
        K_timo = beam.local_stiffness_matrix(BeamFormulation.Timoshenko)

        # Released moments should still be zero with Timoshenko
        assert abs(K_timo[4, 4]) < 1e-6, "RY_i should be released"
        assert abs(K_timo[5, 5]) < 1e-6, "RZ_i should be released"

        # Shear stiffness should be present
        assert K_timo[1, 1] > 1e3, "Shear stiffness should remain"

    def test_beam_element_has_releases_member(self):
        """BeamElement has releases member that can be accessed"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 4.0, 0.0, 0.0)

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "Rect", A=0.06, Iy=0.00045, Iz=0.0004, J=0.0001)

        beam = BeamElement(1, node_i, node_j, mat, sec)

        # Should have releases attribute
        assert hasattr(beam, 'releases')

        # Should be able to modify it
        beam.releases.release_ry_i = True
        assert beam.releases.release_ry_i == True
        assert beam.releases.has_any_release() == True


class TestWarpingOffsetTransformation:
    """Tests for Task 2.7 extension: offset transformation for 14×14 matrices"""

    def test_offset_transformation_matrix_warping_size(self):
        """Offset transformation matrix for warping elements is 14×14"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # Set an offset
        beam.offset_i = np.array([0.0, 0.2, 0.0])

        # Get offset transformation matrix
        T = beam.offset_transformation_matrix_warping()

        # Should be 14×14
        assert T.shape == (14, 14)

    def test_offset_transformation_matrix_warping_no_offsets(self):
        """With no offsets, transformation matrix is identity"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # No offsets (default is zero)
        T = beam.offset_transformation_matrix_warping()

        # Should be identity matrix
        I = np.eye(14)
        np.testing.assert_allclose(T, I, rtol=1e-12)

    def test_offset_transformation_warping_dof_uncoupled(self):
        """Warping DOFs are not coupled with offsets (identity transformation)"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # Set offsets at both ends
        beam.offset_i = np.array([0.0, 0.3, 0.1])
        beam.offset_j = np.array([0.0, -0.2, 0.15])

        T = beam.offset_transformation_matrix_warping()

        # Warping DOF rows (6 and 13) should be identity
        # Row 6 should be [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
        assert T[6, 6] == 1.0
        for j in range(14):
            if j != 6:
                assert abs(T[6, j]) < 1e-12, f"T[6,{j}] should be zero"

        # Row 13 should be [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
        assert T[13, 13] == 1.0
        for j in range(14):
            if j != 13:
                assert abs(T[13, j]) < 1e-12, f"T[13,{j}] should be zero"

    def test_offset_transformation_translation_rotation_coupling(self):
        """Offsets create coupling between translations and rotations (not warping)"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # Set offset at node i in Z direction
        beam.offset_i = np.array([0.0, 0.0, 0.5])

        T = beam.offset_transformation_matrix_warping()

        # Translation-rotation coupling should exist
        # T[0:3, 3:6] should be non-zero (skew-symmetric matrix)
        # For offset in Z, should see coupling with RX and RY
        assert abs(T[0, 4]) > 1e-6 or abs(T[0, 5]) > 1e-6, "Should have trans-rot coupling"

    def test_stiffness_with_offsets_and_warping(self):
        """Stiffness matrix with offsets and warping is symmetric"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # Set offsets
        beam.offset_i = np.array([0.0, 0.2, 0.1])
        beam.offset_j = np.array([0.0, -0.15, 0.2])

        K = beam.local_stiffness_matrix_warping()

        # Should be symmetric
        np.testing.assert_allclose(K, K.T, rtol=1e-10)

    def test_mass_with_offsets_and_warping(self):
        """Mass matrix with offsets and warping is symmetric"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # Set offsets
        beam.offset_i = np.array([0.0, 0.25, 0.0])

        M = beam.local_mass_matrix_warping()

        # Should be symmetric
        np.testing.assert_allclose(M, M.T, rtol=1e-10)

    def test_global_matrices_with_offsets_and_warping(self):
        """Global stiffness and mass matrices work with offsets and warping"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        config = BeamConfig()
        config.include_warping = True
        beam = BeamElement(1, node_i, node_j, mat, sec, config)

        # Set offsets
        beam.offset_i = np.array([0.0, 0.3, 0.0])
        beam.offset_j = np.array([0.0, -0.2, 0.0])

        K_global = beam.global_stiffness_matrix_warping()
        M_global = beam.global_mass_matrix_warping()

        # Both should be symmetric
        np.testing.assert_allclose(K_global, K_global.T, rtol=1e-10)
        np.testing.assert_allclose(M_global, M_global.T, rtol=1e-10)

        # Verify matrices are well-formed
        # Stiffness should have non-zero diagonal
        assert np.all(np.abs(np.diag(K_global)) > 1e-12), "Stiffness should have non-zero diagonal"

        # Mass matrix warping DOF (indices 6 and 13) are zero (expected - warping inertia neglected)
        # All other diagonal terms should be positive
        for i in range(14):
            if i not in [6, 13]:
                assert np.abs(M_global[i, i]) > 1e-12, f"Mass diagonal at {i} should be non-zero"

    def test_offset_consistency_12dof_vs_14dof(self):
        """Offset behavior is consistent between 12-DOF and 14-DOF elements"""
        node_i = Node(1, 0.0, 0.0, 0.0)
        node_j = Node(2, 6.0, 0.0, 0.0)

        # Enable warping for 14-DOF test
        node_i.enable_warping_dof()
        node_j.enable_warping_dof()

        mat = Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-6)
        sec = Section(1, "I-Beam", A=0.01, Iy=2e-4, Iz=5e-5, J=1e-5)
        sec.enable_warping(1e-6, 0.1)

        # Create 12-DOF beam with offset
        beam_12 = BeamElement(1, node_i, node_j, mat, sec)
        beam_12.offset_i = np.array([0.0, 0.2, 0.0])

        # Create 14-DOF beam with same offset
        config = BeamConfig()
        config.include_warping = True
        beam_14 = BeamElement(2, node_i, node_j, mat, sec, config)
        beam_14.offset_i = np.array([0.0, 0.2, 0.0])

        K_12 = beam_12.local_stiffness_matrix()
        K_14 = beam_14.local_stiffness_matrix_warping()

        # Extract the 12×12 sub-block from the 14×14 matrix
        # Mapping: [0-2, 3-5] from 12×12 → [0-2, 3-5] in 14×14 (node i without warp)
        #          [6-8, 9-11] from 12×12 → [7-9, 10-12] in 14×14 (node j without warp)

        # Check translation terms (should match exactly)
        np.testing.assert_allclose(K_14[0:3, 0:3], K_12[0:3, 0:3], rtol=1e-10)
        np.testing.assert_allclose(K_14[0:3, 7:10], K_12[0:3, 6:9], rtol=1e-10)
        np.testing.assert_allclose(K_14[7:10, 0:3], K_12[6:9, 0:3], rtol=1e-10)
        np.testing.assert_allclose(K_14[7:10, 7:10], K_12[6:9, 6:9], rtol=1e-10)

        # Bending terms (RY) should match
        # RY at node i: index 4 in both
        np.testing.assert_allclose(K_14[4, 4], K_12[4, 4], rtol=1e-10)

        # Note: Torsion (RX and RZ) will differ because 14-DOF includes warping torsion coupling


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
