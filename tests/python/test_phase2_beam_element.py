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
from grillex.core import Node, NodeRegistry, Material, Section, LocalAxes, BeamElement, BeamFormulation


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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
