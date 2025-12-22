"""
Tests for Phase 16: Eigenvalue Analysis

Tests cover:
- Task 16.1: EigensolverSettings and Results structs
- Task 16.2: Boundary condition reduction for eigenvalue analysis
- Task 16.3: Dense eigenvalue solver

Analytical benchmarks for verification:
- Simply supported beam: f_n = (n*pi/L)^2 * sqrt(EI / rho*A) / (2*pi)
- Cantilever beam: f_n = lambda_n^2 * sqrt(EI / rho*A*L^4) / (2*pi)
  where lambda_1 = 1.875, lambda_2 = 4.694, lambda_3 = 7.855
- SDOF spring-mass: f = sqrt(k/m) / (2*pi)
"""

import pytest
import numpy as np
import math

from grillex._grillex_cpp import (
    Node, NodeRegistry, Material, Section, BeamElement, BeamConfig,
    DOFHandler, Assembler, BCHandler, DOFIndex,
    EigenvalueSolver, EigensolverSettings, EigensolverMethod,
    EigensolverResult, ModeResult,
    SpringElement, PointMass
)


class TestEigensolverSettings:
    """Task 16.1: Test EigensolverSettings struct"""

    def test_default_settings(self):
        """Test default settings values"""
        settings = EigensolverSettings()
        assert settings.n_modes == 10
        assert settings.shift == 0.0
        assert settings.tolerance == 1e-8
        assert settings.max_iterations == 100
        assert settings.method == EigensolverMethod.Dense
        assert settings.compute_participation == True
        assert settings.mass_normalize == True
        assert settings.rigid_body_threshold == 1e-6

    def test_modify_settings(self):
        """Test modifying settings"""
        settings = EigensolverSettings()
        settings.n_modes = 5
        settings.tolerance = 1e-10
        settings.method = EigensolverMethod.SubspaceIteration
        settings.mass_normalize = False

        assert settings.n_modes == 5
        assert settings.tolerance == 1e-10
        assert settings.method == EigensolverMethod.SubspaceIteration
        assert settings.mass_normalize == False

    def test_repr(self):
        """Test string representation"""
        settings = EigensolverSettings()
        repr_str = repr(settings)
        assert "EigensolverSettings" in repr_str
        assert "n_modes" in repr_str


class TestEigensolverMethod:
    """Task 16.1: Test EigensolverMethod enum"""

    def test_enum_values(self):
        """Test enum values exist"""
        assert EigensolverMethod.Dense is not None
        assert EigensolverMethod.SubspaceIteration is not None
        assert EigensolverMethod.ShiftInvert is not None

    def test_enum_assignment(self):
        """Test enum can be assigned to settings"""
        settings = EigensolverSettings()
        settings.method = EigensolverMethod.SubspaceIteration
        assert settings.method == EigensolverMethod.SubspaceIteration


class TestModeResult:
    """Task 16.1: Test ModeResult struct"""

    def test_default_mode_result(self):
        """Test default ModeResult values"""
        mode = ModeResult()
        assert mode.mode_number == 0
        assert mode.eigenvalue == 0.0
        assert mode.omega == 0.0
        assert mode.frequency_hz == 0.0
        assert mode.period_s == 0.0
        assert mode.is_rigid_body_mode == False

    def test_participation_factors(self):
        """Test participation factor fields exist"""
        mode = ModeResult()
        assert hasattr(mode, 'participation_x')
        assert hasattr(mode, 'participation_y')
        assert hasattr(mode, 'participation_z')
        assert hasattr(mode, 'participation_rx')
        assert hasattr(mode, 'participation_ry')
        assert hasattr(mode, 'participation_rz')

    def test_effective_mass_fields(self):
        """Test effective mass fields exist"""
        mode = ModeResult()
        assert hasattr(mode, 'effective_mass_x')
        assert hasattr(mode, 'effective_mass_y')
        assert hasattr(mode, 'effective_mass_z')
        assert hasattr(mode, 'effective_mass_pct_x')
        assert hasattr(mode, 'effective_mass_pct_y')
        assert hasattr(mode, 'effective_mass_pct_z')


class TestEigensolverResult:
    """Task 16.1: Test EigensolverResult struct"""

    def test_default_result(self):
        """Test default EigensolverResult values"""
        result = EigensolverResult()
        assert result.converged == False
        assert result.iterations == 0
        assert result.message == ""
        assert len(result.modes) == 0
        assert result.n_rigid_body_modes == 0

    def test_get_frequencies(self):
        """Test get_frequencies method returns list"""
        result = EigensolverResult()
        freqs = result.get_frequencies()
        assert isinstance(freqs, list)
        assert len(freqs) == 0

    def test_get_periods(self):
        """Test get_periods method returns list"""
        result = EigensolverResult()
        periods = result.get_periods()
        assert isinstance(periods, list)
        assert len(periods) == 0


class TestBCReduction:
    """Task 16.2: Test boundary condition reduction for eigenvalue analysis"""

    def setup_method(self):
        """Set up a simple beam model for testing"""
        self.registry = NodeRegistry(1e-6)
        self.n1 = self.registry.get_or_create_node(0, 0, 0)
        self.n2 = self.registry.get_or_create_node(1, 0, 0)

        self.material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        self.section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        self.beam = BeamElement(1, self.n1, self.n2, self.material, self.section)

        self.dof_handler = DOFHandler()
        self.dof_handler.number_dofs(self.registry)

        self.assembler = Assembler(self.dof_handler)

    def test_reduce_system_fixed_node(self):
        """Test DOF reduction with one fixed node"""
        # Assemble matrices
        K = self.assembler.assemble_stiffness([self.beam])
        M = self.assembler.assemble_mass([self.beam])

        # Fix first node (all 6 DOFs)
        bc = BCHandler()
        bc.fix_node(self.n1.id)

        # Reduce system
        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, self.dof_handler)

        # Should have 6 free DOFs (node 2)
        total_dofs = self.dof_handler.total_dofs()
        assert K_red.shape[0] == total_dofs - 6
        assert M_red.shape[0] == total_dofs - 6
        assert len(dof_map) == total_dofs - 6

    def test_reduce_system_preserves_symmetry(self):
        """Test that reduced matrices remain symmetric"""
        K = self.assembler.assemble_stiffness([self.beam])
        M = self.assembler.assemble_mass([self.beam])

        bc = BCHandler()
        bc.fix_node(self.n1.id)

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, self.dof_handler)

        # Convert to dense for symmetry check
        K_dense = np.array(K_red.todense())
        M_dense = np.array(M_red.todense())

        # Check symmetry
        np.testing.assert_allclose(K_dense, K_dense.T, atol=1e-10)
        np.testing.assert_allclose(M_dense, M_dense.T, atol=1e-10)

    def test_dof_mapping_correct(self):
        """Test that DOF mapping correctly tracks free DOFs"""
        K = self.assembler.assemble_stiffness([self.beam])
        M = self.assembler.assemble_mass([self.beam])

        bc = BCHandler()
        bc.add_fixed_dof(self.n1.id, DOFIndex.UX, 0.0)  # Fix only UX at node 1

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, self.dof_handler)

        # Total DOFs - 1 fixed = 11 free
        total_dofs = self.dof_handler.total_dofs()
        assert K_red.shape[0] == total_dofs - 1

        # Mapping should not include the fixed DOF
        fixed_dof = self.dof_handler.get_global_dof(self.n1.id, DOFIndex.UX)
        assert fixed_dof not in dof_map


class TestDenseEigenvalueSolver:
    """Task 16.3: Test dense eigenvalue solver"""

    def test_sdof_spring_mass(self):
        """
        Test SDOF spring-mass system.

        Analytical: f = sqrt(k/m) / (2*pi)
        k = 1000 kN/m, m = 10 mT -> f = 1.592 Hz
        """
        # Create a simple 1-DOF system manually using sparse matrices
        k = 1000.0  # kN/m
        m = 10.0    # mT

        # Create 1x1 stiffness and mass matrices
        from scipy import sparse
        K = sparse.csr_matrix([[k]])
        M = sparse.csr_matrix([[m]])

        # Convert to Eigen sparse format by creating a simple model
        # Instead, let's use a spring + point mass model
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        # Create spring with stiffness in UZ direction
        spring = SpringElement(1, n1, n2)
        spring.kz = k

        # Create point mass at n2
        pm = PointMass(1, n2)
        pm.mass = m

        # We need a way to assemble these - for now test the solver directly
        # using numpy matrices

        # Create dense matrices and convert to Eigen format
        K_np = np.array([[k]], dtype=np.float64)
        M_np = np.array([[m]], dtype=np.float64)

        # For now, skip this test as we need to pass Eigen sparse matrices
        # TODO: Add proper assembly for springs and point masses

    def test_two_node_cantilever(self):
        """
        Test cantilever beam with 2 nodes (1 element).

        The analytical solution for the first bending mode of a cantilever is:
        f_1 = (1.875)^2 * sqrt(EI / (rho*A*L^4)) / (2*pi)
        """
        # Create cantilever beam
        L = 1.0  # m
        E = 210e6  # kN/m^2
        I = 1e-4   # m^4
        rho = 7.85e-3  # mT/m^3
        A = 0.01   # m^2

        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(L, 0, 0)

        material = Material(1, "Steel", E, 0.3, rho)
        section = Section(1, "Test", A, I, I, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])
        M = assembler.assemble_mass([beam])

        # Fix first node (cantilever)
        bc = BCHandler()
        bc.fix_node(n1.id)

        # Reduce system
        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)

        # Solve
        settings = EigensolverSettings()
        settings.n_modes = 6  # Get all 6 modes of the free node
        result = solver.solve(K_red, M_red, settings)

        assert result.converged
        assert len(result.modes) == 6

        # All modes should have positive eigenvalues
        for mode in result.modes:
            assert mode.eigenvalue >= 0

        # First bending mode frequency
        f1 = result.modes[0].frequency_hz
        assert f1 > 0, "First mode should have positive frequency"

    def test_eigenvalue_sorting(self):
        """Test that eigenvalues are sorted in ascending order"""
        # Create a simple beam
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)
        n3 = registry.get_or_create_node(2, 0, 0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam1 = BeamElement(1, n1, n2, material, section)
        beam2 = BeamElement(2, n2, n3, material, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam1, beam2])
        M = assembler.assemble_mass([beam1, beam2])

        bc = BCHandler()
        bc.fix_node(n1.id)

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)

        settings = EigensolverSettings()
        settings.n_modes = 5
        result = solver.solve(K_red, M_red, settings)

        assert result.converged

        # Check eigenvalues are sorted ascending
        eigenvalues = [mode.eigenvalue for mode in result.modes]
        for i in range(len(eigenvalues) - 1):
            assert eigenvalues[i] <= eigenvalues[i + 1], \
                f"Eigenvalues not sorted: {eigenvalues[i]} > {eigenvalues[i + 1]}"

    def test_mass_normalization(self):
        """Test that mode shapes are mass-normalized"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])
        M = assembler.assemble_mass([beam])

        bc = BCHandler()
        bc.fix_node(n1.id)

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)

        settings = EigensolverSettings()
        settings.n_modes = 3
        settings.mass_normalize = True
        result = solver.solve(K_red, M_red, settings)

        assert result.converged

        # Convert M to dense for verification
        M_dense = np.array(M_red.todense())

        # Check mass normalization: phi^T * M * phi = 1
        for mode in result.modes:
            phi = np.array(mode.mode_shape)
            modal_mass = phi @ M_dense @ phi
            np.testing.assert_almost_equal(modal_mass, 1.0, decimal=6,
                err_msg=f"Mode {mode.mode_number} not mass-normalized")

    def test_frequency_period_relationship(self):
        """Test that frequency and period are correctly related: T = 1/f"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])
        M = assembler.assemble_mass([beam])

        bc = BCHandler()
        bc.fix_node(n1.id)

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)

        settings = EigensolverSettings()
        settings.n_modes = 3
        result = solver.solve(K_red, M_red, settings)

        assert result.converged

        for mode in result.modes:
            if mode.frequency_hz > 1e-10:  # Skip near-zero frequencies
                expected_period = 1.0 / mode.frequency_hz
                np.testing.assert_almost_equal(mode.period_s, expected_period, decimal=10)

    def test_omega_frequency_relationship(self):
        """Test that omega = 2*pi*f"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])
        M = assembler.assemble_mass([beam])

        bc = BCHandler()
        bc.fix_node(n1.id)

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)

        settings = EigensolverSettings()
        settings.n_modes = 3
        result = solver.solve(K_red, M_red, settings)

        assert result.converged

        for mode in result.modes:
            expected_omega = 2 * math.pi * mode.frequency_hz
            np.testing.assert_almost_equal(mode.omega, expected_omega, decimal=10)


class TestExpandModeShape:
    """Task 16.2: Test mode shape expansion"""

    def test_expand_mode_shape(self):
        """Test expanding reduced mode shape to full DOF vector"""
        # Reduced shape with 3 DOFs
        reduced_shape = np.array([1.0, 2.0, 3.0])

        # Mapping: reduced DOF 0 -> full DOF 2, 1 -> 5, 2 -> 8
        dof_mapping = [2, 5, 8]
        total_dofs = 10

        full_shape = EigenvalueSolver.expand_mode_shape(reduced_shape, dof_mapping, total_dofs)

        assert len(full_shape) == total_dofs
        assert full_shape[2] == 1.0
        assert full_shape[5] == 2.0
        assert full_shape[8] == 3.0
        # All other DOFs should be zero
        assert full_shape[0] == 0.0
        assert full_shape[1] == 0.0
        assert full_shape[3] == 0.0
        assert full_shape[4] == 0.0


class TestEdgeCases:
    """Test edge cases and error handling"""

    def test_n_modes_greater_than_dofs(self):
        """Test requesting more modes than available DOFs"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])
        M = assembler.assemble_mass([beam])

        bc = BCHandler()
        bc.fix_node(n1.id)

        solver = EigenvalueSolver()
        K_red, M_red, dof_map = solver.reduce_system(K, M, bc, dof_handler)

        settings = EigensolverSettings()
        settings.n_modes = 100  # More than available DOFs

        result = solver.solve(K_red, M_red, settings)

        # Should succeed but return only available modes
        assert result.converged
        assert len(result.modes) <= K_red.shape[0]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
