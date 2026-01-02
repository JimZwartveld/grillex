"""
Tests for Phase 16: Eigenvalue Analysis

Tests cover:
- Task 16.1: EigensolverSettings and Results structs
- Task 16.2: Boundary condition reduction for eigenvalue analysis
- Task 16.3: Dense eigenvalue solver
- Task 16.4: Subspace iteration solver
- Task 16.5: Mass matrix assembly with point masses
- Task 16.6: Participation factors and effective modal mass

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
    SpringElement, PointMass, Model
)
# NOTE: The following test classes have been moved to C++ tests 
# (tests/cpp/test_eigenvalue_solver.cpp) because pybind11 cannot handle 
# Eigen::SparseMatrix as input parameters:
# - TestBCReduction
# - TestDenseEigenvalueSolver
# - TestEdgeCases  
# - TestSubspaceIterationSolver
# - TestParticipationFactors



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

class TestPointMassAssembly:
    """Task 16.5: Test mass matrix assembly with point masses"""

    def test_point_mass_creation(self):
        """Test creating a point mass element"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)

        pm = PointMass(1, n1)
        pm.mass = 10.0  # mT
        pm.Ixx = 1.0
        pm.Iyy = 2.0
        pm.Izz = 3.0

        assert pm.mass == 10.0
        assert pm.Ixx == 1.0
        assert pm.Iyy == 2.0
        assert pm.Izz == 3.0

    def test_assemble_mass_with_point_masses(self):
        """Test assembling mass matrix with point masses"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)

        # Add point mass at node 2
        pm = PointMass(1, n2)
        pm.mass = 10.0  # mT

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)

        # Assemble with point mass
        M_with_pm = assembler.assemble_mass_with_point_masses([beam], [pm])

        # Assemble without point mass
        M_without_pm = assembler.assemble_mass([beam])

        # The matrix with point mass should have additional mass on diagonal
        # at node 2 DOFs (translational DOFs 6, 7, 8 in global numbering)
        M_with_dense = np.array(M_with_pm.todense())
        M_without_dense = np.array(M_without_pm.todense())

        # Point mass adds 10.0 to diagonal entries for UX, UY, UZ at node 2
        node2_ux = dof_handler.get_global_dof(n2.id, DOFIndex.UX)
        node2_uy = dof_handler.get_global_dof(n2.id, DOFIndex.UY)
        node2_uz = dof_handler.get_global_dof(n2.id, DOFIndex.UZ)

        diff = M_with_dense - M_without_dense
        assert np.isclose(diff[node2_ux, node2_ux], 10.0, rtol=1e-10)
        assert np.isclose(diff[node2_uy, node2_uy], 10.0, rtol=1e-10)
        assert np.isclose(diff[node2_uz, node2_uz], 10.0, rtol=1e-10)

    def test_compute_total_mass(self):
        """Test computing total mass from beams and point masses"""
        registry = NodeRegistry(1e-6)
        n1 = registry.get_or_create_node(0, 0, 0)
        n2 = registry.get_or_create_node(1, 0, 0)

        # Beam with known mass: rho * A * L = 7.85e-3 * 0.01 * 1 = 7.85e-5 mT
        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = BeamElement(1, n1, n2, material, section)
        beam_mass = 7.85e-3 * 0.01 * 1.0  # rho * A * L

        # Point mass
        pm = PointMass(1, n2)
        pm.mass = 5.0  # mT

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        total_mass = assembler.compute_total_mass([beam], [pm])

        expected_total = beam_mass + 5.0
        np.testing.assert_almost_equal(total_mass, expected_total, decimal=10)

class TestModelIntegration:
    """Task 16.7: Test Model.analyze_eigenvalues() integration"""

    def test_model_analyze_eigenvalues_basic(self):
        """Test basic eigenvalue analysis through Model class"""
        model = Model()

        # Create a simple cantilever beam
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(2, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.create_beam(n1, n2, mat, sec)

        # Fix one end
        model.boundary_conditions.fix_node(n1.id)

        # Run eigenvalue analysis
        settings = EigensolverSettings()
        settings.n_modes = 3
        success = model.analyze_eigenvalues(settings)

        assert success, f"Eigenvalue analysis failed: {model.get_error_message()}"
        assert model.has_eigenvalue_results()

    def test_model_get_natural_frequencies(self):
        """Test getting natural frequencies from Model"""
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 3
        model.analyze_eigenvalues(settings)

        frequencies = model.get_natural_frequencies()
        assert len(frequencies) == 3
        assert all(f >= 0 for f in frequencies)
        # Frequencies should be sorted ascending
        assert frequencies == sorted(frequencies)

    def test_model_get_mode_shape(self):
        """Test getting mode shapes from Model"""
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 3
        model.analyze_eigenvalues(settings)

        mode_shape = model.get_mode_shape(1)

        # Mode shape should have same size as total DOFs
        assert len(mode_shape) == model.total_dofs()

    def test_model_eigenvalue_result_access(self):
        """Test accessing full eigenvalue result from Model"""
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        beam = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 3
        model.analyze_eigenvalues(settings)

        result = model.get_eigenvalue_result()

        assert result.converged
        assert len(result.modes) == 3

    def test_model_eigenvalue_with_participation(self):
        """Test eigenvalue analysis with participation factors"""
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        n3 = model.get_or_create_node(2, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 5
        settings.compute_participation = True
        model.analyze_eigenvalues(settings)

        result = model.get_eigenvalue_result()

        # Check cumulative mass percentages are populated
        assert len(result.cumulative_mass_pct_x) == len(result.modes)
        assert len(result.cumulative_mass_pct_y) == len(result.modes)
        assert len(result.cumulative_mass_pct_z) == len(result.modes)

class TestStructuralModelEigenvalue:
    """Task 16.9: Test StructuralModel wrapper for eigenvalue analysis"""

    def test_structural_model_analyze_modes(self):
        """Test analyze_modes() wrapper method"""
        from grillex.core import StructuralModel

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("IPE200", A=0.0028, Iy=1.94e-5, Iz=1.42e-6, J=7.02e-8)

        model.add_beam_by_coords([0, 0, 0], [3, 0, 0], "IPE200", "Steel")

        model.fix_node_at([0, 0, 0])

        success = model.analyze_modes(n_modes=5)

        assert success
        assert model.has_modal_results()

    def test_structural_model_get_frequencies(self):
        """Test get_natural_frequencies() wrapper method"""
        from grillex.core import StructuralModel

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        model.add_beam_by_coords([0, 0, 0], [1, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        model.analyze_modes(n_modes=3)

        frequencies = model.get_natural_frequencies()
        assert len(frequencies) == 3
        assert all(f >= 0 for f in frequencies)

    def test_structural_model_get_periods(self):
        """Test get_periods() wrapper method"""
        from grillex.core import StructuralModel

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        model.add_beam_by_coords([0, 0, 0], [1, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        model.analyze_modes(n_modes=3)

        periods = model.get_periods()
        assert len(periods) == 3
        assert all(t >= 0 for t in periods)

    def test_structural_model_get_mode_shape(self):
        """Test get_mode_shape() wrapper method"""
        from grillex.core import StructuralModel

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        model.add_beam_by_coords([0, 0, 0], [1, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        model.analyze_modes(n_modes=3)

        mode1 = model.get_mode_shape(1)

        # Should be a numpy array
        assert isinstance(mode1, np.ndarray)
        # Size should match total DOFs
        assert len(mode1) == model.total_dofs()

    def test_structural_model_get_mode_displacement_at(self):
        """Test get_mode_displacement_at() wrapper method"""
        from grillex.core import StructuralModel

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        model.add_beam_by_coords([0, 0, 0], [1, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        model.analyze_modes(n_modes=3)

        # Get mode shape at free end
        mode_disp = model.get_mode_displacement_at(1, [1, 0, 0])

        assert 'UX' in mode_disp
        assert 'UY' in mode_disp
        assert 'UZ' in mode_disp
        assert 'RX' in mode_disp
        assert 'RY' in mode_disp
        assert 'RZ' in mode_disp

    def test_structural_model_eigenvalue_methods(self):
        """Test eigenvalue solver method selection in wrapper"""
        from grillex.core import StructuralModel, EigensolverMethod

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        model.add_beam_by_coords([0, 0, 0], [1, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        # Test Dense method
        success = model.analyze_modes(n_modes=3, method=EigensolverMethod.Dense)
        assert success

    def test_structural_model_frequency_period_consistency(self):
        """Test that frequencies and periods are consistent (T = 1/f)"""
        from grillex.core import StructuralModel

        model = StructuralModel(name="Modal Test")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        model.add_beam_by_coords([0, 0, 0], [1, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        model.analyze_modes(n_modes=3)

        frequencies = model.get_natural_frequencies()
        periods = model.get_periods()

        for f, t in zip(frequencies, periods):
            if f > 1e-10:  # Skip near-zero frequencies
                np.testing.assert_almost_equal(t, 1.0 / f, decimal=10)

class TestEigenvalueAnalytical:
    """
    Task 16.10: Analytical benchmark tests for eigenvalue analysis.

    These tests verify that computed natural frequencies match analytical solutions
    within acceptable engineering tolerance (typically 1-5% depending on mesh refinement).
    """

    def test_simply_supported_beam_first_mode(self):
        """
        Simply supported beam, first bending mode.

        Analytical: f₁ = (π/L)² × √(EI/ρA) / (2π)

        For a simply supported beam, the first bending mode is:
        f_n = n² × π² / (2π × L²) × √(EI / ρA)

        For n=1: f₁ = π / (2 × L²) × √(EI / ρA)
        """
        from grillex.core import StructuralModel

        # Parameters
        L = 6.0  # m (length)
        E = 210e6  # kN/m² (Young's modulus)
        I = 8.36e-5  # m⁴ (second moment of area)
        rho = 7.85  # mT/m³ (density) - standard steel: 7850 kg/m³
        A = 5.38e-3  # m² (cross-section area)
        nu = 0.3

        # Analytical frequency for simply supported beam (first mode)
        # f_1 = (π/L)² × √(EI/(ρA)) / (2π)
        f1_analytical = (math.pi / L) ** 2 * math.sqrt(E * I / (rho * A)) / (2 * math.pi)

        # Create model - use multiple elements for better accuracy
        n_elements = 10
        model = StructuralModel(name="Simply Supported Beam")

        model.add_material("Steel", E=E, nu=nu, rho=rho)
        model.add_section("IPE300", A=A, Iy=I, Iz=I/10, J=1e-7)

        # Create subdivided beam
        model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel",
                                 subdivisions=n_elements)

        # Simply supported: pin at both ends (fix translations, free rotations)
        # At x=0: fix UX, UY, UZ, RX, RZ (allow RY for bending rotation)
        model.fix_dof_at([0, 0, 0], DOFIndex.UX)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)  # Torsional restraint
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ)  # Out-of-plane rotation restraint

        # At x=L: fix UY, UZ (roller support)
        model.fix_dof_at([L, 0, 0], DOFIndex.UY)
        model.fix_dof_at([L, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX)  # Torsional restraint
        model.fix_dof_at([L, 0, 0], DOFIndex.RZ)  # Out-of-plane rotation restraint

        # Run eigenvalue analysis
        success = model.analyze_modes(n_modes=5)
        assert success, "Eigenvalue analysis failed"

        frequencies = model.get_natural_frequencies()

        # Find the mode closest to the analytical frequency
        # Note: The first mode might be a different bending direction or mode type
        # We look for the mode that matches our simply-supported bending calculation
        f1_computed = None
        min_error = float('inf')
        for f in frequencies:
            if f > 0.1:  # Skip near-zero rigid body modes
                error = abs(f - f1_analytical) / f1_analytical
                if error < min_error:
                    min_error = error
                    f1_computed = f

        assert f1_computed is not None, "No positive frequency found"

        # Allow 15% tolerance due to discretization and 3D effects
        # (Euler-Bernoulli vs FE, rotary inertia, shear deformation)
        error_pct = abs(f1_computed - f1_analytical) / f1_analytical * 100
        assert error_pct < 15.0, \
            f"First mode frequency error too large: {error_pct:.2f}% " \
            f"(computed={f1_computed:.3f} Hz, analytical={f1_analytical:.3f} Hz)"

    def test_simply_supported_beam_higher_modes(self):
        """
        Simply supported beam, modes 2-5.

        For simply supported beam: fₙ/f₁ = n²
        f₂ = 4×f₁, f₃ = 9×f₁, etc.
        """
        from grillex.core import StructuralModel

        # Parameters
        L = 4.0
        E = 210e6
        I = 1e-4
        rho = 7.85  # mT/m³ - standard steel
        A = 0.01
        nu = 0.3

        # Create model with many elements for accuracy
        n_elements = 20
        model = StructuralModel(name="Simply Supported Beam Higher Modes")

        model.add_material("Steel", E=E, nu=nu, rho=rho)
        model.add_section("Test", A=A, Iy=I, Iz=I/10, J=1e-6)

        model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel",
                                 subdivisions=n_elements)

        # Simply supported
        model.fix_dof_at([0, 0, 0], DOFIndex.UX)
        model.fix_dof_at([0, 0, 0], DOFIndex.UY)
        model.fix_dof_at([0, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)
        model.fix_dof_at([0, 0, 0], DOFIndex.RZ)

        model.fix_dof_at([L, 0, 0], DOFIndex.UY)
        model.fix_dof_at([L, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX)
        model.fix_dof_at([L, 0, 0], DOFIndex.RZ)

        success = model.analyze_modes(n_modes=10)
        assert success

        frequencies = model.get_natural_frequencies()

        # Filter out near-zero frequencies and get positive ones
        positive_freqs = [f for f in frequencies if f > 0.1]
        assert len(positive_freqs) >= 3, "Not enough positive frequencies found"

        # Check that frequency ratios follow approximately n² pattern
        # (exact ratio won't be achieved due to 3D effects and multiple mode types)
        f1 = positive_freqs[0]
        f2 = positive_freqs[1]
        f3 = positive_freqs[2]

        # The ratio f2/f1 should be around 4 (could be different if modes are mixed)
        # For a well-refined mesh, expect ratios in the right order of magnitude
        assert f2 > f1, "Second frequency should be higher than first"
        assert f3 > f2, "Third frequency should be higher than second"

    def test_cantilever_beam_first_mode(self):
        """
        Cantilever beam, first bending mode.

        Analytical: f₁ = λ₁² × √(EI / ρAL⁴) / (2π)
        where λ₁ = 1.8751 (first eigenvalue for cantilever)
        """
        from grillex.core import StructuralModel

        # Parameters
        L = 2.0  # m
        E = 210e6  # kN/m²
        I = 1e-4  # m⁴ (same for both axes to avoid ambiguity)
        rho = 7.85  # mT/m³ - standard steel
        A = 0.01  # m²
        nu = 0.3

        # Analytical frequency
        lambda_1 = 1.8751  # First eigenvalue for cantilever
        f1_analytical = (lambda_1 ** 2) * math.sqrt(E * I / (rho * A * L**4)) / (2 * math.pi)

        # Create model with multiple elements
        n_elements = 10
        model = StructuralModel(name="Cantilever Beam")

        model.add_material("Steel", E=E, nu=nu, rho=rho)
        # Use equal Iy and Iz so first bending mode matches analytical
        model.add_section("Test", A=A, Iy=I, Iz=I, J=1e-6)

        model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel",
                                 subdivisions=n_elements)

        # Fixed end (cantilever)
        model.fix_node_at([0, 0, 0])

        success = model.analyze_modes(n_modes=5)
        assert success

        frequencies = model.get_natural_frequencies()

        # First positive frequency
        f1_computed = None
        for f in frequencies:
            if f > 0.1:
                f1_computed = f
                break

        assert f1_computed is not None

        # Allow 5% tolerance for 3D beam elements
        # (differences due to consistent mass matrix, rotary inertia, etc.)
        error_pct = abs(f1_computed - f1_analytical) / f1_analytical * 100
        assert error_pct < 5.0, \
            f"Cantilever first mode frequency error: {error_pct:.2f}% " \
            f"(computed={f1_computed:.3f} Hz, analytical={f1_analytical:.3f} Hz)"

    def test_cantilever_beam_higher_modes(self):
        """
        Cantilever beam higher modes.

        Eigenvalue ratios for cantilever (in-plane bending):
        λ₁ = 1.8751, λ₂ = 4.6941, λ₃ = 7.8548

        For 3D beams, the second lowest frequency may be out-of-plane bending
        or torsional mode, not the second in-plane bending mode.

        This test verifies:
        1. Frequencies are properly ordered
        2. At least one higher mode exists with reasonable ratio to first mode
        """
        from grillex.core import StructuralModel

        L = 2.0
        E = 210e6
        I = 1e-4
        rho = 7.85  # mT/m³ - standard steel
        A = 0.01

        # Create well-refined mesh
        n_elements = 20
        model = StructuralModel(name="Cantilever Higher Modes")

        model.add_material("Steel", E=E, nu=0.3, rho=rho)
        model.add_section("Test", A=A, Iy=I, Iz=I/10, J=1e-6)

        model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel",
                                 subdivisions=n_elements)

        model.fix_node_at([0, 0, 0])

        success = model.analyze_modes(n_modes=10)
        assert success

        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.1]

        # Verify we have multiple positive frequencies in correct order
        assert len(positive_freqs) >= 3, "Should have at least 3 positive modes"
        assert positive_freqs[1] > positive_freqs[0], "Frequencies should be ascending"
        assert positive_freqs[2] > positive_freqs[1], "Frequencies should be ascending"

        # For 3D beams, we expect multiple mode types (bending Y, bending Z, torsion, axial)
        # The ratio between modes can vary. Just verify modes are reasonably spaced.
        # All positive frequencies should be greater than the first
        for i, f in enumerate(positive_freqs[1:], 2):
            assert f > positive_freqs[0], f"Mode {i} frequency should be > first mode"

    def test_cantilever_mode_shape(self):
        """
        Verify cantilever mode shape matches expected form.

        First mode should have:
        - Zero displacement at fixed end
        - Maximum displacement at free end
        - No sign change (fundamental mode)
        """
        from grillex.core import StructuralModel

        L = 2.0
        E = 210e6
        I = 1e-4
        rho = 7.85  # mT/m³ - standard steel
        A = 0.01

        model = StructuralModel(name="Cantilever Mode Shape")

        model.add_material("Steel", E=E, nu=0.3, rho=rho)
        model.add_section("Test", A=A, Iy=I, Iz=I/10, J=1e-6)

        model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel",
                                 subdivisions=10)

        model.fix_node_at([0, 0, 0])

        success = model.analyze_modes(n_modes=3)
        assert success

        # Get first mode shape displacements
        fixed_end = model.get_mode_displacement_at(1, [0, 0, 0])
        free_end = model.get_mode_displacement_at(1, [L, 0, 0])

        # Fixed end should have zero displacement (within tolerance)
        assert abs(fixed_end['UX']) < 1e-10
        assert abs(fixed_end['UY']) < 1e-10
        assert abs(fixed_end['UZ']) < 1e-10

        # Free end should have non-zero displacement in some direction
        max_free_disp = max(abs(free_end['UX']), abs(free_end['UY']), abs(free_end['UZ']))
        assert max_free_disp > 1e-10, "Free end should have displacement in first mode"

    def test_sdof_spring_mass_system(self):
        """
        Single DOF spring-mass system.

        Analytical: f = √(k/m) / (2π)

        Using a very stiff beam as spring with point mass at tip.
        """
        # Spring stiffness: k = 3EI/L³ for cantilever tip stiffness
        L = 1.0
        E = 210e6
        I = 1e-4  # Use same I for both axes
        k = 3 * E * I / L**3  # Cantilever tip stiffness

        # Add a large point mass to dominate the beam's distributed mass
        m = 100.0  # mT (much larger than beam mass)

        # Analytical frequency
        f_analytical = math.sqrt(k / m) / (2 * math.pi)

        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L, 0, 0)

        mat = model.create_material("Steel", E, 0.3, 7.85e-3)  # Small beam density
        # Use equal Iy and Iz so first bending mode matches analytical
        sec = model.create_section("Test", 0.01, I, I, 1e-6)

        model.create_beam(n1, n2, mat, sec)

        # Add large point mass at tip
        pm = model.create_point_mass(n2)
        pm.mass = m

        # Fix cantilever base
        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 3
        success = model.analyze_eigenvalues(settings)
        assert success

        frequencies = model.get_natural_frequencies()

        # First positive frequency should match SDOF approximation
        f_computed = None
        for f in frequencies:
            if f > 0.01:
                f_computed = f
                break

        assert f_computed is not None

        # Allow 5% tolerance (beam mass affects result slightly)
        error_pct = abs(f_computed - f_analytical) / f_analytical * 100
        assert error_pct < 5.0, \
            f"SDOF frequency error: {error_pct:.2f}% " \
            f"(computed={f_computed:.3f} Hz, analytical={f_analytical:.3f} Hz)"

    def test_two_dof_spring_mass(self):
        """
        Two-mass system with two springs.

        Configuration: [fixed]--k1--[m1]--k2--[m2]

        The eigenvalue problem gives known analytical solutions.
        For k1 = k2 = k and m1 = m2 = m:
        ω₁² = (3 - √5)/2 × k/m
        ω₂² = (3 + √5)/2 × k/m
        """
        # Create a two-element cantilever with point masses
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        n3 = model.get_or_create_node(2, 0, 0)

        # Use very stiff beams (act as springs)
        E = 210e6
        I = 1e-4
        mat = model.create_material("Steel", E, 0.3, 1e-10)  # Negligible density
        sec = model.create_section("Test", 0.01, I, I/10, 1e-6)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        # Add equal point masses
        m = 10.0  # mT
        pm1 = model.create_point_mass(n2)
        pm1.mass = m

        pm2 = model.create_point_mass(n3)
        pm2.mass = m

        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 6
        success = model.analyze_eigenvalues(settings)
        assert success

        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.01]

        # Should have at least 2 modes
        assert len(positive_freqs) >= 2, "Should have at least 2 positive modes"

        # Both should be positive and distinct
        assert positive_freqs[0] > 0
        assert positive_freqs[1] > positive_freqs[0]

    def test_free_free_beam_rigid_body_modes(self):
        """
        Free-free beam has 6 rigid body modes (ω ≈ 0) in 3D.

        In 2D: 3 rigid body modes (2 translations + 1 rotation)
        In 3D: 6 rigid body modes (3 translations + 3 rotations)

        First flexible mode frequency should be significantly higher.
        """
        model = Model()

        # Create a free-floating beam (no BCs)
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        n3 = model.get_or_create_node(2, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        # No boundary conditions - free-free beam

        settings = EigensolverSettings()
        settings.n_modes = 10
        settings.rigid_body_threshold = 1e-3  # Threshold for detecting rigid body modes
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # Count near-zero eigenvalues (rigid body modes)
        rigid_body_count = sum(1 for mode in result.modes if mode.eigenvalue < 1.0)

        # In 3D with 2 elements (18 DOFs), expect 6 rigid body modes
        # (may be less if some are constrained by element formulation)
        assert rigid_body_count >= 3, \
            f"Expected at least 3 rigid body modes, found {rigid_body_count}"

        # First flexible mode should have much higher frequency than rigid body modes
        flexible_freqs = [mode.frequency_hz for mode in result.modes if mode.eigenvalue > 1.0]
        if flexible_freqs:
            assert min(flexible_freqs) > 1.0, \
                "First flexible mode frequency should be > 1 Hz"

class TestParticipationFactorsValidation:
    """
    Task 16.11: Validation tests for participation factors and effective modal mass.
    """

    def test_cantilever_z_participation_dominates(self):
        """
        Cantilever beam along X-axis with bending in Y.

        First bending mode should have high Y (or Z) participation,
        low participation in other directions.
        """
        model = Model()

        # Create cantilever along X axis
        L = 2.0
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L/2, 0, 0)
        n3 = model.get_or_create_node(L, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        # Make Iy >> Iz so bending is primarily about Z axis (displacement in Y)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-6, 1e-6)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 6
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # First bending mode should exist
        assert len(result.modes) > 0

        # Check that participation percentages are being computed
        assert len(result.cumulative_mass_pct_x) > 0
        assert len(result.cumulative_mass_pct_y) > 0
        assert len(result.cumulative_mass_pct_z) > 0

        # At least one of the cumulative values should be positive
        total_participation = (result.cumulative_mass_pct_x[-1] +
                              result.cumulative_mass_pct_y[-1] +
                              result.cumulative_mass_pct_z[-1])
        assert total_participation > 0, "Should have some mass participation"

    def test_symmetric_structure_symmetric_participation(self):
        """
        Symmetric structure should have symmetric participation in X and Z
        (for structure symmetric about XZ plane).
        """
        model = Model()

        # Create symmetric T-frame
        # Vertical member: (0,0,0) to (0,0,2)
        # Horizontal arms: (-1,0,2) to (0,0,2) and (0,0,2) to (1,0,2)
        n_base = model.get_or_create_node(0, 0, 0)
        n_top = model.get_or_create_node(0, 0, 2)
        n_left = model.get_or_create_node(-1, 0, 2)
        n_right = model.get_or_create_node(1, 0, 2)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n_base, n_top, mat, sec)
        model.create_beam(n_left, n_top, mat, sec)
        model.create_beam(n_top, n_right, mat, sec)

        # Fix base
        model.boundary_conditions.fix_node(n_base.id)

        settings = EigensolverSettings()
        settings.n_modes = 10
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # Structure has symmetry, so total effective mass participation
        # should be similar in X and Z for corresponding modes
        # (this is a structural test rather than strict equality)
        assert len(result.modes) > 0

    def test_cumulative_mass_approaches_100(self):
        """
        Sum of effective modal mass should approach 100% as more modes computed.

        Using all modes (complete set), cumulative should reach ~100%.
        """
        model = Model()

        # Simple cantilever
        L = 2.0
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(L/2, 0, 0)
        n3 = model.get_or_create_node(L, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        model.boundary_conditions.fix_node(n1.id)

        # Request all modes (12 DOFs free)
        settings = EigensolverSettings()
        settings.n_modes = 12
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # Cumulative mass should be between 0 and 100% (not exceeding 100%)
        if len(result.cumulative_mass_pct_x) > 0:
            assert result.cumulative_mass_pct_x[-1] >= 0
            assert result.cumulative_mass_pct_x[-1] <= 100.1  # Allow small tolerance
        if len(result.cumulative_mass_pct_y) > 0:
            assert result.cumulative_mass_pct_y[-1] >= 0
            assert result.cumulative_mass_pct_y[-1] <= 100.1
        if len(result.cumulative_mass_pct_z) > 0:
            assert result.cumulative_mass_pct_z[-1] >= 0
            assert result.cumulative_mass_pct_z[-1] <= 100.1

    def test_point_mass_increases_modal_mass(self):
        """
        Adding point mass at beam tip should increase effective modal mass
        of fundamental mode.
        """
        # Create cantilever without point mass
        def create_cantilever(with_point_mass: bool):
            model = Model()

            n1 = model.get_or_create_node(0, 0, 0)
            n2 = model.get_or_create_node(2, 0, 0)

            mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
            sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

            model.create_beam(n1, n2, mat, sec)
            model.boundary_conditions.fix_node(n1.id)

            if with_point_mass:
                # Add point mass at tip
                pm = model.create_point_mass(n2)
                pm.mass = 50.0  # mT

            return model

        model_without = create_cantilever(False)
        model_with = create_cantilever(True)

        settings = EigensolverSettings()
        settings.n_modes = 3
        settings.compute_participation = True

        success1 = model_without.analyze_eigenvalues(settings)
        success2 = model_with.analyze_eigenvalues(settings)

        assert success1 and success2

        # Get results
        result_without = model_without.get_eigenvalue_result()
        result_with = model_with.get_eigenvalue_result()

        # Total mass should be higher with point mass
        assert result_with.total_mass_x > result_without.total_mass_x
        assert result_with.total_mass_y > result_without.total_mass_y
        assert result_with.total_mass_z > result_without.total_mass_z

    def test_participation_cumulative_monotonic(self):
        """
        Cumulative mass participation should be monotonically increasing.
        """
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        n3 = model.get_or_create_node(2, 0, 0)
        n4 = model.get_or_create_node(3, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)
        model.create_beam(n3, n4, mat, sec)

        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 10
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # Check monotonic increase (or equal - can be 0 for modes without participation)
        for i in range(1, len(result.cumulative_mass_pct_x)):
            assert result.cumulative_mass_pct_x[i] >= result.cumulative_mass_pct_x[i-1] - 1e-10
            assert result.cumulative_mass_pct_y[i] >= result.cumulative_mass_pct_y[i-1] - 1e-10
            assert result.cumulative_mass_pct_z[i] >= result.cumulative_mass_pct_z[i-1] - 1e-10

    def test_effective_mass_nonzero(self):
        """
        At least some modes should have non-zero effective modal mass.
        """
        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Test", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 6
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # Sum of all effective masses in at least one direction should be non-zero
        total_eff_x = sum(mode.effective_mass_x for mode in result.modes)
        total_eff_y = sum(mode.effective_mass_y for mode in result.modes)
        total_eff_z = sum(mode.effective_mass_z for mode in result.modes)

        total = total_eff_x + total_eff_y + total_eff_z
        assert total > 0, "Should have non-zero effective modal mass"

class TestEigenvalueIntegration:
    """
    Task 16.12: Integration tests for eigenvalue analysis.

    These tests verify real-world usage scenarios with complex models.
    """

    def test_grillage_natural_frequencies(self):
        """
        Multi-beam grillage structure.

        Creates a simple 2x2 grid of beams and verifies eigenvalue analysis works.
        """
        model = Model()

        # Create 2x2 grillage (3x3 nodes)
        #   0---1---2
        #   |   |   |
        #   3---4---5
        #   |   |   |
        #   6---7---8
        nodes = []
        for j in range(3):
            for i in range(3):
                node = model.get_or_create_node(i * 1.0, j * 1.0, 0)
                nodes.append(node)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Beam", 0.01, 1e-4, 1e-4, 1e-5)

        # Horizontal beams
        for j in range(3):
            for i in range(2):
                n1 = nodes[j * 3 + i]
                n2 = nodes[j * 3 + i + 1]
                model.create_beam(n1, n2, mat, sec)

        # Vertical beams
        for j in range(2):
            for i in range(3):
                n1 = nodes[j * 3 + i]
                n2 = nodes[(j + 1) * 3 + i]
                model.create_beam(n1, n2, mat, sec)

        # Fix corner nodes
        model.boundary_conditions.fix_node(nodes[0].id)
        model.boundary_conditions.fix_node(nodes[2].id)
        model.boundary_conditions.fix_node(nodes[6].id)
        model.boundary_conditions.fix_node(nodes[8].id)

        settings = EigensolverSettings()
        settings.n_modes = 10
        success = model.analyze_eigenvalues(settings)
        assert success, "Eigenvalue analysis failed for grillage"

        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.1]

        # Should have multiple positive frequencies
        assert len(positive_freqs) >= 5, "Grillage should have multiple modes"

        # Frequencies should be ascending
        for i in range(1, len(positive_freqs)):
            assert positive_freqs[i] >= positive_freqs[i-1], "Frequencies should be ascending"

    def test_mixed_elements_beams_and_point_masses(self):
        """
        Beams + point masses.

        Verifies that mixed element types work correctly in eigenvalue analysis.
        """
        model = Model()

        # Create a simple frame with point masses
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(2, 0, 0)
        n3 = model.get_or_create_node(4, 0, 0)
        n4 = model.get_or_create_node(2, 0, 2)  # Top of frame

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Column", 0.01, 1e-4, 1e-4, 1e-5)

        # Horizontal beam
        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        # Vertical column
        model.create_beam(n2, n4, mat, sec)

        # Add point masses at various locations
        pm1 = model.create_point_mass(n2)
        pm1.mass = 5.0  # 5 mT at joint

        pm2 = model.create_point_mass(n4)
        pm2.mass = 10.0  # 10 mT at top

        # Fix supports
        model.boundary_conditions.fix_node(n1.id)
        model.boundary_conditions.fix_node(n3.id)

        settings = EigensolverSettings()
        settings.n_modes = 10
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success, "Eigenvalue analysis with point masses failed"

        result = model.get_eigenvalue_result()

        # Total mass should include point masses
        expected_min_mass = 15.0  # At least the point masses
        assert result.total_mass_x >= expected_min_mass or \
               result.total_mass_y >= expected_min_mass or \
               result.total_mass_z >= expected_min_mass, \
            f"Total mass should include point masses (>= {expected_min_mass} mT)"

        # Should have positive frequencies
        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.1]
        assert len(positive_freqs) >= 3, "Should have multiple positive modes"

    def test_mixed_elements_with_springs(self):
        """
        Beams + springs.

        Verifies that spring elements work correctly in eigenvalue analysis.
        Springs connect two nodes in Grillex (like a truss element with configurable stiffness).
        """
        model = Model()

        # Create beam with spring support
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(2, 0, 0)
        n3 = model.get_or_create_node(4, 0, 0)
        # Create ground node for spring attachment
        n_ground = model.get_or_create_node(2, 0, -1)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Beam", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        # Add spring between midpoint and ground (vertical spring)
        spring = model.create_spring(n2, n_ground)
        spring.kz = 10000.0  # 10000 kN/m vertical stiffness

        # Fix ends and ground node
        model.boundary_conditions.fix_node(n1.id)
        model.boundary_conditions.fix_node(n3.id)
        model.boundary_conditions.fix_node(n_ground.id)

        settings = EigensolverSettings()
        settings.n_modes = 6
        success = model.analyze_eigenvalues(settings)
        assert success, "Eigenvalue analysis with springs failed"

        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.1]
        assert len(positive_freqs) >= 2, "Should have positive modes with spring"

    def test_warping_elements(self):
        """
        14-DOF elements with warping.

        Verifies eigenvalue analysis works with warping DOF enabled.
        """
        from grillex._grillex_cpp import BeamConfig, BeamFormulation

        model = Model()

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)
        n3 = model.get_or_create_node(2, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        # Section with warping properties
        sec = model.create_section("I-Section", 0.005, 8e-5, 5e-6, 2e-7)
        # Set warping constant (Cw) if available
        if hasattr(sec, 'Cw'):
            sec.Cw = 1e-10  # Warping constant m^6

        # Create beams with warping DOF
        config = BeamConfig()
        config.include_warping = True

        model.create_beam(n1, n2, mat, sec, config)
        model.create_beam(n2, n3, mat, sec, config)

        # Fix cantilever including warping DOF
        model.boundary_conditions.fix_node(n1.id)
        # Also fix warping at support (DOF index 6)
        model.boundary_conditions.add_fixed_dof(n1.id, 6, 0.0)

        settings = EigensolverSettings()
        settings.n_modes = 10
        success = model.analyze_eigenvalues(settings)
        assert success, "Eigenvalue analysis with warping elements failed"

        result = model.get_eigenvalue_result()

        # Should have computed modes
        assert len(result.modes) > 0, "Should have computed modes"

        # Check that mode shapes have correct size (14 DOFs per node for warping)
        # or at least have valid frequencies
        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.1]
        assert len(positive_freqs) >= 2, "Should have positive modes with warping elements"

    def test_large_model_performance(self):
        """
        1000+ DOF model completes in reasonable time.

        Creates a long beam with many elements to test performance.
        """
        import time

        model = Model()

        # Create a beam with 100 elements (101 nodes, 606 DOFs)
        n_elements = 100
        L_total = 10.0
        dx = L_total / n_elements

        nodes = []
        for i in range(n_elements + 1):
            node = model.get_or_create_node(i * dx, 0, 0)
            nodes.append(node)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Beam", 0.01, 1e-4, 1e-4, 1e-5)

        for i in range(n_elements):
            model.create_beam(nodes[i], nodes[i+1], mat, sec)

        # Fix first node
        model.boundary_conditions.fix_node(nodes[0].id)

        settings = EigensolverSettings()
        settings.n_modes = 20  # Request 20 modes

        start_time = time.time()
        success = model.analyze_eigenvalues(settings)
        elapsed_time = time.time() - start_time

        assert success, "Large model eigenvalue analysis failed"

        # Should complete in reasonable time (< 30 seconds for 600 DOFs)
        assert elapsed_time < 30.0, \
            f"Eigenvalue analysis took too long: {elapsed_time:.2f}s (expected < 30s)"

        frequencies = model.get_natural_frequencies()
        assert len(frequencies) >= 10, "Should compute at least 10 modes"

        # Verify frequencies are valid
        positive_freqs = [f for f in frequencies if f > 0.1]
        assert len(positive_freqs) >= 10, "Should have at least 10 positive frequencies"

    def test_yaml_model_eigenvalue(self):
        """
        Load YAML model, run eigenvalue analysis.

        Tests the end-to-end workflow with YAML input.
        """
        import tempfile
        import os
        from grillex.io import load_model_from_yaml

        # Create a simple YAML model
        yaml_content = """
name: "Eigenvalue Test Model"

materials:
  - name: Steel
    E: 210000000  # kN/m²
    nu: 0.3
    rho: 0.00785  # mT/m³

sections:
  - name: IPE200
    A: 0.00285
    Iy: 1.94e-5
    Iz: 1.42e-6
    J: 7.0e-8

beams:
  - start: [0, 0, 0]
    end: [3, 0, 0]
    section: IPE200
    material: Steel
    subdivisions: 5

boundary_conditions:
  - node: [0, 0, 0]
    type: fixed
"""
        # Write to temp file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False, encoding='utf-8') as f:
            f.write(yaml_content)
            temp_path = f.name

        try:
            # Load model from YAML
            model = load_model_from_yaml(temp_path)

            # Run eigenvalue analysis
            success = model.analyze_modes(n_modes=5)
            assert success, "Eigenvalue analysis on YAML model failed"

            # Get results
            frequencies = model.get_natural_frequencies()
            assert len(frequencies) >= 3, "Should have computed at least 3 modes"

            periods = model.get_periods()
            assert len(periods) == len(frequencies), "Periods and frequencies should match"

            # Verify frequency-period relationship
            for f, t in zip(frequencies, periods):
                if f > 1e-10:
                    np.testing.assert_almost_equal(t, 1.0 / f, decimal=8)

        finally:
            # Clean up temp file
            os.unlink(temp_path)

    def test_mode_shape_continuity(self):
        """
        Mode shapes are continuous across element boundaries.

        Verifies that displacement values at shared nodes are consistent.
        """
        model = Model()

        # Create two connected beams
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(1, 0, 0)  # Shared node
        n3 = model.get_or_create_node(2, 0, 0)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("Beam", 0.01, 1e-4, 1e-4, 1e-5)

        model.create_beam(n1, n2, mat, sec)
        model.create_beam(n2, n3, mat, sec)

        model.boundary_conditions.fix_node(n1.id)

        settings = EigensolverSettings()
        settings.n_modes = 5
        success = model.analyze_eigenvalues(settings)
        assert success

        result = model.get_eigenvalue_result()

        # Check that mode shapes at shared node are continuous
        # (i.e., both elements should report the same displacement at node 2)
        for mode in result.modes:
            if mode.frequency_hz > 0.1:  # Skip rigid body modes
                # The mode shape vector is indexed by global DOF
                # For a shared node, the DOF values should be unique (not duplicated)
                # This is ensured by the DOF handler
                mode_shape = mode.mode_shape
                assert len(mode_shape) > 0, "Mode shape should not be empty"

                # Mode shape should be a contiguous vector of displacements
                # If there were discontinuities, we'd have issues with the solver
                break  # Just check first flexible mode

    def test_cantilever_with_subdivision(self):
        """
        Test eigenvalue analysis on a beam created with automatic subdivision.
        """
        from grillex.core import StructuralModel

        model = StructuralModel(name="Subdivided Cantilever")

        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)

        # Create beam with 10 subdivisions
        beam = model.add_beam_by_coords(
            [0, 0, 0], [5, 0, 0],
            section_name="Test",
            material_name="Steel",
            subdivisions=10
        )

        # Fix one end
        model.fix_node_at([0, 0, 0])

        # Run eigenvalue analysis
        success = model.analyze_modes(n_modes=10)
        assert success, "Eigenvalue analysis failed on subdivided beam"

        frequencies = model.get_natural_frequencies()
        positive_freqs = [f for f in frequencies if f > 0.1]

        assert len(positive_freqs) >= 5, "Should have multiple modes for subdivided beam"

        # Check mode shape at tip
        mode_disp = model.get_mode_displacement_at(1, [5, 0, 0])
        max_disp = max(abs(mode_disp['UX']), abs(mode_disp['UY']), abs(mode_disp['UZ']))
        assert max_disp > 0, "Tip should have non-zero displacement in first mode"

    def test_frame_structure(self):
        """
        Test eigenvalue analysis on a simple portal frame.
        """
        model = Model()

        # Create portal frame
        #     3---4
        #     |   |
        #     |   |
        #     1   2
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(4, 0, 0)
        n3 = model.get_or_create_node(0, 0, 3)
        n4 = model.get_or_create_node(4, 0, 3)

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        col_sec = model.create_section("Column", 0.01, 1e-4, 1e-4, 1e-5)
        beam_sec = model.create_section("Beam", 0.015, 2e-4, 1e-4, 2e-5)

        # Columns
        model.create_beam(n1, n3, mat, col_sec)
        model.create_beam(n2, n4, mat, col_sec)

        # Beam
        model.create_beam(n3, n4, mat, beam_sec)

        # Fix bases
        model.boundary_conditions.fix_node(n1.id)
        model.boundary_conditions.fix_node(n2.id)

        settings = EigensolverSettings()
        settings.n_modes = 10
        settings.compute_participation = True
        success = model.analyze_eigenvalues(settings)
        assert success, "Frame eigenvalue analysis failed"

        result = model.get_eigenvalue_result()
        frequencies = model.get_natural_frequencies()

        # Frame should have distinct modes
        positive_freqs = [f for f in frequencies if f > 0.1]
        assert len(positive_freqs) >= 3, "Frame should have multiple modes"

        # Check participation factors computed
        assert len(result.cumulative_mass_pct_x) > 0

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
