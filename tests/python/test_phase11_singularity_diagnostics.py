"""Tests for Phase 11 (Task 11.3): Singularity Diagnostics.

This module tests the singularity diagnostics system that uses eigenvalue
analysis to detect rigid body modes and identify unconstrained DOFs.

Acceptance Criteria (Task 11.3):
- Free-floating model detected as singular
- Specific unconstrained DOFs identified
- Helpful diagnostic message generated
"""

import pytest
import numpy as np

from grillex.core import (
    Node,
    NodeRegistry,
    Material,
    Section,
    DOFHandler,
    BeamConfig,
    BeamFormulation,
    create_beam_element,
    Assembler,
    BCHandler,
    DOFIndex,
    # Singularity Diagnostics types
    RigidBodyModeType,
    RigidBodyModeInfo,
    DOFParticipation,
    SingularityDiagnostics,
    SingularityAnalyzerSettings,
    SingularityAnalyzer,
)


class TestSingularityAnalyzerSettings:
    """Tests for SingularityAnalyzerSettings."""

    def test_default_values(self):
        """Test default settings values."""
        settings = SingularityAnalyzerSettings()
        assert settings.eigenvalue_threshold == pytest.approx(1e-8)
        assert settings.n_modes_to_check == 10
        assert settings.participation_threshold == pytest.approx(0.01)
        assert settings.max_dofs_to_report == 20
        assert settings.use_mass_matrix is True

    def test_custom_settings(self):
        """Test setting custom values."""
        settings = SingularityAnalyzerSettings()
        settings.eigenvalue_threshold = 1e-6
        settings.n_modes_to_check = 20
        settings.participation_threshold = 0.05
        settings.max_dofs_to_report = 50
        settings.use_mass_matrix = False

        assert settings.eigenvalue_threshold == pytest.approx(1e-6)
        assert settings.n_modes_to_check == 20
        assert settings.participation_threshold == pytest.approx(0.05)
        assert settings.max_dofs_to_report == 50
        assert settings.use_mass_matrix is False


class TestRigidBodyModeType:
    """Tests for RigidBodyModeType enum."""

    def test_translation_modes(self):
        """Test translation mode types."""
        assert RigidBodyModeType.TranslationX is not None
        assert RigidBodyModeType.TranslationY is not None
        assert RigidBodyModeType.TranslationZ is not None

    def test_rotation_modes(self):
        """Test rotation mode types."""
        assert RigidBodyModeType.RotationX is not None
        assert RigidBodyModeType.RotationY is not None
        assert RigidBodyModeType.RotationZ is not None

    def test_special_modes(self):
        """Test special mode types."""
        assert RigidBodyModeType.Warping is not None
        assert RigidBodyModeType.Mixed is not None


class TestSingularityDiagnostics:
    """Tests for SingularityDiagnostics result struct."""

    def test_default_values(self):
        """Test default diagnostics values."""
        diag = SingularityDiagnostics()
        assert diag.is_singular is False
        assert diag.n_rigid_body_modes == 0
        assert len(diag.rigid_body_modes) == 0
        assert len(diag.unconstrained_dofs) == 0
        assert len(diag.suggested_fixes) == 0
        assert len(diag.nodes_needing_constraints) == 0
        assert len(diag.dofs_to_constrain) == 0

    def test_bool_conversion(self):
        """Test bool conversion (True if singular)."""
        diag = SingularityDiagnostics()
        assert not bool(diag)  # Not singular

        diag.is_singular = True
        assert bool(diag)  # Singular


class TestSingularityAnalyzerFreeFloatingBeam:
    """Tests for singularity detection on a free-floating (unconstrained) beam.

    ACCEPTANCE CRITERIA:
    - Free-floating model detected as singular
    - Specific unconstrained DOFs identified
    - Helpful diagnostic message generated
    """

    @pytest.fixture
    def free_floating_beam(self):
        """Create a free-floating beam with no boundary conditions."""
        # Create nodes
        registry = NodeRegistry(1e-6)
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(5.0, 0.0, 0.0)

        # Create material and section
        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)  # Steel
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)  # IPE300

        # Create beam element
        config = BeamConfig()
        config.formulation = BeamFormulation.EulerBernoulli
        beam = create_beam_element(1, node_i, node_j, material, section, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble stiffness matrix (no boundary conditions)
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])

        return K, dof_handler, registry

    def test_free_floating_detected_as_singular(self, free_floating_beam):
        """AC: Free-floating model detected as singular."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        assert result.is_singular is True
        # A free-floating 3D beam has 6 rigid body modes
        assert result.n_rigid_body_modes >= 6

    def test_specific_unconstrained_dofs_identified(self, free_floating_beam):
        """AC: Specific unconstrained DOFs identified."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Should identify unconstrained DOFs
        assert len(result.unconstrained_dofs) > 0

        # Check that DOF information is provided
        for dof in result.unconstrained_dofs:
            assert dof.global_dof >= 0
            # Participation should be between 0 and 1
            assert 0 <= dof.participation <= 1

    def test_nodes_needing_constraints_identified(self, free_floating_beam):
        """Test that nodes needing constraints are identified."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Should identify at least one node needing constraints
        assert len(result.nodes_needing_constraints) > 0

    def test_helpful_diagnostic_message_generated(self, free_floating_beam):
        """AC: Helpful diagnostic message generated."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Summary message should indicate singular system
        assert "SINGULAR" in result.summary_message.upper()
        assert str(result.n_rigid_body_modes) in result.summary_message

        # Detailed message should explain the problem
        assert len(result.detailed_message) > 50
        assert "rigid body" in result.detailed_message.lower()

        # Should have suggested fixes
        assert len(result.suggested_fixes) > 0

        # to_string should combine all information
        full_message = result.to_string()
        assert "rigid body" in full_message.lower()

    def test_suggested_fixes_are_actionable(self, free_floating_beam):
        """Test that suggested fixes are actionable."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Each fix should mention a specific action and DOF
        for fix in result.suggested_fixes:
            # Should mention a direction or DOF type
            assert any(dof in fix.upper() for dof in
                      ["UX", "UY", "UZ", "RX", "RY", "RZ", "X", "Y", "Z",
                       "SUPPORT", "FIX", "CONSTRAIN", "RESTRAINT"])

    def test_rigid_body_mode_info_available(self, free_floating_beam):
        """Test that rigid body mode information is detailed."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Should have detailed mode info
        assert len(result.rigid_body_modes) >= 6

        for mode in result.rigid_body_modes:
            # Mode should have near-zero eigenvalue
            assert abs(mode.eigenvalue) < 1e-6
            # Mode should have a type
            assert mode.mode_type is not None
            # Mode should have description
            assert len(mode.description) > 0

    def test_quick_singularity_check(self, free_floating_beam):
        """Test quick is_singular() method."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        assert analyzer.is_singular(K) is True

    def test_count_rigid_body_modes(self, free_floating_beam):
        """Test count_rigid_body_modes() method."""
        K, dof_handler, _ = free_floating_beam

        analyzer = SingularityAnalyzer()
        count = analyzer.count_rigid_body_modes(K)

        # Free-floating 3D beam has 6 rigid body modes
        assert count >= 6


class TestSingularityAnalyzerWellConstrained:
    """Tests for a well-constrained beam (should not be singular)."""

    @pytest.fixture
    def cantilever_beam(self):
        """Create a properly constrained cantilever beam."""
        # Create nodes
        registry = NodeRegistry(1e-6)
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(5.0, 0.0, 0.0)

        # Create material and section
        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)  # Steel
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)  # IPE300

        # Create beam element
        config = BeamConfig()
        config.formulation = BeamFormulation.EulerBernoulli
        beam = create_beam_element(1, node_i, node_j, material, section, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble stiffness matrix
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])

        # Apply fixed boundary conditions at node i (cantilever root)
        bc = BCHandler()
        bc.add_fixed_dof(node_i.id, DOFIndex.UX, 0.0)
        bc.add_fixed_dof(node_i.id, DOFIndex.UY, 0.0)
        bc.add_fixed_dof(node_i.id, DOFIndex.UZ, 0.0)
        bc.add_fixed_dof(node_i.id, DOFIndex.RX, 0.0)
        bc.add_fixed_dof(node_i.id, DOFIndex.RY, 0.0)
        bc.add_fixed_dof(node_i.id, DOFIndex.RZ, 0.0)

        # Create force vector
        n_dofs = dof_handler.total_dofs()
        F = np.zeros(n_dofs)

        # Apply boundary conditions using penalty method
        K_bc, F_bc = bc.apply_to_system(K, F, dof_handler)

        return K_bc, dof_handler

    def test_well_constrained_not_singular(self, cantilever_beam):
        """Well-constrained cantilever should not be detected as singular."""
        K, dof_handler = cantilever_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        assert result.is_singular is False
        assert result.n_rigid_body_modes == 0
        assert len(result.rigid_body_modes) == 0

    def test_well_constrained_no_unconstrained_dofs(self, cantilever_beam):
        """Well-constrained beam should have no unconstrained DOFs."""
        K, dof_handler = cantilever_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        assert len(result.unconstrained_dofs) == 0
        assert len(result.nodes_needing_constraints) == 0
        assert len(result.dofs_to_constrain) == 0

    def test_well_constrained_message(self, cantilever_beam):
        """Well-constrained system should have appropriate message."""
        K, dof_handler = cantilever_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Message should indicate system is OK
        full_message = result.to_string()
        assert "well-constrained" in full_message.lower() or \
               "no rigid body" in full_message.lower()


class TestSingularityAnalyzerPartialConstraints:
    """Tests for analyzing raw stiffness matrices before BC application.

    Note: The penalty BC method makes matrices artificially non-singular,
    so these tests work with raw stiffness matrices to verify that
    singularity detection works correctly for diagnostic purposes.
    """

    @pytest.fixture
    def raw_beam_matrix(self):
        """Create a raw stiffness matrix without any BCs (for diagnostic testing)."""
        # Create nodes
        registry = NodeRegistry(1e-6)
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(5.0, 0.0, 0.0)

        # Create material and section
        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)  # Steel
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)  # IPE300

        # Create beam element
        config = BeamConfig()
        config.formulation = BeamFormulation.EulerBernoulli
        beam = create_beam_element(1, node_i, node_j, material, section, config)

        # Number DOFs
        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        # Assemble stiffness matrix (no BCs applied)
        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])

        return K, dof_handler

    def test_raw_matrix_is_singular(self, raw_beam_matrix):
        """A raw stiffness matrix (no BCs) should be detected as singular."""
        K, dof_handler = raw_beam_matrix

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        # Should be singular (all 6 rigid body modes are free)
        assert result.is_singular is True
        # A 3D beam has 6 rigid body modes
        assert result.n_rigid_body_modes >= 6

    def test_mode_count_is_consistent(self, raw_beam_matrix):
        """Quick is_singular and count_rigid_body_modes should be consistent."""
        K, dof_handler = raw_beam_matrix

        analyzer = SingularityAnalyzer()

        is_sing = analyzer.is_singular(K)
        count = analyzer.count_rigid_body_modes(K)

        assert is_sing == (count > 0)
        # Should have 6 rigid body modes for a free beam
        assert count >= 6


class TestSingularityAnalyzerJSON:
    """Tests for JSON output format."""

    @pytest.fixture
    def free_floating_beam(self):
        """Create a free-floating beam."""
        registry = NodeRegistry(1e-6)
        node_i = registry.get_or_create_node(0.0, 0.0, 0.0)
        node_j = registry.get_or_create_node(5.0, 0.0, 0.0)

        material = Material(1, "Steel", 210e6, 0.3, 7.85e-3)
        section = Section(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

        config = BeamConfig()
        config.formulation = BeamFormulation.EulerBernoulli
        beam = create_beam_element(1, node_i, node_j, material, section, config)

        dof_handler = DOFHandler()
        dof_handler.number_dofs(registry)

        assembler = Assembler(dof_handler)
        K = assembler.assemble_stiffness([beam])

        return K, dof_handler

    def test_json_output_parseable(self, free_floating_beam):
        """Test that to_json() produces valid JSON-like output."""
        K, dof_handler = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        json_str = result.to_json()

        # Should contain key JSON markers
        assert "{" in json_str
        assert "}" in json_str
        assert "is_singular" in json_str
        assert "n_rigid_body_modes" in json_str
        assert "nodes_needing_constraints" in json_str
        assert "rigid_body_modes" in json_str

    def test_json_contains_singular_flag(self, free_floating_beam):
        """Test JSON contains is_singular field."""
        K, dof_handler = free_floating_beam

        analyzer = SingularityAnalyzer()
        result = analyzer.analyze(K, dof_handler)

        json_str = result.to_json()
        assert '"is_singular": true' in json_str


class TestSingularityAnalyzerRepr:
    """Tests for string representation."""

    def test_repr_non_singular(self):
        """Test __repr__ for non-singular result."""
        diag = SingularityDiagnostics()
        repr_str = repr(diag)
        assert "well-constrained" in repr_str.lower()

    def test_repr_singular(self):
        """Test __repr__ for singular result."""
        diag = SingularityDiagnostics()
        diag.is_singular = True
        diag.n_rigid_body_modes = 3
        repr_str = repr(diag)
        assert "3" in repr_str
        assert "rigid body" in repr_str.lower()
