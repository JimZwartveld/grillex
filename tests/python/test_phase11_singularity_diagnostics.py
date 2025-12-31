"""Tests for Phase 11 (Task 11.3): Singularity Diagnostics.

This module tests the singularity diagnostics data structures exposed to Python.

NOTE: Tests that use SingularityAnalyzer.analyze, is_singular, and count_rigid_body_modes
have been moved to C++ tests (tests/cpp/test_singularity_diagnostics.cpp) because
pybind11 cannot handle Eigen::SparseMatrix as input parameters.

The following tests remain in Python:
- SingularityAnalyzerSettings struct tests
- RigidBodyModeType enum tests
- SingularityDiagnostics struct tests
"""

import pytest

from grillex.core import (
    # Singularity Diagnostics types
    RigidBodyModeType,
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


class TestSingularityAnalyzer:
    """Tests for SingularityAnalyzer class creation."""

    def test_analyzer_creation(self):
        """Test that SingularityAnalyzer can be instantiated."""
        analyzer = SingularityAnalyzer()
        assert analyzer is not None
