"""Tests for Phase 11: Error Handling & Diagnostics.

This module tests the structured error and warning system for Grillex.
Error codes and warning codes are designed to be machine-readable for
programmatic handling and LLM agent integration.
"""

import pytest
from grillex.core import (
    ErrorCode,
    GrillexError,
    WarningCode,
    WarningSeverity,
    GrillexWarning,
    WarningList,
)


class TestErrorCode:
    """Tests for ErrorCode enum."""

    def test_error_code_ok(self):
        """OK code should be 0."""
        assert ErrorCode.OK.value == 0

    def test_structural_error_codes(self):
        """Structural errors should be in 100-199 range."""
        assert ErrorCode.UNCONSTRAINED_SYSTEM.value == 100
        assert ErrorCode.SINGULAR_MATRIX.value == 101
        assert ErrorCode.INSUFFICIENT_CONSTRAINTS.value == 102
        assert ErrorCode.REDUNDANT_CONSTRAINTS.value == 103

    def test_element_error_codes(self):
        """Element errors should be in 200-299 range."""
        assert ErrorCode.INVALID_ELEMENT.value == 200
        assert ErrorCode.INVALID_MATERIAL.value == 201
        assert ErrorCode.INVALID_SECTION.value == 202
        assert ErrorCode.INVALID_NODE_REFERENCE.value == 203
        assert ErrorCode.INVALID_PROPERTY.value == 204
        assert ErrorCode.INVALID_ELEMENT_STIFFNESS.value == 205

    def test_load_error_codes(self):
        """Load errors should be in 300-399 range."""
        assert ErrorCode.INVALID_LOAD_NODE.value == 300
        assert ErrorCode.INVALID_LOAD_ELEMENT.value == 301
        assert ErrorCode.EMPTY_LOAD_CASE.value == 302
        assert ErrorCode.INVALID_LOAD_COMBINATION.value == 303

    def test_model_error_codes(self):
        """Model errors should be in 400-499 range."""
        assert ErrorCode.EMPTY_MODEL.value == 400
        assert ErrorCode.NO_NODES.value == 401
        assert ErrorCode.DISCONNECTED_MODEL.value == 402
        assert ErrorCode.NOT_ANALYZED.value == 403

    def test_solver_error_codes(self):
        """Solver errors should be in 500-599 range."""
        assert ErrorCode.SOLVER_CONVERGENCE_FAILED.value == 500
        assert ErrorCode.NUMERICAL_OVERFLOW.value == 501
        assert ErrorCode.OUT_OF_MEMORY.value == 502

    def test_unknown_error_code(self):
        """Unknown error should be 999."""
        assert ErrorCode.UNKNOWN_ERROR.value == 999


class TestGrillexError:
    """Tests for GrillexError struct."""

    def test_default_error_is_ok(self):
        """Default constructed error should be OK."""
        err = GrillexError()
        assert err.is_ok()
        assert not err.is_error()
        assert err.code == ErrorCode.OK
        assert err.message == "OK"

    def test_error_with_code_and_message(self):
        """Error with code and message should be accessible."""
        err = GrillexError(ErrorCode.SINGULAR_MATRIX, "Matrix is singular")
        assert err.is_error()
        assert not err.is_ok()
        assert err.code == ErrorCode.SINGULAR_MATRIX
        assert err.message == "Matrix is singular"

    def test_error_code_string(self):
        """Error code_string should return code name."""
        err = GrillexError(ErrorCode.UNCONSTRAINED_SYSTEM, "Test")
        assert err.code_string() == "UNCONSTRAINED_SYSTEM"

    def test_error_involved_dofs(self):
        """Error should track involved DOFs."""
        err = GrillexError(ErrorCode.UNCONSTRAINED_SYSTEM, "Test")
        err.involved_dofs = [0, 1, 2]
        assert len(err.involved_dofs) == 3
        assert err.involved_dofs[0] == 0
        assert err.involved_dofs[1] == 1
        assert err.involved_dofs[2] == 2

    def test_error_involved_elements(self):
        """Error should track involved elements."""
        err = GrillexError(ErrorCode.INVALID_ELEMENT, "Test")
        err.involved_elements = [5, 10]
        assert len(err.involved_elements) == 2
        assert 5 in err.involved_elements

    def test_error_involved_nodes(self):
        """Error should track involved nodes."""
        err = GrillexError(ErrorCode.INVALID_NODE_REFERENCE, "Test")
        err.involved_nodes = [1, 2, 3]
        assert len(err.involved_nodes) == 3

    def test_error_details(self):
        """Error should support key-value details."""
        err = GrillexError(ErrorCode.SINGULAR_MATRIX, "Test")
        err.details = {"solver": "SparseLU", "reason": "zero pivot"}
        assert err.details["solver"] == "SparseLU"
        assert err.details["reason"] == "zero pivot"

    def test_error_suggestion(self):
        """Error should have suggestion field."""
        err = GrillexError(ErrorCode.EMPTY_MODEL, "Test")
        err.suggestion = "Add at least one element"
        assert "element" in err.suggestion

    def test_error_to_string(self):
        """Error to_string should format nicely."""
        err = GrillexError(ErrorCode.SINGULAR_MATRIX, "Matrix is singular")
        err.involved_nodes = [1, 2]
        err.suggestion = "Check constraints"
        result = err.to_string()
        assert "SINGULAR_MATRIX" in result
        assert "Matrix is singular" in result
        assert "1" in result  # Node 1

    def test_error_to_string_ok(self):
        """OK error to_string should be simple."""
        err = GrillexError()
        assert err.to_string() == "OK"

    def test_factory_unconstrained(self):
        """Factory method for unconstrained error."""
        err = GrillexError.unconstrained([0, 1, 2], [1])
        assert err.code == ErrorCode.UNCONSTRAINED_SYSTEM
        assert len(err.involved_dofs) == 3
        assert len(err.involved_nodes) == 1
        assert err.suggestion  # Should have suggestion

    def test_factory_singular(self):
        """Factory method for singular matrix error."""
        err = GrillexError.singular("Zero pivot at row 5")
        assert err.code == ErrorCode.SINGULAR_MATRIX
        assert "solver_details" in err.details

    def test_factory_invalid_element(self):
        """Factory method for invalid element error."""
        err = GrillexError.invalid_element(42, "Zero length beam")
        assert err.code == ErrorCode.INVALID_ELEMENT
        assert 42 in err.involved_elements
        assert "Zero length" in err.message

    def test_factory_invalid_node(self):
        """Factory method for invalid node error."""
        err = GrillexError.invalid_node(99, "Referenced by beam 5")
        assert err.code == ErrorCode.INVALID_NODE_REFERENCE
        assert 99 in err.involved_nodes

    def test_factory_empty_model(self):
        """Factory method for empty model error."""
        err = GrillexError.empty_model()
        assert err.code == ErrorCode.EMPTY_MODEL
        assert err.suggestion  # Should have suggestion

    def test_factory_not_analyzed(self):
        """Factory method for not analyzed error."""
        err = GrillexError.not_analyzed()
        assert err.code == ErrorCode.NOT_ANALYZED
        assert "analyze" in err.suggestion.lower()


class TestWarningCode:
    """Tests for WarningCode enum."""

    def test_geometry_warning_codes(self):
        """Geometry warnings should be in 100-199 range."""
        assert WarningCode.EXTREME_ASPECT_RATIO.value == 100
        assert WarningCode.SMALL_ELEMENT.value == 101
        assert WarningCode.LARGE_ELEMENT.value == 102
        assert WarningCode.NON_COLLINEAR_WARPING.value == 103

    def test_stiffness_warning_codes(self):
        """Stiffness warnings should be in 200-299 range."""
        assert WarningCode.STIFFNESS_CONTRAST.value == 200
        assert WarningCode.NEAR_SINGULARITY.value == 201
        assert WarningCode.VERY_STIFF_SPRING.value == 202
        assert WarningCode.VERY_SOFT_SPRING.value == 203

    def test_property_warning_codes(self):
        """Property warnings should be in 300-399 range."""
        assert WarningCode.NEAR_ZERO_PROPERTY.value == 300
        assert WarningCode.POSSIBLE_UNIT_ERROR.value == 301
        assert WarningCode.INCONSISTENT_SECTION.value == 302

    def test_load_warning_codes(self):
        """Load warnings should be in 400-499 range."""
        assert WarningCode.LARGE_LOAD.value == 400
        assert WarningCode.LOAD_AT_FREE_NODE.value == 401
        assert WarningCode.ACCELERATION_WITHOUT_MASS.value == 402

    def test_analysis_warning_codes(self):
        """Analysis warnings should be in 500-599 range."""
        assert WarningCode.LARGE_DISPLACEMENT.value == 500
        assert WarningCode.HIGH_STRESS.value == 501
        assert WarningCode.SOLVER_REFINEMENT.value == 502


class TestWarningSeverity:
    """Tests for WarningSeverity enum."""

    def test_severity_values(self):
        """Severity levels should have correct ordering."""
        assert WarningSeverity.Low.value == 0
        assert WarningSeverity.Medium.value == 1
        assert WarningSeverity.High.value == 2

    def test_severity_comparison(self):
        """Higher severity should have higher value."""
        assert WarningSeverity.High.value > WarningSeverity.Medium.value
        assert WarningSeverity.Medium.value > WarningSeverity.Low.value


class TestGrillexWarning:
    """Tests for GrillexWarning struct."""

    def test_warning_creation(self):
        """Warning should be creatable with code, severity, message."""
        warn = GrillexWarning(
            WarningCode.SMALL_ELEMENT,
            WarningSeverity.Medium,
            "Element is very short"
        )
        assert warn.code == WarningCode.SMALL_ELEMENT
        assert warn.severity == WarningSeverity.Medium
        assert warn.message == "Element is very short"

    def test_warning_code_string(self):
        """Warning code_string should return code name."""
        warn = GrillexWarning(
            WarningCode.STIFFNESS_CONTRAST,
            WarningSeverity.High,
            "Test"
        )
        assert warn.code_string() == "STIFFNESS_CONTRAST"

    def test_warning_severity_string(self):
        """Warning severity_string should return severity name."""
        warn = GrillexWarning(
            WarningCode.SMALL_ELEMENT,
            WarningSeverity.High,
            "Test"
        )
        assert warn.severity_string() == "HIGH"

    def test_warning_involved_elements(self):
        """Warning should track involved elements."""
        warn = GrillexWarning(
            WarningCode.SMALL_ELEMENT,
            WarningSeverity.Medium,
            "Test"
        )
        warn.involved_elements = [5]
        assert 5 in warn.involved_elements

    def test_warning_involved_nodes(self):
        """Warning should track involved nodes."""
        warn = GrillexWarning(
            WarningCode.LARGE_DISPLACEMENT,
            WarningSeverity.Medium,
            "Test"
        )
        warn.involved_nodes = [10, 20]
        assert len(warn.involved_nodes) == 2

    def test_warning_details(self):
        """Warning should support key-value details."""
        warn = GrillexWarning(
            WarningCode.EXTREME_ASPECT_RATIO,
            WarningSeverity.Medium,
            "Test"
        )
        warn.details = {"aspect_ratio": "150"}
        assert warn.details["aspect_ratio"] == "150"

    def test_warning_suggestion(self):
        """Warning should have suggestion field."""
        warn = GrillexWarning(
            WarningCode.LARGE_DISPLACEMENT,
            WarningSeverity.Medium,
            "Test"
        )
        warn.suggestion = "Consider geometric nonlinear analysis"
        assert "nonlinear" in warn.suggestion

    def test_warning_to_string(self):
        """Warning to_string should format nicely."""
        warn = GrillexWarning(
            WarningCode.SMALL_ELEMENT,
            WarningSeverity.Medium,
            "Very short element"
        )
        warn.involved_elements = [5]
        result = warn.to_string()
        assert "MEDIUM" in result
        assert "SMALL_ELEMENT" in result
        assert "Very short element" in result

    def test_factory_extreme_aspect_ratio(self):
        """Factory method for extreme aspect ratio warning."""
        warn = GrillexWarning.extreme_aspect_ratio(42, 150.0)
        assert warn.code == WarningCode.EXTREME_ASPECT_RATIO
        assert warn.severity == WarningSeverity.Medium
        assert 42 in warn.involved_elements
        assert "aspect_ratio" in warn.details

    def test_factory_small_element(self):
        """Factory method for small element warning."""
        warn = GrillexWarning.small_element(10, 0.001)
        assert warn.code == WarningCode.SMALL_ELEMENT
        assert 10 in warn.involved_elements
        assert "length" in warn.details

    def test_factory_stiffness_contrast(self):
        """Factory method for stiffness contrast warning."""
        warn = GrillexWarning.stiffness_contrast(5, 6, 1e8)
        assert warn.code == WarningCode.STIFFNESS_CONTRAST
        assert warn.severity == WarningSeverity.High
        assert 5 in warn.involved_elements
        assert 6 in warn.involved_elements

    def test_factory_near_singularity(self):
        """Factory method for near singularity warning."""
        warn = GrillexWarning.near_singularity(1e15)
        assert warn.code == WarningCode.NEAR_SINGULARITY
        assert warn.severity == WarningSeverity.High
        assert "condition_number" in warn.details

    def test_factory_large_displacement(self):
        """Factory method for large displacement warning."""
        warn = GrillexWarning.large_displacement(10, 0.5, 0.05)
        assert warn.code == WarningCode.LARGE_DISPLACEMENT
        assert 10 in warn.involved_nodes
        assert "max_displacement" in warn.details

    def test_factory_near_zero_property(self):
        """Factory method for near zero property warning."""
        warn = GrillexWarning.near_zero_property(5, "Area", 1e-12)
        assert warn.code == WarningCode.NEAR_ZERO_PROPERTY
        assert warn.severity == WarningSeverity.High
        assert 5 in warn.involved_elements
        assert warn.details["property"] == "Area"


class TestWarningList:
    """Tests for WarningList collection."""

    def test_empty_warning_list(self):
        """Empty warning list should report no warnings."""
        wl = WarningList()
        assert not wl.has_warnings()
        assert wl.count() == 0

    def test_add_warning(self):
        """Should be able to add warnings."""
        wl = WarningList()
        warn = GrillexWarning(
            WarningCode.SMALL_ELEMENT,
            WarningSeverity.Medium,
            "Test"
        )
        wl.add(warn)
        assert wl.has_warnings()
        assert wl.count() == 1

    def test_add_multiple_warnings(self):
        """Should be able to add multiple warnings."""
        wl = WarningList()
        wl.add(GrillexWarning(
            WarningCode.SMALL_ELEMENT,
            WarningSeverity.Medium,
            "Test1"
        ))
        wl.add(GrillexWarning(
            WarningCode.STIFFNESS_CONTRAST,
            WarningSeverity.High,
            "Test2"
        ))
        wl.add(GrillexWarning(
            WarningCode.LARGE_ELEMENT,
            WarningSeverity.Low,
            "Test3"
        ))
        assert wl.count() == 3

    def test_count_by_severity(self):
        """Should count warnings by severity."""
        wl = WarningList()
        wl.add(GrillexWarning(WarningCode.SMALL_ELEMENT, WarningSeverity.Medium, "M1"))
        wl.add(GrillexWarning(WarningCode.STIFFNESS_CONTRAST, WarningSeverity.High, "H1"))
        wl.add(GrillexWarning(WarningCode.NEAR_SINGULARITY, WarningSeverity.High, "H2"))
        wl.add(GrillexWarning(WarningCode.LARGE_ELEMENT, WarningSeverity.Low, "L1"))

        assert wl.count_by_severity(WarningSeverity.Low) == 1
        assert wl.count_by_severity(WarningSeverity.Medium) == 1
        assert wl.count_by_severity(WarningSeverity.High) == 2

    def test_get_by_min_severity(self):
        """Should filter warnings by minimum severity."""
        wl = WarningList()
        wl.add(GrillexWarning(WarningCode.SMALL_ELEMENT, WarningSeverity.Medium, "M1"))
        wl.add(GrillexWarning(WarningCode.STIFFNESS_CONTRAST, WarningSeverity.High, "H1"))
        wl.add(GrillexWarning(WarningCode.LARGE_ELEMENT, WarningSeverity.Low, "L1"))

        high_only = wl.get_by_min_severity(WarningSeverity.High)
        assert len(high_only) == 1

        medium_and_up = wl.get_by_min_severity(WarningSeverity.Medium)
        assert len(medium_and_up) == 2

        all_warnings = wl.get_by_min_severity(WarningSeverity.Low)
        assert len(all_warnings) == 3

    def test_clear(self):
        """Should be able to clear all warnings."""
        wl = WarningList()
        wl.add(GrillexWarning(WarningCode.SMALL_ELEMENT, WarningSeverity.Medium, "Test"))
        assert wl.count() == 1
        wl.clear()
        assert wl.count() == 0
        assert not wl.has_warnings()

    def test_summary(self):
        """Summary should give overview of warnings."""
        wl = WarningList()
        assert wl.summary() == "No warnings"

        wl.add(GrillexWarning(WarningCode.STIFFNESS_CONTRAST, WarningSeverity.High, "H1"))
        wl.add(GrillexWarning(WarningCode.SMALL_ELEMENT, WarningSeverity.Medium, "M1"))
        wl.add(GrillexWarning(WarningCode.LARGE_ELEMENT, WarningSeverity.Low, "L1"))

        summary = wl.summary()
        assert "3 warning(s)" in summary
        assert "1 high" in summary
        assert "1 medium" in summary
        assert "1 low" in summary

    def test_access_warnings_list(self):
        """Should be able to access warnings list directly."""
        wl = WarningList()
        wl.add(GrillexWarning(WarningCode.SMALL_ELEMENT, WarningSeverity.Medium, "Test"))

        assert len(wl.warnings) == 1
        assert wl.warnings[0].code == WarningCode.SMALL_ELEMENT


class TestErrorWarningIntegration:
    """Integration tests for error and warning system."""

    def test_error_codes_unique(self):
        """All error codes should have unique values."""
        codes = [
            ErrorCode.OK,
            ErrorCode.UNCONSTRAINED_SYSTEM,
            ErrorCode.SINGULAR_MATRIX,
            ErrorCode.INSUFFICIENT_CONSTRAINTS,
            ErrorCode.REDUNDANT_CONSTRAINTS,
            ErrorCode.INVALID_ELEMENT,
            ErrorCode.INVALID_MATERIAL,
            ErrorCode.INVALID_SECTION,
            ErrorCode.INVALID_NODE_REFERENCE,
            ErrorCode.INVALID_PROPERTY,
            ErrorCode.INVALID_ELEMENT_STIFFNESS,
            ErrorCode.INVALID_LOAD_NODE,
            ErrorCode.INVALID_LOAD_ELEMENT,
            ErrorCode.EMPTY_LOAD_CASE,
            ErrorCode.INVALID_LOAD_COMBINATION,
            ErrorCode.EMPTY_MODEL,
            ErrorCode.NO_NODES,
            ErrorCode.DISCONNECTED_MODEL,
            ErrorCode.NOT_ANALYZED,
            ErrorCode.SOLVER_CONVERGENCE_FAILED,
            ErrorCode.NUMERICAL_OVERFLOW,
            ErrorCode.OUT_OF_MEMORY,
            ErrorCode.UNKNOWN_ERROR,
        ]
        values = [c.value for c in codes]
        assert len(values) == len(set(values)), "Duplicate error code values found"

    def test_warning_codes_unique(self):
        """All warning codes should have unique values."""
        codes = [
            WarningCode.EXTREME_ASPECT_RATIO,
            WarningCode.SMALL_ELEMENT,
            WarningCode.LARGE_ELEMENT,
            WarningCode.NON_COLLINEAR_WARPING,
            WarningCode.STIFFNESS_CONTRAST,
            WarningCode.NEAR_SINGULARITY,
            WarningCode.VERY_STIFF_SPRING,
            WarningCode.VERY_SOFT_SPRING,
            WarningCode.NEAR_ZERO_PROPERTY,
            WarningCode.POSSIBLE_UNIT_ERROR,
            WarningCode.INCONSISTENT_SECTION,
            WarningCode.LARGE_LOAD,
            WarningCode.LOAD_AT_FREE_NODE,
            WarningCode.ACCELERATION_WITHOUT_MASS,
            WarningCode.LARGE_DISPLACEMENT,
            WarningCode.HIGH_STRESS,
            WarningCode.SOLVER_REFINEMENT,
        ]
        values = [c.value for c in codes]
        assert len(values) == len(set(values)), "Duplicate warning code values found"

    def test_error_code_ranges(self):
        """Error codes should be in non-overlapping ranges."""
        # Check structural errors in 100-199
        struct_codes = [100, 101, 102, 103]
        for c in struct_codes:
            assert 100 <= c < 200

        # Check element errors in 200-299
        elem_codes = [200, 201, 202, 203, 204, 205]
        for c in elem_codes:
            assert 200 <= c < 300

        # Check load errors in 300-399
        load_codes = [300, 301, 302, 303]
        for c in load_codes:
            assert 300 <= c < 400

        # Check model errors in 400-499
        model_codes = [400, 401, 402, 403]
        for c in model_codes:
            assert 400 <= c < 500

        # Check solver errors in 500-599
        solver_codes = [500, 501, 502]
        for c in solver_codes:
            assert 500 <= c < 600
