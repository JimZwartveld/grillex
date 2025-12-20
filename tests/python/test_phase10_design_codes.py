"""Tests for Phase 10: Design Code Plugin Architecture.

These tests verify the design code base classes and plugin structure.
"""

import pytest
from dataclasses import dataclass
from typing import List, Any

from grillex.design_codes import CheckResult, DesignCheck, DesignCode


# =============================================================================
# Test Fixtures - Mock section and material
# =============================================================================


@dataclass
class MockSection:
    """Mock section for testing."""

    A: float = 0.01  # m²
    Iy: float = 1e-4  # m⁴
    Iz: float = 1e-5  # m⁴
    Wy: float = 0.001  # m³ (elastic section modulus)
    Wz: float = 0.0005  # m³


@dataclass
class MockMaterial:
    """Mock material for testing."""

    fy: float = 355000.0  # kN/m² (355 MPa)
    fu: float = 510000.0  # kN/m² (510 MPa)
    E: float = 210e6  # kN/m² (210 GPa)


# =============================================================================
# Test Fixtures - Concrete implementations for testing
# =============================================================================


class SimpleAxialCheck(DesignCheck):
    """Simple axial capacity check for testing."""

    @property
    def name(self) -> str:
        return "Axial Capacity"

    def compute_utilization(
        self, actions: dict, section: Any, material: Any, **kwargs
    ) -> float:
        N = abs(actions.get("N", 0.0))
        gamma_M0 = kwargs.get("gamma_M0", 1.0)
        N_Rd = section.A * material.fy / gamma_M0
        return N / N_Rd if N_Rd > 0 else 0.0


class SimpleBendingCheck(DesignCheck):
    """Simple bending capacity check for testing."""

    @property
    def name(self) -> str:
        return "Bending Capacity (My)"

    def compute_utilization(
        self, actions: dict, section: Any, material: Any, **kwargs
    ) -> float:
        My = abs(actions.get("My", 0.0))
        gamma_M0 = kwargs.get("gamma_M0", 1.0)
        M_Rd = section.Wy * material.fy / gamma_M0
        return My / M_Rd if M_Rd > 0 else 0.0


class SimpleDesignCode(DesignCode):
    """Simple design code for testing."""

    @property
    def name(self) -> str:
        return "Simple Test Code v1.0"

    @property
    def version(self) -> str:
        return "1.0-test"

    def get_checks(self) -> List[DesignCheck]:
        return [SimpleAxialCheck(), SimpleBendingCheck()]

    def check_beam(
        self, beam: Any, result_case: Any, combination: Any
    ) -> List[CheckResult]:
        """Simplified check_beam for testing."""
        results = []
        section = beam.section
        material = beam.material

        # Check at start, middle, and end
        locations = [0.0, 0.5, 1.0]

        for loc in locations:
            # Get actions at location (mocked)
            actions = result_case.get_actions_at(beam.id, loc)

            for check in self.get_checks():
                result = check.check(
                    actions=actions,
                    section=section,
                    material=material,
                    element_id=beam.id,
                    location=loc,
                    load_combination=combination.name,
                )
                results.append(result)

        return results


@dataclass
class MockBeam:
    """Mock beam for testing."""

    id: int
    section: MockSection
    material: MockMaterial


class MockResultCase:
    """Mock result case for testing."""

    def __init__(self, actions_by_location: dict = None):
        self.actions_by_location = actions_by_location or {}

    def get_actions_at(self, element_id: int, location: float) -> dict:
        key = (element_id, location)
        return self.actions_by_location.get(
            key, {"N": 0.0, "Vy": 0.0, "Vz": 0.0, "Mx": 0.0, "My": 0.0, "Mz": 0.0}
        )


@dataclass
class MockCombination:
    """Mock load combination for testing."""

    name: str


# =============================================================================
# Tests for CheckResult
# =============================================================================


class TestCheckResult:
    """Tests for CheckResult dataclass."""

    def test_check_result_creation(self):
        """Test basic CheckResult creation."""
        result = CheckResult(
            element_id=1,
            location=0.5,
            check_name="Axial Capacity",
            utilization=0.75,
            load_combination="ULS1",
        )

        assert result.element_id == 1
        assert result.location == 0.5
        assert result.check_name == "Axial Capacity"
        assert result.utilization == 0.75
        assert result.load_combination == "ULS1"
        assert result.governing is False
        assert result.details == {}

    def test_check_result_status_pass(self):
        """Test status is PASS when utilization <= 1.0."""
        result = CheckResult(
            element_id=1,
            location=0.0,
            check_name="Test",
            utilization=0.99,
            load_combination="LC1",
        )
        assert result.status == "PASS"

        result_exact = CheckResult(
            element_id=1,
            location=0.0,
            check_name="Test",
            utilization=1.0,
            load_combination="LC1",
        )
        assert result_exact.status == "PASS"

    def test_check_result_status_fail(self):
        """Test status is FAIL when utilization > 1.0."""
        result = CheckResult(
            element_id=1,
            location=0.0,
            check_name="Test",
            utilization=1.01,
            load_combination="LC1",
        )
        assert result.status == "FAIL"

    def test_check_result_with_details(self):
        """Test CheckResult with additional details."""
        result = CheckResult(
            element_id=1,
            location=0.5,
            check_name="Buckling",
            utilization=0.85,
            load_combination="ULS1",
            details={"lambda_bar": 0.75, "chi": 0.82},
        )

        assert result.details["lambda_bar"] == 0.75
        assert result.details["chi"] == 0.82

    def test_check_result_governing_flag(self):
        """Test governing flag can be set."""
        result = CheckResult(
            element_id=1,
            location=0.0,
            check_name="Test",
            utilization=0.95,
            load_combination="LC1",
            governing=True,
        )
        assert result.governing is True

    def test_check_result_repr(self):
        """Test string representation."""
        result = CheckResult(
            element_id=1,
            location=0.5,
            check_name="Axial",
            utilization=0.85,
            load_combination="ULS1",
        )
        repr_str = repr(result)
        assert "Axial" in repr_str
        assert "0.85" in repr_str
        assert "PASS" in repr_str

        result.governing = True
        repr_str = repr(result)
        assert "GOVERNING" in repr_str


# =============================================================================
# Tests for DesignCheck
# =============================================================================


class TestDesignCheck:
    """Tests for DesignCheck abstract base class."""

    def test_design_check_is_abstract(self):
        """Test that DesignCheck cannot be instantiated directly."""
        with pytest.raises(TypeError):
            DesignCheck()

    def test_concrete_check_implementation(self):
        """Test concrete implementation of DesignCheck."""
        check = SimpleAxialCheck()
        assert check.name == "Axial Capacity"

    def test_compute_utilization_axial(self):
        """Test axial utilization computation."""
        check = SimpleAxialCheck()
        section = MockSection(A=0.01)  # 100 cm²
        material = MockMaterial(fy=355000.0)  # 355 MPa

        # Capacity = 0.01 * 355000 = 3550 kN
        actions = {"N": 1775.0}  # 50% utilization
        util = check.compute_utilization(actions, section, material)
        assert pytest.approx(util, rel=1e-6) == 0.5

    def test_compute_utilization_with_safety_factor(self):
        """Test utilization with safety factor."""
        check = SimpleAxialCheck()
        section = MockSection(A=0.01)
        material = MockMaterial(fy=355000.0)

        # Capacity = 0.01 * 355000 / 1.1 = 3227.27 kN
        actions = {"N": 1613.64}  # ~50% utilization with gamma_M0=1.1
        util = check.compute_utilization(actions, section, material, gamma_M0=1.1)
        assert pytest.approx(util, rel=1e-3) == 0.5

    def test_check_method(self):
        """Test the convenience check method."""
        check = SimpleAxialCheck()
        section = MockSection(A=0.01)
        material = MockMaterial(fy=355000.0)

        actions = {"N": 1775.0}
        result = check.check(
            actions=actions,
            section=section,
            material=material,
            element_id=42,
            location=0.25,
            load_combination="ULS1",
        )

        assert isinstance(result, CheckResult)
        assert result.element_id == 42
        assert result.location == 0.25
        assert result.check_name == "Axial Capacity"
        assert pytest.approx(result.utilization, rel=1e-6) == 0.5
        assert result.load_combination == "ULS1"
        assert result.status == "PASS"


# =============================================================================
# Tests for DesignCode
# =============================================================================


class TestDesignCode:
    """Tests for DesignCode abstract base class."""

    def test_design_code_is_abstract(self):
        """Test that DesignCode cannot be instantiated directly."""
        with pytest.raises(TypeError):
            DesignCode()

    def test_concrete_code_implementation(self):
        """Test concrete implementation of DesignCode."""
        code = SimpleDesignCode()
        assert code.name == "Simple Test Code v1.0"
        assert code.version == "1.0-test"

    def test_get_checks(self):
        """Test get_checks returns list of checks."""
        code = SimpleDesignCode()
        checks = code.get_checks()

        assert len(checks) == 2
        assert all(isinstance(c, DesignCheck) for c in checks)
        check_names = [c.name for c in checks]
        assert "Axial Capacity" in check_names
        assert "Bending Capacity (My)" in check_names

    def test_check_beam(self):
        """Test check_beam performs all checks."""
        code = SimpleDesignCode()
        beam = MockBeam(id=1, section=MockSection(), material=MockMaterial())

        # Define actions at different locations
        result_case = MockResultCase(
            {
                (1, 0.0): {"N": 1000.0, "My": 100.0},
                (1, 0.5): {"N": 500.0, "My": 200.0},
                (1, 1.0): {"N": 0.0, "My": 50.0},
            }
        )
        combination = MockCombination(name="ULS1")

        results = code.check_beam(beam, result_case, combination)

        # 3 locations × 2 checks = 6 results
        assert len(results) == 6
        assert all(isinstance(r, CheckResult) for r in results)
        assert all(r.element_id == 1 for r in results)
        assert all(r.load_combination == "ULS1" for r in results)

    def test_check_all_beams(self):
        """Test check_all_beams aggregates results."""
        code = SimpleDesignCode()
        beams = [
            MockBeam(id=1, section=MockSection(), material=MockMaterial()),
            MockBeam(id=2, section=MockSection(), material=MockMaterial()),
        ]

        result_case = MockResultCase(
            {
                (1, 0.0): {"N": 1000.0, "My": 100.0},
                (1, 0.5): {"N": 500.0, "My": 200.0},
                (1, 1.0): {"N": 0.0, "My": 50.0},
                (2, 0.0): {"N": 2000.0, "My": 150.0},
                (2, 0.5): {"N": 1000.0, "My": 300.0},
                (2, 1.0): {"N": 500.0, "My": 100.0},
            }
        )
        combination = MockCombination(name="ULS1")

        results = code.check_all_beams(beams, result_case, combination)

        # 2 beams × 3 locations × 2 checks = 12 results
        assert len(results) == 12

    def test_get_governing_results(self):
        """Test get_governing_results marks max utilization per element."""
        code = SimpleDesignCode()

        results = [
            CheckResult(1, 0.0, "Axial", 0.5, "LC1"),
            CheckResult(1, 0.5, "Axial", 0.8, "LC1"),  # Max for element 1
            CheckResult(1, 1.0, "Axial", 0.3, "LC1"),
            CheckResult(2, 0.0, "Axial", 0.6, "LC1"),
            CheckResult(2, 0.5, "Axial", 0.9, "LC1"),  # Max for element 2
        ]

        code.get_governing_results(results)

        # Check that correct results are marked as governing
        governing_results = [r for r in results if r.governing]
        assert len(governing_results) == 2

        elem1_governing = [r for r in governing_results if r.element_id == 1]
        assert len(elem1_governing) == 1
        assert elem1_governing[0].utilization == 0.8

        elem2_governing = [r for r in governing_results if r.element_id == 2]
        assert len(elem2_governing) == 1
        assert elem2_governing[0].utilization == 0.9

    def test_get_summary(self):
        """Test get_summary produces correct statistics."""
        code = SimpleDesignCode()

        results = [
            CheckResult(1, 0.0, "Axial", 0.5, "LC1"),
            CheckResult(1, 0.5, "Axial", 0.8, "LC1"),
            CheckResult(1, 1.0, "Bending", 1.2, "LC1"),  # Fails
            CheckResult(2, 0.0, "Axial", 0.6, "LC1"),
            CheckResult(2, 0.5, "Bending", 0.95, "LC1"),  # Max utilization for passing
        ]

        summary = code.get_summary(results)

        assert summary["total_checks"] == 5
        assert summary["passed"] == 4
        assert summary["failed"] == 1
        assert summary["max_utilization"] == 1.2
        assert summary["governing_element"] == 1
        assert summary["governing_check"] == "Bending"

    def test_get_summary_empty(self):
        """Test get_summary with empty results."""
        code = SimpleDesignCode()
        summary = code.get_summary([])

        assert summary["total_checks"] == 0
        assert summary["passed"] == 0
        assert summary["failed"] == 0
        assert summary["max_utilization"] == 0.0
        assert summary["governing_element"] is None


# =============================================================================
# Tests for Plugin Architecture
# =============================================================================


class TestPluginArchitecture:
    """Tests verifying the plugin architecture works correctly."""

    def test_multiple_codes_can_coexist(self):
        """Test that multiple design codes can be instantiated."""

        class AnotherDesignCode(DesignCode):
            @property
            def name(self) -> str:
                return "Another Code"

            def get_checks(self) -> List[DesignCheck]:
                return [SimpleAxialCheck()]

            def check_beam(self, beam, result_case, combination) -> List[CheckResult]:
                return []

        code1 = SimpleDesignCode()
        code2 = AnotherDesignCode()

        assert code1.name != code2.name
        assert len(code1.get_checks()) == 2
        assert len(code2.get_checks()) == 1

    def test_custom_check_implementation(self):
        """Test custom check can be created and used."""

        class CombinedCheck(DesignCheck):
            """Combined axial + bending interaction check."""

            @property
            def name(self) -> str:
                return "Axial-Bending Interaction"

            def compute_utilization(
                self, actions: dict, section: Any, material: Any, **kwargs
            ) -> float:
                N = abs(actions.get("N", 0.0))
                My = abs(actions.get("My", 0.0))

                N_Rd = section.A * material.fy
                M_Rd = section.Wy * material.fy

                # Simple linear interaction
                return N / N_Rd + My / M_Rd

        check = CombinedCheck()
        section = MockSection(A=0.01, Wy=0.001)
        material = MockMaterial(fy=355000.0)

        # N_Rd = 3550 kN, M_Rd = 355 kNm
        # 25% axial + 25% bending = 50% combined
        actions = {"N": 887.5, "My": 88.75}
        util = check.compute_utilization(actions, section, material)
        assert pytest.approx(util, rel=1e-6) == 0.5

    def test_check_results_include_all_required_info(self):
        """Verify CheckResult contains all required information."""
        result = CheckResult(
            element_id=1,
            location=0.5,
            check_name="Test Check",
            utilization=0.75,
            load_combination="ULS1",
            governing=True,
            details={"extra_info": 42},
        )

        # Required fields as per Task 10.1
        assert hasattr(result, "element_id")
        assert hasattr(result, "location")
        assert hasattr(result, "check_name")
        assert hasattr(result, "utilization")
        assert hasattr(result, "load_combination")
        assert hasattr(result, "governing")

        # Also has status property
        assert hasattr(result, "status")

        # And optional details
        assert hasattr(result, "details")


# =============================================================================
# Import Tests
# =============================================================================


class TestImports:
    """Test that design code classes can be imported correctly."""

    def test_import_from_design_codes(self):
        """Test imports from grillex.design_codes."""
        from grillex.design_codes import CheckResult, DesignCheck, DesignCode

        assert CheckResult is not None
        assert DesignCheck is not None
        assert DesignCode is not None

    def test_import_from_base(self):
        """Test imports from grillex.design_codes.base."""
        from grillex.design_codes.base import CheckResult, DesignCheck, DesignCode

        assert CheckResult is not None
        assert DesignCheck is not None
        assert DesignCode is not None

    def test_import_eurocode3(self):
        """Test imports of Eurocode 3 classes."""
        from grillex.design_codes import (
            Eurocode3,
            EC3AxialCheck,
            EC3BendingYCheck,
            EC3BendingZCheck,
            EC3ShearYCheck,
            EC3ShearZCheck,
            EC3CombinedCheck,
        )

        assert Eurocode3 is not None
        assert EC3AxialCheck is not None
        assert EC3BendingYCheck is not None
        assert EC3BendingZCheck is not None
        assert EC3ShearYCheck is not None
        assert EC3ShearZCheck is not None
        assert EC3CombinedCheck is not None


# =============================================================================
# Tests for Eurocode 3 Implementation (Task 10.2)
# =============================================================================


class TestEC3AxialCheck:
    """Tests for EC3 axial capacity check."""

    def test_axial_check_name(self):
        """Test check name is correct."""
        from grillex.design_codes import EC3AxialCheck

        check = EC3AxialCheck()
        assert "Axial" in check.name
        assert "6.2.4" in check.name

    def test_axial_utilization_50_percent(self):
        """Test axial utilization at 50%."""
        from grillex.design_codes import EC3AxialCheck

        check = EC3AxialCheck()
        section = MockSection(A=0.01)  # 100 cm²
        material = MockMaterial(fy=355000.0)  # 355 MPa

        # N_Rd = 0.01 * 355000 / 1.0 = 3550 kN
        actions = {"N": 1775.0}  # 50% utilization
        util = check.compute_utilization(actions, section, material)
        assert pytest.approx(util, rel=1e-6) == 0.5

    def test_axial_utilization_with_gamma(self):
        """Test axial utilization with safety factor."""
        from grillex.design_codes import EC3AxialCheck

        check = EC3AxialCheck()
        section = MockSection(A=0.01)
        material = MockMaterial(fy=355000.0)

        # N_Rd = 0.01 * 355000 / 1.1 = 3227.27 kN
        # 50% of 3227.27 = 1613.64 kN
        actions = {"N": 1613.64}
        util = check.compute_utilization(actions, section, material, gamma_M0=1.1)
        assert pytest.approx(util, rel=1e-3) == 0.5

    def test_axial_zero_force(self):
        """Test axial utilization with zero force."""
        from grillex.design_codes import EC3AxialCheck

        check = EC3AxialCheck()
        section = MockSection(A=0.01)
        material = MockMaterial(fy=355000.0)

        actions = {"N": 0.0}
        util = check.compute_utilization(actions, section, material)
        assert util == 0.0

    def test_axial_tension_and_compression(self):
        """Test that both tension and compression give same utilization."""
        from grillex.design_codes import EC3AxialCheck

        check = EC3AxialCheck()
        section = MockSection(A=0.01)
        material = MockMaterial(fy=355000.0)

        util_tension = check.compute_utilization({"N": 1000.0}, section, material)
        util_compression = check.compute_utilization({"N": -1000.0}, section, material)
        assert util_tension == util_compression


class TestEC3BendingChecks:
    """Tests for EC3 bending capacity checks."""

    def test_bending_y_check_name(self):
        """Test My check name."""
        from grillex.design_codes import EC3BendingYCheck

        check = EC3BendingYCheck()
        assert "Bending" in check.name
        assert "My" in check.name

    def test_bending_z_check_name(self):
        """Test Mz check name."""
        from grillex.design_codes import EC3BendingZCheck

        check = EC3BendingZCheck()
        assert "Bending" in check.name
        assert "Mz" in check.name

    def test_bending_y_utilization(self):
        """Test bending My utilization."""
        from grillex.design_codes import EC3BendingYCheck

        check = EC3BendingYCheck()
        section = MockSection(Iy=1e-4, Wy=0.001)  # W_el = 1000 cm³
        material = MockMaterial(fy=355000.0)

        # M_Rd = 0.001 * 355000 = 355 kNm
        actions = {"My": 177.5}  # 50% utilization
        util = check.compute_utilization(actions, section, material, W_el_y=0.001)
        assert pytest.approx(util, rel=1e-6) == 0.5

    def test_bending_z_utilization(self):
        """Test bending Mz utilization."""
        from grillex.design_codes import EC3BendingZCheck

        check = EC3BendingZCheck()
        section = MockSection(Iz=1e-5, Wz=0.0005)  # W_el = 500 cm³
        material = MockMaterial(fy=355000.0)

        # M_Rd = 0.0005 * 355000 = 177.5 kNm
        actions = {"Mz": 88.75}  # 50% utilization
        util = check.compute_utilization(actions, section, material, W_el_z=0.0005)
        assert pytest.approx(util, rel=1e-6) == 0.5


class TestEC3ShearChecks:
    """Tests for EC3 shear capacity checks."""

    def test_shear_y_check_name(self):
        """Test Vy check name."""
        from grillex.design_codes import EC3ShearYCheck

        check = EC3ShearYCheck()
        assert "Shear" in check.name
        assert "Vy" in check.name

    def test_shear_z_check_name(self):
        """Test Vz check name."""
        from grillex.design_codes import EC3ShearZCheck

        check = EC3ShearZCheck()
        assert "Shear" in check.name
        assert "Vz" in check.name

    def test_shear_utilization(self):
        """Test shear utilization calculation."""
        import math
        from grillex.design_codes import EC3ShearYCheck

        check = EC3ShearYCheck()
        section = MockSection(A=0.01)  # 100 cm²
        material = MockMaterial(fy=355000.0)

        # Av = 0.6 * 0.01 = 0.006 m² (default)
        # V_Rd = 0.006 * (355000 / sqrt(3)) = 1229.5 kN
        actions = {"Vy": 614.75}  # 50% utilization
        util = check.compute_utilization(actions, section, material)
        assert pytest.approx(util, rel=1e-3) == 0.5


class TestEC3CombinedCheck:
    """Tests for EC3 combined axial + bending check."""

    def test_combined_check_name(self):
        """Test combined check name."""
        from grillex.design_codes import EC3CombinedCheck

        check = EC3CombinedCheck()
        assert "Combined" in check.name
        assert "N+M" in check.name

    def test_combined_utilization_linear_interaction(self):
        """Test combined utilization uses linear interaction."""
        from grillex.design_codes import EC3CombinedCheck

        check = EC3CombinedCheck()
        section = MockSection(A=0.01, Wy=0.001, Wz=0.0005)
        material = MockMaterial(fy=355000.0)

        # N_Rd = 3550 kN, M_y_Rd = 355 kNm, M_z_Rd = 177.5 kNm
        # 20% axial + 30% My + 0% Mz = 50% combined
        actions = {"N": 710.0, "My": 106.5, "Mz": 0.0}
        util = check.compute_utilization(
            actions, section, material, W_el_y=0.001, W_el_z=0.0005
        )
        assert pytest.approx(util, rel=1e-3) == 0.5

    def test_combined_all_three_actions(self):
        """Test combined with N, My, and Mz."""
        from grillex.design_codes import EC3CombinedCheck

        check = EC3CombinedCheck()
        section = MockSection(A=0.01, Wy=0.001, Wz=0.0005)
        material = MockMaterial(fy=355000.0)

        # 20% + 20% + 20% = 60%
        actions = {"N": 710.0, "My": 71.0, "Mz": 35.5}
        util = check.compute_utilization(
            actions, section, material, W_el_y=0.001, W_el_z=0.0005
        )
        assert pytest.approx(util, rel=1e-2) == 0.6


class TestEurocode3:
    """Tests for the Eurocode3 design code class."""

    def test_eurocode3_name(self):
        """Test Eurocode 3 name and version."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3()
        assert "Eurocode 3" in code.name
        assert "1993-1-1" in code.name
        assert code.version == "2005"

    def test_eurocode3_get_checks(self):
        """Test get_checks returns all implemented checks."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3()
        checks = code.get_checks()

        assert len(checks) == 6
        check_names = [c.name for c in checks]
        assert any("Axial" in n for n in check_names)
        assert any("Bending" in n and "My" in n for n in check_names)
        assert any("Bending" in n and "Mz" in n for n in check_names)
        assert any("Shear" in n and "Vy" in n for n in check_names)
        assert any("Shear" in n and "Vz" in n for n in check_names)
        assert any("Combined" in n for n in check_names)

    def test_eurocode3_custom_gamma(self):
        """Test custom safety factors."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3(gamma_M0=1.1, gamma_M1=1.2)
        assert code.gamma_M0 == 1.1
        assert code.gamma_M1 == 1.2

    def test_eurocode3_custom_check_locations(self):
        """Test custom check locations."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3(check_locations=[0.0, 0.5, 1.0])
        assert code.check_locations == [0.0, 0.5, 1.0]

    def test_eurocode3_check_beam(self):
        """Test check_beam performs all checks at all locations."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3(check_locations=[0.0, 0.5, 1.0])
        beam = MockBeam(id=1, section=MockSection(), material=MockMaterial())

        result_case = MockResultCase(
            {
                (1, 0.0): {"N": 1000.0, "Vy": 50.0, "Vz": 50.0, "My": 100.0, "Mz": 50.0},
                (1, 0.5): {"N": 500.0, "Vy": 0.0, "Vz": 0.0, "My": 150.0, "Mz": 30.0},
                (1, 1.0): {"N": 0.0, "Vy": 50.0, "Vz": 50.0, "My": 0.0, "Mz": 0.0},
            }
        )
        combination = MockCombination(name="ULS1")

        results = code.check_beam(beam, result_case, combination)

        # 3 locations × 6 checks = 18 results
        assert len(results) == 18
        assert all(r.element_id == 1 for r in results)
        assert all(r.load_combination == "ULS1" for r in results)

    def test_eurocode3_governing_identified(self):
        """Test that governing check is identified."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3(check_locations=[0.0, 0.5, 1.0])
        beam = MockBeam(id=1, section=MockSection(), material=MockMaterial())

        # High bending at midspan
        result_case = MockResultCase(
            {
                (1, 0.0): {"N": 100.0, "Vy": 50.0, "Vz": 50.0, "My": 50.0, "Mz": 10.0},
                (1, 0.5): {"N": 100.0, "Vy": 0.0, "Vz": 0.0, "My": 300.0, "Mz": 10.0},
                (1, 1.0): {"N": 100.0, "Vy": 50.0, "Vz": 50.0, "My": 50.0, "Mz": 10.0},
            }
        )
        combination = MockCombination(name="ULS1")

        results = code.check_beam(beam, result_case, combination)

        # Find governing result
        governing = [r for r in results if r.governing]
        assert len(governing) == 1

        # Should be the combined check at midspan (highest utilization)
        gov = governing[0]
        assert gov.location == 0.5

    def test_eurocode3_utilization_computed_correctly(self):
        """Test that utilizations are computed correctly (acceptance criterion)."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3()
        beam = MockBeam(id=1, section=MockSection(A=0.01), material=MockMaterial(fy=355000.0))

        # 50% axial load: N = 1775 kN (N_Rd = 3550 kN)
        result_case = MockResultCase(
            {
                (1, 0.0): {"N": 1775.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 0.25): {"N": 1775.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 0.5): {"N": 1775.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 0.75): {"N": 1775.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 1.0): {"N": 1775.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
            }
        )
        combination = MockCombination(name="ULS1")

        results = code.check_beam(beam, result_case, combination)

        # Find axial check results
        axial_results = [r for r in results if "Axial" in r.check_name]
        assert len(axial_results) == 5  # 5 locations

        for r in axial_results:
            assert pytest.approx(r.utilization, rel=1e-3) == 0.5
            assert r.status == "PASS"

    def test_eurocode3_failing_check(self):
        """Test detection of failing checks."""
        from grillex.design_codes import Eurocode3

        code = Eurocode3()
        beam = MockBeam(id=1, section=MockSection(A=0.01), material=MockMaterial(fy=355000.0))

        # 120% axial load: N = 4260 kN (N_Rd = 3550 kN)
        result_case = MockResultCase(
            {
                (1, 0.0): {"N": 4260.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 0.25): {"N": 4260.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 0.5): {"N": 4260.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 0.75): {"N": 4260.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
                (1, 1.0): {"N": 4260.0, "Vy": 0.0, "Vz": 0.0, "My": 0.0, "Mz": 0.0},
            }
        )
        combination = MockCombination(name="ULS1")

        results = code.check_beam(beam, result_case, combination)
        summary = code.get_summary(results)

        # Should have failures
        assert summary["failed"] > 0
        assert summary["max_utilization"] > 1.0

        # Axial checks should fail
        axial_results = [r for r in results if "Axial" in r.check_name]
        assert all(r.status == "FAIL" for r in axial_results)
        assert all(r.utilization > 1.0 for r in axial_results)
