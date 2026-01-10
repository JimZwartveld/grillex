"""Tests for sectionbuilder integration module.

These tests verify the sections module functionality without requiring
sectionbuilder to be installed. Mock objects are used to simulate
sectionbuilder behavior.
"""

import pytest
import math
from unittest.mock import Mock, MagicMock

from grillex.sections import (
    ScaleFactors,
    get_scale_factors,
    convert_section_properties,
    convert_section_dict,
    SectionInfo,
    GrillexSectionAdapter,
    SectionLibraryRegistry,
    ExtendedSectionProperties,
    SECTIONBUILDER_AVAILABLE
)


class TestScaleFactors:
    """Tests for scale factor calculation."""

    def test_mm_scale_factors(self):
        """Scale factors for mm input should convert to meters."""
        sf = get_scale_factors("mm")

        assert sf.length == 1e-3
        assert sf.area == pytest.approx(1e-6)
        assert sf.I == pytest.approx(1e-12)
        assert sf.Iw == pytest.approx(1e-18)
        assert sf.S == pytest.approx(1e-9)

    def test_m_scale_factors(self):
        """Scale factors for m input should be unity."""
        sf = get_scale_factors("m")

        assert sf.length == 1.0
        assert sf.area == 1.0
        assert sf.I == 1.0
        assert sf.Iw == 1.0
        assert sf.S == 1.0

    def test_invalid_units_raises(self):
        """Invalid units should raise ValueError."""
        with pytest.raises(ValueError, match="Invalid input_units"):
            get_scale_factors("cm")


class TestConvertSectionProperties:
    """Tests for converting sectionbuilder properties to grillex."""

    def create_mock_sb_props(self):
        """Create a mock sectionbuilder properties object."""
        props = Mock()
        # IPE300-like properties in mm
        props.area = 5380  # mm²
        props.Ixx = 83600000  # mm⁴ (strong axis)
        props.Iyy = 6040000  # mm⁴ (weak axis)
        props.J = 201000  # mm⁴
        props.Cw = 126000000000  # mm⁶
        props.omega_max = 22500  # mm² (h*bf/4 = 300*150/4)
        props.Avx = 2000  # mm²
        props.Avy = 3000  # mm²
        props.y_top = 150  # mm
        props.y_bottom = 150  # mm
        props.x_right = 75  # mm
        props.x_left = 75  # mm
        return props

    def test_convert_mm_to_m(self):
        """Conversion from mm to m should apply correct scaling."""
        props = self.create_mock_sb_props()
        result = convert_section_properties(props, input_units="mm")

        # A: 5380 mm² -> 0.00538 m²
        assert result["A"] == pytest.approx(0.00538, rel=1e-6)

        # Iz (strong): 83600000 mm⁴ -> 8.36e-5 m⁴
        assert result["Iz"] == pytest.approx(8.36e-5, rel=1e-6)

        # Iy (weak): 6040000 mm⁴ -> 6.04e-6 m⁴
        assert result["Iy"] == pytest.approx(6.04e-6, rel=1e-6)

        # J: 201000 mm⁴ -> 2.01e-7 m⁴
        assert result["J"] == pytest.approx(2.01e-7, rel=1e-6)

        # Iw: 126e9 mm⁶ -> 1.26e-7 m⁶
        assert result["Iw"] == pytest.approx(1.26e-7, rel=1e-6)

        # omega_max: 22500 mm² -> 0.0225 m² (22500 * 1e-6)
        assert result["omega_max"] == pytest.approx(0.0225, rel=1e-6)

    def test_axis_mapping(self):
        """Axis convention mapping should swap strong/weak correctly."""
        props = self.create_mock_sb_props()
        result = convert_section_properties(props, input_units="mm")

        # SB Avy (vertical shear) -> Asy
        # SB Avx (horizontal shear) -> Asz
        # 3000 mm² * 1e-6 = 0.003 m², 2000 mm² * 1e-6 = 0.002 m²
        assert result["Asy"] == pytest.approx(0.003, rel=1e-6)  # 3000 mm²
        assert result["Asz"] == pytest.approx(0.002, rel=1e-6)  # 2000 mm²

        # Fibre distances
        assert result["zy_top"] == pytest.approx(0.150, rel=1e-6)
        assert result["zz_top"] == pytest.approx(0.075, rel=1e-6)

    def test_requires_warping_flag(self):
        """requires_warping should be True when Iw > 0."""
        props = self.create_mock_sb_props()
        result = convert_section_properties(props, input_units="mm")
        assert result["requires_warping"] is True

        # Set Cw to 0
        props.Cw = 0
        result = convert_section_properties(props, input_units="mm")
        assert result["requires_warping"] is False

    def test_missing_properties_default_to_zero(self):
        """Missing properties should default to 0."""
        # Create a simple object with only area defined
        class MinimalProps:
            area = 1000  # mm²
        props = MinimalProps()

        result = convert_section_properties(props, input_units="mm")

        # 1000 mm² * 1e-6 = 0.001 m²
        assert result["A"] == pytest.approx(0.001, rel=1e-6)
        assert result["Iz"] == 0.0
        assert result["Iw"] == 0.0
        assert result["omega_max"] == 0.0


class TestConvertSectionDict:
    """Tests for dict-based conversion."""

    def test_convert_dict(self):
        """Dict conversion should work same as object conversion."""
        props_dict = {
            "area": 5380,
            "Ixx": 83600000,
            "Iyy": 6040000,
            "J": 201000,
        }

        result = convert_section_dict(props_dict, input_units="mm")

        assert result["A"] == pytest.approx(0.00538, rel=1e-6)
        assert result["Iz"] == pytest.approx(8.36e-5, rel=1e-6)

    def test_convert_dict_with_none_values(self):
        """None values should be treated as 0."""
        props_dict = {
            "area": 1000,
            "Ixx": None,
            "Iyy": None,
        }

        result = convert_section_dict(props_dict, input_units="mm")

        # 1000 mm² * 1e-6 = 0.001 m²
        assert result["A"] == pytest.approx(0.001, rel=1e-6)
        assert result["Iz"] == 0.0


class TestGrillexSectionAdapter:
    """Tests for GrillexSectionAdapter."""

    def create_mock_sb_adapter(self):
        """Create a mock sectionbuilder library adapter."""
        adapter = Mock()

        # Create mock section properties
        heb300_props = Mock()
        heb300_props.area = 14910
        heb300_props.Ixx = 251700000
        heb300_props.Iyy = 85630000
        heb300_props.J = 1850000
        heb300_props.Cw = 1690000000000
        heb300_props.omega_max = 33750  # 300*150/4 approx
        heb300_props.Avx = 5000
        heb300_props.Avy = 7000
        heb300_props.y_top = 150
        heb300_props.y_bottom = 150
        heb300_props.x_left = 75
        heb300_props.x_right = 75

        # Setup adapter methods
        adapter.get_section = Mock(return_value=heb300_props)
        adapter.search = Mock(return_value=[
            Mock(designation="HEB300", description="Wide flange beam"),
            Mock(designation="HEB320", description="Wide flange beam"),
        ])
        adapter.list_sections = Mock(return_value=["HEB300", "HEB320", "HEB360"])
        adapter.name = "EurocodeAdapter"

        return adapter

    def test_init_valid_units(self):
        """Adapter should accept valid unit systems."""
        sb_adapter = self.create_mock_sb_adapter()

        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")
        assert adapter.input_units == "mm"

        adapter = GrillexSectionAdapter(sb_adapter, input_units="m")
        assert adapter.input_units == "m"

    def test_init_invalid_units_raises(self):
        """Adapter should reject invalid unit systems."""
        sb_adapter = self.create_mock_sb_adapter()

        with pytest.raises(ValueError, match="Invalid input_units"):
            GrillexSectionAdapter(sb_adapter, input_units="cm")

    def test_get_grillex_section_params(self):
        """get_grillex_section_params should return converted properties."""
        sb_adapter = self.create_mock_sb_adapter()
        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")

        params = adapter.get_grillex_section_params("HEB300")

        # Verify conversion applied
        assert params["A"] == pytest.approx(14910e-6, rel=1e-6)  # m²
        assert params["Iz"] == pytest.approx(251700000e-12, rel=1e-6)  # m⁴
        assert params["Iy"] == pytest.approx(85630000e-12, rel=1e-6)  # m⁴

        # Verify underlying adapter was called
        sb_adapter.get_section.assert_called_once_with("HEB300")

    def test_caching(self):
        """Results should be cached by default."""
        sb_adapter = self.create_mock_sb_adapter()
        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")

        # First call
        params1 = adapter.get_grillex_section_params("HEB300")
        assert sb_adapter.get_section.call_count == 1

        # Second call - should use cache
        params2 = adapter.get_grillex_section_params("HEB300")
        assert sb_adapter.get_section.call_count == 1  # Not called again

        # Verify results are equal
        assert params1 == params2

    def test_cache_bypass(self):
        """Cache can be bypassed with use_cache=False."""
        sb_adapter = self.create_mock_sb_adapter()
        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")

        adapter.get_grillex_section_params("HEB300")
        adapter.get_grillex_section_params("HEB300", use_cache=False)

        assert sb_adapter.get_section.call_count == 2

    def test_search(self):
        """search should return SectionInfo objects."""
        sb_adapter = self.create_mock_sb_adapter()
        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")

        results = adapter.search("HEB")

        assert len(results) == 2
        assert all(isinstance(r, SectionInfo) for r in results)
        assert results[0].designation == "HEB300"

    def test_list_sections(self):
        """list_sections should return all section names."""
        sb_adapter = self.create_mock_sb_adapter()
        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")

        sections = adapter.list_sections()

        assert "HEB300" in sections
        assert "HEB320" in sections

    def test_contains(self):
        """__contains__ should check if section exists."""
        sb_adapter = self.create_mock_sb_adapter()
        adapter = GrillexSectionAdapter(sb_adapter, input_units="mm")

        assert "HEB300" in adapter

        sb_adapter.get_section.side_effect = KeyError("Not found")
        assert "INVALID" not in adapter


class TestSectionLibraryRegistry:
    """Tests for SectionLibraryRegistry."""

    def create_mock_adapter(self, name="TestAdapter"):
        """Create a mock GrillexSectionAdapter."""
        sb_adapter = Mock()
        sb_adapter.get_section = Mock(return_value=Mock(
            area=5000, Ixx=50000000, Iyy=20000000, J=100000,
            Cw=0, omega_max=0, Avx=2000, Avy=3000,
            y_top=100, y_bottom=100, x_left=50, x_right=50
        ))
        sb_adapter.search = Mock(return_value=[])
        sb_adapter.list_sections = Mock(return_value=["SEC1", "SEC2"])
        sb_adapter.name = name

        return GrillexSectionAdapter(sb_adapter, input_units="mm")

    def test_register_and_get(self):
        """Should be able to register and retrieve adapters."""
        registry = SectionLibraryRegistry()
        adapter = self.create_mock_adapter()

        registry.register("test", adapter)

        assert "test" in registry
        assert registry.get("test") is adapter
        assert registry["test"] is adapter

    def test_register_duplicate_raises(self):
        """Registering duplicate name should raise."""
        registry = SectionLibraryRegistry()
        adapter1 = self.create_mock_adapter("Adapter1")
        adapter2 = self.create_mock_adapter("Adapter2")

        registry.register("test", adapter1)

        with pytest.raises(ValueError, match="already registered"):
            registry.register("test", adapter2)

    def test_register_duplicate_with_replace(self):
        """Duplicate registration with replace=True should work."""
        registry = SectionLibraryRegistry()
        adapter1 = self.create_mock_adapter("Adapter1")
        adapter2 = self.create_mock_adapter("Adapter2")

        registry.register("test", adapter1)
        registry.register("test", adapter2, replace=True)

        assert registry["test"].adapter_name == "Adapter2"

    def test_unregister(self):
        """Should be able to unregister adapters."""
        registry = SectionLibraryRegistry()
        adapter = self.create_mock_adapter()

        registry.register("test", adapter)
        removed = registry.unregister("test")

        assert removed is adapter
        assert "test" not in registry

    def test_list_libraries(self):
        """list_libraries should return all registered names."""
        registry = SectionLibraryRegistry()

        registry.register("lib1", self.create_mock_adapter())
        registry.register("lib2", self.create_mock_adapter())

        libs = registry.list_libraries()
        assert "lib1" in libs
        assert "lib2" in libs

    def test_get_section_params(self):
        """get_section_params should retrieve from registered libraries."""
        registry = SectionLibraryRegistry()
        adapter = self.create_mock_adapter()
        registry.register("test", adapter)

        params = registry.get_section_params("SEC1")

        assert params["A"] > 0

    def test_get_section_params_not_found_raises(self):
        """get_section_params should raise if section not found."""
        registry = SectionLibraryRegistry()

        with pytest.raises(KeyError, match="not found"):
            registry.get_section_params("INVALID")


class TestExtendedSectionProperties:
    """Tests for ExtendedSectionProperties."""

    def test_from_grillex_params(self):
        """Should create from grillex params dict."""
        params = {
            "A": 0.00538,
            "Iy": 6.04e-6,
            "Iz": 8.36e-5,
            "J": 2.01e-7,
            "Iw": 1.26e-7,
            "omega_max": 2.25e-5,
        }

        props = ExtendedSectionProperties.from_grillex_params(
            params,
            designation="IPE300",
            section_type="I"
        )

        assert props.A == pytest.approx(0.00538)
        assert props.designation == "IPE300"
        assert props.section_type == "I"

    def test_to_grillex_params(self):
        """to_grillex_params should return dict suitable for add_section."""
        props = ExtendedSectionProperties(
            A=0.00538,
            Iy=6.04e-6,
            Iz=8.36e-5,
            J=2.01e-7,
            Iw=1.26e-7,
        )

        params = props.to_grillex_params()

        assert params["A"] == pytest.approx(0.00538)
        assert params["Iy"] == pytest.approx(6.04e-6)
        assert "A" in params
        assert "Iy" in params
        assert "Iz" in params
        assert "J" in params

    def test_calculate_derived(self):
        """Derived properties should be calculated."""
        props = ExtendedSectionProperties(
            A=0.00538,
            Iy=6.04e-6,
            Iz=8.36e-5,
            J=2.01e-7,
            Iw=1.26e-7,
            zy_top=0.15,
            zy_bot=0.15,
        )
        props._calculate_derived()

        # Radius of gyration: sqrt(I/A)
        expected_iy = math.sqrt(6.04e-6 / 0.00538)
        expected_iz = math.sqrt(8.36e-5 / 0.00538)

        assert props.iy == pytest.approx(expected_iy, rel=1e-3)
        assert props.iz == pytest.approx(expected_iz, rel=1e-3)

        # Warping flag
        assert props.requires_warping is True


class TestStructuralModelIntegration:
    """Integration tests for StructuralModel section library methods."""

    def test_list_section_libraries_empty(self):
        """Empty model should have no registered libraries."""
        from grillex.core import StructuralModel

        model = StructuralModel()
        assert model.list_section_libraries() == []

    def test_add_section_from_library_no_libraries_raises(self):
        """Adding from library with none registered should raise."""
        from grillex.core import StructuralModel

        model = StructuralModel()

        with pytest.raises(ValueError, match="No section libraries registered"):
            model.add_section_from_library("HEB300")

    def test_search_sections_unknown_library_raises(self):
        """Searching unknown library should raise KeyError."""
        from grillex.core import StructuralModel

        model = StructuralModel()

        with pytest.raises(KeyError, match="not registered"):
            model.search_sections("HEB", library="unknown")
