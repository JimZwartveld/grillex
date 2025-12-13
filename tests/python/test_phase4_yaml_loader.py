"""
Test suite for Phase 4: YAML Input Parser (Task 4.2)

Tests the YAML loader functionality:
1. Loading valid YAML files
2. All entity types (materials, sections, beams, BCs, loads)
3. Error handling for invalid YAML

Acceptance Criteria (Task 4.2):
- AC1: Valid YAML files load without error
- AC2: All entity types are supported
- AC3: Clear error messages for invalid YAML
"""

import pytest
from pathlib import Path
import tempfile
import yaml

from grillex.io import (
    load_model_from_yaml,
    build_model_from_dict,
    YAMLLoadError
)
from grillex.core import DOFIndex


# Path to test data directory
TEST_DATA_DIR = Path(__file__).parent.parent / "test_data"


class TestYAMLLoading:
    """Test loading valid YAML files"""

    def test_load_simple_cantilever(self):
        """Test loading a simple cantilever beam model - AC1"""
        yaml_path = TEST_DATA_DIR / "simple_cantilever.yaml"
        model = load_model_from_yaml(str(yaml_path))

        assert model is not None
        assert model.name == "Simple Cantilever Beam"
        assert model.num_nodes() == 2
        assert model.num_beams() == 1
        assert model.num_elements() == 1

    def test_load_multi_span_beam(self):
        """Test loading a multi-span beam model - AC1"""
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))

        assert model is not None
        assert model.name == "Two-Span Beam"
        assert model.num_nodes() == 3
        assert model.num_beams() == 2
        assert model.num_elements() == 2

    def test_loaded_model_can_be_analyzed(self):
        """Test that loaded model can be analyzed successfully - AC1"""
        yaml_path = TEST_DATA_DIR / "simple_cantilever.yaml"
        model = load_model_from_yaml(str(yaml_path))

        # Analyze
        success = model.analyze()
        assert success
        assert model.is_analyzed()

        # Check results
        disp = model.get_displacement_at([6, 0, 0], DOFIndex.UY)
        assert disp < 0  # Downward displacement
        assert abs(disp) > 0  # Non-zero

    def test_loaded_model_with_multiple_load_cases(self):
        """Test loading model with multiple load cases - AC2"""
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))

        # Should have 2 load cases
        load_cases = model.cpp_model.get_load_cases()
        assert len(load_cases) == 2

        # Analyze
        success = model.analyze()
        assert success


class TestEntityTypes:
    """Test that all entity types are supported - AC2"""

    def test_materials_loaded(self):
        """Test that materials are loaded correctly - AC2"""
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))

        # Should have 2 materials
        assert model.get_material("Steel") is not None
        assert model.get_material("Aluminum") is not None

        steel = model.get_material("Steel")
        assert steel.name == "Steel"
        assert steel.E == pytest.approx(210000000)

    def test_sections_loaded(self):
        """Test that sections are loaded correctly - AC2"""
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))

        # Should have 2 sections
        assert model.get_section("IPE300") is not None
        assert model.get_section("IPE400") is not None

        ipe300 = model.get_section("IPE300")
        assert ipe300.name == "IPE300"
        assert ipe300.A == pytest.approx(0.00538)

    def test_beams_loaded(self):
        """Test that beams are loaded correctly - AC2"""
        yaml_path = TEST_DATA_DIR / "simple_cantilever.yaml"
        model = load_model_from_yaml(str(yaml_path))

        assert model.num_beams() == 1
        beam = model.beams[0]
        assert beam.length == pytest.approx(6.0)

    def test_boundary_conditions_loaded(self):
        """Test that boundary conditions are loaded correctly - AC2"""
        yaml_path = TEST_DATA_DIR / "simple_cantilever.yaml"
        model = load_model_from_yaml(str(yaml_path))

        # Should have 6 fixed DOFs (all DOFs at one node)
        assert model.boundary_conditions.num_fixed_dofs() == 6

    def test_boundary_condition_types(self):
        """Test different BC types: fixed, pinned, custom - AC2"""
        yaml_data = {
            'name': 'BC Test',
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [
                {'start': [0, 0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'},
                {'start': [6, 0, 0], 'end': [12, 0, 0], 'section': 'Test', 'material': 'Steel'},
                {'start': [12, 0, 0], 'end': [18, 0, 0], 'section': 'Test', 'material': 'Steel'}
            ],
            'boundary_conditions': [
                {'node': [0, 0, 0], 'type': 'fixed'},
                {'node': [6, 0, 0], 'type': 'pinned'},
                {'node': [12, 0, 0], 'type': 'custom', 'dofs': ['UY', 'RZ']}
            ]
        }

        model = build_model_from_dict(yaml_data)

        # Fixed: 6 DOFs, Pinned: 3 DOFs, Custom: 2 DOFs = 11 total
        assert model.boundary_conditions.num_fixed_dofs() == 11

    def test_load_cases_loaded(self):
        """Test that load cases are loaded correctly - AC2"""
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))

        load_cases = model.cpp_model.get_load_cases()
        assert len(load_cases) == 2

        # Check names
        names = [lc.name for lc in load_cases]
        assert "Dead Load" in names
        assert "Live Load" in names


class TestErrorHandling:
    """Test error handling for invalid YAML - AC3"""

    def test_file_not_found(self):
        """Test error when YAML file doesn't exist - AC3"""
        with pytest.raises(FileNotFoundError):
            load_model_from_yaml("nonexistent_file.yaml")

    def test_empty_yaml_file(self):
        """Test error for empty YAML file - AC3"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("")
            temp_path = f.name

        try:
            with pytest.raises(YAMLLoadError, match="empty"):
                load_model_from_yaml(temp_path)
        finally:
            Path(temp_path).unlink()

    def test_invalid_yaml_syntax(self):
        """Test error for invalid YAML syntax - AC3"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("invalid: yaml: syntax: [}")
            temp_path = f.name

        try:
            with pytest.raises(YAMLLoadError, match="Invalid YAML syntax"):
                load_model_from_yaml(temp_path)
        finally:
            Path(temp_path).unlink()

    def test_yaml_root_not_dict(self):
        """Test error when YAML root is not a dictionary - AC3"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write("- this\n- is\n- a\n- list")
            temp_path = f.name

        try:
            with pytest.raises(YAMLLoadError, match="must be a dictionary"):
                load_model_from_yaml(temp_path)
        finally:
            Path(temp_path).unlink()

    def test_missing_required_material_field(self):
        """Test error when material is missing required field - AC3"""
        yaml_data = {
            'materials': [
                {'name': 'Steel', 'E': 210e6}  # Missing 'nu' and 'rho'
            ]
        }

        with pytest.raises(YAMLLoadError, match="missing required field"):
            build_model_from_dict(yaml_data)

    def test_missing_required_section_field(self):
        """Test error when section is missing required field - AC3"""
        yaml_data = {
            'sections': [
                {'name': 'IPE300', 'A': 0.01}  # Missing Iy, Iz, J
            ]
        }

        with pytest.raises(YAMLLoadError, match="missing required field"):
            build_model_from_dict(yaml_data)

    def test_missing_required_beam_field(self):
        """Test error when beam is missing required field - AC3"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [
                {'start': [0, 0, 0], 'section': 'Test'}  # Missing 'end' and 'material'
            ]
        }

        with pytest.raises(YAMLLoadError, match="missing required field"):
            build_model_from_dict(yaml_data)

    def test_invalid_beam_coordinates(self):
        """Test error for invalid beam coordinates - AC3"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [
                {'start': [0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'}  # Only 2 coords
            ]
        }

        with pytest.raises(YAMLLoadError, match="must be a list of 3 coordinates"):
            build_model_from_dict(yaml_data)

    def test_beam_references_nonexistent_material(self):
        """Test error when beam references non-existent material - AC3"""
        yaml_path = TEST_DATA_DIR / "invalid_missing_material.yaml"

        with pytest.raises(YAMLLoadError, match="not found"):
            load_model_from_yaml(str(yaml_path))

    def test_invalid_dof_name(self):
        """Test error for invalid DOF name - AC3"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [{'start': [0, 0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'}],
            'boundary_conditions': [
                {'node': [0, 0, 0], 'type': 'custom', 'dofs': ['INVALID_DOF']}
            ]
        }

        with pytest.raises(YAMLLoadError, match="Invalid DOF"):
            build_model_from_dict(yaml_data)

    def test_invalid_load_case_type(self):
        """Test error for invalid load case type - AC3"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [{'start': [0, 0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'}],
            'load_cases': [
                {'name': 'Test', 'type': 'INVALID_TYPE', 'loads': []}
            ]
        }

        with pytest.raises(YAMLLoadError, match="Invalid load case type"):
            build_model_from_dict(yaml_data)

    def test_invalid_bc_type(self):
        """Test error for invalid boundary condition type - AC3"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [{'start': [0, 0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'}],
            'boundary_conditions': [
                {'node': [0, 0, 0], 'type': 'invalid_type'}
            ]
        }

        with pytest.raises(YAMLLoadError, match="unknown type"):
            build_model_from_dict(yaml_data)


class TestDefaultValues:
    """Test default values and optional fields"""

    def test_model_name_defaults_to_filename(self):
        """Test that model name defaults to filename if not specified"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [{'start': [0, 0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'}]
        }

        model = build_model_from_dict(yaml_data, default_name="test_model")
        assert model.name == "test_model"

    def test_load_case_type_defaults_to_variable(self):
        """Test that load case type defaults to Variable"""
        yaml_data = {
            'materials': [{'name': 'Steel', 'E': 210e6, 'nu': 0.3, 'rho': 7.85e-6}],
            'sections': [{'name': 'Test', 'A': 0.01, 'Iy': 1e-5, 'Iz': 1e-5, 'J': 1e-5}],
            'beams': [{'start': [0, 0, 0], 'end': [6, 0, 0], 'section': 'Test', 'material': 'Steel'}],
            'load_cases': [
                {'name': 'Test', 'loads': []}  # No 'type' specified
            ]
        }

        model = build_model_from_dict(yaml_data)
        # Should not raise error - defaults to Variable


class TestAcceptanceCriteria:
    """Explicit tests for all acceptance criteria"""

    def test_ac1_valid_yaml_files_load_without_error(self):
        """AC1: Valid YAML files load without error"""
        # Test simple cantilever
        yaml_path = TEST_DATA_DIR / "simple_cantilever.yaml"
        model = load_model_from_yaml(str(yaml_path))
        assert model is not None

        # Test multi-span beam
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))
        assert model is not None

    def test_ac2_all_entity_types_supported(self):
        """AC2: All entity types are supported"""
        yaml_path = TEST_DATA_DIR / "multi_span_beam.yaml"
        model = load_model_from_yaml(str(yaml_path))

        # Materials
        assert model.get_material("Steel") is not None
        assert model.get_material("Aluminum") is not None

        # Sections
        assert model.get_section("IPE300") is not None
        assert model.get_section("IPE400") is not None

        # Beams
        assert model.num_beams() == 2

        # Boundary conditions
        assert model.boundary_conditions.num_fixed_dofs() > 0

        # Load cases
        load_cases = model.cpp_model.get_load_cases()
        assert len(load_cases) == 2

    def test_ac3_clear_error_messages_for_invalid_yaml(self):
        """AC3: Clear error messages for invalid YAML"""
        # Test various invalid scenarios
        test_cases = [
            # (yaml_data, expected_error_substring)
            ({}, ""),  # Empty dict is technically valid
            ({'materials': 'not_a_list'}, "'materials' must be a list"),
            ({'sections': [{'name': 'Test'}]}, "missing required field"),
            ({'beams': [{'start': [0, 0, 0]}]}, "missing required field"),  # Missing 'end'
        ]

        for i, (yaml_data, expected_msg) in enumerate(test_cases):
            if expected_msg:  # Only test cases that should raise errors
                try:
                    build_model_from_dict(yaml_data)
                    # If we get here without error, test should fail
                    assert False, f"Test case {i} should have raised YAMLLoadError"
                except YAMLLoadError as e:
                    # Check that error message is informative
                    assert len(str(e)) > 10  # At least some description
                    if expected_msg:
                        assert expected_msg.lower() in str(e).lower()
