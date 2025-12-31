"""
Test suite for Phase 4: JSON Result Export (Task 4.3)

Tests the JSON result export functionality:
1. Exporting results to valid JSON
2. JSON structure is human-readable
3. All result types are included

Acceptance Criteria (Task 4.3):
- AC1: Results export to valid JSON
- AC2: JSON structure is human-readable
- AC3: All result types are included
"""

import pytest
import json
import tempfile
from pathlib import Path

from grillex.core import StructuralModel, DOFIndex, LoadCaseType
from grillex.io import (
    NodeResult,
    ElementResult,
    LoadCaseInfo,
    ModelInfo,
    ResultCase,
    export_results_to_json,
    build_result_case,
    export_all_load_cases_to_json
)


class TestDataClasses:
    """Test result data classes"""

    def test_node_result_creation(self):
        """Test creating NodeResult"""
        node_result = NodeResult(
            node_id=1,
            position=[0.0, 0.0, 0.0],
            displacements=[0.001, -0.002, 0.0, 0.0, 0.0, 0.001],
            reactions=[10.0, 20.0, 0.0, 0.0, 5.0, 0.0]
        )

        assert node_result.node_id == 1
        assert len(node_result.displacements) == 6
        assert len(node_result.reactions) == 6

    def test_element_result_creation(self):
        """Test creating ElementResult"""
        elem_result = ElementResult(
            element_id=0,
            node_i_id=1,
            node_j_id=2,
            length=6.0,
            end_forces_i=[0, 10, 0, 0, 0, 30],
            end_forces_j=[0, 10, 0, 0, 0, -30]
        )

        assert elem_result.element_id == 0
        assert elem_result.length == 6.0
        assert len(elem_result.end_forces_i) == 6

    def test_load_case_info_creation(self):
        """Test creating LoadCaseInfo"""
        lc_info = LoadCaseInfo(
            name="Dead Load",
            type="Permanent",
            num_nodal_loads=3
        )

        assert lc_info.name == "Dead Load"
        assert lc_info.type == "Permanent"

    def test_model_info_creation(self):
        """Test creating ModelInfo"""
        model_info = ModelInfo(
            name="Test Model",
            num_nodes=10,
            num_elements=8,
            num_beams=8,
            total_dofs=60,
            num_load_cases=2
        )

        assert model_info.name == "Test Model"
        assert model_info.num_nodes == 10


class TestResultCase:
    """Test ResultCase class"""

    def test_result_case_creation(self):
        """Test creating ResultCase"""
        model_info = ModelInfo("Test", 2, 1, 1, 12, 1)
        lc_info = LoadCaseInfo("Test Load", "Variable", 1)
        nodes = [NodeResult(1, [0, 0, 0], [0] * 6, [0] * 6)]
        elements = [ElementResult(0, 1, 2, 6.0, [0] * 6, [0] * 6)]

        result_case = ResultCase(
            model_info=model_info,
            load_case_info=lc_info,
            nodes=nodes,
            elements=elements
        )

        assert result_case.success is True
        assert result_case.error_message is None
        assert len(result_case.nodes) == 1
        assert len(result_case.elements) == 1

    def test_result_case_to_dict(self):
        """Test converting ResultCase to dictionary"""
        model_info = ModelInfo("Test", 2, 1, 1, 12, 1)
        lc_info = LoadCaseInfo("Test Load", "Variable", 1)
        nodes = [NodeResult(1, [0, 0, 0], [0] * 6)]
        elements = [ElementResult(0, 1, 2, 6.0, [0] * 6, [0] * 6)]

        result_case = ResultCase(
            model_info=model_info,
            load_case_info=lc_info,
            nodes=nodes,
            elements=elements
        )

        result_dict = result_case.to_dict()

        assert isinstance(result_dict, dict)
        assert 'model_info' in result_dict
        assert 'load_case_info' in result_dict
        assert 'nodes' in result_dict
        assert 'elements' in result_dict
        assert 'units' in result_dict

    def test_result_case_to_json_string(self):
        """Test converting ResultCase to JSON string"""
        model_info = ModelInfo("Test", 2, 1, 1, 12, 1)
        lc_info = LoadCaseInfo("Test Load", "Variable", 1)
        nodes = [NodeResult(1, [0, 0, 0], [0] * 6)]
        elements = [ElementResult(0, 1, 2, 6.0, [0] * 6, [0] * 6)]

        result_case = ResultCase(
            model_info=model_info,
            load_case_info=lc_info,
            nodes=nodes,
            elements=elements
        )

        json_str = result_case.to_json_string()

        assert isinstance(json_str, str)
        # Should be valid JSON
        parsed = json.loads(json_str)
        assert isinstance(parsed, dict)

    def test_result_case_to_json_file(self):
        """Test exporting ResultCase to JSON file - AC1"""
        model_info = ModelInfo("Test", 2, 1, 1, 12, 1)
        lc_info = LoadCaseInfo("Test Load", "Variable", 1)
        nodes = [NodeResult(1, [0, 0, 0], [0] * 6)]
        elements = [ElementResult(0, 1, 2, 6.0, [0] * 6, [0] * 6)]

        result_case = ResultCase(
            model_info=model_info,
            load_case_info=lc_info,
            nodes=nodes,
            elements=elements
        )

        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, encoding='utf-8') as f:
            temp_path = f.name

        try:
            result_case.to_json(temp_path)

            # Verify file exists and is valid JSON
            assert Path(temp_path).exists()

            with open(temp_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            assert isinstance(data, dict)
            assert 'model_info' in data
            assert 'load_case_info' in data
        finally:
            Path(temp_path).unlink()


class TestExportFunctions:
    """Test export functions with actual models"""

    def test_export_simple_cantilever(self):
        """Test exporting results from simple cantilever - AC1"""
        # Create and analyze model
        model = StructuralModel(name="Test Cantilever")
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        # Export to JSON
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, encoding='utf-8') as f:
            temp_path = f.name

        try:
            export_results_to_json(model, temp_path)

            # Verify file exists and is valid JSON
            assert Path(temp_path).exists()

            with open(temp_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            assert isinstance(data, dict)
            assert data['success'] is True
        finally:
            Path(temp_path).unlink()

    def test_build_result_case(self):
        """Test building ResultCase from model"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        result_case = build_result_case(model)

        assert isinstance(result_case, ResultCase)
        assert result_case.success is True
        assert len(result_case.nodes) == 2
        assert len(result_case.elements) == 1

    def test_export_before_analysis_raises_error(self):
        """Test that exporting before analysis raises error"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        with pytest.raises(ValueError, match="must be analyzed"):
            export_results_to_json(model, "test.json")

    def test_export_all_load_cases(self):
        """Test exporting all load cases to separate files"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        # Create multiple load cases
        lc1 = model.create_load_case("Dead Load", LoadCaseType.Permanent)
        lc1.add_nodal_load([6, 0, 0], [0, -5.0, 0])

        lc2 = model.create_load_case("Live Load", LoadCaseType.Variable)
        lc2.add_nodal_load([6, 0, 0], [0, -10.0, 0])

        model.analyze()

        # Export all load cases
        with tempfile.TemporaryDirectory() as temp_dir:
            created_files = export_all_load_cases_to_json(model, temp_dir)

            assert len(created_files) == 2
            assert all(Path(f).exists() for f in created_files)

            # Verify each file is valid JSON
            for file_path in created_files:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                assert isinstance(data, dict)
                assert data['success'] is True


class TestJSONStructure:
    """Test JSON structure is human-readable and complete - AC2 and AC3"""

    def test_json_has_clear_field_names(self):
        """Test that JSON has clear, descriptive field names - AC2"""
        model = StructuralModel(name="Readability Test")
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        result_case = build_result_case(model)
        json_str = result_case.to_json_string()
        data = json.loads(json_str)

        # Check top-level keys
        expected_keys = ['model_info', 'load_case_info', 'nodes', 'elements', 'units', 'success']
        for key in expected_keys:
            assert key in data, f"Missing expected key: {key}"

        # Check model_info has clear fields
        assert 'name' in data['model_info']
        assert 'num_nodes' in data['model_info']
        assert 'num_elements' in data['model_info']

        # Check nodes have clear structure
        assert len(data['nodes']) > 0
        node = data['nodes'][0]
        assert 'node_id' in node
        assert 'position' in node
        assert 'displacements' in node

    def test_json_includes_all_result_types(self):
        """Test that JSON includes all result types - AC3"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        result_case = build_result_case(model)
        data = result_case.to_dict()

        # Model information
        assert 'model_info' in data
        assert data['model_info']['name'] is not None
        assert data['model_info']['num_nodes'] > 0

        # Load case information
        assert 'load_case_info' in data
        assert data['load_case_info']['name'] is not None
        assert data['load_case_info']['type'] is not None

        # Node results (displacements and reactions)
        assert 'nodes' in data
        assert len(data['nodes']) > 0
        node = data['nodes'][0]
        assert 'displacements' in node
        assert len(node['displacements']) == 6  # 6 DOFs

        # All nodes should have reactions (zeros for unconstrained)
        for node in data['nodes']:
            assert 'reactions' in node
            assert node['reactions'] is not None
            assert len(node['reactions']) == 6

        # Element results
        assert 'elements' in data
        assert len(data['elements']) > 0
        element = data['elements'][0]
        assert 'element_id' in element
        assert 'length' in element
        assert 'end_forces_i' in element
        assert 'end_forces_j' in element

        # Units documentation
        assert 'units' in data
        assert 'length' in data['units']
        assert 'force' in data['units']
        assert 'moment' in data['units']

    def test_json_is_properly_formatted(self):
        """Test that JSON is properly formatted with indentation - AC2"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        result_case = build_result_case(model)
        json_str = result_case.to_json_string(indent=2)

        # Should have newlines (pretty printed)
        assert '\n' in json_str

        # Should have indentation
        assert '  ' in json_str

        # Should be parseable
        data = json.loads(json_str)
        assert isinstance(data, dict)

    def test_json_includes_model_metadata(self):
        """Test that JSON includes comprehensive model metadata - AC3"""
        model = StructuralModel(name="Metadata Test Model")
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.add_beam_by_coords([6, 0, 0], [12, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.pin_node_at([12, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        result_case = build_result_case(model)
        data = result_case.to_dict()

        # Model metadata
        model_info = data['model_info']
        assert model_info['name'] == "Metadata Test Model"
        assert model_info['num_nodes'] == 3  # 3 nodes for 2 beams
        assert model_info['num_elements'] == 2  # 2 elements
        assert model_info['num_beams'] == 2  # 2 beams
        assert model_info['total_dofs'] > 0
        assert model_info['num_load_cases'] >= 1


class TestAcceptanceCriteria:
    """Explicit tests for all acceptance criteria"""

    def test_ac1_results_export_to_valid_json(self):
        """AC1: Results export to valid JSON"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, encoding='utf-8') as f:
            temp_path = f.name

        try:
            # Export results
            export_results_to_json(model, temp_path)

            # File should exist
            assert Path(temp_path).exists()

            # Should be valid JSON
            with open(temp_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # Should be a dictionary
            assert isinstance(data, dict)

            # Should contain expected structure
            assert 'model_info' in data
            assert 'nodes' in data
            assert 'elements' in data
        finally:
            Path(temp_path).unlink()

    def test_ac2_json_structure_is_human_readable(self):
        """AC2: JSON structure is human-readable"""
        model = StructuralModel(name="Human Readable Test")
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])

        lc = model.create_load_case("Tip Load", LoadCaseType.Variable)
        lc.add_nodal_load([6, 0, 0], [0, -10.0, 0])

        model.analyze()

        result_case = build_result_case(model)
        json_str = result_case.to_json_string(indent=2)

        # Human-readable features:
        # 1. Pretty printed with indentation
        assert '  ' in json_str
        assert '\n' in json_str

        # 2. Clear field names (not abbreviated)
        assert '"model_info"' in json_str
        assert '"node_id"' in json_str
        assert '"displacements"' in json_str
        assert '"reactions"' in json_str

        # 3. Units documented
        data = json.loads(json_str)
        assert 'units' in data
        assert data['units']['length'] == 'm'
        assert data['units']['force'] == 'kN'

        # 4. Model name preserved
        assert data['model_info']['name'] == "Human Readable Test"

        # 5. Load case name preserved
        assert data['load_case_info']['name'] == "Tip Load"

    def test_ac3_all_result_types_included(self):
        """AC3: All result types are included"""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 1e-5, 2e-5, 1.5e-5)
        model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], force=[0, -10.0, 0])
        model.analyze()

        result_case = build_result_case(model)
        data = result_case.to_dict()

        # All result types that should be included:

        # 1. Model information
        assert 'model_info' in data
        assert all(k in data['model_info'] for k in
                   ['name', 'num_nodes', 'num_elements', 'num_beams', 'total_dofs'])

        # 2. Load case information
        assert 'load_case_info' in data
        assert all(k in data['load_case_info'] for k in
                   ['name', 'type', 'num_nodal_loads'])

        # 3. Node results with displacements
        assert 'nodes' in data
        assert len(data['nodes']) > 0
        for node in data['nodes']:
            assert 'node_id' in node
            assert 'position' in node
            assert 'displacements' in node
            assert len(node['displacements']) == 6

        # 4. Node reactions (for constrained nodes)
        constrained_nodes = [n for n in data['nodes'] if n['reactions'] is not None]
        assert len(constrained_nodes) > 0
        for node in constrained_nodes:
            assert len(node['reactions']) == 6

        # 5. Element results
        assert 'elements' in data
        assert len(data['elements']) > 0
        for element in data['elements']:
            assert 'element_id' in element
            assert 'node_i_id' in element
            assert 'node_j_id' in element
            assert 'length' in element
            assert 'end_forces_i' in element
            assert 'end_forces_j' in element

        # 6. Units documentation
        assert 'units' in data
        required_units = ['length', 'displacement_translation',
                         'displacement_rotation', 'force', 'moment']
        for unit_type in required_units:
            assert unit_type in data['units']

        # 7. Analysis status
        assert 'success' in data
        assert isinstance(data['success'], bool)
