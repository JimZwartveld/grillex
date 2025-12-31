"""Tests for Phase 12: LLM Tooling.

This module tests the MCP tool definitions, tool executor, and
self-healing diagnostics functionality.
"""

import pytest
from grillex.core import (
    StructuralModel,
    DOFIndex,
    ErrorCode,
    GrillexError,
    WarningCode,
    WarningSeverity,
    GrillexWarning,
)
from grillex.llm import (
    TOOLS,
    ToolResult,
    ToolExecutor,
    execute_tool,
    get_tool_by_name,
    FixSuggestion,
    get_fix_suggestions,
    get_warning_advice,
    analyze_model_health,
)


class TestToolDefinitions:
    """Tests for MCP tool definitions."""

    def test_tools_is_list(self):
        """TOOLS should be a list of tool definitions."""
        assert isinstance(TOOLS, list)
        assert len(TOOLS) > 0

    def test_tool_has_required_fields(self):
        """Each tool should have name, description, and input_schema."""
        for tool in TOOLS:
            assert "name" in tool
            assert "description" in tool
            assert "input_schema" in tool
            assert isinstance(tool["name"], str)
            assert isinstance(tool["description"], str)
            assert isinstance(tool["input_schema"], dict)

    def test_tool_schema_has_type_and_properties(self):
        """Each tool schema should have type and properties."""
        for tool in TOOLS:
            schema = tool["input_schema"]
            assert schema.get("type") == "object"
            assert "properties" in schema
            assert "required" in schema

    def test_get_tool_by_name(self):
        """Should find tool by name."""
        tool = get_tool_by_name("create_model")
        assert tool is not None
        assert tool["name"] == "create_model"

    def test_get_tool_by_name_not_found(self):
        """Should return None for unknown tool."""
        tool = get_tool_by_name("nonexistent_tool")
        assert tool is None

    def test_essential_tools_exist(self):
        """Essential tools should exist."""
        essential_tools = [
            "create_model",
            "add_material",
            "add_section",
            "create_beam",
            "fix_node",
            "pin_node",
            "fix_dof",
            "add_point_load",
            "analyze",
            "get_displacement",
            "get_reactions",
            "get_model_info",
        ]
        for tool_name in essential_tools:
            tool = get_tool_by_name(tool_name)
            assert tool is not None, f"Tool '{tool_name}' not found"


class TestToolResult:
    """Tests for ToolResult dataclass."""

    def test_success_result(self):
        """Should create success result."""
        result = ToolResult(success=True, result={"key": "value"})
        assert result.success is True
        assert result.result == {"key": "value"}
        assert result.error is None

    def test_error_result(self):
        """Should create error result."""
        result = ToolResult(success=False, error="Something went wrong")
        assert result.success is False
        assert result.error == "Something went wrong"

    def test_result_with_suggestion(self):
        """Should include suggestion."""
        result = ToolResult(
            success=False,
            error="Material not found",
            suggestion="Add the material first"
        )
        assert result.suggestion == "Add the material first"

    def test_to_dict(self):
        """Should convert to dictionary."""
        result = ToolResult(success=True, result={"data": 123})
        d = result.to_dict()
        assert d["success"] is True
        assert d["result"] == {"data": 123}


class TestToolExecutor:
    """Tests for ToolExecutor."""

    def test_create_model(self):
        """Should create a new model."""
        executor = ToolExecutor()
        result = executor.execute("create_model", {"name": "Test Model"})

        assert result.success is True
        assert executor.model is not None
        assert executor.model.name == "Test Model"

    def test_add_material(self):
        """Should add material to model."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Test"})

        result = executor.execute("add_material", {
            "name": "Steel",
            "E": 210e6,
            "nu": 0.3,
            "rho": 7.85e-3
        })

        assert result.success is True
        assert executor.model.get_material("Steel") is not None

    def test_add_section(self):
        """Should add section to model."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Test"})

        result = executor.execute("add_section", {
            "name": "IPE300",
            "A": 0.00538,
            "Iy": 8.36e-5,
            "Iz": 6.04e-6,
            "J": 2.01e-7
        })

        assert result.success is True
        assert executor.model.get_section("IPE300") is not None

    def test_create_beam(self):
        """Should create beam element."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Test"})
        executor.execute("add_material", {"name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3})
        executor.execute("add_section", {"name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7})

        result = executor.execute("create_beam", {
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        assert result.success is True
        assert executor.model.num_beams() == 1

    def test_fix_node(self):
        """Should fix node."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Test"})
        executor.execute("add_material", {"name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3})
        executor.execute("add_section", {"name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7})
        executor.execute("create_beam", {
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        result = executor.execute("fix_node", {"position": [0, 0, 0]})
        assert result.success is True

    def test_add_point_load(self):
        """Should add point load."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Test"})
        executor.execute("add_material", {"name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3})
        executor.execute("add_section", {"name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7})
        executor.execute("create_beam", {
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        result = executor.execute("add_point_load", {
            "position": [6, 0, 0],
            "force": [0, 0, -10.0]
        })
        assert result.success is True

    def test_analyze_and_get_results(self):
        """Should analyze and get results."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Cantilever"})
        executor.execute("add_material", {"name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3})
        executor.execute("add_section", {"name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7})
        executor.execute("create_beam", {
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })
        executor.execute("fix_node", {"position": [0, 0, 0]})
        executor.execute("add_point_load", {"position": [6, 0, 0], "force": [0, 0, -10.0]})

        # Analyze
        result = executor.execute("analyze", {})
        assert result.success is True

        # Get displacement
        result = executor.execute("get_displacement", {
            "position": [6, 0, 0],
            "dof": "UZ"
        })
        assert result.success is True
        assert "value" in result.result
        assert result.result["value"] < 0  # Downward displacement

        # Get reactions
        result = executor.execute("get_reactions", {"position": [0, 0, 0]})
        assert result.success is True
        assert "reactions" in result.result

    def test_get_model_info(self):
        """Should get model info."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "Test"})
        executor.execute("add_material", {"name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3})
        executor.execute("add_section", {"name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7})
        executor.execute("create_beam", {
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        result = executor.execute("get_model_info", {})
        assert result.success is True
        assert result.result["name"] == "Test"
        assert result.result["num_beams"] == 1
        assert "Steel" in result.result["materials"]
        assert "IPE300" in result.result["sections"]

    def test_error_no_model(self):
        """Should error if no model created."""
        executor = ToolExecutor()
        result = executor.execute("add_material", {"name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3})

        assert result.success is False
        assert "No model created" in result.error

    def test_error_unknown_tool(self):
        """Should error for unknown tool."""
        executor = ToolExecutor()
        result = executor.execute("nonexistent_tool", {})

        assert result.success is False
        assert "Unknown tool" in result.error


class TestExecuteTool:
    """Tests for execute_tool convenience function."""

    def test_execute_tool_creates_model(self):
        """Should execute tool without pre-existing model."""
        result = execute_tool(None, "create_model", {"name": "Quick Test"})
        assert result.success is True


class TestFixSuggestion:
    """Tests for FixSuggestion dataclass."""

    def test_fix_suggestion_creation(self):
        """Should create fix suggestion."""
        suggestion = FixSuggestion(
            description="Fix the node",
            tool_name="fix_node",
            tool_params={"position": [0, 0, 0]},
            priority=1,
            confidence=0.9
        )
        assert suggestion.description == "Fix the node"
        assert suggestion.tool_name == "fix_node"
        assert suggestion.priority == 1
        assert suggestion.confidence == 0.9

    def test_to_dict(self):
        """Should convert to dictionary."""
        suggestion = FixSuggestion(
            description="Add material",
            tool_name="add_material",
            tool_params={"name": "Steel"}
        )
        d = suggestion.to_dict()
        assert d["description"] == "Add material"
        assert d["tool"] == "add_material"
        assert d["params"] == {"name": "Steel"}


class TestGetFixSuggestions:
    """Tests for get_fix_suggestions function."""

    def test_ok_error_returns_empty(self):
        """OK error should return no suggestions."""
        error = GrillexError()  # Default is OK
        suggestions = get_fix_suggestions(error)
        assert len(suggestions) == 0

    def test_unconstrained_system_suggestions(self):
        """Should suggest fixing nodes for unconstrained system."""
        error = GrillexError.unconstrained([0, 1, 2, 3, 4, 5], [0])
        suggestions = get_fix_suggestions(error)

        assert len(suggestions) > 0
        # Should suggest fix_node tool
        tool_names = [s.tool_name for s in suggestions]
        assert "fix_node" in tool_names

    def test_singular_matrix_suggestions(self):
        """Should suggest fixes for singular matrix."""
        error = GrillexError.singular("Zero pivot")
        suggestions = get_fix_suggestions(error)

        assert len(suggestions) > 0
        # Should include fix_node and get_model_info
        tool_names = [s.tool_name for s in suggestions]
        assert "fix_node" in tool_names

    def test_not_analyzed_suggestions(self):
        """Should suggest running analysis."""
        error = GrillexError.not_analyzed()
        suggestions = get_fix_suggestions(error)

        assert len(suggestions) == 1
        assert suggestions[0].tool_name == "analyze"
        assert suggestions[0].confidence == 1.0

    def test_empty_model_suggestions(self):
        """Should suggest adding elements."""
        error = GrillexError.empty_model()
        suggestions = get_fix_suggestions(error)

        assert len(suggestions) > 0
        tool_names = [s.tool_name for s in suggestions]
        assert "create_beam" in tool_names or "add_material" in tool_names

    def test_suggestions_sorted_by_priority(self):
        """Suggestions should be sorted by priority."""
        error = GrillexError.unconstrained([0, 1, 2], [0])
        suggestions = get_fix_suggestions(error)

        priorities = [s.priority for s in suggestions]
        assert priorities == sorted(priorities)


class TestGetWarningAdvice:
    """Tests for get_warning_advice function."""

    def test_extreme_aspect_ratio_advice(self):
        """Should give advice for extreme aspect ratio."""
        warning = GrillexWarning.extreme_aspect_ratio(1, 150.0)
        advice = get_warning_advice(warning)

        assert "aspect ratio" in advice.lower()
        assert len(advice) > 20

    def test_small_element_advice(self):
        """Should give advice for small element."""
        warning = GrillexWarning.small_element(1, 0.001)
        advice = get_warning_advice(warning)

        assert "short" in advice.lower() or "small" in advice.lower()

    def test_stiffness_contrast_advice(self):
        """Should give advice for stiffness contrast."""
        warning = GrillexWarning.stiffness_contrast(1, 2, 1e8)
        advice = get_warning_advice(warning)

        assert "stiffness" in advice.lower()

    def test_near_zero_property_advice(self):
        """Should give advice for near-zero property."""
        warning = GrillexWarning.near_zero_property(1, "Area", 1e-15)
        advice = get_warning_advice(warning)

        assert "zero" in advice.lower() or "property" in advice.lower()


class TestAnalyzeModelHealth:
    """Tests for analyze_model_health function."""

    def test_healthy_model(self):
        """Should report OK for no errors/warnings."""
        result = analyze_model_health([], [])

        assert result["status"] == "OK"
        assert result["error_count"] == 0
        assert result["warning_counts"]["high"] == 0

    def test_model_with_errors(self):
        """Should report ERROR for errors."""
        errors = [GrillexError.singular()]
        result = analyze_model_health(errors, [])

        assert result["status"] == "ERROR"
        assert result["error_count"] == 1

    def test_model_with_high_warnings(self):
        """Should report WARNING for high-severity warnings."""
        warnings = [GrillexWarning.stiffness_contrast(1, 2, 1e8)]
        result = analyze_model_health([], warnings)

        assert result["status"] == "WARNING"
        assert result["warning_counts"]["high"] == 1

    def test_model_with_medium_warnings(self):
        """Should report CAUTION for medium-severity warnings."""
        warnings = [GrillexWarning.extreme_aspect_ratio(1, 120)]
        result = analyze_model_health([], warnings)

        assert result["status"] == "CAUTION"
        assert result["warning_counts"]["medium"] == 1

    def test_recommendations_included(self):
        """Should include recommendations."""
        errors = [GrillexError.unconstrained([0, 1, 2], [0])]
        result = analyze_model_health(errors, [])

        assert len(result["recommendations"]) > 0


class TestIntegration:
    """Integration tests for LLM tooling."""

    def test_full_workflow(self):
        """Test complete modeling workflow."""
        executor = ToolExecutor()

        # Create model
        result = executor.execute("create_model", {"name": "Cantilever Beam"})
        assert result.success

        # Add material
        result = executor.execute("add_material", {
            "name": "Steel",
            "E": 210000000,
            "nu": 0.3,
            "rho": 7.85e-3
        })
        assert result.success

        # Add section
        result = executor.execute("add_section", {
            "name": "HEB200",
            "A": 0.00781,
            "Iy": 5.70e-5,
            "Iz": 2.00e-5,
            "J": 5.94e-7
        })
        assert result.success

        # Create beam
        result = executor.execute("create_beam", {
            "start_position": [0, 0, 0],
            "end_position": [4, 0, 0],
            "section": "HEB200",
            "material": "Steel"
        })
        assert result.success
        beam_length = result.result["length"]
        assert abs(beam_length - 4.0) < 0.001

        # Fix support
        result = executor.execute("fix_node", {"position": [0, 0, 0]})
        assert result.success

        # Add tip load
        result = executor.execute("add_point_load", {
            "position": [4, 0, 0],
            "force": [0, 0, -50.0]
        })
        assert result.success

        # Analyze
        result = executor.execute("analyze", {})
        assert result.success

        # Check displacement
        result = executor.execute("get_displacement", {
            "position": [4, 0, 0],
            "dof": "UZ"
        })
        assert result.success
        displacement = result.result["value"]
        assert displacement < 0  # Downward

        # Check reactions
        result = executor.execute("get_reactions", {"position": [0, 0, 0]})
        assert result.success
        reactions = result.result["reactions"]
        assert abs(reactions["UZ"] - 50.0) < 0.01  # Reaction equals load

    def test_error_recovery_workflow(self):
        """Test that error suggestions can fix issues."""
        executor = ToolExecutor()

        # Try to add material without creating model first
        result = executor.execute("add_material", {
            "name": "Steel",
            "E": 210e6,
            "nu": 0.3,
            "rho": 7.85e-3
        })
        assert result.success is False

        # Create model to fix the issue
        result = executor.execute("create_model", {"name": "Fixed Model"})
        assert result.success

        # Now adding material should work
        result = executor.execute("add_material", {
            "name": "Steel",
            "E": 210e6,
            "nu": 0.3,
            "rho": 7.85e-3
        })
        assert result.success


# Skip plate tests if gmsh is not installed
gmsh = pytest.importorskip("gmsh")


class TestPlateMeshingTools:
    """Tests for plate meshing LLM tools."""

    def test_plate_meshing_tools_exist(self):
        """All plate meshing tools should be defined."""
        tool_names = [t["name"] for t in TOOLS]

        expected = [
            "add_plate",
            "set_edge_divisions",
            "couple_plate_to_beam",
            "add_support_curve",
            "mesh_model",
            "get_plate_displacement",
            "get_plate_moments",
            "get_plate_stress"
        ]

        for name in expected:
            assert name in tool_names, f"Missing tool: {name}"

    def test_add_plate_tool(self):
        """Test add_plate tool."""
        executor = ToolExecutor()

        # Create model and add material
        executor.execute("create_model", {"name": "PlateTest"})
        executor.execute("add_material", {
            "name": "Steel",
            "E": 210e6,
            "nu": 0.3,
            "rho": 7.85e-3
        })

        # Add a plate
        result = executor.execute("add_plate", {
            "corners": [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            "thickness": 0.02,
            "material": "Steel",
            "mesh_size": 0.5,
            "element_type": "MITC4"
        })

        assert result.success
        assert "plate_name" in result.result
        assert result.result["n_corners"] == 4
        assert result.result["element_type"] == "MITC4"

    def test_add_plate_invalid_material(self):
        """Test add_plate with invalid material."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "PlateTest"})

        result = executor.execute("add_plate", {
            "corners": [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            "thickness": 0.02,
            "material": "NonExistent"
        })

        assert not result.success
        assert "not found" in result.error.lower()

    def test_set_edge_divisions_tool(self):
        """Test set_edge_divisions tool."""
        executor = ToolExecutor()

        # Create model with plate
        executor.execute("create_model", {"name": "EdgeDivTest"})
        executor.execute("add_material", {
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        plate_result = executor.execute("add_plate", {
            "corners": [[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            "thickness": 0.02,
            "material": "Steel",
            "name": "TestPlate"
        })
        plate_name = plate_result.result["plate_name"]

        # Set edge divisions
        result = executor.execute("set_edge_divisions", {
            "plate_name": plate_name,
            "edge_index": 0,
            "n_elements": 4
        })

        assert result.success
        assert result.result["n_elements"] == 4

    def test_add_support_curve_tool(self):
        """Test add_support_curve tool."""
        executor = ToolExecutor()

        # Create model with plate
        executor.execute("create_model", {"name": "SupportTest"})
        executor.execute("add_material", {
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        plate_result = executor.execute("add_plate", {
            "corners": [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            "thickness": 0.02,
            "material": "Steel",
            "name": "SupportPlate"
        })
        plate_name = plate_result.result["plate_name"]

        # Add support on edge
        result = executor.execute("add_support_curve", {
            "plate_name": plate_name,
            "edge_index": 0,
            "uz": True
        })

        assert result.success
        assert "UZ" in result.result["restrained_dofs"]

    def test_mesh_model_tool(self):
        """Test mesh_model tool."""
        executor = ToolExecutor()

        # Create model with plate
        executor.execute("create_model", {"name": "MeshTest"})
        executor.execute("add_material", {
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        executor.execute("add_plate", {
            "corners": [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            "thickness": 0.02,
            "material": "Steel"
        })

        # Mesh the model
        result = executor.execute("mesh_model", {})

        assert result.success
        assert result.result["n_plate_elements"] > 0
        assert "n_quad_elements" in result.result
        assert "n_tri_elements" in result.result

    def test_plate_not_found_error(self):
        """Test error when plate not found."""
        executor = ToolExecutor()
        executor.execute("create_model", {"name": "NotFoundTest"})

        result = executor.execute("set_edge_divisions", {
            "plate_name": "NonExistentPlate",
            "edge_index": 0,
            "n_elements": 4
        })

        assert not result.success
        assert "not found" in result.error.lower()
        assert result.suggestion is not None
