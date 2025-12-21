"""MCP (Model Context Protocol) tool definitions for LLM integration.

This module provides JSON Schema tool definitions that enable LLMs to
interact with Grillex structural models through a standardized interface.

The tools follow the MCP specification and can be used with various
LLM frameworks including Anthropic's Claude, OpenAI's function calling,
and other compatible systems.

Usage:
    from grillex.llm.tools import TOOLS, execute_tool

    # Get tool definitions for LLM
    tool_defs = TOOLS

    # Execute a tool call
    result = execute_tool(model, "create_beam", {
        "start_position": [0, 0, 0],
        "end_position": [6, 0, 0],
        "section": "IPE300",
        "material": "Steel"
    })
"""

from typing import Any, Dict, List, Optional, Union
from dataclasses import dataclass

from grillex.core import StructuralModel, DOFIndex, LoadCaseType


# =============================================================================
# Tool Result Types
# =============================================================================

@dataclass
class ToolResult:
    """Result of a tool execution.

    Attributes:
        success: Whether the tool executed successfully.
        result: The result data (varies by tool).
        error: Error message if success is False.
        suggestion: Suggested fix if there was an error.
    """
    success: bool
    result: Optional[Any] = None
    error: Optional[str] = None
    suggestion: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        d = {"success": self.success}
        if self.result is not None:
            d["result"] = self.result
        if self.error is not None:
            d["error"] = self.error
        if self.suggestion is not None:
            d["suggestion"] = self.suggestion
        return d


# =============================================================================
# Tool Definitions (JSON Schema)
# =============================================================================

TOOLS: List[Dict[str, Any]] = [
    # =========================================================================
    # Model Creation
    # =========================================================================
    {
        "name": "create_model",
        "description": "Create a new structural model. This should be called first before adding any elements.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the model (e.g., 'Bridge Deck', 'Barge Structure')"
                },
                "node_tolerance": {
                    "type": "number",
                    "description": "Tolerance for node merging in meters. Default 1e-6 (1 micrometer)."
                }
            },
            "required": ["name"]
        }
    },

    # =========================================================================
    # Materials and Sections
    # =========================================================================
    {
        "name": "add_material",
        "description": "Add a material to the model library. Must be called before creating beams that use this material.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Material name (e.g., 'Steel', 'Aluminum')"
                },
                "E": {
                    "type": "number",
                    "description": "Young's modulus in kN/m². Steel is typically 210000000 (210 GPa)."
                },
                "nu": {
                    "type": "number",
                    "description": "Poisson's ratio (dimensionless). Steel is typically 0.3."
                },
                "rho": {
                    "type": "number",
                    "description": "Density in mT/m³ (metric tonnes per cubic meter). Steel is 7.85e-3."
                }
            },
            "required": ["name", "E", "nu", "rho"]
        }
    },
    {
        "name": "add_section",
        "description": "Add a cross-section to the model library. Must be called before creating beams that use this section.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Section name (e.g., 'IPE300', 'HEB200')"
                },
                "A": {
                    "type": "number",
                    "description": "Cross-sectional area in m²"
                },
                "Iy": {
                    "type": "number",
                    "description": "Second moment of area about local y-axis (strong axis) in m⁴"
                },
                "Iz": {
                    "type": "number",
                    "description": "Second moment of area about local z-axis (weak axis) in m⁴"
                },
                "J": {
                    "type": "number",
                    "description": "Torsional constant (St. Venant) in m⁴"
                }
            },
            "required": ["name", "A", "Iy", "Iz", "J"]
        }
    },

    # =========================================================================
    # Beam Creation
    # =========================================================================
    {
        "name": "create_beam",
        "description": "Create a beam element between two points. Material and section must exist in the model.",
        "input_schema": {
            "type": "object",
            "properties": {
                "start_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Start point [x, y, z] in meters"
                },
                "end_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "End point [x, y, z] in meters"
                },
                "section": {
                    "type": "string",
                    "description": "Name of section to use (must exist in model)"
                },
                "material": {
                    "type": "string",
                    "description": "Name of material to use (must exist in model)"
                }
            },
            "required": ["start_position", "end_position", "section", "material"]
        }
    },

    # =========================================================================
    # Boundary Conditions
    # =========================================================================
    {
        "name": "fix_node",
        "description": "Fix all 6 degrees of freedom at a node (fully fixed support). Use for cantilever roots or fully restrained connections.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters. Must match an existing node."
                }
            },
            "required": ["position"]
        }
    },
    {
        "name": "pin_node",
        "description": "Pin a node (fix translations only, rotations free). Use for simple supports that allow rotation.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters. Must match an existing node."
                }
            },
            "required": ["position"]
        }
    },
    {
        "name": "fix_dof",
        "description": "Fix a specific degree of freedom at a node. Use for partial restraints like roller supports.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters"
                },
                "dof": {
                    "type": "string",
                    "enum": ["UX", "UY", "UZ", "RX", "RY", "RZ"],
                    "description": "Degree of freedom to fix: UX/UY/UZ (translations), RX/RY/RZ (rotations)"
                },
                "value": {
                    "type": "number",
                    "description": "Prescribed value. Default 0.0 for zero displacement."
                }
            },
            "required": ["position", "dof"]
        }
    },

    # =========================================================================
    # Loads
    # =========================================================================
    {
        "name": "add_point_load",
        "description": "Apply a concentrated force or moment at a node.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters"
                },
                "dof": {
                    "type": "string",
                    "enum": ["UX", "UY", "UZ", "RX", "RY", "RZ"],
                    "description": "DOF for load direction: UX/UY/UZ for forces, RX/RY/RZ for moments"
                },
                "value": {
                    "type": "number",
                    "description": "Load magnitude in kN (forces) or kN·m (moments). Negative = opposite direction."
                }
            },
            "required": ["position", "dof", "value"]
        }
    },
    {
        "name": "add_line_load",
        "description": "Apply a distributed load along a beam. The load is specified in global coordinates.",
        "input_schema": {
            "type": "object",
            "properties": {
                "beam_start": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Start position of beam [x, y, z] in meters"
                },
                "beam_end": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "End position of beam [x, y, z] in meters"
                },
                "load_start": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Load intensity at start [wx, wy, wz] in kN/m. Negative Z = downward."
                },
                "load_end": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Load intensity at end [wx, wy, wz] in kN/m. If omitted, uses load_start (uniform)."
                }
            },
            "required": ["beam_start", "beam_end", "load_start"]
        }
    },

    # =========================================================================
    # Analysis
    # =========================================================================
    {
        "name": "analyze",
        "description": "Run linear static analysis on the model. Must be called after adding all elements, BCs, and loads.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },

    # =========================================================================
    # Results
    # =========================================================================
    {
        "name": "get_displacement",
        "description": "Get displacement or rotation at a node after analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters"
                },
                "dof": {
                    "type": "string",
                    "enum": ["UX", "UY", "UZ", "RX", "RY", "RZ"],
                    "description": "Degree of freedom: UX/UY/UZ (displacements in m), RX/RY/RZ (rotations in rad)"
                }
            },
            "required": ["position", "dof"]
        }
    },
    {
        "name": "get_reactions",
        "description": "Get reaction forces and moments at a supported node after analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters (must be a supported node)"
                }
            },
            "required": ["position"]
        }
    },
    {
        "name": "get_internal_actions",
        "description": "Get internal forces and moments at a position along a beam after analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "beam_start": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Start position of beam [x, y, z]"
                },
                "beam_end": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "End position of beam [x, y, z]"
                },
                "position_along_beam": {
                    "type": "number",
                    "description": "Position along beam in meters from start (0 = start, L = end)"
                }
            },
            "required": ["beam_start", "beam_end", "position_along_beam"]
        }
    },

    # =========================================================================
    # Model Information
    # =========================================================================
    {
        "name": "get_model_info",
        "description": "Get summary information about the current model state.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
]


# =============================================================================
# Tool Execution
# =============================================================================

def _parse_dof(dof_str: str) -> DOFIndex:
    """Parse DOF string to DOFIndex enum."""
    dof_map = {
        "UX": DOFIndex.UX,
        "UY": DOFIndex.UY,
        "UZ": DOFIndex.UZ,
        "RX": DOFIndex.RX,
        "RY": DOFIndex.RY,
        "RZ": DOFIndex.RZ,
    }
    return dof_map[dof_str.upper()]


class ToolExecutor:
    """Executes tool calls on a StructuralModel.

    This class maintains a reference to a model and executes tool calls,
    returning structured results suitable for LLM consumption.

    Example:
        model = StructuralModel("Test")
        executor = ToolExecutor(model)

        result = executor.execute("add_material", {
            "name": "Steel",
            "E": 210e6,
            "nu": 0.3,
            "rho": 7.85e-3
        })
    """

    def __init__(self, model: Optional[StructuralModel] = None):
        """Initialize the tool executor.

        Args:
            model: Optional StructuralModel to operate on. Can be set later
                   or created via create_model tool.
        """
        self.model = model

    def execute(self, tool_name: str, params: Dict[str, Any]) -> ToolResult:
        """Execute a tool by name with given parameters.

        Args:
            tool_name: Name of the tool to execute.
            params: Parameters for the tool.

        Returns:
            ToolResult with success status and result/error.
        """
        try:
            method = getattr(self, f"_tool_{tool_name}", None)
            if method is None:
                return ToolResult(
                    success=False,
                    error=f"Unknown tool: {tool_name}",
                    suggestion=f"Available tools: {[t['name'] for t in TOOLS]}"
                )
            return method(params)
        except Exception as e:
            return ToolResult(
                success=False,
                error=str(e),
                suggestion=self._get_suggestion_for_error(tool_name, e)
            )

    def _get_suggestion_for_error(self, tool_name: str, error: Exception) -> Optional[str]:
        """Get a suggestion for fixing an error."""
        error_str = str(error).lower()

        if "material" in error_str and "not found" in error_str:
            return "Add the material first using add_material tool."
        if "section" in error_str and "not found" in error_str:
            return "Add the section first using add_section tool."
        if "no node found" in error_str:
            return "The specified position doesn't match any node. Check coordinates or create a beam that passes through this point."
        if "not analyzed" in error_str:
            return "Run the analyze tool first before querying results."
        if "singular" in error_str or "unconstrained" in error_str:
            return "Add boundary conditions to prevent rigid body motion. At minimum, fix one node completely."

        return None

    def _tool_create_model(self, params: Dict[str, Any]) -> ToolResult:
        """Create a new model."""
        name = params.get("name", "Unnamed Model")
        tolerance = params.get("node_tolerance", 1e-6)
        self.model = StructuralModel(name=name, node_tolerance=tolerance)
        return ToolResult(
            success=True,
            result={"model_name": name, "message": "Model created successfully"}
        )

    def _tool_add_material(self, params: Dict[str, Any]) -> ToolResult:
        """Add a material."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        self.model.add_material(
            name=params["name"],
            E=params["E"],
            nu=params["nu"],
            rho=params["rho"]
        )
        return ToolResult(
            success=True,
            result={"material": params["name"], "message": "Material added successfully"}
        )

    def _tool_add_section(self, params: Dict[str, Any]) -> ToolResult:
        """Add a section."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        self.model.add_section(
            name=params["name"],
            A=params["A"],
            Iy=params["Iy"],
            Iz=params["Iz"],
            J=params["J"]
        )
        return ToolResult(
            success=True,
            result={"section": params["name"], "message": "Section added successfully"}
        )

    def _tool_create_beam(self, params: Dict[str, Any]) -> ToolResult:
        """Create a beam."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        beam = self.model.add_beam_by_coords(
            start_pos=params["start_position"],
            end_pos=params["end_position"],
            section_name=params["section"],
            material_name=params["material"]
        )
        return ToolResult(
            success=True,
            result={
                "beam_id": beam.beam_id,
                "length": beam.length,
                "message": f"Beam created with length {beam.length:.3f} m"
            }
        )

    def _tool_fix_node(self, params: Dict[str, Any]) -> ToolResult:
        """Fix a node."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        self.model.fix_node_at(params["position"])
        return ToolResult(
            success=True,
            result={"position": params["position"], "message": "Node fixed (all DOFs)"}
        )

    def _tool_pin_node(self, params: Dict[str, Any]) -> ToolResult:
        """Pin a node."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        self.model.pin_node_at(params["position"])
        return ToolResult(
            success=True,
            result={"position": params["position"], "message": "Node pinned (translations fixed)"}
        )

    def _tool_fix_dof(self, params: Dict[str, Any]) -> ToolResult:
        """Fix a specific DOF."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        dof = _parse_dof(params["dof"])
        value = params.get("value", 0.0)
        self.model.fix_dof_at(params["position"], dof, value)
        return ToolResult(
            success=True,
            result={
                "position": params["position"],
                "dof": params["dof"],
                "value": value,
                "message": f"DOF {params['dof']} fixed to {value}"
            }
        )

    def _tool_add_point_load(self, params: Dict[str, Any]) -> ToolResult:
        """Add a point load."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        dof = _parse_dof(params["dof"])
        self.model.add_point_load(params["position"], dof, params["value"])

        unit = "kN·m" if params["dof"].startswith("R") else "kN"
        return ToolResult(
            success=True,
            result={
                "position": params["position"],
                "dof": params["dof"],
                "value": params["value"],
                "message": f"Point load of {params['value']} {unit} applied"
            }
        )

    def _tool_add_line_load(self, params: Dict[str, Any]) -> ToolResult:
        """Add a line load."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        load_end = params.get("load_end", params["load_start"])
        self.model.add_line_load_by_coords(
            start_pos=params["beam_start"],
            end_pos=params["beam_end"],
            w_start=params["load_start"],
            w_end=load_end
        )
        return ToolResult(
            success=True,
            result={
                "beam_start": params["beam_start"],
                "beam_end": params["beam_end"],
                "load_start": params["load_start"],
                "load_end": load_end,
                "message": "Line load applied to beam"
            }
        )

    def _tool_analyze(self, params: Dict[str, Any]) -> ToolResult:
        """Run analysis."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        success = self.model.analyze()
        if success:
            return ToolResult(
                success=True,
                result={
                    "analyzed": True,
                    "num_dofs": self.model.total_dofs(),
                    "message": "Analysis completed successfully"
                }
            )
        else:
            return ToolResult(
                success=False,
                error="Analysis failed. The system may be unstable or unconstrained.",
                suggestion="Check that boundary conditions prevent all rigid body motion (6 DOFs minimum)."
            )

    def _tool_get_displacement(self, params: Dict[str, Any]) -> ToolResult:
        """Get displacement at a node."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(success=False, error="Model not analyzed. Call analyze first.")

        dof = _parse_dof(params["dof"])
        value = self.model.get_displacement_at(params["position"], dof)

        unit = "rad" if params["dof"].startswith("R") else "m"
        return ToolResult(
            success=True,
            result={
                "position": params["position"],
                "dof": params["dof"],
                "value": value,
                "unit": unit,
                "message": f"{params['dof']} = {value:.6e} {unit}"
            }
        )

    def _tool_get_reactions(self, params: Dict[str, Any]) -> ToolResult:
        """Get reactions at a node."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(success=False, error="Model not analyzed. Call analyze first.")

        reactions = self.model.get_reactions_at(params["position"])

        # Convert DOFIndex keys to strings
        reactions_dict = {}
        for dof, value in reactions.items():
            dof_name = dof.name
            reactions_dict[dof_name] = value

        return ToolResult(
            success=True,
            result={
                "position": params["position"],
                "reactions": reactions_dict,
                "units": {"UX": "kN", "UY": "kN", "UZ": "kN", "RX": "kN·m", "RY": "kN·m", "RZ": "kN·m"}
            }
        )

    def _tool_get_internal_actions(self, params: Dict[str, Any]) -> ToolResult:
        """Get internal actions at a position along a beam."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(success=False, error="Model not analyzed. Call analyze first.")

        beam = self.model.find_beam_by_coords(params["beam_start"], params["beam_end"])
        if beam is None:
            return ToolResult(
                success=False,
                error=f"No beam found from {params['beam_start']} to {params['beam_end']}"
            )

        x = params["position_along_beam"]
        actions = beam.get_internal_actions_at(x, self.model)

        return ToolResult(
            success=True,
            result={
                "position": x,
                "N": actions.N,  # Axial force [kN]
                "Vy": actions.Vy,  # Shear Y [kN]
                "Vz": actions.Vz,  # Shear Z [kN]
                "Mx": actions.Mx,  # Torsion [kN·m]
                "My": actions.My,  # Bending Y [kN·m]
                "Mz": actions.Mz,  # Bending Z [kN·m]
                "units": {
                    "N": "kN", "Vy": "kN", "Vz": "kN",
                    "Mx": "kN·m", "My": "kN·m", "Mz": "kN·m"
                }
            }
        )

    def _tool_get_model_info(self, params: Dict[str, Any]) -> ToolResult:
        """Get model summary."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        return ToolResult(
            success=True,
            result={
                "name": self.model.name,
                "num_nodes": self.model.num_nodes(),
                "num_beams": self.model.num_beams(),
                "num_elements": self.model.num_elements(),
                "total_dofs": self.model.total_dofs(),
                "analyzed": self.model.is_analyzed(),
                "materials": list(self.model._materials.keys()),
                "sections": list(self.model._sections.keys())
            }
        )


def execute_tool(
    model: Optional[StructuralModel],
    tool_name: str,
    params: Dict[str, Any]
) -> ToolResult:
    """Convenience function to execute a tool on a model.

    Args:
        model: StructuralModel to operate on (can be None for create_model).
        tool_name: Name of the tool to execute.
        params: Tool parameters.

    Returns:
        ToolResult with execution result.
    """
    executor = ToolExecutor(model)
    return executor.execute(tool_name, params)


def get_tool_by_name(name: str) -> Optional[Dict[str, Any]]:
    """Get a tool definition by name.

    Args:
        name: Tool name.

    Returns:
        Tool definition dict or None if not found.
    """
    for tool in TOOLS:
        if tool["name"] == name:
            return tool
    return None
