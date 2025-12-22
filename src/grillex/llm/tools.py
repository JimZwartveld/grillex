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
    {
        "name": "analyze_modes",
        "description": "Perform eigenvalue analysis to find natural frequencies and mode shapes. Use this to check for resonance, understand dynamic behavior, or prepare for response spectrum analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "n_modes": {
                    "type": "integer",
                    "description": "Number of modes to compute (lowest frequencies first). Default 10.",
                    "default": 10
                },
                "method": {
                    "type": "string",
                    "enum": ["dense", "subspace", "shift_invert"],
                    "description": "Solver method. Use 'subspace' for large models (>500 DOFs). Default 'subspace'.",
                    "default": "subspace"
                }
            },
            "required": []
        }
    },
    {
        "name": "get_modal_summary",
        "description": "Get summary of modal analysis results including natural frequencies, periods, and participation factors. Must run analyze_modes first.",
        "input_schema": {
            "type": "object",
            "properties": {
                "n_modes_to_show": {
                    "type": "integer",
                    "description": "Number of modes to include in summary. Default shows all computed modes.",
                    "default": 10
                }
            },
            "required": []
        }
    },
    {
        "name": "check_resonance",
        "description": "Check if any natural frequency is within a specified range of an excitation frequency. Use this to identify potential resonance issues.",
        "input_schema": {
            "type": "object",
            "properties": {
                "excitation_frequency": {
                    "type": "number",
                    "description": "Excitation frequency to check against [Hz]"
                },
                "tolerance_percent": {
                    "type": "number",
                    "description": "Percentage band around excitation frequency. Default 15%.",
                    "default": 15
                }
            },
            "required": ["excitation_frequency"]
        }
    },
    {
        "name": "get_mode_shape",
        "description": "Get mode shape displacements at a specific node for a given mode number.",
        "input_schema": {
            "type": "object",
            "properties": {
                "mode_number": {
                    "type": "integer",
                    "description": "Mode number (1-based, 1 = fundamental mode)"
                },
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Node position [x, y, z] in meters"
                }
            },
            "required": ["mode_number", "position"]
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

        # Eigenvalue-specific errors
        if "eigenvalue" in error_str or "modal" in error_str:
            if "mass" in error_str and ("singular" in error_str or "zero" in error_str):
                return "The mass matrix is singular. Ensure material density (rho) is non-zero and all elements have valid sections."
            if "convergence" in error_str:
                return "Eigenvalue solver did not converge. Try increasing the number of iterations or using a different method (e.g., 'subspace')."
            if "not computed" in error_str or "not available" in error_str:
                return "Run analyze_modes first to compute natural frequencies and mode shapes."
        if "mode" in error_str and "not found" in error_str:
            return "The requested mode number is out of range. Check how many modes were computed."

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

    def _tool_analyze_modes(self, params: Dict[str, Any]) -> ToolResult:
        """Run eigenvalue analysis."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        n_modes = params.get("n_modes", 10)
        method = params.get("method", "subspace")

        success = self.model.analyze_modes(n_modes=n_modes)

        if success:
            frequencies = self.model.get_natural_frequencies()
            periods = self.model.get_periods()

            # Build results summary
            modes_summary = []
            for i, (f, t) in enumerate(zip(frequencies, periods), 1):
                modes_summary.append({
                    "mode": i,
                    "frequency_hz": round(f, 4),
                    "period_s": round(t, 6) if t < 1e10 else "inf"
                })

            return ToolResult(
                success=True,
                result={
                    "n_modes_computed": len(frequencies),
                    "fundamental_frequency_hz": round(frequencies[0], 4) if frequencies else None,
                    "modes": modes_summary[:min(5, len(modes_summary))],  # Show first 5
                    "message": f"Computed {len(frequencies)} modes. Fundamental frequency: {frequencies[0]:.2f} Hz"
                }
            )
        else:
            return ToolResult(
                success=False,
                error="Eigenvalue analysis failed.",
                suggestion="Check boundary conditions and ensure material density (rho) is non-zero."
            )

    def _tool_get_modal_summary(self, params: Dict[str, Any]) -> ToolResult:
        """Get modal analysis summary."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        frequencies = self.model.get_natural_frequencies()
        if not frequencies:
            return ToolResult(
                success=False,
                error="No modal analysis results available.",
                suggestion="Run analyze_modes first to compute natural frequencies and mode shapes."
            )

        periods = self.model.get_periods()
        n_modes_to_show = params.get("n_modes_to_show", len(frequencies))

        # Build mode table
        modes = []
        for i in range(min(n_modes_to_show, len(frequencies))):
            mode_info = {
                "mode": i + 1,
                "frequency_hz": round(frequencies[i], 4),
                "period_s": round(periods[i], 6) if periods[i] < 1e10 else "inf"
            }
            modes.append(mode_info)

        # Identify frequency ranges
        positive_freqs = [f for f in frequencies if f > 0.1]
        freq_range = {
            "min_hz": round(min(positive_freqs), 4) if positive_freqs else 0,
            "max_hz": round(max(frequencies[:n_modes_to_show]), 4)
        }

        return ToolResult(
            success=True,
            result={
                "n_modes": len(frequencies),
                "frequency_range": freq_range,
                "modes": modes,
                "units": {"frequency": "Hz", "period": "s"}
            }
        )

    def _tool_check_resonance(self, params: Dict[str, Any]) -> ToolResult:
        """Check for resonance with an excitation frequency."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        frequencies = self.model.get_natural_frequencies()
        if not frequencies:
            return ToolResult(
                success=False,
                error="No modal analysis results available.",
                suggestion="Run analyze_modes first."
            )

        excitation_freq = params["excitation_frequency"]
        tolerance_pct = params.get("tolerance_percent", 15)

        # Calculate frequency band
        f_low = excitation_freq * (1 - tolerance_pct / 100)
        f_high = excitation_freq * (1 + tolerance_pct / 100)

        # Find modes within the band
        resonant_modes = []
        for i, f in enumerate(frequencies, 1):
            if f_low <= f <= f_high:
                resonant_modes.append({
                    "mode": i,
                    "frequency_hz": round(f, 4),
                    "ratio": round(f / excitation_freq, 3)
                })

        has_resonance = len(resonant_modes) > 0

        return ToolResult(
            success=True,
            result={
                "excitation_frequency_hz": excitation_freq,
                "tolerance_percent": tolerance_pct,
                "frequency_band": {"low_hz": round(f_low, 4), "high_hz": round(f_high, 4)},
                "has_resonance_risk": has_resonance,
                "resonant_modes": resonant_modes,
                "message": (
                    f"WARNING: {len(resonant_modes)} mode(s) within {tolerance_pct}% of {excitation_freq} Hz"
                    if has_resonance else
                    f"No modes within {tolerance_pct}% of {excitation_freq} Hz - OK"
                )
            }
        )

    def _tool_get_mode_shape(self, params: Dict[str, Any]) -> ToolResult:
        """Get mode shape at a position."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        frequencies = self.model.get_natural_frequencies()
        if not frequencies:
            return ToolResult(
                success=False,
                error="No modal analysis results available.",
                suggestion="Run analyze_modes first."
            )

        mode_number = params["mode_number"]
        position = params["position"]

        if mode_number < 1 or mode_number > len(frequencies):
            return ToolResult(
                success=False,
                error=f"Mode {mode_number} not found. Only {len(frequencies)} modes were computed.",
                suggestion=f"Use mode_number between 1 and {len(frequencies)}."
            )

        mode_disp = self.model.get_mode_displacement_at(mode_number, position)

        return ToolResult(
            success=True,
            result={
                "mode_number": mode_number,
                "frequency_hz": round(frequencies[mode_number - 1], 4),
                "position": position,
                "displacements": {
                    "UX": float(mode_disp["UX"]),
                    "UY": float(mode_disp["UY"]),
                    "UZ": float(mode_disp["UZ"]),
                    "RX": float(mode_disp["RX"]),
                    "RY": float(mode_disp["RY"]),
                    "RZ": float(mode_disp["RZ"])
                },
                "units": {"translations": "normalized", "rotations": "normalized"}
            }
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
