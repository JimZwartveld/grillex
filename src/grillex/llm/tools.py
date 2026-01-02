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

from typing import Any, Dict, List, Optional
from dataclasses import dataclass

from grillex.core import StructuralModel, DOFIndex, LoadCaseType, SpringBehavior, LoadingCondition


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
                    "description": "Density in mT/m³ (metric tonnes per cubic meter). Steel is 7.85."
                },
                "fy": {
                    "type": "number",
                    "description": "Yield stress in kN/m². Optional. Steel S355 is typically 355000."
                },
                "fu": {
                    "type": "number",
                    "description": "Ultimate tensile strength in kN/m². Optional. Steel S355 is typically 470000-630000."
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
                "name": {
                    "type": "string",
                    "description": "Optional name for the beam (e.g., 'Main Girder', 'Cross Beam 1')"
                },
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
                },
                "subdivisions": {
                    "type": "integer",
                    "description": "Number of FEM elements to subdivide the beam into (default: 1)",
                    "minimum": 1,
                    "default": 1
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
        "description": "Apply a concentrated force and/or moment at a position.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Position [x, y, z] in meters"
                },
                "force": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Force vector [Fx, Fy, Fz] in kN. Default: [0, 0, 0]"
                },
                "moment": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Moment vector [Mx, My, Mz] in kNm. Default: [0, 0, 0]"
                }
            },
            "required": ["position"]
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
                },
                "load_case_id": {
                    "type": "integer",
                    "description": "Optional load case ID. If not specified, uses the first/default load case."
                }
            },
            "required": ["beam_start", "beam_end", "load_start"]
        }
    },
    {
        "name": "add_load_case",
        "description": "Create a new load case to organize loads. Load cases can be combined according to design codes for ultimate and serviceability limit states.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the load case (e.g., 'Dead Load', 'Live Load 1', 'Wind X+')"
                },
                "load_case_type": {
                    "type": "string",
                    "enum": ["permanent", "variable", "environmental", "accidental"],
                    "description": "Type of load case for combination factors: permanent (dead loads), variable (live loads), environmental (wind/wave), accidental. Default 'variable'.",
                    "default": "variable"
                }
            },
            "required": ["name"]
        }
    },
    {
        "name": "add_load_combination",
        "description": "Create a load combination with type-based factors. Common presets: SLS(1,1,0,1), ULS-a(1.3,1.3,0,0.7), ULS-b(1.0,1.0,0,1.3), ALS(1.0,0,1.0,0).",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the combination (e.g., 'Main Load', 'Dead + Live')"
                },
                "type": {
                    "type": "string",
                    "enum": ["ULS-a", "ULS-b", "SLS", "ALS"],
                    "description": "Combination type. ULS-a: max permanent, ULS-b: max environmental, SLS: serviceability, ALS: accidental"
                },
                "permanent_factor": {
                    "type": "number",
                    "description": "Factor for permanent load cases. Default 1.0.",
                    "default": 1.0
                },
                "variable_factor": {
                    "type": "number",
                    "description": "Factor for variable load cases. Default 1.0.",
                    "default": 1.0
                },
                "environmental_factor": {
                    "type": "number",
                    "description": "Factor for environmental load cases (wind/wave). Default 1.0.",
                    "default": 1.0
                },
                "accidental_factor": {
                    "type": "number",
                    "description": "Factor for accidental load cases. Default 0.0.",
                    "default": 0.0
                }
            },
            "required": ["name", "type"]
        }
    },
    {
        "name": "add_load_case_to_combination",
        "description": "Add a load case to a load combination. The factor is determined by the load case type unless an override factor is provided.",
        "input_schema": {
            "type": "object",
            "properties": {
                "combination_id": {
                    "type": "integer",
                    "description": "ID of the load combination"
                },
                "load_case_id": {
                    "type": "integer",
                    "description": "ID of the load case to add"
                },
                "override_factor": {
                    "type": "number",
                    "description": "Optional override factor. If not provided, uses type-based factor from the combination."
                }
            },
            "required": ["combination_id", "load_case_id"]
        }
    },
    {
        "name": "remove_load_case_from_combination",
        "description": "Remove a load case from a load combination.",
        "input_schema": {
            "type": "object",
            "properties": {
                "combination_id": {
                    "type": "integer",
                    "description": "ID of the load combination"
                },
                "load_case_id": {
                    "type": "integer",
                    "description": "ID of the load case to remove"
                }
            },
            "required": ["combination_id", "load_case_id"]
        }
    },
    {
        "name": "update_load_case_override",
        "description": "Update the override factor for a load case in a combination. Set to null to remove the override.",
        "input_schema": {
            "type": "object",
            "properties": {
                "combination_id": {
                    "type": "integer",
                    "description": "ID of the load combination"
                },
                "load_case_id": {
                    "type": "integer",
                    "description": "ID of the load case"
                },
                "override_factor": {
                    "type": ["number", "null"],
                    "description": "New override factor, or null to remove override and use type-based factor"
                }
            },
            "required": ["combination_id", "load_case_id", "override_factor"]
        }
    },
    {
        "name": "delete_load_combination",
        "description": "Delete a load combination.",
        "input_schema": {
            "type": "object",
            "properties": {
                "combination_id": {
                    "type": "integer",
                    "description": "ID of the load combination to delete"
                }
            },
            "required": ["combination_id"]
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
    # Spring Elements (Nonlinear)
    # =========================================================================
    {
        "name": "add_spring",
        "description": "Add a spring element between two nodes. Springs can be linear, tension-only, compression-only, or have gaps. Use for bearing pads, cables, contact problems, or elastic connections.",
        "input_schema": {
            "type": "object",
            "properties": {
                "position1": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "First node position [x, y, z] in meters"
                },
                "position2": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Second node position [x, y, z] in meters"
                },
                "kx": {
                    "type": "number",
                    "description": "Translational stiffness in X direction [kN/m]. Default 0."
                },
                "ky": {
                    "type": "number",
                    "description": "Translational stiffness in Y direction [kN/m]. Default 0."
                },
                "kz": {
                    "type": "number",
                    "description": "Translational stiffness in Z direction [kN/m]. Default 0."
                },
                "krx": {
                    "type": "number",
                    "description": "Rotational stiffness about X axis [kN·m/rad]. Default 0."
                },
                "kry": {
                    "type": "number",
                    "description": "Rotational stiffness about Y axis [kN·m/rad]. Default 0."
                },
                "krz": {
                    "type": "number",
                    "description": "Rotational stiffness about Z axis [kN·m/rad]. Default 0."
                },
                "behavior": {
                    "type": "string",
                    "enum": ["Linear", "TensionOnly", "CompressionOnly"],
                    "description": "Spring behavior: Linear (bidirectional), TensionOnly (cables/hooks), CompressionOnly (bearing pads). Default Linear."
                },
                "gap": {
                    "type": "number",
                    "description": "Gap before spring engages [m for translation, rad for rotation]. Use for contact with clearance. Default 0."
                }
            },
            "required": ["position1", "position2"]
        }
    },
    {
        "name": "analyze_nonlinear",
        "description": "Run nonlinear analysis for models with tension/compression-only springs or gap springs. Uses iterative solver to determine spring states.",
        "input_schema": {
            "type": "object",
            "properties": {
                "max_iterations": {
                    "type": "integer",
                    "description": "Maximum solver iterations. Default 50."
                },
                "gap_tolerance": {
                    "type": "number",
                    "description": "Tolerance for gap state changes [m]. Default 1e-6."
                }
            },
            "required": []
        }
    },
    {
        "name": "get_spring_states",
        "description": "Get the state and forces of all springs after nonlinear analysis. Shows which springs are active/inactive and their forces.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },
    {
        "name": "update_spring",
        "description": "Update spring element properties including position, stiffness, behavior, and gap. Position updates only allowed if node is not shared with other elements.",
        "input_schema": {
            "type": "object",
            "properties": {
                "spring_id": {
                    "type": "integer",
                    "description": "ID of the spring to update"
                },
                "position1": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New position [x, y, z] for first node. Only allowed if node is not shared."
                },
                "position2": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New position [x, y, z] for second node. Only allowed if node is not shared."
                },
                "kx": {
                    "type": "number",
                    "description": "Translational stiffness in X direction [kN/m]"
                },
                "ky": {
                    "type": "number",
                    "description": "Translational stiffness in Y direction [kN/m]"
                },
                "kz": {
                    "type": "number",
                    "description": "Translational stiffness in Z direction [kN/m]"
                },
                "krx": {
                    "type": "number",
                    "description": "Rotational stiffness about X axis [kN·m/rad]"
                },
                "kry": {
                    "type": "number",
                    "description": "Rotational stiffness about Y axis [kN·m/rad]"
                },
                "krz": {
                    "type": "number",
                    "description": "Rotational stiffness about Z axis [kN·m/rad]"
                },
                "behavior": {
                    "type": "string",
                    "enum": ["Linear", "TensionOnly", "CompressionOnly"],
                    "description": "Spring behavior type"
                },
                "gap": {
                    "type": "number",
                    "description": "Gap before spring engages [m for translation, rad for rotation]"
                },
                "loading_condition": {
                    "type": "string",
                    "enum": ["All", "Static", "Dynamic"],
                    "description": "Loading condition: All (default), Static (permanent only), Dynamic (variable/environmental only)"
                }
            },
            "required": ["spring_id"]
        }
    },
    {
        "name": "delete_spring",
        "description": "Delete a spring element from the model.",
        "input_schema": {
            "type": "object",
            "properties": {
                "spring_id": {
                    "type": "integer",
                    "description": "ID of the spring to delete"
                }
            },
            "required": ["spring_id"]
        }
    },

    # =========================================================================
    # Rigid Links / Constraints
    # =========================================================================
    {
        "name": "add_rigid_link",
        "description": "Add a rigid link constraint between slave and master nodes. The slave node's motion follows the master node according to rigid body kinematics: u_slave = u_master + theta_master × offset.",
        "input_schema": {
            "type": "object",
            "properties": {
                "slave_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Position of slave (constrained) node [x, y, z] in meters"
                },
                "master_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Position of master (independent) node [x, y, z] in meters"
                }
            },
            "required": ["slave_position", "master_position"]
        }
    },
    {
        "name": "update_rigid_link",
        "description": "Update a rigid link constraint. Can update slave or master node positions (if not shared) or the offset.",
        "input_schema": {
            "type": "object",
            "properties": {
                "link_index": {
                    "type": "integer",
                    "description": "Index of the rigid link to update (0-based)"
                },
                "slave_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New position for slave node [x, y, z]. Only allowed if node is not shared."
                },
                "master_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New position for master node [x, y, z]. Only allowed if node is not shared."
                },
                "offset": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New offset vector from master to slave [x, y, z] in meters"
                }
            },
            "required": ["link_index"]
        }
    },
    {
        "name": "delete_rigid_link",
        "description": "Delete a rigid link constraint.",
        "input_schema": {
            "type": "object",
            "properties": {
                "link_index": {
                    "type": "integer",
                    "description": "Index of the rigid link to delete (0-based)"
                }
            },
            "required": ["link_index"]
        }
    },
    {
        "name": "get_rigid_links",
        "description": "Get all rigid link constraints in the model.",
        "input_schema": {
            "type": "object",
            "properties": {},
            "required": []
        }
    },

    # =========================================================================
    # Element Modification
    # =========================================================================
    {
        "name": "update_material",
        "description": "Update material properties. Changes will affect all beams using this material.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the material to update"
                },
                "E": {
                    "type": "number",
                    "description": "New Young's modulus in kN/m². Optional."
                },
                "nu": {
                    "type": "number",
                    "description": "New Poisson's ratio. Optional."
                },
                "rho": {
                    "type": "number",
                    "description": "New density in mT/m³. Optional."
                },
                "fy": {
                    "type": "number",
                    "description": "New yield stress in kN/m². Optional."
                },
                "fu": {
                    "type": "number",
                    "description": "New ultimate tensile strength in kN/m². Optional."
                }
            },
            "required": ["name"]
        }
    },
    {
        "name": "delete_material",
        "description": "Delete a material from the model. Cannot delete if material is in use by beams.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the material to delete"
                }
            },
            "required": ["name"]
        }
    },
    {
        "name": "update_section",
        "description": "Update section properties. Changes will affect all beams using this section.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the section to update"
                },
                "A": {
                    "type": "number",
                    "description": "New cross-sectional area in m². Optional."
                },
                "Iy": {
                    "type": "number",
                    "description": "New second moment of area about y-axis in m⁴. Optional."
                },
                "Iz": {
                    "type": "number",
                    "description": "New second moment of area about z-axis in m⁴. Optional."
                },
                "J": {
                    "type": "number",
                    "description": "New torsional constant in m⁴. Optional."
                }
            },
            "required": ["name"]
        }
    },
    {
        "name": "delete_section",
        "description": "Delete a section from the model. Cannot delete if section is in use by beams.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name of the section to delete"
                }
            },
            "required": ["name"]
        }
    },
    {
        "name": "update_beam",
        "description": "Update a beam's properties including name, position, material, section, formulation, warping, and offsets.",
        "input_schema": {
            "type": "object",
            "properties": {
                "beam_id": {
                    "type": "integer",
                    "description": "ID of the beam to update"
                },
                "name": {
                    "type": "string",
                    "description": "User-defined beam name for display. Optional."
                },
                "start_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New start position [x, y, z] in meters. Optional - only provide if changing. WARNING: This will move the node, affecting any other beams connected to it."
                },
                "end_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New end position [x, y, z] in meters. Optional - only provide if changing. WARNING: This will move the node, affecting any other beams connected to it."
                },
                "material": {
                    "type": "string",
                    "description": "New material name (must exist in model). Optional - only provide if changing."
                },
                "section": {
                    "type": "string",
                    "description": "New section name (must exist in model). Optional - only provide if changing."
                },
                "roll_angle": {
                    "type": "number",
                    "description": "Roll angle about beam axis in degrees. Optional."
                },
                "formulation": {
                    "type": "string",
                    "enum": ["euler_bernoulli", "timoshenko"],
                    "description": "Beam formulation type. 'euler_bernoulli' for slender beams (no shear deformation), 'timoshenko' for stocky beams (includes shear deformation). Optional."
                },
                "warping_enabled": {
                    "type": "boolean",
                    "description": "Enable 7th DOF for warping of thin-walled open sections. Optional."
                },
                "offset_i": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "End offset at start node [x, y, z] in local beam coordinates (meters). Optional."
                },
                "offset_j": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "End offset at end node [x, y, z] in local beam coordinates (meters). Optional."
                },
                "releases_i": {
                    "type": "array",
                    "items": {"type": "boolean"},
                    "minItems": 7,
                    "maxItems": 7,
                    "description": "DOF releases at start end [UX, UY, UZ, RX, RY, RZ, WARP]. True = released (hinge). Optional."
                },
                "releases_j": {
                    "type": "array",
                    "items": {"type": "boolean"},
                    "minItems": 7,
                    "maxItems": 7,
                    "description": "DOF releases at end [UX, UY, UZ, RX, RY, RZ, WARP]. True = released (hinge). Optional."
                }
            },
            "required": ["beam_id"]
        }
    },
    {
        "name": "delete_beam",
        "description": "Delete a beam from the model. WARNING: This cannot be undone.",
        "input_schema": {
            "type": "object",
            "properties": {
                "beam_id": {
                    "type": "integer",
                    "description": "ID of the beam to delete"
                }
            },
            "required": ["beam_id"]
        }
    },
    {
        "name": "remove_boundary_condition",
        "description": "Remove boundary conditions from a node. Can remove all BCs or specific DOFs.",
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
                    "enum": ["UX", "UY", "UZ", "RX", "RY", "RZ", "ALL"],
                    "description": "Specific DOF to unfix, or 'ALL' to remove all BCs at this node"
                }
            },
            "required": ["position", "dof"]
        }
    },
    {
        "name": "add_cargo",
        "description": "Add a cargo item to the model. The cargo is represented as a point mass at the CoG with optional visualization dimensions.",
        "input_schema": {
            "type": "object",
            "properties": {
                "name": {
                    "type": "string",
                    "description": "Name for the cargo (e.g., 'Generator', 'Container 1')"
                },
                "cog_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Center of gravity position [x, y, z] in meters"
                },
                "mass": {
                    "type": "number",
                    "description": "Total mass in metric tonnes (mT)"
                },
                "dimensions": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Cargo dimensions [length_x, width_y, height_z] in meters for visualization. Optional."
                }
            },
            "required": ["name", "cog_position", "mass"]
        }
    },
    {
        "name": "update_cargo",
        "description": "Update cargo properties including name, mass, CoG position, and dimensions.",
        "input_schema": {
            "type": "object",
            "properties": {
                "cargo_id": {
                    "type": "integer",
                    "description": "ID of the cargo to update"
                },
                "name": {
                    "type": "string",
                    "description": "New name for the cargo. Optional."
                },
                "mass": {
                    "type": "number",
                    "description": "New mass in metric tonnes (mT). Optional."
                },
                "cog_position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New center of gravity position [x, y, z] in meters. Optional."
                },
                "dimensions": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New cargo dimensions [length_x, width_y, height_z] in meters. Optional."
                }
            },
            "required": ["cargo_id"]
        }
    },
    {
        "name": "delete_cargo",
        "description": "Delete a cargo from the model. WARNING: This cannot be undone.",
        "input_schema": {
            "type": "object",
            "properties": {
                "cargo_id": {
                    "type": "integer",
                    "description": "ID of the cargo to delete"
                }
            },
            "required": ["cargo_id"]
        }
    },
    {
        "name": "add_cargo_connection",
        "description": "Add a connection point to a cargo item. Connection points define where the cargo attaches to the structure. Defaults to stiff connection if stiffness not specified.",
        "input_schema": {
            "type": "object",
            "properties": {
                "cargo_id": {
                    "type": "integer",
                    "description": "ID of the cargo to add connection to"
                },
                "name": {
                    "type": "string",
                    "description": "Name identifier for this connection point (e.g., 'Corner 1', 'SF-A'). Auto-generated if not provided."
                },
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Position [x, y, z] of the connection point in meters (global coordinates)"
                },
                "stiffness": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 6,
                    "description": "Spring stiffnesses. Can be [kx, ky, kz] (3 values, rotational defaults to 0) or [kx, ky, kz, krx, kry, krz] (6 values). Units: translations [kN/m], rotations [kN·m/rad]. Defaults to stiff (1e9) if not provided."
                },
                "cargo_offset": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Optional offset from cargo CoG to connection point on cargo [dx, dy, dz] in meters"
                },
                "loading_condition": {
                    "type": "string",
                    "enum": ["all", "dynamic"],
                    "description": "When the connection is active: 'all' (always, default) or 'dynamic' (only during environmental/variable loads)"
                }
            },
            "required": ["cargo_id", "position"]
        }
    },
    {
        "name": "delete_cargo_connection",
        "description": "Delete a connection point from a cargo item by name.",
        "input_schema": {
            "type": "object",
            "properties": {
                "cargo_id": {
                    "type": "integer",
                    "description": "ID of the cargo"
                },
                "connection_name": {
                    "type": "string",
                    "description": "Name of the connection point to delete"
                }
            },
            "required": ["cargo_id", "connection_name"]
        }
    },
    {
        "name": "update_cargo_connection",
        "description": "Update a cargo connection point's properties.",
        "input_schema": {
            "type": "object",
            "properties": {
                "cargo_id": {
                    "type": "integer",
                    "description": "ID of the cargo"
                },
                "connection_name": {
                    "type": "string",
                    "description": "Name of the connection point to update"
                },
                "new_name": {
                    "type": "string",
                    "description": "New name for the connection point. Optional."
                },
                "position": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "New position [x, y, z] in meters. Optional."
                },
                "stiffness": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                    "description": "Translational spring stiffnesses [kx, ky, kz] in kN/m. Optional."
                },
                "loading_condition": {
                    "type": "string",
                    "enum": ["all", "static", "dynamic"],
                    "description": "When connection is active: 'all' (always), 'static' (permanent loads only), 'dynamic' (variable/environmental loads only). Optional."
                }
            },
            "required": ["cargo_id", "connection_name"]
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

    # =========================================================================
    # Plate Meshing (Phase 19)
    # =========================================================================
    {
        "name": "add_plate",
        "description": "Add a plate region to the model defined by corner points. The plate will be meshed when mesh() is called. Requires gmsh to be installed.",
        "input_schema": {
            "type": "object",
            "properties": {
                "corners": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "number"}},
                    "description": "List of [x, y, z] corner coordinates in meters. At least 3 corners for triangular plate, 4 for quadrilateral."
                },
                "thickness": {
                    "type": "number",
                    "description": "Plate thickness in meters"
                },
                "material": {
                    "type": "string",
                    "description": "Name of material to use (must already exist in model)"
                },
                "name": {
                    "type": "string",
                    "description": "Optional name for the plate"
                },
                "mesh_size": {
                    "type": "number",
                    "description": "Target element size in meters (default 0.5)"
                },
                "element_type": {
                    "type": "string",
                    "enum": ["MITC4", "MITC8", "MITC9", "DKT"],
                    "description": "Plate element type. MITC4 (default) is a 4-node Mindlin plate. DKT is a 3-node thin plate (Kirchhoff)."
                }
            },
            "required": ["corners", "thickness", "material"]
        }
    },
    {
        "name": "set_edge_divisions",
        "description": "Set number of elements along a plate edge for structured meshing. Takes precedence over mesh_size.",
        "input_schema": {
            "type": "object",
            "properties": {
                "plate_name": {
                    "type": "string",
                    "description": "Name of the plate"
                },
                "edge_index": {
                    "type": "integer",
                    "description": "Edge index (0-based, edge i goes from corner i to corner i+1)"
                },
                "n_elements": {
                    "type": "integer",
                    "description": "Number of elements along this edge"
                }
            },
            "required": ["plate_name", "edge_index", "n_elements"]
        }
    },
    {
        "name": "couple_plate_to_beam",
        "description": "Couple a plate edge to a beam using rigid links. Use when plate edge is offset from beam centroid.",
        "input_schema": {
            "type": "object",
            "properties": {
                "plate_name": {
                    "type": "string",
                    "description": "Name of the plate"
                },
                "edge_index": {
                    "type": "integer",
                    "description": "Edge index (0-based)"
                },
                "beam_name": {
                    "type": "string",
                    "description": "Name of the beam to couple to"
                },
                "offset": {
                    "type": "array",
                    "items": {"type": "number"},
                    "description": "[dx, dy, dz] offset from plate edge to beam centroid in meters"
                }
            },
            "required": ["plate_name", "edge_index", "beam_name"]
        }
    },
    {
        "name": "add_support_curve",
        "description": "Add boundary condition support along a plate edge. Apply after meshing or during mesh.",
        "input_schema": {
            "type": "object",
            "properties": {
                "plate_name": {
                    "type": "string",
                    "description": "Name of the plate"
                },
                "edge_index": {
                    "type": "integer",
                    "description": "Edge index (0-based)"
                },
                "ux": {
                    "type": "boolean",
                    "description": "Restrain X translation (default false)"
                },
                "uy": {
                    "type": "boolean",
                    "description": "Restrain Y translation (default false)"
                },
                "uz": {
                    "type": "boolean",
                    "description": "Restrain Z translation (default false)"
                },
                "rotation_about_edge": {
                    "type": "boolean",
                    "description": "Restrain rotation about the edge (default false)"
                }
            },
            "required": ["plate_name", "edge_index"]
        }
    },
    {
        "name": "mesh_model",
        "description": "Generate mesh for all plates and apply boundary conditions. Call after defining plates and supports, before analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "verbose": {
                    "type": "boolean",
                    "description": "Print meshing progress (default false)"
                }
            },
            "required": []
        }
    },
    {
        "name": "get_plate_displacement",
        "description": "Get displacement at a point within a plate element after analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "element_id": {
                    "type": "integer",
                    "description": "Plate element ID"
                },
                "xi": {
                    "type": "number",
                    "description": "Natural coordinate xi (-1 to 1 for quads, 0 to 1 for triangles). Default 0."
                },
                "eta": {
                    "type": "number",
                    "description": "Natural coordinate eta (-1 to 1 for quads, 0 to 1 for triangles). Default 0."
                }
            },
            "required": ["element_id"]
        }
    },
    {
        "name": "get_plate_moments",
        "description": "Get internal bending moments at a point within a plate element after analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "element_id": {
                    "type": "integer",
                    "description": "Plate element ID"
                },
                "xi": {
                    "type": "number",
                    "description": "Natural coordinate xi. Default 0."
                },
                "eta": {
                    "type": "number",
                    "description": "Natural coordinate eta. Default 0."
                }
            },
            "required": ["element_id"]
        }
    },
    {
        "name": "get_plate_stress",
        "description": "Get stress at a point within a plate element after analysis.",
        "input_schema": {
            "type": "object",
            "properties": {
                "element_id": {
                    "type": "integer",
                    "description": "Plate element ID"
                },
                "surface": {
                    "type": "string",
                    "enum": ["top", "bottom", "middle"],
                    "description": "Surface to evaluate stress at (default 'top')"
                },
                "xi": {
                    "type": "number",
                    "description": "Natural coordinate xi. Default 0."
                },
                "eta": {
                    "type": "number",
                    "description": "Natural coordinate eta. Default 0."
                }
            },
            "required": ["element_id"]
        }
    },

    # =========================================================================
    # Beam Line Diagram Data
    # =========================================================================
    {
        "name": "get_beam_line_data",
        "description": "Get internal action diagram data for a beam. Returns x positions and values for plotting moment, shear, axial, or torsion diagrams. Includes extrema (min/max values and positions).",
        "input_schema": {
            "type": "object",
            "properties": {
                "beam_id": {
                    "type": "integer",
                    "description": "ID of the beam"
                },
                "action_type": {
                    "type": "string",
                    "enum": ["moment_y", "moment_z", "shear_y", "shear_z", "axial", "torsion", "bimoment", "warping_torsion"],
                    "description": "Type of internal action diagram to retrieve"
                },
                "num_points": {
                    "type": "integer",
                    "description": "Number of sample points along beam (default 100)"
                },
                "load_case_id": {
                    "type": "integer",
                    "description": "Optional load case ID (uses active load case if not specified)"
                }
            },
            "required": ["beam_id", "action_type"]
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
            "rho": 7.85
            "rho": 7.85
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

        # Plate meshing errors
        if "plate" in error_str and "not found" in error_str:
            return "Use add_plate tool first to create a plate, then refer to it by the returned plate_name."
        if "edge" in error_str and ("index" in error_str or "out of range" in error_str):
            return "Edge index is 0-based. For a quad plate, use 0-3; for a triangle, use 0-2."
        if "coplanar" in error_str:
            return "Plate corner points must all lie in the same plane. Check the z-coordinates."
        if "mesh" in error_str and ("fail" in error_str or "error" in error_str):
            return "Mesh generation failed. Try using a coarser mesh_size or check that the plate geometry is valid."
        if "gmsh" in error_str:
            return "Gmsh meshing error. Ensure gmsh is installed (pip install gmsh) and plate geometry is valid."

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
            rho=params["rho"],
            fy=params.get("fy", 0.0),
            fu=params.get("fu", 0.0)
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

        # Generate default name if not provided
        beam_name = params.get("name")
        if not beam_name:
            beam_name = f"Beam {len(self.model.beams) + 1}"

        beam = self.model.add_beam_by_coords(
            start_pos=params["start_position"],
            end_pos=params["end_position"],
            section_name=params["section"],
            material_name=params["material"],
            subdivisions=params.get("subdivisions", 1),
            name=beam_name
        )
        return ToolResult(
            success=True,
            result={
                "beam_id": beam.beam_id,
                "name": beam.name,
                "length": beam.length,
                "message": f"Beam '{beam.name}' created with length {beam.length:.3f} m"
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

        force = params.get("force")
        moment = params.get("moment")

        if force is None and moment is None:
            return ToolResult(
                success=False,
                error="At least one of 'force' or 'moment' must be provided"
            )

        self.model.add_point_load(params["position"], force=force, moment=moment)

        # Build descriptive message
        parts = []
        if force is not None:
            parts.append(f"F={force} kN")
        if moment is not None:
            parts.append(f"M={moment} kNm")

        return ToolResult(
            success=True,
            result={
                "position": params["position"],
                "force": force,
                "moment": moment,
                "message": f"Point load applied: {', '.join(parts)}"
            }
        )

    def _tool_add_line_load(self, params: Dict[str, Any]) -> ToolResult:
        """Add a line load."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        load_end = params.get("load_end", params["load_start"])

        # Find load case by ID if specified, otherwise use active load case
        load_case = None
        load_case_id = params.get("load_case_id")
        if load_case_id is not None:
            for lc in self.model._cpp_model.get_load_cases():
                if lc.id == load_case_id:
                    load_case = lc
                    break
            if load_case is None:
                return ToolResult(
                    success=False,
                    error=f"Load case with ID {load_case_id} not found"
                )
        else:
            # Use active load case instead of default
            load_case = self.model._cpp_model.get_active_load_case()
            if load_case is not None:
                load_case_id = load_case.id

        self.model.add_line_load_by_coords(
            start_pos=params["beam_start"],
            end_pos=params["beam_end"],
            w_start=params["load_start"],
            w_end=load_end,
            load_case=load_case
        )
        return ToolResult(
            success=True,
            result={
                "beam_start": params["beam_start"],
                "beam_end": params["beam_end"],
                "load_start": params["load_start"],
                "load_end": load_end,
                "load_case_id": load_case_id,
                "message": "Line load applied to beam"
            }
        )

    def _tool_add_load_case(self, params: Dict[str, Any]) -> ToolResult:
        """Create a new load case."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        name = params["name"]
        load_type_str = params.get("load_case_type", "variable").lower()

        # Map string to LoadCaseType enum
        type_map = {
            "permanent": LoadCaseType.Permanent,
            "variable": LoadCaseType.Variable,
            "environmental": LoadCaseType.Environmental,
            "accidental": LoadCaseType.Accidental,
        }
        load_type = type_map.get(load_type_str, LoadCaseType.Variable)

        load_case = self.model.create_load_case(name, load_type)
        return ToolResult(
            success=True,
            result={
                "id": load_case.id,
                "name": name,
                "type": load_type_str,
                "message": f"Load case '{name}' created with type '{load_type_str}'"
            }
        )

    def _tool_add_load_combination(self, params: Dict[str, Any]) -> ToolResult:
        """Create a load combination with type-based factors."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        name = params["name"]
        combo_type = params["type"]
        permanent_factor = params.get("permanent_factor", 1.0)
        variable_factor = params.get("variable_factor", 1.0)
        environmental_factor = params.get("environmental_factor", 1.0)
        accidental_factor = params.get("accidental_factor", 0.0)

        combo = self.model.create_load_combination(
            name,
            combo_type,
            permanent_factor,
            variable_factor,
            environmental_factor,
            accidental_factor
        )
        return ToolResult(
            success=True,
            result={
                "id": combo["id"],
                "name": name,
                "type": combo_type,
                "permanent_factor": permanent_factor,
                "variable_factor": variable_factor,
                "environmental_factor": environmental_factor,
                "accidental_factor": accidental_factor,
                "message": f"Load combination '{combo_type} - {name}' created"
            }
        )

    def _tool_add_load_case_to_combination(self, params: Dict[str, Any]) -> ToolResult:
        """Add a load case to a load combination."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        combination_id = params["combination_id"]
        load_case_id = params["load_case_id"]
        override_factor = params.get("override_factor")

        try:
            combo = self.model.add_load_case_to_combination(
                combination_id,
                load_case_id,
                override_factor
            )
            return ToolResult(
                success=True,
                result={
                    "combination_id": combination_id,
                    "load_case_id": load_case_id,
                    "override_factor": override_factor,
                    "has_override": override_factor is not None,
                    "total_load_cases": len(combo["load_cases"]),
                    "message": f"Load case {load_case_id} added to combination {combination_id}"
                }
            )
        except ValueError as e:
            return ToolResult(success=False, error=str(e))

    def _tool_remove_load_case_from_combination(self, params: Dict[str, Any]) -> ToolResult:
        """Remove a load case from a load combination."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        combination_id = params["combination_id"]
        load_case_id = params["load_case_id"]

        try:
            combo = self.model.remove_load_case_from_combination(
                combination_id,
                load_case_id
            )
            return ToolResult(
                success=True,
                result={
                    "combination_id": combination_id,
                    "load_case_id": load_case_id,
                    "remaining_load_cases": len(combo["load_cases"]),
                    "message": f"Load case {load_case_id} removed from combination {combination_id}"
                }
            )
        except ValueError as e:
            return ToolResult(success=False, error=str(e))

    def _tool_delete_load_combination(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a load combination."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        combination_id = params["combination_id"]

        try:
            self.model.delete_load_combination(combination_id)
            return ToolResult(
                success=True,
                result={
                    "combination_id": combination_id,
                    "message": f"Load combination {combination_id} deleted"
                }
            )
        except ValueError as e:
            return ToolResult(success=False, error=str(e))

    def _tool_update_load_case_override(self, params: Dict[str, Any]) -> ToolResult:
        """Update the override factor for a load case in a combination."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        combination_id = params["combination_id"]
        load_case_id = params["load_case_id"]
        override_factor = params.get("override_factor")

        try:
            self.model.update_load_case_override(
                combination_id,
                load_case_id,
                override_factor
            )
            return ToolResult(
                success=True,
                result={
                    "combination_id": combination_id,
                    "load_case_id": load_case_id,
                    "override_factor": override_factor,
                    "has_override": override_factor is not None,
                    "message": f"Override factor updated for load case {load_case_id} in combination {combination_id}"
                }
            )
        except ValueError as e:
            return ToolResult(success=False, error=str(e))

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

    def _tool_add_spring(self, params: Dict[str, Any]) -> ToolResult:
        """Add a spring element."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        # Parse behavior
        behavior_str = params.get("behavior", "Linear")
        behavior_map = {
            "Linear": SpringBehavior.Linear,
            "TensionOnly": SpringBehavior.TensionOnly,
            "CompressionOnly": SpringBehavior.CompressionOnly,
        }
        behavior = behavior_map.get(behavior_str, SpringBehavior.Linear)

        spring = self.model.add_spring(
            node1=params["position1"],
            node2=params["position2"],
            kx=params.get("kx", 0.0),
            ky=params.get("ky", 0.0),
            kz=params.get("kz", 0.0),
            krx=params.get("krx", 0.0),
            kry=params.get("kry", 0.0),
            krz=params.get("krz", 0.0),
            behavior=behavior,
            gap=params.get("gap", 0.0),
        )

        return ToolResult(
            success=True,
            result={
                "spring_id": spring.id(),
                "behavior": behavior_str,
                "gap": params.get("gap", 0.0),
                "message": f"Spring created with behavior={behavior_str}"
            }
        )

    def _tool_analyze_nonlinear(self, params: Dict[str, Any]) -> ToolResult:
        """Run nonlinear analysis with springs."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        # Configure settings if provided
        settings = self.model.get_nonlinear_settings()
        if "max_iterations" in params:
            settings.max_iterations = params["max_iterations"]
        if "gap_tolerance" in params:
            settings.gap_tolerance = params["gap_tolerance"]

        # Check if model has nonlinear springs
        has_nonlinear = self.model.has_nonlinear_springs()

        # Run analysis
        results = self.model.analyze_with_nonlinear_springs()

        if results:
            # Get first result for summary
            first_result = next(iter(results.values()))
            return ToolResult(
                success=True,
                result={
                    "analyzed": True,
                    "has_nonlinear_springs": has_nonlinear,
                    "num_load_cases": len(results),
                    "iterations": first_result.iterations if hasattr(first_result, 'iterations') else 1,
                    "message": f"Nonlinear analysis completed for {len(results)} load case(s)"
                }
            )
        else:
            return ToolResult(
                success=False,
                error="Nonlinear analysis failed.",
                suggestion="Check boundary conditions and spring configuration."
            )

    def _tool_get_spring_states(self, params: Dict[str, Any]) -> ToolResult:
        """Get spring states after analysis."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(
                success=False,
                error="Model not analyzed. Call analyze or analyze_nonlinear first.",
                suggestion="Run analysis before querying spring states."
            )

        # Get all springs from model
        springs = self.model._cpp_model.springs
        if not springs:
            return ToolResult(
                success=True,
                result={
                    "num_springs": 0,
                    "springs": [],
                    "message": "No springs in model"
                }
            )

        spring_info = []
        for spring in springs:
            # Get states for each DOF (active or not)
            states = []
            for dof in range(6):
                is_active = spring.is_active(dof)
                states.append(is_active)

            spring_info.append({
                "id": spring.id(),
                "is_nonlinear": spring.is_nonlinear(),
                "has_gap": spring.has_gap(),
                "active_dofs": states,
                "stiffness": {
                    "kx": spring.kx,
                    "ky": spring.ky,
                    "kz": spring.kz,
                    "krx": spring.krx,
                    "kry": spring.kry,
                    "krz": spring.krz,
                }
            })

        return ToolResult(
            success=True,
            result={
                "num_springs": len(springs),
                "springs": spring_info,
                "units": {"k_trans": "kN/m", "k_rot": "kN·m/rad"}
            }
        )

    def _tool_update_spring(self, params: Dict[str, Any]) -> ToolResult:
        """Update spring element properties including position."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        spring_id = params["spring_id"]

        # Find the spring
        spring = None
        for s in self.model._cpp_model.spring_elements:
            if s.id == spring_id:
                spring = s
                break

        if spring is None:
            return ToolResult(
                success=False,
                error=f"Spring with ID {spring_id} not found",
                suggestion="Use get_spring_states to list all springs"
            )

        changes = []

        # Helper to check if a node is shared
        def is_node_shared(node_id: int, exclude_spring_id: int) -> bool:
            # Check beams
            for b in self.model.beams:
                for elem in b.elements:
                    if elem.node_i.id == node_id or elem.node_j.id == node_id:
                        return True
            # Check other springs
            for s in self.model._cpp_model.spring_elements:
                if s.id == exclude_spring_id:
                    continue
                if s.node_i.id == node_id or s.node_j.id == node_id:
                    return True
            return False

        # Handle position1 update (node_i)
        if "position1" in params and params["position1"] is not None:
            new_pos = params["position1"]
            node_i = spring.node_i
            if is_node_shared(node_i.id, spring_id):
                return ToolResult(
                    success=False,
                    error=f"Cannot update position1: node {node_i.id} is shared with other elements",
                    suggestion="Position updates only allowed on non-shared nodes"
                )
            node_i.x = new_pos[0]
            node_i.y = new_pos[1]
            node_i.z = new_pos[2]
            changes.append(f"position1 -> {new_pos}")

        # Handle position2 update (node_j)
        if "position2" in params and params["position2"] is not None:
            new_pos = params["position2"]
            node_j = spring.node_j
            if is_node_shared(node_j.id, spring_id):
                return ToolResult(
                    success=False,
                    error=f"Cannot update position2: node {node_j.id} is shared with other elements",
                    suggestion="Position updates only allowed on non-shared nodes"
                )
            node_j.x = new_pos[0]
            node_j.y = new_pos[1]
            node_j.z = new_pos[2]
            changes.append(f"position2 -> {new_pos}")

        # Update stiffness properties
        if "kx" in params and params["kx"] is not None:
            spring.kx = params["kx"]
            changes.append(f"kx -> {params['kx']}")
        if "ky" in params and params["ky"] is not None:
            spring.ky = params["ky"]
            changes.append(f"ky -> {params['ky']}")
        if "kz" in params and params["kz"] is not None:
            spring.kz = params["kz"]
            changes.append(f"kz -> {params['kz']}")
        if "krx" in params and params["krx"] is not None:
            spring.krx = params["krx"]
            changes.append(f"krx -> {params['krx']}")
        if "kry" in params and params["kry"] is not None:
            spring.kry = params["kry"]
            changes.append(f"kry -> {params['kry']}")
        if "krz" in params and params["krz"] is not None:
            spring.krz = params["krz"]
            changes.append(f"krz -> {params['krz']}")

        # Update behavior
        if "behavior" in params and params["behavior"] is not None:
            behavior_str = params["behavior"]
            behavior_map = {
                "Linear": SpringBehavior.Linear,
                "TensionOnly": SpringBehavior.TensionOnly,
                "CompressionOnly": SpringBehavior.CompressionOnly,
            }
            if behavior_str in behavior_map:
                spring.set_all_behavior(behavior_map[behavior_str])
                changes.append(f"behavior -> {behavior_str}")

        # Update gap
        if "gap" in params and params["gap"] is not None:
            spring.set_all_gaps(params["gap"])
            changes.append(f"gap -> {params['gap']}")

        # Update loading condition
        if "loading_condition" in params and params["loading_condition"] is not None:
            cond_str = params["loading_condition"]
            cond_map = {
                "All": LoadingCondition.All,
                "Static": LoadingCondition.Static,
                "Dynamic": LoadingCondition.Dynamic,
            }
            if cond_str in cond_map:
                spring.loading_condition = cond_map[cond_str]
                changes.append(f"loading_condition -> {cond_str}")

        # Clear analysis cache if any changes made
        if changes:
            try:
                self.model._cpp_model.clear_analysis()
            except Exception:
                pass
            self.model._is_analyzed = False

        return ToolResult(
            success=True,
            result={
                "spring_id": spring_id,
                "changes": changes,
                "message": f"Spring updated: {', '.join(changes)}" if changes else "No changes made"
            }
        )

    def _tool_delete_spring(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a spring element."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        spring_id = params["spring_id"]

        # Find and remove the spring
        springs = self.model._cpp_model.spring_elements
        found = False
        for i, s in enumerate(springs):
            if s.id == spring_id:
                # Note: We can't directly remove from vector exposed via pybind11
                # So we need a C++ method or workaround
                found = True
                break

        if not found:
            return ToolResult(
                success=False,
                error=f"Spring with ID {spring_id} not found",
                suggestion="Use get_spring_states to list all springs"
            )

        # For now, mark as deleted by setting all stiffness to 0
        # A proper implementation would require a C++ delete method
        for s in springs:
            if s.id == spring_id:
                s.kx = 0.0
                s.ky = 0.0
                s.kz = 0.0
                s.krx = 0.0
                s.kry = 0.0
                s.krz = 0.0
                break

        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass
        self.model._is_analyzed = False

        return ToolResult(
            success=True,
            result={
                "spring_id": spring_id,
                "message": f"Spring {spring_id} deleted (stiffness set to zero)"
            }
        )

    def _tool_add_rigid_link(self, params: Dict[str, Any]) -> ToolResult:
        """Add a rigid link constraint."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        import numpy as np

        slave_pos = params["slave_position"]
        master_pos = params["master_position"]

        # Get or create nodes at the specified positions
        slave_node = self.model.get_or_create_node(slave_pos[0], slave_pos[1], slave_pos[2])
        master_node = self.model.get_or_create_node(master_pos[0], master_pos[1], master_pos[2])

        # Calculate offset from master to slave
        offset = np.array([
            slave_pos[0] - master_pos[0],
            slave_pos[1] - master_pos[1],
            slave_pos[2] - master_pos[2]
        ])

        # Add the rigid link
        self.model._cpp_model.add_rigid_link(slave_node, master_node, offset)

        # Clear analysis cache
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass
        self.model._is_analyzed = False

        return ToolResult(
            success=True,
            result={
                "slave_node_id": slave_node.id,
                "master_node_id": master_node.id,
                "offset": offset.tolist(),
                "message": f"Rigid link created: slave node {slave_node.id} constrained to master node {master_node.id}"
            }
        )

    def _tool_update_rigid_link(self, params: Dict[str, Any]) -> ToolResult:
        """Update a rigid link constraint."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        import numpy as np

        link_index = params["link_index"]
        rigid_links = self.model._cpp_model.constraints.get_rigid_links()

        if link_index < 0 or link_index >= len(rigid_links):
            return ToolResult(
                success=False,
                error=f"Rigid link index {link_index} out of range. Have {len(rigid_links)} links.",
                suggestion="Use get_rigid_links to see all links"
            )

        link = rigid_links[link_index]
        changes = []

        # Helper to check if a node is shared
        def is_node_shared(node_id: int) -> bool:
            # Check beams
            for b in self.model.beams:
                for elem in b.elements:
                    if elem.node_i.id == node_id or elem.node_j.id == node_id:
                        return True
            # Check springs
            for s in self.model._cpp_model.spring_elements:
                if s.node_i.id == node_id or s.node_j.id == node_id:
                    return True
            # Check other rigid links (referencing same node in different constraints)
            count = 0
            for rl in rigid_links:
                if rl.slave_node_id == node_id or rl.master_node_id == node_id:
                    count += 1
            return count > 1  # More than one reference means shared

        # Helper to get node by ID
        def get_node_by_id(node_id: int):
            for node in self.model._cpp_model.get_all_nodes():
                if node.id == node_id:
                    return node
            return None

        # Handle slave_position update
        if "slave_position" in params and params["slave_position"] is not None:
            new_pos = params["slave_position"]
            if is_node_shared(link.slave_node_id):
                return ToolResult(
                    success=False,
                    error=f"Cannot update slave position: node {link.slave_node_id} is shared",
                    suggestion="Position updates only allowed on non-shared nodes"
                )
            slave_node = get_node_by_id(link.slave_node_id)
            if slave_node:
                slave_node.x = new_pos[0]
                slave_node.y = new_pos[1]
                slave_node.z = new_pos[2]
                # Recalculate offset
                master_node = get_node_by_id(link.master_node_id)
                if master_node:
                    link.offset = np.array([
                        slave_node.x - master_node.x,
                        slave_node.y - master_node.y,
                        slave_node.z - master_node.z
                    ])
                changes.append(f"slave_position -> {new_pos}")

        # Handle master_position update
        if "master_position" in params and params["master_position"] is not None:
            new_pos = params["master_position"]
            if is_node_shared(link.master_node_id):
                return ToolResult(
                    success=False,
                    error=f"Cannot update master position: node {link.master_node_id} is shared",
                    suggestion="Position updates only allowed on non-shared nodes"
                )
            master_node = get_node_by_id(link.master_node_id)
            if master_node:
                master_node.x = new_pos[0]
                master_node.y = new_pos[1]
                master_node.z = new_pos[2]
                # Recalculate offset
                slave_node = get_node_by_id(link.slave_node_id)
                if slave_node:
                    link.offset = np.array([
                        slave_node.x - master_node.x,
                        slave_node.y - master_node.y,
                        slave_node.z - master_node.z
                    ])
                changes.append(f"master_position -> {new_pos}")

        # Handle offset update directly
        if "offset" in params and params["offset"] is not None:
            new_offset = np.array(params["offset"])
            link.offset = new_offset
            changes.append(f"offset -> {params['offset']}")

        # Clear analysis cache
        if changes:
            try:
                self.model._cpp_model.clear_analysis()
            except Exception:
                pass
            self.model._is_analyzed = False

        return ToolResult(
            success=True,
            result={
                "link_index": link_index,
                "changes": changes,
                "message": f"Rigid link updated: {', '.join(changes)}" if changes else "No changes made"
            }
        )

    def _tool_delete_rigid_link(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a rigid link constraint."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        link_index = params["link_index"]
        rigid_links = self.model._cpp_model.constraints.get_rigid_links()

        if link_index < 0 or link_index >= len(rigid_links):
            return ToolResult(
                success=False,
                error=f"Rigid link index {link_index} out of range. Have {len(rigid_links)} links.",
                suggestion="Use get_rigid_links to see all links"
            )

        # We can't directly remove from the list, but we can clear all and re-add
        # Store all links except the one to delete
        links_to_keep = []
        for i, rl in enumerate(rigid_links):
            if i != link_index:
                links_to_keep.append({
                    'slave_node_id': rl.slave_node_id,
                    'master_node_id': rl.master_node_id,
                    'offset': rl.offset.copy()
                })

        # Clear and re-add
        self.model._cpp_model.constraints.clear()

        # Helper to get node by ID
        def get_node_by_id(node_id: int):
            for node in self.model._cpp_model.get_all_nodes():
                if node.id == node_id:
                    return node
            return None

        for link_data in links_to_keep:
            slave_node = get_node_by_id(link_data['slave_node_id'])
            master_node = get_node_by_id(link_data['master_node_id'])
            if slave_node and master_node:
                # Use Model's add_rigid_link which takes Node objects
                self.model._cpp_model.add_rigid_link(
                    slave_node, master_node, link_data['offset']
                )

        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass
        self.model._is_analyzed = False

        return ToolResult(
            success=True,
            result={
                "link_index": link_index,
                "remaining_links": len(links_to_keep),
                "message": f"Rigid link {link_index} deleted"
            }
        )

    def _tool_get_rigid_links(self, params: Dict[str, Any]) -> ToolResult:
        """Get all rigid links."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        rigid_links = self.model._cpp_model.constraints.get_rigid_links()

        # Helper to get node position by ID
        def get_node_pos(node_id: int):
            for node in self.model._cpp_model.get_all_nodes():
                if node.id == node_id:
                    return [node.x, node.y, node.z]
            return None

        links_info = []
        for i, rl in enumerate(rigid_links):
            links_info.append({
                "index": i,
                "slave_node_id": rl.slave_node_id,
                "master_node_id": rl.master_node_id,
                "slave_position": get_node_pos(rl.slave_node_id),
                "master_position": get_node_pos(rl.master_node_id),
                "offset": [rl.offset[0], rl.offset[1], rl.offset[2]]
            })

        return ToolResult(
            success=True,
            result={
                "num_rigid_links": len(rigid_links),
                "rigid_links": links_info
            }
        )

    def _tool_get_model_info(self, params: Dict[str, Any]) -> ToolResult:
        """Get model summary."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        num_springs = len(self.model._cpp_model.springs) if hasattr(self.model._cpp_model, 'springs') else 0

        return ToolResult(
            success=True,
            result={
                "name": self.model.name,
                "num_nodes": self.model.num_nodes(),
                "num_beams": self.model.num_beams(),
                "num_springs": num_springs,
                "num_elements": self.model.num_elements(),
                "total_dofs": self.model.total_dofs(),
                "analyzed": self.model.is_analyzed(),
                "has_nonlinear_springs": self.model.has_nonlinear_springs(),
                "materials": list(self.model._materials.keys()),
                "sections": list(self.model._sections.keys())
            }
        )

    def _tool_update_material(self, params: Dict[str, Any]) -> ToolResult:
        """Update material properties."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        name = params["name"]
        if name not in self.model._materials:
            return ToolResult(
                success=False,
                error=f"Material '{name}' not found",
                suggestion=f"Available materials: {list(self.model._materials.keys())}"
            )

        material = self.model._materials[name]
        changes = []

        if "E" in params and params["E"] is not None:
            material.E = params["E"]
            changes.append(f"E -> {params['E']}")

        if "nu" in params and params["nu"] is not None:
            material.nu = params["nu"]
            changes.append(f"nu -> {params['nu']}")

        if "rho" in params and params["rho"] is not None:
            material.rho = params["rho"]
            changes.append(f"rho -> {params['rho']}")

        if "fy" in params and params["fy"] is not None:
            material.fy = params["fy"]
            changes.append(f"fy -> {params['fy']}")

        if "fu" in params and params["fu"] is not None:
            material.fu = params["fu"]
            changes.append(f"fu -> {params['fu']}")

        if not changes:
            return ToolResult(
                success=True,
                result={"name": name, "message": "No changes specified"}
            )

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "name": name,
                "changes": changes,
                "message": f"Material '{name}' updated: {', '.join(changes)}. Re-analysis required."
            }
        )

    def _tool_delete_material(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a material from the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        name = params["name"]
        if name not in self.model._materials:
            return ToolResult(
                success=False,
                error=f"Material '{name}' not found",
                suggestion=f"Available materials: {list(self.model._materials.keys())}"
            )

        # Check if material is in use
        beams_using = [b.beam_id for b in self.model.beams if b.material.name == name]
        if beams_using:
            return ToolResult(
                success=False,
                error=f"Cannot delete material '{name}': in use by beams {beams_using}",
                suggestion="Update the beams to use a different material first."
            )

        del self.model._materials[name]

        return ToolResult(
            success=True,
            result={
                "name": name,
                "message": f"Material '{name}' deleted."
            }
        )

    def _tool_update_section(self, params: Dict[str, Any]) -> ToolResult:
        """Update section properties."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        name = params["name"]
        if name not in self.model._sections:
            return ToolResult(
                success=False,
                error=f"Section '{name}' not found",
                suggestion=f"Available sections: {list(self.model._sections.keys())}"
            )

        section = self.model._sections[name]
        changes = []

        if "A" in params and params["A"] is not None:
            section.A = params["A"]
            changes.append(f"A -> {params['A']}")

        if "Iy" in params and params["Iy"] is not None:
            section.Iy = params["Iy"]
            changes.append(f"Iy -> {params['Iy']}")

        if "Iz" in params and params["Iz"] is not None:
            section.Iz = params["Iz"]
            changes.append(f"Iz -> {params['Iz']}")

        if "J" in params and params["J"] is not None:
            section.J = params["J"]
            changes.append(f"J -> {params['J']}")

        if not changes:
            return ToolResult(
                success=True,
                result={"name": name, "message": "No changes specified"}
            )

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "name": name,
                "changes": changes,
                "message": f"Section '{name}' updated: {', '.join(changes)}. Re-analysis required."
            }
        )

    def _tool_delete_section(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a section from the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        name = params["name"]
        if name not in self.model._sections:
            return ToolResult(
                success=False,
                error=f"Section '{name}' not found",
                suggestion=f"Available sections: {list(self.model._sections.keys())}"
            )

        # Check if section is in use
        beams_using = [b.beam_id for b in self.model.beams if b.section.name == name]
        if beams_using:
            return ToolResult(
                success=False,
                error=f"Cannot delete section '{name}': in use by beams {beams_using}",
                suggestion="Update the beams to use a different section first."
            )

        del self.model._sections[name]

        return ToolResult(
            success=True,
            result={
                "name": name,
                "message": f"Section '{name}' deleted."
            }
        )

    def _tool_update_beam(self, params: Dict[str, Any]) -> ToolResult:
        """Update a beam's properties including material, section, formulation, warping, and offsets."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        beam_id = params["beam_id"]

        # Find the beam by ID
        beam = None
        for b in self.model.beams:
            if b.beam_id == beam_id:
                beam = b
                break

        if beam is None:
            return ToolResult(
                success=False,
                error=f"Beam with ID {beam_id} not found",
                suggestion=f"Available beam IDs: {[b.beam_id for b in self.model.beams]}"
            )

        changes = []
        import numpy as np

        # Update name if provided
        if "name" in params and params["name"] is not None:
            old_name = beam.name
            beam.name = params["name"]
            changes.append(f"name '{old_name}' -> '{beam.name}'")

        # Helper to check if a node is shared with other elements
        def is_node_shared(node_id: int, exclude_beam_id: int) -> bool:
            for b in self.model.beams:
                if b.beam_id == exclude_beam_id:
                    continue
                for elem in b.elements:
                    if elem.node_i.id == node_id or elem.node_j.id == node_id:
                        return True
            # Also check springs
            for s in self.model._cpp_model.spring_elements:
                if s.node_i.id == node_id or s.node_j.id == node_id:
                    return True
            return False

        # Helper to recalculate element geometry (length and local_axes)
        def recalculate_element_geometry(elem):
            from grillex.core.data_types import LocalAxes
            pos_i = np.array([elem.node_i.x, elem.node_i.y, elem.node_i.z])
            pos_j = np.array([elem.node_j.x, elem.node_j.y, elem.node_j.z])
            elem.length = float(np.linalg.norm(pos_j - pos_i))
            # Preserve the current roll angle
            current_roll = elem.local_axes.roll
            elem.local_axes = LocalAxes(pos_i, pos_j, current_roll)

        positions_changed = False

        # Update start position if provided
        if "start_position" in params and params["start_position"] is not None:
            new_pos = params["start_position"]
            if len(beam.elements) > 0:
                node = beam.elements[0].node_i
                if is_node_shared(node.id, beam.beam_id):
                    return ToolResult(
                        success=False,
                        error=f"Cannot update start position: node {node.id} is shared with other elements. Moving it would affect all connected beams.",
                        suggestion="Delete this beam and create a new one at the desired position, or disconnect the shared elements first."
                    )
                old_pos = [node.x, node.y, node.z]
                node.x, node.y, node.z = new_pos[0], new_pos[1], new_pos[2]
                # Update beam's Python-level start_pos
                beam.start_pos = np.array(new_pos)
                positions_changed = True
                changes.append(f"start_position {old_pos} -> {new_pos}")

        # Update end position if provided
        if "end_position" in params and params["end_position"] is not None:
            new_pos = params["end_position"]
            if len(beam.elements) > 0:
                node = beam.elements[-1].node_j
                if is_node_shared(node.id, beam.beam_id):
                    return ToolResult(
                        success=False,
                        error=f"Cannot update end position: node {node.id} is shared with other elements. Moving it would affect all connected beams.",
                        suggestion="Delete this beam and create a new one at the desired position, or disconnect the shared elements first."
                    )
                old_pos = [node.x, node.y, node.z]
                node.x, node.y, node.z = new_pos[0], new_pos[1], new_pos[2]
                # Update beam's Python-level end_pos
                beam.end_pos = np.array(new_pos)
                positions_changed = True
                changes.append(f"end_position {old_pos} -> {new_pos}")

        # Recalculate geometry for all elements if positions changed
        if positions_changed:
            beam.length = float(np.linalg.norm(beam.end_pos - beam.start_pos))
            for elem in beam.elements:
                recalculate_element_geometry(elem)

        # Update material if provided
        if "material" in params and params["material"]:
            material_name = params["material"]
            if material_name not in self.model._materials:
                return ToolResult(
                    success=False,
                    error=f"Material '{material_name}' not found",
                    suggestion=f"Available materials: {list(self.model._materials.keys())}"
                )
            beam.material = self.model._materials[material_name]
            # Update material on underlying elements
            for elem in beam.elements:
                elem.material = beam.material
            changes.append(f"material -> {material_name}")

        # Update section if provided
        if "section" in params and params["section"]:
            section_name = params["section"]
            if section_name not in self.model._sections:
                return ToolResult(
                    success=False,
                    error=f"Section '{section_name}' not found",
                    suggestion=f"Available sections: {list(self.model._sections.keys())}"
                )
            beam.section = self.model._sections[section_name]
            # Update section on underlying elements
            for elem in beam.elements:
                elem.section = beam.section
            changes.append(f"section -> {section_name}")

        # Update formulation if provided
        if "formulation" in params and params["formulation"] is not None:
            formulation_str = params["formulation"].lower()
            from grillex.core.data_types import BeamFormulation
            if formulation_str == "timoshenko":
                new_formulation = BeamFormulation.Timoshenko
            else:
                new_formulation = BeamFormulation.EulerBernoulli
            # Update config on underlying elements
            for elem in beam.elements:
                elem.config.formulation = new_formulation
                if formulation_str == "timoshenko":
                    elem.config.include_shear_deformation = True
            changes.append(f"formulation -> {formulation_str}")

        # Update warping if provided
        if "warping_enabled" in params and params["warping_enabled"] is not None:
            warping = params["warping_enabled"]
            for elem in beam.elements:
                elem.config.include_warping = warping
                # When warping is enabled, release warping DOFs by default
                if warping:
                    elem.releases.release_warp_i = True
                    elem.releases.release_warp_j = True
            changes.append(f"warping_enabled -> {warping}")

        # Update offsets if provided
        if "offset_i" in params and params["offset_i"] is not None:
            import numpy as np
            offset_i = np.array(params["offset_i"])
            for elem in beam.elements:
                elem.offset_i = offset_i
            changes.append(f"offset_i -> {params['offset_i']}")

        if "offset_j" in params and params["offset_j"] is not None:
            import numpy as np
            offset_j = np.array(params["offset_j"])
            for elem in beam.elements:
                elem.offset_j = offset_j
            changes.append(f"offset_j -> {params['offset_j']}")

        # Update roll angle if provided (input in degrees, converted to radians)
        if "roll_angle" in params and params["roll_angle"] is not None:
            roll_deg = params["roll_angle"]
            roll_rad = np.radians(roll_deg)
            from grillex.core.data_types import LocalAxes
            for elem in beam.elements:
                # Recalculate local_axes with the new roll angle
                pos_i = np.array([elem.node_i.x, elem.node_i.y, elem.node_i.z])
                pos_j = np.array([elem.node_j.x, elem.node_j.y, elem.node_j.z])
                elem.local_axes = LocalAxes(pos_i, pos_j, roll_rad)
            changes.append(f"roll_angle -> {roll_deg}°")

        # Update releases if provided
        if "releases_i" in params and params["releases_i"] is not None:
            rel = params["releases_i"]
            for elem in beam.elements:
                elem.releases.release_ux_i = rel[0]
                elem.releases.release_uy_i = rel[1]
                elem.releases.release_uz_i = rel[2]
                elem.releases.release_rx_i = rel[3]
                elem.releases.release_ry_i = rel[4]
                elem.releases.release_rz_i = rel[5]
                elem.releases.release_warp_i = rel[6] if len(rel) > 6 else False
            changes.append(f"releases_i -> {rel}")

        if "releases_j" in params and params["releases_j"] is not None:
            rel = params["releases_j"]
            for elem in beam.elements:
                elem.releases.release_ux_j = rel[0]
                elem.releases.release_uy_j = rel[1]
                elem.releases.release_uz_j = rel[2]
                elem.releases.release_rx_j = rel[3]
                elem.releases.release_ry_j = rel[4]
                elem.releases.release_rz_j = rel[5]
                elem.releases.release_warp_j = rel[6] if len(rel) > 6 else False
            changes.append(f"releases_j -> {rel}")

        if not changes:
            return ToolResult(
                success=True,
                result={"beam_id": beam_id, "message": "No changes specified"}
            )

        # Mark model as not analyzed (changes require re-analysis)
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "beam_id": beam_id,
                "changes": changes,
                "message": f"Beam {beam_id} updated: {', '.join(changes)}. Re-analysis required."
            }
        )

    def _tool_delete_beam(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a beam from the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        beam_id = params["beam_id"]

        # Find the beam index
        beam_index = None
        for i, b in enumerate(self.model.beams):
            if b.beam_id == beam_id:
                beam_index = i
                break

        if beam_index is None:
            return ToolResult(
                success=False,
                error=f"Beam with ID {beam_id} not found",
                suggestion=f"Available beam IDs: {[b.beam_id for b in self.model.beams]}"
            )

        # Remove from Python list
        removed_beam = self.model.beams.pop(beam_index)

        # Try to remove from C++ model if possible
        try:
            self.model._cpp_model.remove_beam(beam_id)
        except Exception:
            # C++ model might not support removal, but Python side is updated
            pass

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "beam_id": beam_id,
                "start": list(removed_beam.start_pos),
                "end": list(removed_beam.end_pos),
                "message": f"Beam {beam_id} deleted. Re-analysis required."
            }
        )

    def _tool_remove_boundary_condition(self, params: Dict[str, Any]) -> ToolResult:
        """Remove boundary conditions from a node."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        position = params["position"]
        dof_str = params["dof"]

        try:
            if dof_str == "ALL":
                # Remove all BCs at this node
                self.model._cpp_model.remove_all_bcs_at(position)
                message = f"All boundary conditions removed at {position}"
            else:
                # Remove specific DOF
                dof = _parse_dof(dof_str)
                self.model._cpp_model.remove_bc_at(position, dof)
                message = f"Boundary condition for {dof_str} removed at {position}"

            # Mark model as not analyzed
            try:
                self.model._cpp_model.clear_analysis()
            except Exception:
                pass

            return ToolResult(
                success=True,
                result={
                    "position": position,
                    "dof": dof_str,
                    "message": message
                }
            )
        except Exception as e:
            return ToolResult(
                success=False,
                error=str(e),
                suggestion="Ensure the position matches an existing node with boundary conditions."
            )

    def _tool_add_cargo(self, params: Dict[str, Any]) -> ToolResult:
        """Add a cargo item to the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        from grillex.core.cargo import Cargo

        name = params["name"]
        cog_position = params["cog_position"]
        mass = params["mass"]
        dimensions = params.get("dimensions", [1.0, 1.0, 1.0])

        # Create the cargo
        cargo = Cargo(name)
        cargo.set_cog(cog_position)
        cargo.set_mass(mass)
        cargo.set_dimensions(dimensions)

        # Note: We add the cargo to the list but don't generate FE elements
        # since there are no connections yet. The cargo is purely for tracking/visualization.
        self.model.cargos.append(cargo)

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "cargo_id": cargo.id,
                "name": name,
                "cog_position": cog_position,
                "mass": mass,
                "dimensions": dimensions,
                "message": f"Cargo '{name}' added at COG {cog_position} with mass {mass} mT."
            }
        )

    def _tool_update_cargo(self, params: Dict[str, Any]) -> ToolResult:
        """Update cargo properties."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        cargo_id = params["cargo_id"]

        # Find the cargo by ID
        cargo = None
        for c in self.model.cargos:
            if c.id == cargo_id:
                cargo = c
                break

        if cargo is None:
            available_ids = [c.id for c in self.model.cargos]
            return ToolResult(
                success=False,
                error=f"Cargo with ID {cargo_id} not found",
                suggestion=f"Available cargo IDs: {available_ids}" if available_ids else "No cargos in model"
            )

        changes = []

        # Update name if provided
        if "name" in params and params["name"]:
            cargo.name = params["name"]
            changes.append(f"name -> {params['name']}")

        # Update mass if provided
        if "mass" in params and params["mass"] is not None:
            new_mass = params["mass"]
            cargo.mass = new_mass
            # Update the underlying point mass if already generated
            if cargo._point_mass is not None:
                cargo._point_mass.mass = new_mass
            changes.append(f"mass -> {new_mass} mT")

        # Update CoG position if provided
        if "cog_position" in params and params["cog_position"] is not None:
            new_cog = params["cog_position"]
            cargo.cog_position = new_cog
            changes.append(f"cog_position -> {new_cog}")

        # Update dimensions if provided
        if "dimensions" in params and params["dimensions"] is not None:
            new_dims = params["dimensions"]
            cargo.dimensions = new_dims
            changes.append(f"dimensions -> {new_dims}")

        if not changes:
            return ToolResult(
                success=True,
                result={"cargo_id": cargo_id, "message": "No changes specified"}
            )

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "cargo_id": cargo_id,
                "changes": changes,
                "message": f"Cargo '{cargo.name}' updated: {', '.join(changes)}. Re-analysis required."
            }
        )

    def _tool_delete_cargo(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a cargo from the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        cargo_id = params["cargo_id"]

        # Find the cargo by ID
        cargo_index = None
        for idx, c in enumerate(self.model.cargos):
            if c.id == cargo_id:
                cargo_index = idx
                break

        if cargo_index is None:
            available_ids = [c.id for c in self.model.cargos]
            return ToolResult(
                success=False,
                error=f"Cargo with ID {cargo_id} not found",
                suggestion=f"Available cargo IDs: {available_ids}" if available_ids else "No cargos in model"
            )

        # Remove from Python list
        removed_cargo = self.model.cargos.pop(cargo_index)

        # Note: The underlying C++ elements (point mass, springs) are not removed
        # This is a limitation of the current implementation
        # The model will need to be re-created for full removal

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "cargo_id": cargo_id,
                "name": removed_cargo.name,
                "message": f"Cargo '{removed_cargo.name}' removed from model. Note: underlying elements may remain. Re-analysis required."
            }
        )

    def _tool_add_cargo_connection(self, params: Dict[str, Any]) -> ToolResult:
        """Add a connection point to a cargo item."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        cargo_id = params["cargo_id"]
        position = params["position"]
        name = params.get("name")
        stiffness = params.get("stiffness")  # Defaults to stiff in add_connection
        cargo_offset = params.get("cargo_offset")
        loading_condition = params.get("loading_condition", "all")

        # Handle 3-element stiffness (expand to 6 with zero rotational stiffness)
        if stiffness is not None and len(stiffness) == 3:
            stiffness = [stiffness[0], stiffness[1], stiffness[2], 0.0, 0.0, 0.0]

        # Find the cargo by ID
        cargo = None
        for c in self.model.cargos:
            if c.id == cargo_id:
                cargo = c
                break

        if cargo is None:
            available_ids = [c.id for c in self.model.cargos]
            return ToolResult(
                success=False,
                error=f"Cargo with ID {cargo_id} not found",
                suggestion=f"Available cargo IDs: {available_ids}" if available_ids else "No cargos in model"
            )

        # Add the connection
        cargo.add_connection(
            structural_position=position,
            stiffness=stiffness,
            cargo_offset=cargo_offset,
            loading_condition=loading_condition,
            name=name
        )

        # Get the added connection (last one)
        added_connection = cargo.connections[-1]

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "cargo_id": cargo_id,
                "cargo_name": cargo.name,
                "connection_name": added_connection.name,
                "position": position,
                "stiffness": added_connection.stiffness,
                "message": f"Connection '{added_connection.name}' added to cargo '{cargo.name}' at position {position}"
            }
        )

    def _tool_delete_cargo_connection(self, params: Dict[str, Any]) -> ToolResult:
        """Delete a connection point from a cargo item."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        cargo_id = params["cargo_id"]
        connection_name = params["connection_name"]

        # Find the cargo by ID
        cargo = None
        for c in self.model.cargos:
            if c.id == cargo_id:
                cargo = c
                break

        if cargo is None:
            available_ids = [c.id for c in self.model.cargos]
            return ToolResult(
                success=False,
                error=f"Cargo with ID {cargo_id} not found",
                suggestion=f"Available cargo IDs: {available_ids}" if available_ids else "No cargos in model"
            )

        # Find and remove the connection
        connection_index = None
        for idx, conn in enumerate(cargo.connections):
            if conn.name == connection_name:
                connection_index = idx
                break

        if connection_index is None:
            available_names = [conn.name for conn in cargo.connections]
            return ToolResult(
                success=False,
                error=f"Connection '{connection_name}' not found in cargo '{cargo.name}'",
                suggestion=f"Available connections: {available_names}" if available_names else "No connections in cargo"
            )

        # Remove the connection
        cargo.connections.pop(connection_index)

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "cargo_id": cargo_id,
                "cargo_name": cargo.name,
                "connection_name": connection_name,
                "message": f"Connection '{connection_name}' removed from cargo '{cargo.name}'"
            }
        )

    def _tool_update_cargo_connection(self, params: Dict[str, Any]) -> ToolResult:
        """Update a cargo connection point's properties."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")

        cargo_id = params["cargo_id"]
        connection_name = params["connection_name"]

        # Find the cargo by ID
        cargo = None
        for c in self.model.cargos:
            if c.id == cargo_id:
                cargo = c
                break

        if cargo is None:
            available_ids = [c.id for c in self.model.cargos]
            return ToolResult(
                success=False,
                error=f"Cargo with ID {cargo_id} not found",
                suggestion=f"Available cargo IDs: {available_ids}" if available_ids else "No cargos in model"
            )

        # Find the connection
        connection = None
        for conn in cargo.connections:
            if conn.name == connection_name:
                connection = conn
                break

        if connection is None:
            available_names = [conn.name for conn in cargo.connections]
            return ToolResult(
                success=False,
                error=f"Connection '{connection_name}' not found in cargo '{cargo.name}'",
                suggestion=f"Available connections: {available_names}" if available_names else "No connections in cargo"
            )

        changes = []

        # Update name if provided
        if "new_name" in params and params["new_name"]:
            connection.name = params["new_name"]
            changes.append(f"name -> {params['new_name']}")

        # Update position if provided
        if "position" in params and params["position"] is not None:
            connection.structural_position = list(params["position"])
            changes.append(f"position -> {params['position']}")

        # Update stiffness if provided (only kx, ky, kz)
        if "stiffness" in params and params["stiffness"] is not None:
            new_stiffness = params["stiffness"]
            # Keep rotational stiffness (indices 3-5) unchanged, update translational (0-2)
            connection.stiffness[0] = new_stiffness[0]
            connection.stiffness[1] = new_stiffness[1]
            connection.stiffness[2] = new_stiffness[2]
            changes.append(f"stiffness [kx, ky, kz] -> {new_stiffness}")

        # Update loading condition if provided
        if "loading_condition" in params and params["loading_condition"]:
            from grillex.core.cargo import VALID_LOADING_CONDITIONS
            if params["loading_condition"] in VALID_LOADING_CONDITIONS:
                connection.loading_condition = params["loading_condition"]
                changes.append(f"loading_condition -> {params['loading_condition']}")
            else:
                return ToolResult(
                    success=False,
                    error=f"Invalid loading_condition: {params['loading_condition']}",
                    suggestion=f"Valid values: {VALID_LOADING_CONDITIONS}"
                )

        if not changes:
            return ToolResult(
                success=True,
                result={"message": "No changes specified"}
            )

        # Mark model as not analyzed
        try:
            self.model._cpp_model.clear_analysis()
        except Exception:
            pass

        return ToolResult(
            success=True,
            result={
                "cargo_id": cargo_id,
                "cargo_name": cargo.name,
                "connection_name": connection.name,
                "changes": changes,
                "message": f"Connection '{connection.name}' updated: {', '.join(changes)}"
            }
        )

    # ===== Plate Meshing Tools =====

    def _tool_add_plate(self, params: Dict[str, Any]) -> ToolResult:
        """Add a plate region to the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        plate = self.model.add_plate(
            corners=params["corners"],
            thickness=params["thickness"],
            material=params["material"],
            mesh_size=params.get("mesh_size", 0.5),
            element_type=params.get("element_type", "MITC4"),
            name=params.get("name")
        )

        return ToolResult(
            success=True,
            result={
                "plate_name": plate.name,
                "n_corners": len(plate.corners),
                "element_type": plate.element_type,
                "thickness": plate.thickness,
                "message": f"Plate '{plate.name}' created with {len(plate.corners)} corners"
            }
        )

    def _tool_set_edge_divisions(self, params: Dict[str, Any]) -> ToolResult:
        """Set edge divisions for structured meshing."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        plate_name = params["plate_name"]
        plates = self.model.get_plates()
        plate = None
        for p in plates:
            if p.name == plate_name:
                plate = p
                break

        if plate is None:
            return ToolResult(
                success=False,
                error=f"Plate '{plate_name}' not found",
                suggestion=f"Available plates: {[p.name for p in plates]}"
            )

        self.model.set_edge_divisions(
            plate=plate,
            edge_index=params["edge_index"],
            n_elements=params["n_elements"]
        )

        return ToolResult(
            success=True,
            result={
                "plate_name": plate_name,
                "edge_index": params["edge_index"],
                "n_elements": params["n_elements"],
                "message": f"Edge {params['edge_index']} set to {params['n_elements']} elements"
            }
        )

    def _tool_couple_plate_to_beam(self, params: Dict[str, Any]) -> ToolResult:
        """Couple a plate edge to a beam."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        # Find plate
        plate_name = params["plate_name"]
        plates = self.model.get_plates()
        plate = None
        for p in plates:
            if p.name == plate_name:
                plate = p
                break

        if plate is None:
            return ToolResult(
                success=False,
                error=f"Plate '{plate_name}' not found",
                suggestion=f"Available plates: {[p.name for p in plates]}"
            )

        # Find beam
        beam_id = params["beam_id"]
        beam = None
        for b in self.model.beams:
            if b.beam_id == beam_id:
                beam = b
                break

        if beam is None:
            return ToolResult(
                success=False,
                error=f"Beam with ID {beam_id} not found",
                suggestion=f"Available beam IDs: {[b.beam_id for b in self.model.beams]}"
            )

        offset = params.get("offset")
        releases = params.get("releases")

        self.model.couple_plate_to_beam(
            plate=plate,
            edge_index=params["edge_index"],
            beam=beam,
            offset=offset,
            releases=releases
        )

        return ToolResult(
            success=True,
            result={
                "plate_name": plate_name,
                "edge_index": params["edge_index"],
                "beam_id": beam_id,
                "has_offset": offset is not None,
                "message": f"Plate edge {params['edge_index']} coupled to beam {beam_id}"
            }
        )

    def _tool_add_support_curve(self, params: Dict[str, Any]) -> ToolResult:
        """Add a support along a plate edge."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        plate_name = params["plate_name"]
        plates = self.model.get_plates()
        plate = None
        for p in plates:
            if p.name == plate_name:
                plate = p
                break

        if plate is None:
            return ToolResult(
                success=False,
                error=f"Plate '{plate_name}' not found",
                suggestion=f"Available plates: {[p.name for p in plates]}"
            )

        self.model.add_support_curve(
            plate=plate,
            edge_index=params["edge_index"],
            ux=params.get("ux", False),
            uy=params.get("uy", False),
            uz=params.get("uz", False),
            rotation_about_edge=params.get("rotation_about_edge", False)
        )

        restrained = []
        if params.get("ux", False):
            restrained.append("UX")
        if params.get("uy", False):
            restrained.append("UY")
        if params.get("uz", False):
            restrained.append("UZ")
        if params.get("rotation_about_edge", False):
            restrained.append("R_EDGE")

        return ToolResult(
            success=True,
            result={
                "plate_name": plate_name,
                "edge_index": params["edge_index"],
                "restrained_dofs": restrained,
                "message": f"Support added to edge {params['edge_index']}: {', '.join(restrained) or 'none'}"
            }
        )

    def _tool_mesh_model(self, params: Dict[str, Any]) -> ToolResult:
        """Mesh all plates in the model."""
        if self.model is None:
            return ToolResult(success=False, error="No model created. Call create_model first.")

        verbose = params.get("verbose", False)

        stats = self.model.mesh(verbose=verbose)

        return ToolResult(
            success=True,
            result={
                "n_plate_nodes": stats.n_plate_nodes,
                "n_plate_elements": stats.n_plate_elements,
                "n_quad_elements": stats.n_quad_elements,
                "n_tri_elements": stats.n_tri_elements,
                "n_support_dofs": stats.n_support_dofs,
                "n_rigid_links": stats.n_rigid_links,
                "message": f"Meshed {stats.n_plate_elements} plate elements ({stats.n_quad_elements} quad, {stats.n_tri_elements} tri)"
            }
        )

    def _tool_get_plate_displacement(self, params: Dict[str, Any]) -> ToolResult:
        """Get displacement at a point within a plate element."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(success=False, error="Model not analyzed. Call analyze first.")

        element_id = params["element_id"]
        xi = params.get("xi", 0.0)
        eta = params.get("eta", 0.0)

        # Find element by ID
        element = None
        for elem in self.model.get_plate_elements():
            if elem.id == element_id:
                element = elem
                break

        if element is None:
            elements = self.model.get_plate_elements()
            return ToolResult(
                success=False,
                error=f"Plate element with ID {element_id} not found",
                suggestion=f"Available element IDs: {[e.id for e in elements[:10]]}{'...' if len(elements) > 10 else ''}"
            )

        disp = self.model.get_plate_displacement(element, xi=xi, eta=eta)

        return ToolResult(
            success=True,
            result={
                "element_id": element_id,
                "xi": xi,
                "eta": eta,
                "displacements": {
                    "UX": float(disp["UX"]),
                    "UY": float(disp["UY"]),
                    "UZ": float(disp["UZ"]),
                    "RX": float(disp["RX"]),
                    "RY": float(disp["RY"]),
                    "RZ": float(disp["RZ"])
                },
                "units": {"translations": "m", "rotations": "rad"}
            }
        )

    def _tool_get_plate_moments(self, params: Dict[str, Any]) -> ToolResult:
        """Get bending moments at a point within a plate element."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(success=False, error="Model not analyzed. Call analyze first.")

        element_id = params["element_id"]
        xi = params.get("xi", 0.0)
        eta = params.get("eta", 0.0)

        # Find element by ID
        element = None
        for elem in self.model.get_plate_elements():
            if elem.id == element_id:
                element = elem
                break

        if element is None:
            elements = self.model.get_plate_elements()
            return ToolResult(
                success=False,
                error=f"Plate element with ID {element_id} not found",
                suggestion=f"Available element IDs: {[e.id for e in elements[:10]]}{'...' if len(elements) > 10 else ''}"
            )

        moments = self.model.get_plate_moments(element, xi=xi, eta=eta)

        return ToolResult(
            success=True,
            result={
                "element_id": element_id,
                "xi": xi,
                "eta": eta,
                "moments": {
                    "Mx": float(moments["Mx"]),
                    "My": float(moments["My"]),
                    "Mxy": float(moments["Mxy"])
                },
                "units": "kN·m/m"
            }
        )

    def _tool_get_plate_stress(self, params: Dict[str, Any]) -> ToolResult:
        """Get stress at a point within a plate element."""
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(success=False, error="Model not analyzed. Call analyze first.")

        element_id = params["element_id"]
        surface = params.get("surface", "top")
        xi = params.get("xi", 0.0)
        eta = params.get("eta", 0.0)

        # Find element by ID
        element = None
        for elem in self.model.get_plate_elements():
            if elem.id == element_id:
                element = elem
                break

        if element is None:
            elements = self.model.get_plate_elements()
            return ToolResult(
                success=False,
                error=f"Plate element with ID {element_id} not found",
                suggestion=f"Available element IDs: {[e.id for e in elements[:10]]}{'...' if len(elements) > 10 else ''}"
            )

        stress = self.model.get_plate_stress(element, surface=surface, xi=xi, eta=eta)

        return ToolResult(
            success=True,
            result={
                "element_id": element_id,
                "surface": surface,
                "xi": xi,
                "eta": eta,
                "stress": {
                    "sigma_x": float(stress["sigma_x"]),
                    "sigma_y": float(stress["sigma_y"]),
                    "tau_xy": float(stress["tau_xy"])
                },
                "units": "kN/m²"
            }
        )

    def _tool_get_beam_line_data(self, params: Dict[str, Any]) -> ToolResult:
        """Get internal action diagram data for a beam.

        Returns x positions and values for plotting moment, shear, axial,
        or torsion diagrams along a beam.
        """
        if self.model is None:
            return ToolResult(success=False, error="No model created.")
        if not self.model.is_analyzed():
            return ToolResult(
                success=False,
                error="Model not analyzed. Call analyze first.",
                suggestion="Run the analyze tool before querying beam line data."
            )

        beam_id = params["beam_id"]
        action_type = params["action_type"]
        num_points = params.get("num_points", 100)
        load_case_id = params.get("load_case_id")

        # Find beam by ID
        beam = None
        for b in self.model.beams:
            if b.beam_id == beam_id:
                beam = b
                break

        if beam is None:
            available_ids = [b.beam_id for b in self.model.beams]
            return ToolResult(
                success=False,
                error=f"Beam with ID {beam_id} not found",
                suggestion=f"Available beam IDs: {available_ids}"
            )

        # Get load case if specified
        load_case = None
        if load_case_id is not None:
            load_cases = self.model._cpp_model.get_load_cases()
            for lc in load_cases:
                if lc.id == load_case_id:
                    load_case = lc
                    break
            if load_case is None:
                available_ids = [lc.id for lc in load_cases]
                return ToolResult(
                    success=False,
                    error=f"Load case with ID {load_case_id} not found",
                    suggestion=f"Available load case IDs: {available_ids}"
                )

        try:
            # If no load case specified, use the default
            if load_case is None:
                load_case = self.model._cpp_model.get_default_load_case()

            data = beam.get_line_data(
                action_type=action_type,
                model=self.model,
                num_points=num_points,
                load_case=load_case
            )

            # Add load case info to the result
            data["load_case_id"] = load_case.id if load_case else None
            data["load_case_name"] = load_case.name if load_case else None

            return ToolResult(success=True, result=data)
        except ValueError as e:
            return ToolResult(
                success=False,
                error=str(e),
                suggestion="Check that the action_type is valid: moment_y, moment_z, shear_y, shear_z, axial, torsion, bimoment, warping_torsion"
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
