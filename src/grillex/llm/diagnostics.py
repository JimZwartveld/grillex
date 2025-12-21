"""Diagnostics and self-healing support for LLM integration.

This module provides error analysis and fix suggestions to enable LLMs
to automatically diagnose and fix structural model issues in a self-healing
loop pattern.

The key function `get_fix_suggestions` takes a GrillexError and returns
a list of suggested tool calls that can fix the issue.

Usage:
    from grillex.core import GrillexError, ErrorCode
    from grillex.llm.diagnostics import get_fix_suggestions

    # After an analysis failure
    error = GrillexError(ErrorCode.UNCONSTRAINED_SYSTEM, "Model has rigid body modes")
    error.involved_dofs = [0, 1, 2, 3, 4, 5]  # First node unconstrained
    error.involved_nodes = [0]

    suggestions = get_fix_suggestions(error)
    # Returns list of tool calls that can fix the issue
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass

from grillex.core import ErrorCode, GrillexError, WarningCode, WarningSeverity, GrillexWarning


@dataclass
class FixSuggestion:
    """A suggested fix for an error or warning.

    Attributes:
        description: Human-readable description of the fix.
        tool_name: Name of the tool to call.
        tool_params: Parameters for the tool call.
        priority: Priority level (1 = highest, try first).
        confidence: Confidence that this fix will resolve the issue (0-1).
    """
    description: str
    tool_name: str
    tool_params: Dict[str, Any]
    priority: int = 1
    confidence: float = 0.8

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "description": self.description,
            "tool": self.tool_name,
            "params": self.tool_params,
            "priority": self.priority,
            "confidence": self.confidence
        }


def get_fix_suggestions(error: GrillexError) -> List[FixSuggestion]:
    """Generate fix suggestions for an error.

    Analyzes the error code and context to suggest tool calls that
    can resolve the issue.

    Args:
        error: GrillexError with code and diagnostic information.

    Returns:
        List of FixSuggestion objects, sorted by priority (1 = first to try).

    Example:
        >>> error = GrillexError.unconstrained([0, 1, 2], [0])
        >>> suggestions = get_fix_suggestions(error)
        >>> for s in suggestions:
        ...     print(f"{s.tool_name}: {s.description}")
    """
    if error.is_ok():
        return []

    suggestions = []

    # Dispatch based on error code
    if error.code == ErrorCode.UNCONSTRAINED_SYSTEM:
        suggestions.extend(_suggest_for_unconstrained(error))
    elif error.code == ErrorCode.SINGULAR_MATRIX:
        suggestions.extend(_suggest_for_singular(error))
    elif error.code == ErrorCode.INSUFFICIENT_CONSTRAINTS:
        suggestions.extend(_suggest_for_insufficient_constraints(error))
    elif error.code == ErrorCode.INVALID_MATERIAL:
        suggestions.extend(_suggest_for_invalid_material(error))
    elif error.code == ErrorCode.INVALID_SECTION:
        suggestions.extend(_suggest_for_invalid_section(error))
    elif error.code == ErrorCode.EMPTY_MODEL:
        suggestions.extend(_suggest_for_empty_model(error))
    elif error.code == ErrorCode.NOT_ANALYZED:
        suggestions.extend(_suggest_for_not_analyzed(error))
    elif error.code == ErrorCode.INVALID_NODE_REFERENCE:
        suggestions.extend(_suggest_for_invalid_node(error))
    elif error.code == ErrorCode.EMPTY_LOAD_CASE:
        suggestions.extend(_suggest_for_empty_load_case(error))
    else:
        # Generic suggestion
        suggestions.append(FixSuggestion(
            description="Check the model for issues based on the error message.",
            tool_name="get_model_info",
            tool_params={},
            priority=1,
            confidence=0.3
        ))

    # Sort by priority
    suggestions.sort(key=lambda s: s.priority)

    return suggestions


def _suggest_for_unconstrained(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for unconstrained system errors."""
    suggestions = []

    # If we know which DOFs are unconstrained
    if error.involved_dofs:
        # Calculate node ID (assuming 6 DOFs per node)
        node_ids = set()
        for dof in error.involved_dofs:
            node_ids.add(dof // 6)

        # For each unconstrained node, suggest fixing it
        for node_id in list(node_ids)[:3]:  # Limit to first 3 nodes
            suggestions.append(FixSuggestion(
                description=f"Fix node {node_id} to prevent rigid body motion",
                tool_name="fix_node",
                tool_params={
                    "position": f"[position of node {node_id}]"  # LLM needs to fill this
                },
                priority=1,
                confidence=0.9
            ))

    # If we know which nodes are involved
    if error.involved_nodes:
        for node_id in error.involved_nodes[:3]:
            suggestions.append(FixSuggestion(
                description=f"Fix node {node_id} (identified as unconstrained)",
                tool_name="fix_node",
                tool_params={
                    "position": f"[position of node {node_id}]"
                },
                priority=1,
                confidence=0.9
            ))

    # General suggestion
    suggestions.append(FixSuggestion(
        description="Add at least one fully fixed support to prevent rigid body motion. "
                    "Fix the first node of the structure (typically at origin).",
        tool_name="fix_node",
        tool_params={"position": "[0, 0, 0]"},
        priority=2,
        confidence=0.7
    ))

    # For 3D problems, may need to fix torsion
    suggestions.append(FixSuggestion(
        description="If using pin supports, also fix torsional DOF (RX) at one end "
                    "to prevent spinning about the beam axis.",
        tool_name="fix_dof",
        tool_params={"position": "[0, 0, 0]", "dof": "RX"},
        priority=3,
        confidence=0.5
    ))

    return suggestions


def _suggest_for_singular(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for singular matrix errors."""
    suggestions = []

    suggestions.append(FixSuggestion(
        description="Singular matrix typically means insufficient boundary conditions. "
                    "Add a fixed support at one end of the structure.",
        tool_name="fix_node",
        tool_params={"position": "[first_node_position]"},
        priority=1,
        confidence=0.8
    ))

    suggestions.append(FixSuggestion(
        description="Check for disconnected parts in the model. All elements should "
                    "be connected to form a continuous structure.",
        tool_name="get_model_info",
        tool_params={},
        priority=2,
        confidence=0.5
    ))

    suggestions.append(FixSuggestion(
        description="Ensure material and section properties are non-zero (E, A, I, J > 0).",
        tool_name="get_model_info",
        tool_params={},
        priority=3,
        confidence=0.4
    ))

    return suggestions


def _suggest_for_insufficient_constraints(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for insufficient constraints."""
    suggestions = []

    # Need at least 6 DOFs fixed for 3D problems
    suggestions.append(FixSuggestion(
        description="For a 3D problem, at least 6 DOFs must be constrained to prevent "
                    "rigid body motion. Fully fix one node (all 6 DOFs).",
        tool_name="fix_node",
        tool_params={"position": "[support_position]"},
        priority=1,
        confidence=0.9
    ))

    suggestions.append(FixSuggestion(
        description="For a simply supported beam, pin both ends (translations only) "
                    "and fix torsion at one end.",
        tool_name="pin_node",
        tool_params={"position": "[first_support]"},
        priority=2,
        confidence=0.7
    ))

    return suggestions


def _suggest_for_invalid_material(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for invalid material errors."""
    suggestions = []

    # Extract material name from error details if available
    material_name = error.details.get("material_name", "Steel")

    suggestions.append(FixSuggestion(
        description=f"Add material '{material_name}' before creating beams that use it.",
        tool_name="add_material",
        tool_params={
            "name": material_name,
            "E": 210000000,  # Steel default
            "nu": 0.3,
            "rho": 7.85e-3
        },
        priority=1,
        confidence=0.95
    ))

    return suggestions


def _suggest_for_invalid_section(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for invalid section errors."""
    suggestions = []

    section_name = error.details.get("section_name", "IPE300")

    suggestions.append(FixSuggestion(
        description=f"Add section '{section_name}' before creating beams that use it.",
        tool_name="add_section",
        tool_params={
            "name": section_name,
            "A": 0.00538,  # IPE300 defaults
            "Iy": 8.36e-5,
            "Iz": 6.04e-6,
            "J": 2.01e-7
        },
        priority=1,
        confidence=0.95
    ))

    return suggestions


def _suggest_for_empty_model(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for empty model errors."""
    suggestions = []

    suggestions.append(FixSuggestion(
        description="The model has no elements. Add at least one beam element.",
        tool_name="create_beam",
        tool_params={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "[section_name]",
            "material": "[material_name]"
        },
        priority=1,
        confidence=0.9
    ))

    suggestions.append(FixSuggestion(
        description="Before creating beams, ensure materials and sections are defined.",
        tool_name="add_material",
        tool_params={
            "name": "Steel",
            "E": 210000000,
            "nu": 0.3,
            "rho": 7.85e-3
        },
        priority=0,  # Do this first
        confidence=0.9
    ))

    return suggestions


def _suggest_for_not_analyzed(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for not analyzed errors."""
    return [
        FixSuggestion(
            description="Run analysis before querying results.",
            tool_name="analyze",
            tool_params={},
            priority=1,
            confidence=1.0
        )
    ]


def _suggest_for_invalid_node(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for invalid node reference errors."""
    suggestions = []

    suggestions.append(FixSuggestion(
        description="The specified node position doesn't exist. Check coordinates match an existing node.",
        tool_name="get_model_info",
        tool_params={},
        priority=1,
        confidence=0.7
    ))

    suggestions.append(FixSuggestion(
        description="Nodes are created automatically when beams are added. "
                    "Ensure the beam endpoints match the desired load/BC positions.",
        tool_name="create_beam",
        tool_params={
            "start_position": "[corrected_position]",
            "end_position": "[other_position]",
            "section": "[section]",
            "material": "[material]"
        },
        priority=2,
        confidence=0.5
    ))

    return suggestions


def _suggest_for_empty_load_case(error: GrillexError) -> List[FixSuggestion]:
    """Suggestions for empty load case errors."""
    return [
        FixSuggestion(
            description="Add at least one load to the model before running analysis.",
            tool_name="add_point_load",
            tool_params={
                "position": "[node_position]",
                "dof": "UZ",
                "value": -10.0
            },
            priority=1,
            confidence=0.9
        )
    ]


# =============================================================================
# Warning Analysis
# =============================================================================

def get_warning_advice(warning: GrillexWarning) -> str:
    """Get human-readable advice for addressing a warning.

    Args:
        warning: GrillexWarning to analyze.

    Returns:
        String with advice for addressing the warning.
    """
    advice_map = {
        WarningCode.EXTREME_ASPECT_RATIO: (
            "Beam has extreme aspect ratio. Very slender beams (L/h > 100) should use "
            "Euler-Bernoulli formulation. Very deep beams (L/h < 2) need Timoshenko "
            "formulation for accurate shear deformation."
        ),
        WarningCode.SMALL_ELEMENT: (
            "Very short element detected. This may cause numerical issues. Consider "
            "merging with adjacent elements or checking for unintended duplicate nodes."
        ),
        WarningCode.LARGE_ELEMENT: (
            "Very long element. For accurate internal force distribution, consider "
            "subdividing into smaller elements (2-3 per span is typical)."
        ),
        WarningCode.STIFFNESS_CONTRAST: (
            "Large stiffness contrast between adjacent elements (ratio > 1e6). This may "
            "cause ill-conditioning. Check for unit errors or unrealistic properties."
        ),
        WarningCode.NEAR_SINGULARITY: (
            "Stiffness matrix is poorly conditioned. Results may be inaccurate. Check "
            "for near-zero stiffnesses, extreme property ratios, or unstable supports."
        ),
        WarningCode.NEAR_ZERO_PROPERTY: (
            "Near-zero property value detected. This may indicate a unit error or "
            "accidentally small value. Verify the property is intentional."
        ),
        WarningCode.POSSIBLE_UNIT_ERROR: (
            "Property value suggests possible unit error. Common mistakes: E in MPa "
            "instead of kN/m² (should be 210000000, not 210000 for steel)."
        ),
        WarningCode.LARGE_DISPLACEMENT: (
            "Large displacements detected (>1% of span). Linear analysis assumes small "
            "displacements. Geometric nonlinear analysis may be needed for accuracy."
        ),
        WarningCode.HIGH_STRESS: (
            "High stress detected (may exceed yield). Check if material nonlinearity "
            "or plastic analysis should be considered."
        ),
    }

    return advice_map.get(
        warning.code,
        f"Review warning: {warning.message}"
    )


def analyze_model_health(
    errors: List[GrillexError],
    warnings: List[GrillexWarning]
) -> Dict[str, Any]:
    """Analyze overall model health based on errors and warnings.

    Args:
        errors: List of errors from model validation/analysis.
        warnings: List of warnings from model validation.

    Returns:
        Dictionary with health assessment and recommendations.
    """
    # Count severity levels
    high_warnings = sum(1 for w in warnings if w.severity == WarningSeverity.High)
    medium_warnings = sum(1 for w in warnings if w.severity == WarningSeverity.Medium)
    low_warnings = sum(1 for w in warnings if w.severity == WarningSeverity.Low)

    # Determine overall status
    if errors:
        status = "ERROR"
        message = f"Model has {len(errors)} error(s) that prevent analysis."
    elif high_warnings > 0:
        status = "WARNING"
        message = f"Model has {high_warnings} high-severity warning(s) that should be reviewed."
    elif medium_warnings > 0:
        status = "CAUTION"
        message = f"Model has {medium_warnings} medium-severity warning(s)."
    else:
        status = "OK"
        message = "Model passes validation checks."

    # Generate recommendations
    recommendations = []
    if errors:
        for error in errors[:3]:  # Limit to first 3 errors
            suggestions = get_fix_suggestions(error)
            if suggestions:
                recommendations.append({
                    "error": error.code_string(),
                    "fix": suggestions[0].description
                })

    for warning in warnings:
        if warning.severity == WarningSeverity.High:
            recommendations.append({
                "warning": warning.code_string(),
                "advice": get_warning_advice(warning)
            })

    return {
        "status": status,
        "message": message,
        "error_count": len(errors),
        "warning_counts": {
            "high": high_warnings,
            "medium": medium_warnings,
            "low": low_warnings
        },
        "recommendations": recommendations
    }
