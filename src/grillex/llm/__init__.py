"""LLM tooling and MCP server integration.

This module provides tools for integrating Grillex with Large Language Models:

- `tools`: MCP tool definitions and executor for LLM function calling
- `diagnostics`: Error analysis and self-healing suggestions

Usage:
    from grillex.llm import TOOLS, ToolExecutor, get_fix_suggestions

    # Get tool definitions for LLM
    tool_definitions = TOOLS

    # Execute tool calls
    executor = ToolExecutor()
    result = executor.execute("create_model", {"name": "My Bridge"})

    # Get fix suggestions for errors
    from grillex.core import GrillexError, ErrorCode
    error = GrillexError(ErrorCode.UNCONSTRAINED_SYSTEM, "...")
    suggestions = get_fix_suggestions(error)
"""

from .tools import (
    TOOLS,
    ToolResult,
    ToolExecutor,
    execute_tool,
    get_tool_by_name,
)

from .diagnostics import (
    FixSuggestion,
    get_fix_suggestions,
    get_warning_advice,
    analyze_model_health,
)

__all__ = [
    # Tools module
    "TOOLS",
    "ToolResult",
    "ToolExecutor",
    "execute_tool",
    "get_tool_by_name",
    # Diagnostics module
    "FixSuggestion",
    "get_fix_suggestions",
    "get_warning_advice",
    "analyze_model_health",
]
