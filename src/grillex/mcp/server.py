#!/usr/bin/env python3
"""MCP Server for Grillex structural analysis.

This module implements a Model Context Protocol (MCP) server that exposes
Grillex's structural analysis tools to LLM agents.

The server maintains a stateful session with a StructuralModel instance,
allowing incremental model building and analysis.

Usage:
    # As a console script (installed via pip)
    grillex-mcp

    # As a Python module
    python -m grillex.mcp.server

    # In Docker
    docker run -i grillex-mcp
"""

from __future__ import annotations

import asyncio
import json
import logging
import sys
from typing import Any, Sequence

# MCP SDK imports
try:
    from mcp.server import Server
    from mcp.server.stdio import stdio_server
    from mcp.types import (
        Tool,
        TextContent,
        ImageContent,
        EmbeddedResource,
        LoggingLevel,
    )
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False
    # Provide stub types for type checking when MCP is not installed
    Server = None
    Tool = dict
    TextContent = dict

# Grillex imports
from grillex.llm.tools import TOOLS, ToolExecutor, ToolResult
from grillex.core import StructuralModel

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    stream=sys.stderr,
)
logger = logging.getLogger("grillex.mcp")


class GrillexMCPServer:
    """MCP Server that exposes Grillex structural analysis tools.

    This server maintains a stateful session with a ToolExecutor instance,
    allowing LLM agents to build and analyze structural models incrementally.

    Attributes:
        server: The MCP Server instance.
        executor: The ToolExecutor for executing Grillex tools.
    """

    def __init__(self) -> None:
        """Initialize the MCP server."""
        if not MCP_AVAILABLE:
            raise ImportError(
                "MCP SDK is not installed. Install it with: pip install mcp"
            )

        self.server = Server("grillex")
        self.executor = ToolExecutor()

        # Register handlers
        self._register_handlers()

        logger.info("Grillex MCP server initialized")

    def _register_handlers(self) -> None:
        """Register MCP protocol handlers."""

        @self.server.list_tools()
        async def list_tools() -> list[Tool]:
            """List all available Grillex tools."""
            logger.debug("Listing %d tools", len(TOOLS))
            return [
                Tool(
                    name=tool["name"],
                    description=tool["description"],
                    inputSchema=tool["input_schema"],
                )
                for tool in TOOLS
            ]

        @self.server.call_tool()
        async def call_tool(
            name: str, arguments: dict[str, Any] | None
        ) -> Sequence[TextContent | ImageContent | EmbeddedResource]:
            """Execute a Grillex tool.

            Args:
                name: The name of the tool to execute.
                arguments: The arguments to pass to the tool.

            Returns:
                A sequence containing the tool result as TextContent.
            """
            logger.info("Executing tool: %s", name)
            logger.debug("Arguments: %s", arguments)

            try:
                # Execute the tool
                result = self.executor.execute(name, arguments or {})

                # Format the result
                response = self._format_result(result)

                logger.info(
                    "Tool %s completed: success=%s", name, result.success
                )

                return [TextContent(type="text", text=response)]

            except Exception as e:
                logger.error("Tool %s failed: %s", name, str(e))
                error_result = ToolResult(
                    success=False,
                    error=str(e),
                    suggestion="Check the tool parameters and try again.",
                )
                return [
                    TextContent(
                        type="text",
                        text=self._format_result(error_result),
                    )
                ]

        @self.server.set_logging_level()
        async def set_logging_level(level: LoggingLevel) -> None:
            """Set the server logging level."""
            level_map = {
                "debug": logging.DEBUG,
                "info": logging.INFO,
                "warning": logging.WARNING,
                "error": logging.ERROR,
            }
            python_level = level_map.get(level.lower(), logging.INFO)
            logger.setLevel(python_level)
            logger.info("Logging level set to: %s", level)

    def _format_result(self, result: ToolResult) -> str:
        """Format a ToolResult as a JSON string.

        Args:
            result: The ToolResult to format.

        Returns:
            A formatted JSON string.
        """
        return json.dumps(result.to_dict(), indent=2, default=str)

    async def run(self) -> None:
        """Run the MCP server using stdio transport."""
        logger.info("Starting Grillex MCP server on stdio")

        async with stdio_server() as (read_stream, write_stream):
            await self.server.run(
                read_stream,
                write_stream,
                self.server.create_initialization_options(),
            )


def create_server() -> GrillexMCPServer:
    """Create a new Grillex MCP server instance.

    Returns:
        A configured GrillexMCPServer instance.

    Raises:
        ImportError: If the MCP SDK is not installed.
    """
    return GrillexMCPServer()


def main() -> None:
    """Entry point for the grillex-mcp console script."""
    if not MCP_AVAILABLE:
        print(
            "Error: MCP SDK is not installed.\n"
            "Install it with: pip install grillex[mcp]\n"
            "Or: pip install mcp",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        server = create_server()
        asyncio.run(server.run())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error("Server error: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
