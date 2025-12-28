"""MCP (Model Context Protocol) server for Grillex structural analysis.

This module provides an MCP server that exposes Grillex's structural analysis
capabilities to LLM agents through the Model Context Protocol.

Example usage with Claude Desktop:
    Add to your claude_desktop_config.json:
    {
        "mcpServers": {
            "grillex": {
                "command": "grillex-mcp"
            }
        }
    }

Example usage with Docker:
    docker run -i grillex-mcp
"""

from .server import main, create_server

__all__ = ["main", "create_server"]
