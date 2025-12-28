# Grillex MCP Server

This directory contains the Docker configuration for running Grillex as an MCP (Model Context Protocol) server.

## Quick Start

### Build the Docker image

```bash
docker build -f mcp/Dockerfile -t grillex-mcp .
```

### Run the MCP server

```bash
docker run -i --rm grillex-mcp
```

## Claude Desktop Integration

Add the following to your `claude_desktop_config.json`:

### Using Docker

```json
{
  "mcpServers": {
    "grillex": {
      "command": "docker",
      "args": ["run", "-i", "--rm", "grillex-mcp"]
    }
  }
}
```

### Using installed package

If you've installed Grillex with MCP support (`pip install grillex[mcp]`):

```json
{
  "mcpServers": {
    "grillex": {
      "command": "grillex-mcp"
    }
  }
}
```

## Available Tools

The MCP server exposes all Grillex structural analysis tools:

| Tool | Description |
|------|-------------|
| `create_model` | Create a new structural model |
| `add_material` | Add a material to the model |
| `add_section` | Add a cross-section to the model |
| `create_beam` | Create a beam element |
| `fix_node` | Fix all DOFs at a node |
| `pin_node` | Pin a node (translations only) |
| `add_point_load` | Apply a point load |
| `add_line_load` | Apply a distributed load |
| `analyze` | Run static analysis |
| `analyze_modes` | Run eigenvalue analysis |
| `get_displacement` | Get displacement results |
| `get_reactions` | Get reaction forces |
| `get_internal_actions` | Get internal forces/moments |
| ... and more |

## Example Conversation

```
User: Create a simple cantilever beam model with a point load

Claude: I'll create a cantilever beam model for you.

[Uses create_model tool]
[Uses add_material tool with Steel properties]
[Uses add_section tool with IPE300]
[Uses create_beam tool]
[Uses fix_node tool at the support]
[Uses add_point_load tool at the tip]
[Uses analyze tool]
[Uses get_displacement tool to show results]

The analysis is complete. The tip displacement is 5.2mm downward.
```

## Development

### Running locally without Docker

```bash
# Install with MCP support
pip install -e .[mcp]

# Run the server
grillex-mcp
```

### Testing the server

```bash
# Test that the server starts correctly
echo '{"jsonrpc": "2.0", "method": "tools/list", "id": 1}' | grillex-mcp
```
