"""Chat endpoint for LLM integration with Claude API.

This module provides the chat endpoint that interfaces with Claude API
for natural language model manipulation using tool calling.
"""

import json
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, asdict

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

from ..services.model_service import ModelService, get_model_service
from grillex.llm.tools import TOOLS

router = APIRouter(prefix="/api", tags=["chat"])


# =============================================================================
# Request/Response Models
# =============================================================================

class ChatMessage(BaseModel):
    """A single message in the conversation."""

    role: str  # "user" or "assistant"
    content: str


class ChatRequest(BaseModel):
    """Request body for chat endpoint."""

    message: str
    history: List[ChatMessage] = []


class ToolCall(BaseModel):
    """A tool call made during the chat."""

    tool_name: str
    params: Dict[str, Any]
    result: Dict[str, Any]


class ChatResponse(BaseModel):
    """Response from chat endpoint."""

    text: str
    tool_calls: List[ToolCall] = []
    model_updated: bool = False
    error: Optional[str] = None


# =============================================================================
# System Prompt
# =============================================================================

SYSTEM_PROMPT = """You are a structural engineering assistant for Grillex,
a finite element analysis tool for offshore structures, barges, and cargo operations.

You can manipulate structural models using the available tools:
- create_model: Start a new model
- add_material, add_section: Define material and cross-section properties
- create_beam: Add beam elements between two points
- fix_node, pin_node, fix_dof: Apply boundary conditions
- add_point_load, add_line_load: Apply loads
- add_spring: Add spring elements (can be nonlinear)
- analyze: Run linear static analysis
- analyze_modes: Run eigenvalue analysis for natural frequencies
- analyze_nonlinear: Run nonlinear analysis for gap/tension-only springs
- get_displacement, get_reactions, get_internal_actions: Query results
- get_model_info: Get summary of current model

**Units:**
- Length: meters (m)
- Force: kilonewtons (kN)
- Mass: metric tonnes (mT)
- Stress: kN/m² (kPa)

**Best Practices:**
1. Always start with create_model before adding elements
2. Add materials and sections before creating beams
3. Apply boundary conditions before analysis
4. Suggest running analysis after adding loads
5. For cantilever: fix one end, load the other
6. For simply supported: pin supports at both ends

**Example workflow:**
User: "Create a 6m steel cantilever beam with a 10kN downward load at the tip"
1. create_model("Cantilever Example")
2. add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
3. add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
4. create_beam([0,0,0], [6,0,0], "IPE300", "Steel")
5. fix_node([0,0,0])
6. add_point_load([6,0,0], "UZ", -10.0)
7. analyze()
8. get_displacement([6,0,0], "UZ") to report deflection
"""


# =============================================================================
# Claude API Integration
# =============================================================================

def convert_tools_to_claude_format(tools: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Convert Grillex tools to Claude API format."""
    claude_tools = []
    for tool in tools:
        claude_tools.append({
            "name": tool["name"],
            "description": tool["description"],
            "input_schema": tool["input_schema"],
        })
    return claude_tools


async def call_claude_with_tools(
    message: str,
    history: List[ChatMessage],
    model_service: ModelService,
) -> ChatResponse:
    """Call Claude API and execute any tool calls.

    Args:
        message: User's message.
        history: Previous conversation messages.
        model_service: ModelService for executing tools.

    Returns:
        ChatResponse with Claude's response and any tool calls made.
    """
    if not ANTHROPIC_AVAILABLE:
        return ChatResponse(
            text="Anthropic SDK not installed. Install with: pip install anthropic",
            error="anthropic_not_installed",
        )

    client = anthropic.AsyncAnthropic()

    # Build messages list
    messages = []
    for msg in history:
        messages.append({"role": msg.role, "content": msg.content})
    messages.append({"role": "user", "content": message})

    # Convert tools to Claude format
    claude_tools = convert_tools_to_claude_format(TOOLS)

    tool_calls: List[ToolCall] = []
    model_updated = False

    try:
        # Tool calling loop
        while True:
            response = await client.messages.create(
                model="claude-sonnet-4-20250514",
                max_tokens=4096,
                system=SYSTEM_PROMPT,
                tools=claude_tools,
                messages=messages,
            )

            # Check for tool use blocks
            tool_use_blocks = [
                block for block in response.content
                if block.type == "tool_use"
            ]

            if not tool_use_blocks:
                # No more tool calls, extract final text response
                text_blocks = [
                    block.text for block in response.content
                    if block.type == "text"
                ]
                final_text = " ".join(text_blocks)
                return ChatResponse(
                    text=final_text,
                    tool_calls=tool_calls,
                    model_updated=model_updated,
                )

            # Execute each tool call
            tool_results = []
            for tool_use in tool_use_blocks:
                result = await model_service.execute_tool(
                    tool_use.name,
                    tool_use.input,
                )

                tool_calls.append(ToolCall(
                    tool_name=tool_use.name,
                    params=tool_use.input,
                    result=result.to_dict(),
                ))

                if result.success:
                    model_updated = True

                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_use.id,
                    "content": json.dumps(result.to_dict()),
                })

            # Add assistant response and tool results to messages
            messages.append({"role": "assistant", "content": response.content})
            messages.append({"role": "user", "content": tool_results})

    except anthropic.APIConnectionError as e:
        return ChatResponse(
            text="Failed to connect to Claude API. Please check your internet connection.",
            error=f"api_connection_error: {str(e)}",
        )
    except anthropic.RateLimitError as e:
        return ChatResponse(
            text="Claude API rate limit exceeded. Please try again later.",
            error=f"rate_limit_error: {str(e)}",
        )
    except anthropic.AuthenticationError as e:
        return ChatResponse(
            text="Invalid API key. Please set ANTHROPIC_API_KEY environment variable.",
            error=f"auth_error: {str(e)}",
        )
    except anthropic.APIStatusError as e:
        return ChatResponse(
            text=f"Claude API error: {e.message}",
            error=f"api_error: {str(e)}",
        )
    except Exception as e:
        return ChatResponse(
            text=f"An error occurred: {str(e)}",
            error=f"unknown_error: {str(e)}",
        )


# =============================================================================
# Endpoints
# =============================================================================

@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    service: ModelService = Depends(get_model_service),
) -> ChatResponse:
    """Process chat message through Claude API with tool calling.

    Args:
        request: Chat request with message and optional history.
        service: ModelService dependency.

    Returns:
        Chat response with Claude's text and any tool calls made.
    """
    return await call_claude_with_tools(
        message=request.message,
        history=request.history,
        model_service=service,
    )


@router.post("/chat/stream")
async def chat_stream(
    request: ChatRequest,
    service: ModelService = Depends(get_model_service),
) -> StreamingResponse:
    """Stream chat response with SSE for real-time feedback.

    This is an optional enhancement that streams the response as it's generated.
    """
    async def generate():
        # Send initial event
        yield f"data: {json.dumps({'type': 'start'})}\n\n"

        try:
            response = await call_claude_with_tools(
                message=request.message,
                history=request.history,
                model_service=service,
            )

            # Stream tool calls as they happen
            for tool_call in response.tool_calls:
                yield f"data: {json.dumps({'type': 'tool_call', 'data': asdict(tool_call) if hasattr(tool_call, '__dict__') else tool_call.model_dump()})}\n\n"

            # Send final response
            yield f"data: {json.dumps({'type': 'complete', 'text': response.text, 'model_updated': response.model_updated})}\n\n"

        except Exception as e:
            yield f"data: {json.dumps({'type': 'error', 'error': str(e)})}\n\n"

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
        },
    )


@router.get("/chat/status")
async def chat_status() -> Dict[str, Any]:
    """Check chat endpoint status and dependencies."""
    return {
        "anthropic_available": ANTHROPIC_AVAILABLE,
        "tools_available": len(TOOLS),
        "model": "claude-sonnet-4-20250514" if ANTHROPIC_AVAILABLE else None,
    }
