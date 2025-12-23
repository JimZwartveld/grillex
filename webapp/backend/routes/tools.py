"""Tool execution endpoints for the Grillex web application.

This module provides endpoints for executing Grillex tools and
retrieving model state. Both the UI and LLM use these same endpoints.
"""

from typing import Any, Dict, List
from fastapi import APIRouter, Depends, HTTPException

from ..services.model_service import ModelService, get_model_service
from ..schemas.responses import ToolResponse, ModelStateResponse

router = APIRouter(prefix="/api", tags=["tools"])


@router.post("/tools/{tool_name}", response_model=ToolResponse)
async def execute_tool(
    tool_name: str,
    params: Dict[str, Any],
    service: ModelService = Depends(get_model_service),
) -> ToolResponse:
    """Execute a Grillex tool by name.

    Args:
        tool_name: Name of the tool to execute.
        params: Tool parameters.
        service: ModelService dependency.

    Returns:
        ToolResponse with success status and result/error.
    """
    result = await service.execute_tool(tool_name, params)
    return ToolResponse(
        success=result.success,
        result=result.result,
        error=result.error,
        suggestion=result.suggestion,
    )


@router.get("/model/state", response_model=ModelStateResponse)
async def get_model_state(
    service: ModelService = Depends(get_model_service),
) -> ModelStateResponse:
    """Get the current model state.

    Returns:
        Complete model state for frontend rendering.
    """
    state = service.get_state_snapshot()
    return ModelStateResponse(**state)


@router.get("/tools", response_model=List[Dict[str, Any]])
async def get_available_tools(
    service: ModelService = Depends(get_model_service),
) -> List[Dict[str, Any]]:
    """Get the list of available tools.

    Returns:
        List of tool definitions with schemas.
    """
    return service.get_available_tools()


@router.get("/health")
async def health_check() -> Dict[str, str]:
    """Health check endpoint.

    Returns:
        Status OK message.
    """
    return {"status": "ok"}
