"""Server-Sent Events endpoint for real-time updates.

This module provides the SSE endpoint that clients connect to
for receiving real-time model state updates.
"""

import asyncio
import json
from fastapi import APIRouter, Depends
from fastapi.responses import StreamingResponse

from ..services.event_service import EventService, get_event_service, ModelEvent

router = APIRouter(prefix="/api", tags=["events"])


@router.get("/events")
async def event_stream(
    service: EventService = Depends(get_event_service),
) -> StreamingResponse:
    """SSE endpoint for real-time model updates.

    Clients connect to this endpoint to receive events when
    the model changes (beams added, analysis complete, etc.).

    Returns:
        StreamingResponse with SSE events.
    """
    queue = await service.subscribe()

    async def generate():
        """Generate SSE events from the queue."""
        try:
            # Send initial connection event
            yield f"data: {json.dumps({'type': 'connected', 'data': {}})}\n\n"

            while True:
                try:
                    # Wait for events with a timeout to allow cleanup
                    event: ModelEvent = await asyncio.wait_for(
                        queue.get(), timeout=30.0
                    )

                    # Format as SSE
                    event_data = {
                        "type": event.event_type,
                        "tool_name": event.tool_name,
                        "data": event.data,
                    }
                    yield f"data: {json.dumps(event_data)}\n\n"

                except asyncio.TimeoutError:
                    # Send keepalive comment
                    yield ": keepalive\n\n"

        except asyncio.CancelledError:
            # Client disconnected
            pass
        finally:
            await service.unsubscribe(queue)

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",  # Disable nginx buffering
        },
    )
