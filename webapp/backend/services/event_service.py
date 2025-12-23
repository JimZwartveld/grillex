"""Server-Sent Events (SSE) service for real-time updates.

This module provides event broadcasting to connected clients via SSE.
When model changes occur (e.g., beam added, analysis run), events are
pushed to all subscribers so the frontend can update in real-time.
"""

import asyncio
import json
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, asdict


@dataclass
class ModelEvent:
    """An event representing a change to the model state.

    Attributes:
        event_type: Type of event (e.g., "model_created", "beam_added").
        tool_name: Name of the tool that triggered this event.
        data: Additional event data.
    """

    event_type: str
    tool_name: str
    data: Dict[str, Any]

    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(asdict(self))


class EventService:
    """Manages SSE connections and broadcasts events to all subscribers.

    This service maintains a list of active subscribers (asyncio Queues)
    and broadcasts events to all of them when model changes occur.

    Example:
        event_service = EventService()

        # In an endpoint, subscribe to events
        queue = await event_service.subscribe()

        # In another endpoint, broadcast an event
        await event_service.broadcast("beam_added", "create_beam", {"beam_id": 1})
    """

    def __init__(self):
        """Initialize the event service."""
        self._subscribers: List[asyncio.Queue] = []
        self._lock = asyncio.Lock()

    async def subscribe(self) -> asyncio.Queue:
        """Add a new SSE subscriber.

        Returns:
            An asyncio Queue that will receive events.
        """
        queue: asyncio.Queue = asyncio.Queue()
        async with self._lock:
            self._subscribers.append(queue)
        return queue

    async def unsubscribe(self, queue: asyncio.Queue) -> None:
        """Remove a subscriber.

        Args:
            queue: The subscriber's queue to remove.
        """
        async with self._lock:
            if queue in self._subscribers:
                self._subscribers.remove(queue)

    async def broadcast(
        self, event_type: str, tool_name: str, data: Dict[str, Any]
    ) -> None:
        """Send an event to all subscribers.

        Args:
            event_type: Type of event.
            tool_name: Name of the tool that triggered this event.
            data: Event data.
        """
        event = ModelEvent(event_type=event_type, tool_name=tool_name, data=data)

        async with self._lock:
            # Remove any closed queues and broadcast to active ones
            active_subscribers = []
            for queue in self._subscribers:
                try:
                    await queue.put(event)
                    active_subscribers.append(queue)
                except Exception:
                    # Queue is closed or broken, skip it
                    pass
            self._subscribers = active_subscribers

    @property
    def subscriber_count(self) -> int:
        """Get the number of active subscribers."""
        return len(self._subscribers)


# Global singleton instance
_event_service: Optional[EventService] = None


def get_event_service() -> EventService:
    """Get the global EventService singleton.

    Returns:
        The EventService instance.
    """
    global _event_service
    if _event_service is None:
        _event_service = EventService()
    return _event_service
