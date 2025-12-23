"""Tests for chat endpoint.

Tests the chat API endpoint for LLM integration.
Note: Full Claude API tests require a valid API key.
"""

import pytest
import sys
import os

# Add src to path for grillex imports
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../src"))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from fastapi.testclient import TestClient

from webapp.backend.main import app
from webapp.backend.services.model_service import get_model_service
from webapp.backend.services import model_service as ms_module


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    # Reset the model service for each test
    ms_module._model_service = None
    return TestClient(app)


class TestChatStatus:
    """Tests for chat status endpoint."""

    def test_chat_status(self, client):
        """Test that chat status returns dependency info."""
        response = client.get("/api/chat/status")
        assert response.status_code == 200
        data = response.json()
        assert "anthropic_available" in data
        assert "tools_available" in data
        assert data["tools_available"] > 0


class TestChatEndpoint:
    """Tests for chat endpoint."""

    def test_chat_request_model(self, client):
        """Test that chat endpoint accepts requests."""
        # This test will fail without an API key, but should return
        # a proper error response rather than crashing
        response = client.post(
            "/api/chat",
            json={"message": "Hello"}
        )
        assert response.status_code == 200
        data = response.json()

        # Response should have the expected structure
        assert "text" in data
        assert "tool_calls" in data
        assert "model_updated" in data

    def test_chat_with_history(self, client):
        """Test that chat endpoint accepts conversation history."""
        response = client.post(
            "/api/chat",
            json={
                "message": "What can you help me with?",
                "history": [
                    {"role": "user", "content": "Hello"},
                    {"role": "assistant", "content": "Hello! I'm a structural engineering assistant."}
                ]
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert "text" in data

    def test_chat_response_structure(self, client):
        """Test that chat response has correct structure."""
        response = client.post(
            "/api/chat",
            json={"message": "test"}
        )
        assert response.status_code == 200
        data = response.json()

        # Verify response structure
        assert isinstance(data.get("text"), str)
        assert isinstance(data.get("tool_calls"), list)
        assert isinstance(data.get("model_updated"), bool)


class TestChatStreamEndpoint:
    """Tests for streaming chat endpoint."""

    def test_chat_stream_connection(self, client):
        """Test that streaming endpoint returns SSE response."""
        with client.stream(
            "POST",
            "/api/chat/stream",
            json={"message": "test"}
        ) as response:
            assert response.status_code == 200
            assert "text/event-stream" in response.headers.get("content-type", "")

            # Read first event (should be 'start')
            for line in response.iter_lines():
                if line.startswith("data:"):
                    import json
                    data = json.loads(line[5:].strip())
                    assert data.get("type") == "start"
                    break


class TestChatToolConversion:
    """Tests for tool format conversion."""

    def test_tools_available(self):
        """Test that Grillex tools are available."""
        from grillex.llm.tools import TOOLS
        assert len(TOOLS) > 0

        # Check tool structure
        for tool in TOOLS:
            assert "name" in tool
            assert "description" in tool
            assert "input_schema" in tool

    def test_convert_tools_format(self):
        """Test tool conversion to Claude format."""
        from webapp.backend.routes.chat import convert_tools_to_claude_format
        from grillex.llm.tools import TOOLS

        claude_tools = convert_tools_to_claude_format(TOOLS)

        assert len(claude_tools) == len(TOOLS)
        for tool in claude_tools:
            assert "name" in tool
            assert "description" in tool
            assert "input_schema" in tool


class TestSystemPrompt:
    """Tests for system prompt."""

    def test_system_prompt_exists(self):
        """Test that system prompt is defined."""
        from webapp.backend.routes.chat import SYSTEM_PROMPT
        assert SYSTEM_PROMPT is not None
        assert len(SYSTEM_PROMPT) > 100  # Should be substantial

    def test_system_prompt_mentions_tools(self):
        """Test that system prompt mentions available tools."""
        from webapp.backend.routes.chat import SYSTEM_PROMPT
        assert "create_model" in SYSTEM_PROMPT
        assert "add_material" in SYSTEM_PROMPT
        assert "create_beam" in SYSTEM_PROMPT
        assert "analyze" in SYSTEM_PROMPT

    def test_system_prompt_mentions_units(self):
        """Test that system prompt mentions units."""
        from webapp.backend.routes.chat import SYSTEM_PROMPT
        assert "meter" in SYSTEM_PROMPT.lower()
        assert "kn" in SYSTEM_PROMPT.lower() or "kilonewton" in SYSTEM_PROMPT.lower()
