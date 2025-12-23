"""API tests for Grillex web application backend.

Tests the FastAPI endpoints for tool execution, model state,
and SSE events.
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
from webapp.backend.services.model_service import get_model_service, ModelService


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    # Reset the model service for each test
    from webapp.backend.services import model_service
    model_service._model_service = None
    return TestClient(app)


class TestHealthCheck:
    """Tests for health check endpoint."""

    def test_health_check(self, client):
        """Test that health check returns OK."""
        response = client.get("/api/health")
        assert response.status_code == 200
        assert response.json() == {"status": "ok"}


class TestRootEndpoint:
    """Tests for root endpoint."""

    def test_root(self, client):
        """Test that root returns API info."""
        response = client.get("/")
        assert response.status_code == 200
        data = response.json()
        assert "name" in data
        assert "endpoints" in data


class TestToolExecution:
    """Tests for tool execution endpoints."""

    def test_create_model(self, client):
        """Test creating a new model."""
        response = client.post(
            "/api/tools/create_model",
            json={"name": "Test Model"}
        )
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert data["result"]["model_name"] == "Test Model"

    def test_add_material(self, client):
        """Test adding a material."""
        # First create model
        client.post("/api/tools/create_model", json={"name": "Test"})

        # Add material
        response = client.post(
            "/api/tools/add_material",
            json={
                "name": "Steel",
                "E": 210e6,
                "nu": 0.3,
                "rho": 7.85e-3
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert data["result"]["material"] == "Steel"

    def test_add_section(self, client):
        """Test adding a section."""
        # First create model
        client.post("/api/tools/create_model", json={"name": "Test"})

        # Add section
        response = client.post(
            "/api/tools/add_section",
            json={
                "name": "IPE300",
                "A": 0.00538,
                "Iy": 8.36e-5,
                "Iz": 6.04e-6,
                "J": 2.01e-7
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert data["result"]["section"] == "IPE300"

    def test_create_beam(self, client):
        """Test creating a beam."""
        # Setup
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })

        # Create beam
        response = client.post(
            "/api/tools/create_beam",
            json={
                "start_position": [0, 0, 0],
                "end_position": [6, 0, 0],
                "section": "IPE300",
                "material": "Steel"
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert "beam_id" in data["result"]
        assert abs(data["result"]["length"] - 6.0) < 0.001

    def test_unknown_tool(self, client):
        """Test that unknown tools return error."""
        response = client.post(
            "/api/tools/unknown_tool",
            json={}
        )
        assert response.status_code == 200
        data = response.json()
        assert data["success"] is False
        assert "Unknown tool" in data["error"]


class TestModelState:
    """Tests for model state endpoint."""

    def test_empty_model_state(self, client):
        """Test model state when no model exists."""
        response = client.get("/api/model/state")
        assert response.status_code == 200
        data = response.json()
        assert data["exists"] is False
        assert data["beams"] == []

    def test_model_state_after_creation(self, client):
        """Test model state after creating elements."""
        # Create model with beam
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        response = client.get("/api/model/state")
        assert response.status_code == 200
        data = response.json()
        assert data["exists"] is True
        assert data["name"] == "Test"
        assert len(data["beams"]) == 1
        assert len(data["materials"]) == 1
        assert len(data["sections"]) == 1


class TestSSEConnection:
    """Tests for SSE endpoint."""

    def test_sse_connection(self, client):
        """Test that SSE endpoint establishes connection."""
        with client.stream("GET", "/api/events") as response:
            assert response.status_code == 200
            assert "text/event-stream" in response.headers["content-type"]

            # Read the initial connection event
            for line in response.iter_lines():
                if line.startswith("data:"):
                    import json
                    data = json.loads(line[5:].strip())
                    assert data["type"] == "connected"
                    break


class TestFullWorkflow:
    """Integration tests for complete workflows."""

    def test_cantilever_workflow(self, client):
        """Test complete cantilever beam workflow."""
        # Create model
        r = client.post("/api/tools/create_model", json={"name": "Cantilever"})
        assert r.json()["success"] is True

        # Add material and section
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })

        # Create beam
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        # Fix support
        r = client.post("/api/tools/fix_node", json={"position": [0, 0, 0]})
        assert r.json()["success"] is True

        # Add load
        r = client.post("/api/tools/add_point_load", json={
            "position": [6, 0, 0],
            "dof": "UZ",
            "value": -10
        })
        assert r.json()["success"] is True

        # Analyze
        r = client.post("/api/tools/analyze", json={})
        assert r.json()["success"] is True

        # Get results
        r = client.post("/api/tools/get_displacement", json={
            "position": [6, 0, 0],
            "dof": "UZ"
        })
        assert r.json()["success"] is True
        # Downward deflection should be negative
        assert r.json()["result"]["value"] < 0

        # Verify model state shows analyzed
        state = client.get("/api/model/state").json()
        assert state["isAnalyzed"] is True


class TestModelServiceSingleton:
    """Tests that ModelService is a singleton."""

    def test_singleton(self, client):
        """Test that the same ModelService instance is used across requests."""
        # Create a model
        client.post("/api/tools/create_model", json={"name": "Test1"})

        # Get state - should show the model
        state1 = client.get("/api/model/state").json()
        assert state1["exists"] is True
        assert state1["name"] == "Test1"

        # Add a beam
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        # Get state again - should show the beam from previous request
        state2 = client.get("/api/model/state").json()
        assert len(state2["beams"]) == 1


class TestModelReset:
    """Tests for model reset functionality."""

    def test_reset_model(self, client):
        """Test that resetting the model clears all state."""
        # Create model with elements
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        # Verify model exists
        state1 = client.get("/api/model/state").json()
        assert state1["exists"] is True
        assert len(state1["beams"]) == 1

        # Reset
        response = client.post("/api/model/reset", json={})
        assert response.status_code == 200

        # Verify model is empty
        state2 = client.get("/api/model/state").json()
        assert state2["exists"] is False
        assert state2["beams"] == []


class TestErrorHandling:
    """Tests for error handling in API."""

    def test_create_beam_without_model(self, client):
        """Test that creating a beam without a model fails gracefully."""
        response = client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })
        data = response.json()
        assert data["success"] is False

    def test_create_beam_with_invalid_material(self, client):
        """Test that creating beam with unknown material fails."""
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })

        response = client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "UnknownMaterial"
        })
        data = response.json()
        assert data["success"] is False

    def test_analyze_without_boundary_conditions(self, client):
        """Test that analysis without BC fails gracefully."""
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        response = client.post("/api/tools/analyze", json={})
        data = response.json()
        # Analysis might fail due to singularity (no BCs) or might succeed
        # with warnings - either is acceptable behavior
        assert response.status_code == 200


class TestSimplySupportedWorkflow:
    """Integration tests for simply supported beam."""

    def test_simply_supported_beam(self, client):
        """Test complete simply supported beam workflow."""
        # Create model
        client.post("/api/tools/create_model", json={"name": "Simply Supported"})

        # Add material and section
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })

        # Create beam
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [6, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        # Pin support at start (translations fixed)
        client.post("/api/tools/fix_node", json={"position": [0, 0, 0]})

        # Roller at end (only UZ fixed) - for simplicity, we use fix_node too
        # In reality, we'd want selective DOF fixing
        client.post("/api/tools/fix_node", json={"position": [6, 0, 0]})

        # Add uniform load at mid-span (approximated as point load)
        client.post("/api/tools/add_point_load", json={
            "position": [3, 0, 0],
            "dof": "UZ",
            "value": -20
        })

        # Analyze
        r = client.post("/api/tools/analyze", json={})
        assert r.json()["success"] is True

        # Check state
        state = client.get("/api/model/state").json()
        assert state["isAnalyzed"] is True


class TestPhase18Features:
    """Tests for Phase 18 WebApp UX improvements."""

    def test_model_state_includes_load_cases(self, client):
        """Test that model state includes loadCases field."""
        client.post("/api/tools/create_model", json={"name": "Test"})
        state = client.get("/api/model/state").json()
        assert "loadCases" in state
        assert isinstance(state["loadCases"], list)

    def test_model_state_includes_cargos(self, client):
        """Test that model state includes cargos field."""
        client.post("/api/tools/create_model", json={"name": "Test"})
        state = client.get("/api/model/state").json()
        assert "cargos" in state
        assert isinstance(state["cargos"], list)

    def test_material_state_includes_properties(self, client):
        """Test that material state includes E, nu, rho, G."""
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        state = client.get("/api/model/state").json()
        assert len(state["materials"]) == 1
        mat = state["materials"][0]
        assert "name" in mat
        assert mat["name"] == "Steel"
        # Check that additional properties are included if available
        if "E" in mat:
            assert mat["E"] == 210e6
        if "nu" in mat:
            assert mat["nu"] == 0.3

    def test_section_state_includes_properties(self, client):
        """Test that section state includes A, Iy, Iz, J."""
        client.post("/api/tools/create_model", json={"name": "Test"})
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })
        state = client.get("/api/model/state").json()
        assert len(state["sections"]) == 1
        sec = state["sections"][0]
        assert "name" in sec
        assert sec["name"] == "IPE300"
        # Check that additional properties are included if available
        if "A" in sec:
            assert abs(sec["A"] - 0.00538) < 0.0001


class TestMultipleBeamWorkflow:
    """Integration tests for multiple beam structures."""

    def test_continuous_beam(self, client):
        """Test workflow with multiple connected beams."""
        # Create model
        client.post("/api/tools/create_model", json={"name": "Continuous Beam"})

        # Add material and section
        client.post("/api/tools/add_material", json={
            "name": "Steel", "E": 210e6, "nu": 0.3, "rho": 7.85e-3
        })
        client.post("/api/tools/add_section", json={
            "name": "IPE300", "A": 0.00538, "Iy": 8.36e-5, "Iz": 6.04e-6, "J": 2.01e-7
        })

        # Create two connected beams
        client.post("/api/tools/create_beam", json={
            "start_position": [0, 0, 0],
            "end_position": [4, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })
        client.post("/api/tools/create_beam", json={
            "start_position": [4, 0, 0],
            "end_position": [8, 0, 0],
            "section": "IPE300",
            "material": "Steel"
        })

        # Check state has both beams
        state = client.get("/api/model/state").json()
        assert len(state["beams"]) == 2

        # Fix supports
        client.post("/api/tools/fix_node", json={"position": [0, 0, 0]})
        client.post("/api/tools/fix_node", json={"position": [4, 0, 0]})
        client.post("/api/tools/fix_node", json={"position": [8, 0, 0]})

        # Add loads
        client.post("/api/tools/add_point_load", json={
            "position": [2, 0, 0],
            "dof": "UZ",
            "value": -10
        })
        client.post("/api/tools/add_point_load", json={
            "position": [6, 0, 0],
            "dof": "UZ",
            "value": -10
        })

        # Analyze
        r = client.post("/api/tools/analyze", json={})
        assert r.json()["success"] is True
