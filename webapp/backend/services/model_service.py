"""Model service for managing the structural model state.

This service wraps the ToolExecutor from grillex.llm.tools and provides
state management for the web application. It broadcasts events via SSE
when the model changes.
"""

from typing import Any, Dict, List, Optional
import sys
import os

# Add the src directory to path if needed
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../src"))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from grillex.core import StructuralModel, DOFIndex
from grillex.llm.tools import ToolExecutor, ToolResult, TOOLS

from .event_service import EventService, get_event_service


class ModelService:
    """Singleton service holding the current model state.

    This service wraps the ToolExecutor and provides:
    - Model state management
    - Tool execution with automatic event broadcasting
    - State snapshot generation for frontend

    Attributes:
        model: The current StructuralModel (or None if not created).
        executor: The ToolExecutor for running tools.
    """

    def __init__(self, event_service: Optional[EventService] = None):
        """Initialize the model service.

        Args:
            event_service: Optional EventService for SSE broadcasting.
        """
        self.model: Optional[StructuralModel] = None
        self.executor: ToolExecutor = ToolExecutor()
        self._event_service = event_service or get_event_service()
        self._command_history: List[Dict[str, Any]] = []  # For future undo/redo

    async def execute_tool(
        self, tool_name: str, params: Dict[str, Any]
    ) -> ToolResult:
        """Execute a tool and broadcast state change.

        Args:
            tool_name: Name of the tool to execute.
            params: Tool parameters.

        Returns:
            ToolResult with success status and result/error.
        """
        # Execute the tool
        result = self.executor.execute(tool_name, params)

        # Update our model reference if a model was created
        if tool_name == "create_model" and result.success:
            self.model = self.executor.model

        # Record command for future undo/redo
        self._command_history.append({
            "tool_name": tool_name,
            "params": params,
            "success": result.success,
        })

        # Broadcast state change if successful
        if result.success:
            event_type = self._get_event_type(tool_name)
            await self._event_service.broadcast(
                event_type=event_type,
                tool_name=tool_name,
                data={
                    "result": result.to_dict(),
                    "state": self.get_state_snapshot(),
                },
            )

        return result

    def _get_event_type(self, tool_name: str) -> str:
        """Map tool name to event type."""
        event_map = {
            "create_model": "model_created",
            "add_material": "material_added",
            "add_section": "section_added",
            "create_beam": "beam_added",
            "fix_node": "bc_added",
            "pin_node": "bc_added",
            "fix_dof": "bc_added",
            "add_point_load": "load_added",
            "add_line_load": "load_added",
            "add_spring": "spring_added",
            "analyze": "analysis_complete",
            "analyze_modes": "modal_analysis_complete",
            "analyze_nonlinear": "nonlinear_analysis_complete",
        }
        return event_map.get(tool_name, "model_updated")

    def get_state_snapshot(self) -> Dict[str, Any]:
        """Return serializable model state for frontend.

        Returns:
            Dictionary containing model state including nodes, beams,
            loads, BCs, analysis status, and results.
        """
        if self.model is None:
            return {
                "exists": False,
                "name": None,
                "nodes": [],
                "beams": [],
                "materials": [],
                "sections": [],
                "loads": [],
                "boundaryConditions": [],
                "springs": [],
                "isAnalyzed": False,
                "results": None,
            }

        # Get materials
        materials = [
            {"name": name}
            for name in self.model._materials.keys()
        ]

        # Get sections
        sections = [
            {"name": name}
            for name in self.model._sections.keys()
        ]

        # Get nodes from C++ model
        nodes = []
        try:
            for node in self.model._cpp_model.get_all_nodes():
                nodes.append({
                    "id": node.id,
                    "position": [node.x, node.y, node.z],
                })
        except Exception:
            pass  # Node access not available

        # Get beams from Python model wrapper
        beams = []
        for beam in self.model.beams:
            beams.append({
                "id": beam.beam_id,
                "start": list(beam.start_pos),
                "end": list(beam.end_pos),
                "section": beam.section.name,
                "material": beam.material.name,
                "length": beam.length,
            })

        # Boundary conditions and loads are managed by the C++ model internally
        # We don't track them separately in the Python wrapper
        boundary_conditions: List[Dict[str, Any]] = []
        loads: List[Dict[str, Any]] = []

        # Get springs
        springs = []
        try:
            if hasattr(self.model._cpp_model, 'springs'):
                for spring in self.model._cpp_model.springs:
                    springs.append({
                        "id": spring.id(),
                        "node1": spring.node_i.id,
                        "node2": spring.node_j.id,
                        "isNonlinear": spring.is_nonlinear(),
                        "hasGap": spring.has_gap(),
                    })
        except Exception:
            pass  # Spring access not available

        # Get results if analyzed
        results = None
        if self.model.is_analyzed():
            results = {
                "displacements": self._get_displacement_results(),
                "reactions": [],  # Reactions will be empty for now
            }

        return {
            "exists": True,
            "name": self.model.name,
            "nodes": nodes,
            "beams": beams,
            "materials": materials,
            "sections": sections,
            "loads": loads,
            "boundaryConditions": boundary_conditions,
            "springs": springs,
            "isAnalyzed": self.model.is_analyzed(),
            "results": results,
        }

    def _get_displacement_results(self) -> List[Dict[str, Any]]:
        """Get displacement results for all nodes."""
        if not self.model or not self.model.is_analyzed():
            return []

        results = []
        try:
            for node in self.model._cpp_model.get_all_nodes():
                pos = [node.x, node.y, node.z]
                try:
                    displacements = {
                        "UX": self.model.get_displacement_at(pos, DOFIndex.UX),
                        "UY": self.model.get_displacement_at(pos, DOFIndex.UY),
                        "UZ": self.model.get_displacement_at(pos, DOFIndex.UZ),
                        "RX": self.model.get_displacement_at(pos, DOFIndex.RX),
                        "RY": self.model.get_displacement_at(pos, DOFIndex.RY),
                        "RZ": self.model.get_displacement_at(pos, DOFIndex.RZ),
                    }
                    results.append({
                        "nodeId": node.id,
                        "position": pos,
                        "displacements": displacements,
                    })
                except Exception:
                    pass
        except Exception:
            pass  # Node access not available
        return results

    def get_available_tools(self) -> List[Dict[str, Any]]:
        """Get the list of available tools.

        Returns:
            List of tool definitions from grillex.llm.tools.TOOLS.
        """
        return TOOLS


# Global singleton instance
_model_service: Optional[ModelService] = None


def get_model_service() -> ModelService:
    """Get the global ModelService singleton.

    Returns:
        The ModelService instance.
    """
    global _model_service
    if _model_service is None:
        _model_service = ModelService()
    return _model_service
