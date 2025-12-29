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
            "update_material": "material_updated",
            "delete_material": "material_deleted",
            "add_section": "section_added",
            "update_section": "section_updated",
            "delete_section": "section_deleted",
            "create_beam": "beam_added",
            "update_beam": "beam_updated",
            "delete_beam": "beam_deleted",
            "fix_node": "bc_added",
            "pin_node": "bc_added",
            "fix_dof": "bc_added",
            "remove_boundary_condition": "bc_removed",
            "add_point_load": "load_added",
            "add_line_load": "load_added",
            "add_spring": "spring_added",
            "update_cargo": "cargo_updated",
            "delete_cargo": "cargo_deleted",
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
                "boundaryConditions": [],
                "loadCases": [],
                "cargos": [],
                "springs": [],
                "isAnalyzed": False,
                "results": None,
            }

        # Get materials with full properties
        materials = []
        for name, mat in self.model._materials.items():
            mat_data = {"name": name}
            try:
                mat_data["E"] = mat.E
                mat_data["nu"] = mat.nu
                mat_data["rho"] = mat.rho
                mat_data["G"] = mat.G
            except AttributeError:
                pass  # Material properties not available
            materials.append(mat_data)

        # Get sections with full properties
        sections = []
        for name, sec in self.model._sections.items():
            sec_data = {"name": name}
            try:
                sec_data["A"] = sec.A
                sec_data["Iy"] = sec.Iy
                sec_data["Iz"] = sec.Iz
                sec_data["J"] = sec.J
                if hasattr(sec, 'Iw'):
                    sec_data["Iw"] = sec.Iw
                if hasattr(sec, 'ky'):
                    sec_data["ky"] = sec.ky
                if hasattr(sec, 'kz'):
                    sec_data["kz"] = sec.kz
            except AttributeError:
                pass  # Section properties not available
            sections.append(sec_data)

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

        # Boundary conditions from C++ model
        boundary_conditions: List[Dict[str, Any]] = []
        try:
            bc_handler = self.model._cpp_model.bc_handler
            for fixed_dof in bc_handler.get_fixed_dofs():
                # Convert DOF index to string
                dof_names = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ', 'WARP']
                dof_str = dof_names[fixed_dof.local_dof] if fixed_dof.local_dof < len(dof_names) else str(fixed_dof.local_dof)
                boundary_conditions.append({
                    "node_id": fixed_dof.node_id,
                    "dof": dof_str,
                    "value": fixed_dof.value,
                })
        except Exception:
            pass  # BC access not available

        # Get load cases from C++ model
        load_cases: List[Dict[str, Any]] = []
        try:
            cpp_load_cases = self.model._cpp_model.get_load_cases()
            for lc in cpp_load_cases:
                lc_data = {
                    "id": lc.id,
                    "name": lc.name,
                    "type": str(lc.type).split('.')[-1].lower(),  # Convert enum to string
                    "loads": [],
                }
                # Get nodal loads from the load case
                for load in lc.get_nodal_loads():
                    # Convert DOF enum to string
                    dof_names = ['UX', 'UY', 'UZ', 'RX', 'RY', 'RZ', 'WARP']
                    dof_str = dof_names[load.dof] if load.dof < len(dof_names) else str(load.dof)
                    lc_data["loads"].append({
                        "node_id": load.node_id,
                        "dof": dof_str,
                        "value": load.value,
                    })
                load_cases.append(lc_data)
        except Exception:
            pass  # Load case access not available

        # Get cargo items from Python model
        cargos: List[Dict[str, Any]] = []
        try:
            if hasattr(self.model, '_cargos'):
                for cargo in self.model._cargos:
                    cargo_data = {
                        "id": cargo.id,
                        "name": cargo.name,
                        "cogPosition": list(cargo.cog_position),
                        "dimensions": list(cargo.dimensions),
                        "mass": cargo.mass,
                        "connections": [],
                    }
                    if hasattr(cargo, 'connections'):
                        for conn in cargo.connections:
                            cargo_data["connections"].append({
                                "node_id": conn.node_id,
                                "cargoOffset": list(conn.offset),
                            })
                    cargos.append(cargo_data)
        except Exception:
            pass  # Cargo access not available

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
            "boundaryConditions": boundary_conditions,
            "loadCases": load_cases,
            "cargos": cargos,
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
