# FEM Solver Web Application — Requirements Summary

## Overview

A web-based dashboard for a finite element beam solver, targeting the offshore industry. The application supports structural analysis of cargo transportation on barges, including seafastening and grillage design. Users can interact with the model through both a traditional UI and natural language via an LLM assistant.

---

## Core Components

### 1. FEM Solver Package
- Python package with C++ backend
- Beam element analysis for offshore cargo transport
- Capabilities: create beams, grillages, loads, boundary conditions, run analysis
- MCP server interface (optional, for external tool access)

### 2. Web Application
- React frontend with Three.js 3D viewer
- FastAPI backend
- Same repository as FEM package for tight integration

### 3. LLM Integration
- Claude API (Sonnet 4) for showcase/development
- Swappable to local LLM (Ollama/Llama 3.1) for office deployment
- Tool calling to manipulate models and query data

### 4. Database (Phase 2)
- PostgreSQL
- Stores projects, cargos (mass, CoG, geometry), vessels, analysis history
- LLM can query database to retrieve historical project data

---

## User Interface Layout

PyCharm-inspired collapsible panel design:

```
┌────────┬─────────────────────────────────┬────────┐
│ [icons]│                                 │[icons] │
│        │                                 │        │
│  Left  │         3D Viewer               │ Right  │
│  Panel │         (center)                │ Panel  │
│        │                                 │        │
│        │                                 │        │
└────────┴─────────────────────────────────┴────────┘
```

### Left Panel (collapsible)
- Model tree: beams, loads, boundary conditions
- Model settings: load case selection, units
- Action buttons: add beam, add load, run analysis

### Center
- Three.js 3D viewer
- Dropdown menu in top-left corner for view mode selection

#### 3D Viewer Modes

| Mode | Description | Display |
|------|-------------|---------|
| **Model View** | 3D rendered geometry | Solid/shaded barge, cargo blocks, beam profiles with realistic sections |
| **FEM View** | Finite element representation | Lines for beam elements, nodes as points, force arrows, boundary condition symbols |
| **Results View** | Analysis output visualization | Deflected shapes (scaled), animated deformation, displacement contours |
| **Capacity View** | Unity checks / utilization | Color-coded elements by utilization ratio (green → yellow → red), legend with scale |

Each mode shares the same camera/view state — switching modes preserves orientation and zoom level.

### Right Panel (collapsible, tabbed)
- Results tab: stress, displacement, utilization, reactions
- Chat tab: LLM conversation interface (messages above, input below)

---

## Interaction Modes

### Direct UI Control
- Buttons and forms to create/edit model elements
- Settings dropdowns for load cases
- Immediate geometry updates in viewer

### LLM Assistant
- Natural language commands: *"Add a grillage under the cargo with 1m spacing"*
- LLM calls tools that execute against the same model service as UI
- Responses displayed in chat panel, geometry updates in viewer

Both modes share a single `ModelService` backend — changes from either source reflect immediately.

---

## Architecture

```
Frontend (React)
    │
    ├── Direct API calls ──► /api/model/* endpoints
    │
    └── Chat API call ─────► /api/chat endpoint
                                  │
                                  ▼
                            Claude API (tool calling)
                                  │
                                  ▼
                            Tool execution
                                  │
    ┌─────────────────────────────┴─────────────────────────────┐
    │                                                           │
    ▼                                                           ▼
ModelService (shared)                                     Database (Phase 2)
    │
    ▼
FEM Solver Package (Python + C++)
```

---

## Repository Structure

```
fem-solver/
├── src/
│   ├── cpp/                    # C++ backend
│   └── python/fem_solver/      # Python package
├── mcp_server/                 # Optional MCP interface
├── webapp/
│   ├── backend/                # FastAPI
│   │   ├── routes/
│   │   │   ├── model.py        # Direct model endpoints
│   │   │   └── chat.py         # LLM chat endpoint
│   │   └── services/
│   │       └── model_service.py
│   └── frontend/               # React + Three.js
│       └── src/components/
│           ├── LeftPanel.tsx
│           ├── Viewer3D.tsx
│           ├── RightPanel.tsx
│           └── ChatPanel.tsx
└── pyproject.toml
```

---

## Phased Rollout

### Phase 1 — Showcase
- Claude API for LLM
- No database
- Direct model manipulation via UI and chat
- 3D visualization of models and results

### Phase 2 — Office Deployment
- Swap to local LLM (Ollama)
- Add PostgreSQL database
- Project/cargo search and retrieval
- Historical analysis lookup

---

## Technology Stack

| Layer | Technology |
|-------|------------|
| Frontend | React, TypeScript, Three.js (React Three Fiber), Tailwind |
| Backend | FastAPI, Python |
| FEM Solver | Python + C++ (your package) |
| LLM (Phase 1) | Claude API (Sonnet 4) |
| LLM (Phase 2) | Ollama + Llama 3.1 8B |
| Database (Phase 2) | PostgreSQL |
| Deployment | Docker Compose |

---

## Key Design Decisions

1. **Single repo** for package + webapp — enables tight integration when UI directly controls model
2. **Shared ModelService** — both UI and LLM use same underlying functions
3. **MCP server optional** — useful for Claude Desktop or external clients, but webapp calls package directly
4. **Swappable LLM provider** — abstract interface allows switching Claude ↔ local LLM via config
5. **Collapsible panels** — maximizes 3D viewer space while keeping tools accessible
