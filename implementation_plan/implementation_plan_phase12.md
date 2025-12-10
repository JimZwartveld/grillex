## Phase 12: LLM Tooling

### Task 12.1: Add Type Hints and Docstrings
**Requirements:** R-LLM-001
**Dependencies:** All previous Python tasks
**Difficulty:** Medium

**Description:**
Ensure all Python functions have complete type hints and docstrings.

**Steps:**
1. Review all public functions in:
   - `grillex/core/model.py`
   - `grillex/core/loads.py`
   - `grillex/io/yaml_loader.py`
   - etc.

2. Add/update docstrings:
   ```python
   def add_beam(
       self,
       end_a_position: list[float],
       end_b_position: list[float],
       section: str,
       material: str,
       roll_angle: float = 0.0
   ) -> Beam:
       """
       Add a beam element to the model.

       Args:
           end_a_position: [x, y, z] coordinates of end A in meters.
           end_b_position: [x, y, z] coordinates of end B in meters.
           section: Name of section to use.
           material: Name of material to use.
           roll_angle: Rotation about beam axis in radians. Default 0.

       Returns:
           The created Beam object.

       Raises:
           ValueError: If section or material not found.
       """
   ```

3. Run type checker (mypy) to verify

**Acceptance Criteria:**
- [ ] All public functions have docstrings
- [ ] All parameters have type hints
- [ ] Units are documented
- [ ] mypy passes

---

### Task 12.2: Create MCP Tool Schemas
**Requirements:** R-LLM-002, R-LLM-003
**Dependencies:** Task 12.1
**Difficulty:** Medium

**Description:**
Create MCP (Model Context Protocol) tool definitions.

**Steps:**
1. Create `grillex/llm/tools.py`:
   ```python
   from typing import TypedDict

   class CreateBeamInput(TypedDict):
       end_a_position: list[float]
       end_b_position: list[float]
       section: str
       material: str
       roll_angle: float

   TOOLS = [
       {
           "name": "create_beam",
           "description": "Create a beam element between two points",
           "input_schema": {
               "type": "object",
               "properties": {
                   "end_a_position": {
                       "type": "array",
                       "items": {"type": "number"},
                       "description": "Start point [x, y, z] in meters"
                   },
                   # ... etc
               },
               "required": ["end_a_position", "end_b_position", "section", "material"]
           }
       },
       # More tools...
   ]
   ```

2. Create tool execution handlers

**Acceptance Criteria:**
- [ ] Tool schemas are valid JSON Schema
- [ ] Schemas are self-documenting
- [ ] Tools cover main modelling operations

---

### Task 12.3: Implement Self-Healing Loop Support
**Requirements:** R-LLM-004, R-LLM-005
**Dependencies:** Tasks 11.1, 12.2
**Difficulty:** Medium

**Description:**
Enable LLM to interpret errors and fix models.

**Steps:**
1. Create `grillex/llm/diagnostics.py`:
   ```python
   def get_fix_suggestions(error: GrillexError) -> list[dict]:
       """
       Generate LLM-friendly fix suggestions for an error.

       Returns list of suggested tool calls to fix the error.
       """
       if error.code == ErrorCode.UNCONSTRAINED_SYSTEM:
           return [
               {
                   "suggestion": "Add supports to constrain the model",
                   "tool": "add_support",
                   "example_params": {
                       "node_id": error.involved_dofs[0] // 6,
                       "dofs": ["UX", "UY", "UZ", "RX", "RY", "RZ"]
                   }
               }
           ]
       # ... other error types
   ```

2. Return suggestions as part of error response

**Acceptance Criteria:**
- [ ] Each error type has fix suggestions
- [ ] Suggestions are actionable tool calls
- [ ] LLM can execute suggestions to fix model

---

