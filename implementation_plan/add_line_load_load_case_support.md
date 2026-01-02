# Implementation Plan: Add Load Case Support to add_line_load Tool

## Status: ✅ COMPLETED (2026-01-02)

## Problem Statement

The `add_line_load` LLM tool currently doesn't support specifying which load case to add the line load to. It always uses the default/first load case.

---

## Current Implementation

**File:** `src/grillex/llm/tools.py`

```python
def _tool_add_line_load(self, params: Dict[str, Any]) -> ToolResult:
    load_end = params.get("load_end", params["load_start"])
    self.model.add_line_load_by_coords(
        start_pos=params["beam_start"],
        end_pos=params["beam_end"],
        w_start=params["load_start"],
        w_end=load_end
        # Missing: load_case parameter
    )
```

The Python wrapper already supports load_case:
```python
def add_line_load_by_coords(self, start_pos, end_pos, w_start, w_end=None, load_case=None):
```

---

## Implementation Steps

### Step 1: Update Tool Schema

**File:** `src/grillex/llm/tools.py`

Find the `add_line_load` tool definition (around line 302) and add `load_case_id` property:

```python
{
    "name": "add_line_load",
    "description": "Apply a distributed load along a beam. The load is specified in global coordinates.",
    "input_schema": {
        "type": "object",
        "properties": {
            "beam_start": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 3,
                "maxItems": 3,
                "description": "Start position of beam [x, y, z] in meters"
            },
            "beam_end": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 3,
                "maxItems": 3,
                "description": "End position of beam [x, y, z] in meters"
            },
            "load_start": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 3,
                "maxItems": 3,
                "description": "Load intensity at start [wx, wy, wz] in kN/m. Negative Z = downward."
            },
            "load_end": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 3,
                "maxItems": 3,
                "description": "Load intensity at end [wx, wy, wz] in kN/m. If omitted, uses load_start (uniform)."
            },
            "load_case_id": {
                "type": "integer",
                "description": "Optional load case ID. If not specified, uses the first/default load case."
            }
        },
        "required": ["beam_start", "beam_end", "load_start"]
    }
}
```

---

### Step 2: Update Tool Handler

**File:** `src/grillex/llm/tools.py`

Update `_tool_add_line_load` method (around line 1767):

```python
def _tool_add_line_load(self, params: Dict[str, Any]) -> ToolResult:
    """Add a line load."""
    if self.model is None:
        return ToolResult(success=False, error="No model created. Call create_model first.")

    load_end = params.get("load_end", params["load_start"])

    # Find load case by ID if specified
    load_case = None
    load_case_id = params.get("load_case_id")
    if load_case_id is not None:
        for lc in self.model._cpp_model.get_load_cases():
            if lc.id == load_case_id:
                load_case = lc
                break
        if load_case is None:
            return ToolResult(
                success=False,
                error=f"Load case with ID {load_case_id} not found"
            )

    self.model.add_line_load_by_coords(
        start_pos=params["beam_start"],
        end_pos=params["beam_end"],
        w_start=params["load_start"],
        w_end=load_end,
        load_case=load_case  # Pass the load case
    )

    return ToolResult(
        success=True,
        result={
            "beam_start": params["beam_start"],
            "beam_end": params["beam_end"],
            "load_start": params["load_start"],
            "load_end": load_end,
            "load_case_id": load_case_id,
            "message": "Line load applied to beam"
        }
    )
```

---

## Files to Modify

| File | Changes |
|------|---------|
| `src/grillex/llm/tools.py` | Add `load_case_id` to schema and handler |

---

## Testing

```python
# Test with default load case
result = executor.execute("add_line_load", {
    "beam_start": [0, 0, 0],
    "beam_end": [6, 0, 0],
    "load_start": [0, 0, -10]
})
assert result.success

# Test with specific load case
lc = model.create_load_case("Live Load", LoadCaseType.Variable)
result = executor.execute("add_line_load", {
    "beam_start": [0, 0, 0],
    "beam_end": [6, 0, 0],
    "load_start": [0, 0, -5],
    "load_case_id": lc.id
})
assert result.success

# Verify load is in correct load case
loads = lc.get_line_loads()
assert len(loads) == 1
```

---

## Execution Notes (Completed 2026-01-02)

**Steps Taken:**
1. Updated tool schema in `src/grillex/llm/tools.py` (line 335) to add `load_case_id` property
2. Updated `_tool_add_line_load` handler method (line 1802) to:
   - Look up load case by ID if `load_case_id` parameter is provided
   - Return error if specified load case ID is not found
   - Pass the found load case to `add_line_load_by_coords`
   - Include `load_case_id` in the result dictionary
3. Added 3 test cases to `tests/python/test_phase12_llm_tooling.py`:
   - `test_add_line_load_default_load_case`: Tests that line loads work with default load case (load_case_id=None)
   - `test_add_line_load_with_load_case_id`: Tests that line loads can be added to specific load cases
   - `test_add_line_load_invalid_load_case_id`: Tests error handling for non-existent load case IDs

**Problems Encountered:**
- None

**Verification:**
- All 3 new tests passing ✓
- Ran: `PYTHONPATH=/home/user/grillex/src python3 -m pytest -o "addopts=-v" tests/python/test_phase12_llm_tooling.py -k "test_add_line_load"`

**Files Modified:**
- `src/grillex/llm/tools.py` - Schema and handler updates
- `tests/python/test_phase12_llm_tooling.py` - Added 3 test methods
