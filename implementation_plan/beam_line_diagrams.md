# Implementation Plan: Beam Line Diagram Data API

## Overview

Add a unified API for retrieving beam internal action line data (moment, shear, axial, torsion, bimoment) for plotting diagrams in the webapp.

## Requirements

- Provide line data (x positions + values) for all internal action types
- Find and return extrema (min/max values and positions)
- Support all 8 action types: moment_y, moment_z, shear_y, shear_z, axial, torsion, bimoment, warping_torsion
- Add LLM tool for programmatic access

---

## Implementation Steps

### Step 1: Add ActionType Enum

**File:** `src/grillex/core/model_wrapper.py`

Add at top of file:

```python
from enum import Enum

class ActionType(Enum):
    """Type of internal action for line diagrams."""
    MOMENT_Y = "moment_y"
    MOMENT_Z = "moment_z"
    SHEAR_Y = "shear_y"
    SHEAR_Z = "shear_z"
    AXIAL = "axial"
    TORSION = "torsion"
    BIMOMENT = "bimoment"
    WARPING_TORSION = "warping_torsion"
```

---

### Step 2: Add `get_line_data()` Method to Beam Class

**File:** `src/grillex/core/model_wrapper.py`

Add to `Beam` class:

```python
def get_line_data(
    self,
    action_type: Union[str, ActionType],
    model: 'StructuralModel',
    num_points: int = 100,
    load_case: Optional[_CppLoadCase] = None
) -> Dict[str, Any]:
    """Get line diagram data for plotting.

    Args:
        action_type: Type of internal action (ActionType enum or string)
        model: StructuralModel (must be analyzed)
        num_points: Number of points to sample along beam
        load_case: Optional load case (uses active if None)

    Returns:
        Dictionary with:
            - x: List of positions along beam [m]
            - values: List of action values [kN or kN.m]
            - units: Unit string for display
            - label: Human-readable label
            - beam_id: Beam ID
            - beam_name: Beam name
            - beam_length: Total beam length [m]
            - extrema: List of {x, value, type, is_global} for min/max
    """
```

**Implementation Logic:**
1. Convert string to ActionType enum if needed
2. Validate model is analyzed
3. Sample `num_points` evenly spaced positions along beam
4. For each position, call `get_internal_actions_at()` and extract relevant field
5. For warping actions (bimoment, warping_torsion), use `get_warping_internal_actions()`
6. Find global min/max from sampled values
7. Return dictionary with all data

---

### Step 3: Add `_find_extrema()` Helper

**File:** `src/grillex/core/model_wrapper.py`

Add to `Beam` class:

```python
def _find_extrema(
    self,
    values: np.ndarray,
    x_positions: np.ndarray
) -> List[Dict[str, Any]]:
    """Find global extrema in values array.

    Returns:
        List of {x, value, type, is_global} dicts
    """
```

**Logic:**
1. Find index of minimum value
2. Find index of maximum value
3. Return list with min and max extrema (if different)

---

### Step 4: Export ActionType

**File:** `src/grillex/core/__init__.py`

Add to imports and `__all__`:

```python
from .model_wrapper import ActionType
__all__ = [..., "ActionType"]
```

---

### Step 5: Add LLM Tool Schema

**File:** `src/grillex/llm/tools.py`

Add to TOOLS list:

```python
{
    "name": "get_beam_line_data",
    "description": "Get internal action diagram data for a beam. Returns x positions and values for plotting moment, shear, axial, or torsion diagrams.",
    "input_schema": {
        "type": "object",
        "properties": {
            "beam_id": {
                "type": "integer",
                "description": "ID of the beam"
            },
            "action_type": {
                "type": "string",
                "enum": ["moment_y", "moment_z", "shear_y", "shear_z", "axial", "torsion", "bimoment", "warping_torsion"],
                "description": "Type of internal action diagram"
            },
            "num_points": {
                "type": "integer",
                "description": "Number of sample points (default 100)"
            },
            "load_case_id": {
                "type": "integer",
                "description": "Optional load case ID"
            }
        },
        "required": ["beam_id", "action_type"]
    }
}
```

---

### Step 6: Add Tool Executor

**File:** `src/grillex/llm/tools.py`

Add method to `ToolExecutor` class:

```python
def _execute_get_beam_line_data(self, params: Dict[str, Any]) -> ToolResult:
    """Execute get_beam_line_data tool."""
    # 1. Validate model exists and is analyzed
    # 2. Find beam by ID
    # 3. Find load case by ID (optional)
    # 4. Call beam.get_line_data()
    # 5. Return ToolResult with data
```

Add dispatch in `execute()` method:

```python
elif tool_name == "get_beam_line_data":
    return self._execute_get_beam_line_data(params)
```

---

## Files to Modify

| File | Changes |
|------|---------|
| `src/grillex/core/model_wrapper.py` | Add `ActionType` enum, `get_line_data()`, `_find_extrema()` |
| `src/grillex/core/__init__.py` | Export `ActionType` |
| `src/grillex/llm/tools.py` | Add tool schema and executor |

---

## Testing Checklist

- [ ] `get_line_data()` returns correct structure for all action types
- [ ] Extrema correctly identifies min/max values
- [ ] Warping actions work for 14-DOF elements
- [ ] Warping actions return zeros for 12-DOF elements
- [ ] LLM tool executes correctly
- [ ] Error handling for non-analyzed model
- [ ] Error handling for invalid beam ID
