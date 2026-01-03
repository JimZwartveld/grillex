# Phase 20: Vessel Motions

## Overview

This phase implements a Python front-end abstraction for vessel (barge) motion-induced loading. Similar to the Cargo abstraction (Phase 9), VesselMotions is a Python-level wrapper that generates load cases and load combinations from motion amplitude inputs.

**Purpose**: Simplify the workflow for marine/offshore structural analysis by automatically generating environmental load cases from vessel motion parameters (roll, pitch, heave) and combining them with gravity loads according to LRFD/ASD design code requirements.

**Input Options**:
1. **From Amplitudes**: User provides translational and rotational acceleration amplitudes directly
2. **From Noble Denton**: User provides motion amplitudes (degrees, G's) and the Noble Denton guidelines are used to compute acceleration combinations

**Output**:
- Environmental load cases (one per motion combination)
- A gravity (static) load case
- ULS-a and ULS-b load combinations for LRFD
- ASD load combinations (if selected)
- Separate combinations for "removal" vs "regular" operations

---

## Requirements Traceability

New requirements to be added:
- **R-VESSEL-001**: Vessel motion abstraction shall generate load cases from acceleration inputs
- **R-VESSEL-002**: Vessel motions shall support both amplitude-based and Noble Denton inputs
- **R-VESSEL-003**: Vessel motions shall generate LRFD and ASD load combinations
- **R-VESSEL-004**: Vessel motions shall support removal vs regular operation conditions

---

## Task 20.1: Define VesselMotions Base Class and Data Structures

**Requirements:** R-VESSEL-001, R-VESSEL-002
**Dependencies:** Phase 5 (Load Cases), Phase 9 (Cargo pattern reference)
**Difficulty:** Medium

**Description:**
Create the VesselMotions base class and supporting data structures in `grillex/core/vessel_motions.py`.

**Steps:**

1. Create `src/grillex/core/vessel_motions.py` with the following structure:

```python
from typing import List, Optional, Tuple, TYPE_CHECKING
from dataclasses import dataclass, field
from enum import Enum
import math
import itertools
import numpy as np

if TYPE_CHECKING:
    from grillex.core.model_wrapper import StructuralModel


class DesignMethod(Enum):
    """Design method for load combinations."""
    LRFD = "lrfd"  # Load and Resistance Factor Design
    ASD = "asd"    # Allowable Stress Design


class OperationType(Enum):
    """Type of marine operation."""
    REGULAR = "regular"   # Normal transport/operations
    REMOVAL = "removal"   # Lift-on/lift-off operations


@dataclass
class Acceleration:
    """Translational accelerations in global coordinates.

    Attributes:
        ax: Surge acceleration [m/s²] (positive toward bow)
        ay: Sway acceleration [m/s²] (positive toward port)
        az: Heave acceleration [m/s²] (positive upward)
    """
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.ax, self.ay, self.az])

    def __mul__(self, factor: float) -> "Acceleration":
        return Acceleration(self.ax * factor, self.ay * factor, self.az * factor)

    def __rmul__(self, factor: float) -> "Acceleration":
        return self.__mul__(factor)


@dataclass
class RotaryAcceleration:
    """Rotational accelerations about global axes.

    Attributes:
        rx: Roll acceleration [rad/s²] (about X axis, positive starboard down)
        ry: Pitch acceleration [rad/s²] (about Y axis, positive bow down)
        rz: Yaw acceleration [rad/s²] (about Z axis, positive clockwise from above)
    """
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.rx, self.ry, self.rz])

    def __mul__(self, factor: float) -> "RotaryAcceleration":
        return RotaryAcceleration(self.rx * factor, self.ry * factor, self.rz * factor)

    def __rmul__(self, factor: float) -> "RotaryAcceleration":
        return self.__mul__(factor)


@dataclass
class MotionCase:
    """A single motion combination with its accelerations.

    Attributes:
        name: Descriptive name (e.g., "Roll_Pos_Heave_Down")
        acceleration: Translational accelerations [m/s²]
        rotary_acceleration: Rotational accelerations [rad/s²]
    """
    name: str
    acceleration: Acceleration
    rotary_acceleration: RotaryAcceleration
```

2. Add the base `VesselMotions` class:

```python
class VesselMotions:
    """
    Base class for vessel motion loading.

    VesselMotions generates environmental load cases from vessel motion parameters.
    Each motion combination produces a load case with translational and rotational
    accelerations that create inertial forces on the structure.

    The coordinate system follows marine convention:
    - X axis: Points from stern to bow (surge positive forward)
    - Y axis: Points to port side (sway positive port)
    - Z axis: Points upward (heave positive up)

    Motion coupling assumptions:
    - Positive roll (rotation about X) causes negative sway acceleration
    - Positive pitch (rotation about Y) causes positive surge acceleration

    Attributes:
        name: Descriptive name for this motion set
        center_of_rotation: [x, y, z] location for acceleration calculations [m]
        motion_cases: List of generated MotionCase objects
        gravity: Gravitational acceleration [m/s²], default -9.81

    Subclasses:
        VesselMotionsFromAmplitudes: Direct amplitude input
        VesselMotionsFromNobleDenton: Noble Denton guidelines input
    """

    def __init__(
        self,
        name: str,
        center_of_rotation: List[float],
        gravity: float = -9.81
    ):
        if len(center_of_rotation) != 3:
            raise ValueError("center_of_rotation must be [x, y, z]")

        self.name = name
        self.center_of_rotation = list(center_of_rotation)
        self.gravity = gravity
        self.motion_cases: List[MotionCase] = []
        self._load_cases: List = []  # Generated load cases
        self._gravity_load_case = None
        self._generated = False

    @property
    def acceleration_combinations(self) -> List[List[float]]:
        """Return acceleration combinations as list of [ax, ay, az, rx, ry, rz].

        Must be implemented by subclasses.
        """
        raise NotImplementedError("Subclasses must implement acceleration_combinations")

    def _build_motion_cases(self) -> None:
        """Build MotionCase objects from acceleration_combinations."""
        self.motion_cases = []
        for i, combo in enumerate(self.acceleration_combinations):
            acc = Acceleration(combo[0], combo[1], combo[2])
            rot = RotaryAcceleration(combo[3], combo[4], combo[5])
            case = MotionCase(
                name=f"{self.name}_Env_{i+1}",
                acceleration=acc,
                rotary_acceleration=rot
            )
            self.motion_cases.append(case)

    def acceleration_at_point(
        self,
        case: MotionCase,
        point: List[float]
    ) -> np.ndarray:
        """
        Calculate total translational acceleration at a point.

        The acceleration at a point includes:
        - The base translational acceleration
        - Additional acceleration from rotary motion about center of rotation

        Args:
            case: MotionCase with accelerations
            point: [x, y, z] location of interest [m]

        Returns:
            [ax, ay, az] total acceleration at point [m/s²]
        """
        # Vector from center of rotation to point
        d = np.array(point) - np.array(self.center_of_rotation)

        # Cross product matrix for rotary acceleration
        r = case.rotary_acceleration.as_array()
        rot_matrix = np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]
        ])

        # Total acceleration = translational + rotational contribution
        return case.acceleration.as_array() + rot_matrix @ d
```

3. Export from `grillex/core/__init__.py`:
   - VesselMotions
   - VesselMotionsFromAmplitudes
   - VesselMotionsFromNobleDenton
   - DesignMethod
   - OperationType
   - Acceleration
   - RotaryAcceleration

**Acceptance Criteria:**
- [ ] VesselMotions base class defined with center_of_rotation
- [ ] Acceleration and RotaryAcceleration dataclasses work correctly
- [ ] acceleration_at_point correctly computes total acceleration including rotary effects
- [ ] DesignMethod and OperationType enums defined
- [ ] Classes are exported from grillex.core module

---

## Task 20.2: Implement VesselMotionsFromAmplitudes

**Requirements:** R-VESSEL-001, R-VESSEL-002
**Dependencies:** Task 20.1
**Difficulty:** Medium

**Description:**
Implement the amplitude-based vessel motions class that generates all combinations of positive/negative amplitudes.

**Steps:**

1. Add `VesselMotionsFromAmplitudes` class to `vessel_motions.py`:

```python
class VesselMotionsFromAmplitudes(VesselMotions):
    """
    Generate vessel motion load cases from acceleration amplitudes.

    Given maximum acceleration amplitudes, generates all physical combinations
    of positive/negative amplitudes while respecting motion coupling:
    - Positive roll causes negative sway (port-down = sway to starboard)
    - Positive pitch causes positive surge (bow-down = surge forward)

    This produces 16 unique combinations (2^4 independent sign choices with
    2 degrees of freedom constrained by coupling).

    Args:
        name: Descriptive name for this motion set
        center_of_rotation: [x, y, z] reference point for accelerations [m]
        acceleration: Maximum translational accelerations [m/s²]
        rotary_acceleration: Maximum rotational accelerations [rad/s²]
        gravity: Gravitational acceleration [m/s²], default -9.81

    Example:
        >>> from grillex.core import VesselMotionsFromAmplitudes, Acceleration, RotaryAcceleration
        >>>
        >>> motions = VesselMotionsFromAmplitudes(
        ...     name="Barge Transport",
        ...     center_of_rotation=[50.0, 0.0, 5.0],
        ...     acceleration=Acceleration(ax=1.5, ay=2.0, az=3.0),
        ...     rotary_acceleration=RotaryAcceleration(rx=0.15, ry=0.10, rz=0.05)
        ... )
        >>>
        >>> print(f"Generated {len(motions.motion_cases)} motion cases")
        Generated 16 motion cases
    """

    def __init__(
        self,
        name: str,
        center_of_rotation: List[float],
        acceleration: Acceleration,
        rotary_acceleration: RotaryAcceleration,
        gravity: float = -9.81
    ):
        super().__init__(name, center_of_rotation, gravity)

        self._acceleration = acceleration
        self._rotary_acceleration = rotary_acceleration

        # Build motion cases from combinations
        self._build_motion_cases()

    @property
    def acceleration(self) -> Acceleration:
        """Maximum translational acceleration amplitudes."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value: Acceleration) -> None:
        self._acceleration = value
        self._build_motion_cases()
        self._generated = False

    @property
    def rotary_acceleration(self) -> RotaryAcceleration:
        """Maximum rotational acceleration amplitudes."""
        return self._rotary_acceleration

    @rotary_acceleration.setter
    def rotary_acceleration(self, value: RotaryAcceleration) -> None:
        self._rotary_acceleration = value
        self._build_motion_cases()
        self._generated = False

    @property
    def acceleration_combinations(self) -> List[List[float]]:
        """
        Generate all combinations of acceleration signs.

        Applies coupling rules:
        - surge sign (ax) = pitch sign (ry): positive pitch -> bow down -> surge forward
        - sway sign (ay) = -roll sign (rx): positive roll -> starboard down -> sway to port

        Returns:
            List of [ax, ay, az, rx, ry, rz] combinations
        """
        # Generate all 2^6 = 64 sign combinations
        all_signs = list(itertools.product([-1, 1], repeat=6))

        # Filter by coupling rules:
        # - sign(ax) == sign(ry): surge couples with pitch
        # - sign(ay) == -sign(rx): sway couples opposite to roll
        valid_signs = [
            s for s in all_signs
            if s[0] == s[4] and s[1] == -s[3]  # ax==ry, ay==-rx
        ]

        # Build amplitude combinations
        amplitudes = [
            self._acceleration.ax,
            self._acceleration.ay,
            self._acceleration.az,
            self._rotary_acceleration.rx,
            self._rotary_acceleration.ry,
            self._rotary_acceleration.rz
        ]

        combinations = [
            [s * a for s, a in zip(signs, amplitudes)]
            for signs in valid_signs
        ]

        # Remove duplicates (can occur if some amplitudes are zero)
        unique = []
        for combo in combinations:
            if combo not in unique:
                unique.append(combo)

        return unique

    def __repr__(self) -> str:
        return (
            f"VesselMotionsFromAmplitudes(name='{self.name}', "
            f"center={self.center_of_rotation}, "
            f"acc={self._acceleration}, rot={self._rotary_acceleration})"
        )
```

2. Add unit tests in `tests/python/test_phase17_vessel_motions.py`:
   - Test that 16 combinations are generated (for non-zero amplitudes)
   - Test coupling rules are correctly applied
   - Test that acceleration_at_point includes rotary effects
   - Test with zero amplitudes produces fewer combinations

**Acceptance Criteria:**
- [ ] VesselMotionsFromAmplitudes generates correct number of combinations
- [ ] Coupling rules correctly constrain surge/pitch and sway/roll signs
- [ ] Changing acceleration/rotary_acceleration rebuilds motion cases
- [ ] Zero amplitudes are handled correctly (reduced combinations)

---

## Task 20.3: Implement VesselMotionsFromNobleDenton

**Requirements:** R-VESSEL-001, R-VESSEL-002
**Dependencies:** Task 20.1
**Difficulty:** Medium

**Description:**
Implement the Noble Denton guidelines-based vessel motions class.

**Steps:**

1. Add `VesselMotionsFromNobleDenton` class to `vessel_motions.py`:

```python
class VesselMotionsFromNobleDenton(VesselMotions):
    """
    Generate vessel motion load cases from Noble Denton guidelines.

    Computes accelerations based on motion amplitudes following Noble Denton
    marine transport guidelines. Produces 8 combinations representing extreme
    roll and pitch conditions with associated heave.

    The combinations consider:
    - Roll motion: Static tilt + dynamic heave acceleration
    - Pitch motion: Static tilt + dynamic heave acceleration
    - Each with positive/negative rotational direction
    - Each with heave in-phase or out-of-phase with tilt

    Note: Gravitational effects are NOT included in these environmental cases.
    Gravity is added separately as a Permanent load case.

    Args:
        name: Descriptive name for this motion set
        center_of_rotation: [x, y, z] reference point for accelerations [m]
        roll_amplitude: Maximum roll angle [degrees], default 20
        pitch_amplitude: Maximum pitch angle [degrees], default 12.5
        heave_amplitude: Maximum heave acceleration [G], default 0.2
        period: Motion period [seconds], default 10
        gravity: Gravitational acceleration [m/s²], default 9.81 (positive)

    Example:
        >>> from grillex.core import VesselMotionsFromNobleDenton
        >>>
        >>> motions = VesselMotionsFromNobleDenton(
        ...     name="Noble Denton Transport",
        ...     center_of_rotation=[50.0, 0.0, 5.0],
        ...     roll_amplitude=20,      # degrees
        ...     pitch_amplitude=12.5,   # degrees
        ...     heave_amplitude=0.2,    # G
        ...     period=10               # seconds
        ... )
        >>>
        >>> print(f"Generated {len(motions.motion_cases)} motion cases")
        Generated 8 motion cases
    """

    def __init__(
        self,
        name: str,
        center_of_rotation: List[float],
        roll_amplitude: float = 20.0,
        pitch_amplitude: float = 12.5,
        heave_amplitude: float = 0.2,
        period: float = 10.0,
        gravity: float = 9.81
    ):
        # Note: gravity is positive here (magnitude for calculations)
        super().__init__(name, center_of_rotation, gravity=-gravity)

        self.roll_amplitude = roll_amplitude
        self.pitch_amplitude = pitch_amplitude
        self.heave_amplitude = heave_amplitude
        self.period = period
        self._g = abs(gravity)  # Use magnitude for calculations

        # Build motion cases
        self._build_motion_cases()

    # === Private calculation properties ===

    @property
    def _ax_static_pitch(self) -> float:
        """Surge due to static pitch tilt."""
        return -self._g * math.sin(math.radians(self.pitch_amplitude))

    @property
    def _ay_static_roll(self) -> float:
        """Sway due to static roll tilt."""
        return -self._g * math.sin(math.radians(self.roll_amplitude))

    @property
    def _az_static_pitch(self) -> float:
        """Heave reduction due to pitch tilt (gravity component)."""
        return -self._g * (1 - math.cos(math.radians(self.pitch_amplitude)))

    @property
    def _az_static_roll(self) -> float:
        """Heave reduction due to roll tilt (gravity component)."""
        return -self._g * (1 - math.cos(math.radians(self.roll_amplitude)))

    @property
    def _ax_heave_pitch(self) -> float:
        """Surge component from heave during pitch."""
        return -self.heave_amplitude * self._g * math.sin(math.radians(self.pitch_amplitude))

    @property
    def _ay_heave_roll(self) -> float:
        """Sway component from heave during roll."""
        return -self.heave_amplitude * self._g * math.sin(math.radians(self.roll_amplitude))

    @property
    def _az_heave_pitch(self) -> float:
        """Heave component during pitch."""
        return -self.heave_amplitude * self._g * math.cos(math.radians(self.pitch_amplitude))

    @property
    def _az_heave_roll(self) -> float:
        """Heave component during roll."""
        return -self.heave_amplitude * self._g * math.cos(math.radians(self.roll_amplitude))

    @property
    def _a_pitch(self) -> float:
        """Angular acceleration from pitch motion."""
        omega = 2 * math.pi / self.period
        return math.radians(self.pitch_amplitude) * omega**2

    @property
    def _a_roll(self) -> float:
        """Angular acceleration from roll motion."""
        omega = 2 * math.pi / self.period
        return math.radians(self.roll_amplitude) * omega**2

    @property
    def acceleration_combinations(self) -> List[List[float]]:
        """
        Generate Noble Denton acceleration combinations.

        Returns 8 combinations:
        - 4 roll cases (positive/negative roll, heave in-phase/out-of-phase)
        - 4 pitch cases (positive/negative pitch, heave in-phase/out-of-phase)

        Each row: [ax, ay, az, rx, ry, rz]
        """
        # Roll cases (surge=0, yaw=0)
        roll_cases = [
            # Negative roll, heave in-phase
            [0,
             self._ay_static_roll - self._ay_heave_roll,
             self._az_static_roll + self._az_heave_roll,
             -self._a_roll, 0, 0],
            # Negative roll, heave out-of-phase
            [0,
             self._ay_static_roll + self._ay_heave_roll,
             self._az_static_roll - self._az_heave_roll,
             -self._a_roll, 0, 0],
            # Positive roll, heave in-phase
            [0,
             -self._ay_static_roll + self._ay_heave_roll,
             self._az_static_roll + self._az_heave_roll,
             self._a_roll, 0, 0],
            # Positive roll, heave out-of-phase
            [0,
             -self._ay_static_roll - self._ay_heave_roll,
             self._az_static_roll - self._az_heave_roll,
             self._a_roll, 0, 0],
        ]

        # Pitch cases (sway=0, yaw=0)
        pitch_cases = [
            # Negative pitch, heave in-phase
            [-self._ax_static_pitch + self._ax_heave_pitch,
             0,
             self._az_static_pitch + self._az_heave_pitch,
             0, -self._a_pitch, 0],
            # Negative pitch, heave out-of-phase
            [-self._ax_static_pitch - self._ax_heave_pitch,
             0,
             self._az_static_pitch - self._az_heave_pitch,
             0, -self._a_pitch, 0],
            # Positive pitch, heave in-phase
            [self._ax_static_pitch - self._ax_heave_pitch,
             0,
             self._az_static_pitch + self._az_heave_pitch,
             0, self._a_pitch, 0],
            # Positive pitch, heave out-of-phase
            [self._ax_static_pitch + self._ax_heave_pitch,
             0,
             self._az_static_pitch - self._az_heave_pitch,
             0, self._a_pitch, 0],
        ]

        return roll_cases + pitch_cases

    def __repr__(self) -> str:
        return (
            f"VesselMotionsFromNobleDenton(name='{self.name}', "
            f"roll={self.roll_amplitude}°, pitch={self.pitch_amplitude}°, "
            f"heave={self.heave_amplitude}G, period={self.period}s)"
        )
```

2. Add tests:
   - Verify 8 combinations are generated
   - Verify roll cases have zero surge and yaw
   - Verify pitch cases have zero sway and yaw
   - Verify computed accelerations match hand calculations

**Acceptance Criteria:**
- [ ] VesselMotionsFromNobleDenton generates 8 motion combinations
- [ ] Roll and pitch cases are correctly separated (orthogonal)
- [ ] Computed accelerations match Noble Denton guideline formulas
- [ ] Default values match typical marine transport assumptions

---

## Task 20.4: Implement Load Case Generation

**Requirements:** R-VESSEL-001
**Dependencies:** Task 20.2, Task 20.3, Phase 5 (Load Cases)
**Difficulty:** Medium

**Description:**
Add methods to VesselMotions to generate load cases from motion combinations.

**Steps:**

1. Add `generate_load_cases` method to `VesselMotions` base class:

```python
def generate_load_cases(self, model: "StructuralModel") -> List:
    """
    Generate load cases for all motion combinations.

    Creates:
    1. One gravity load case (Permanent type) with acceleration [0, 0, gravity]
    2. One Environmental load case per motion combination

    Each environmental load case sets the acceleration field with the
    motion's translational and rotational accelerations about the
    center of rotation.

    Args:
        model: The StructuralModel to add load cases to

    Returns:
        List of created LoadCase objects (gravity + environmental cases)

    Raises:
        RuntimeError: If load cases have already been generated

    Example:
        >>> model = StructuralModel(name="Barge")
        >>> motions = VesselMotionsFromNobleDenton(...)
        >>> load_cases = motions.generate_load_cases(model)
        >>> print(f"Created {len(load_cases)} load cases")
        Created 9 load cases  # 1 gravity + 8 environmental
    """
    if self._generated:
        raise RuntimeError(
            f"Load cases for '{self.name}' have already been generated. "
            "Create a new VesselMotions instance for different load cases."
        )

    from grillex.core import LoadCaseType

    # Get the C++ model
    cpp_model = model._model

    self._load_cases = []

    # 1. Create gravity load case (Permanent)
    gravity_lc = cpp_model.create_load_case(
        f"{self.name}_Gravity",
        LoadCaseType.Permanent
    )

    # Set gravity acceleration field (no rotation)
    import numpy as np
    gravity_accel = np.array([0.0, 0.0, self.gravity, 0.0, 0.0, 0.0])
    ref_point = np.array(self.center_of_rotation)
    gravity_lc.set_acceleration_field(gravity_accel, ref_point)

    self._gravity_load_case = gravity_lc
    self._load_cases.append(gravity_lc)

    # 2. Create environmental load cases for each motion combination
    for motion_case in self.motion_cases:
        env_lc = cpp_model.create_load_case(
            motion_case.name,
            LoadCaseType.Environmental
        )

        # Build 6-component acceleration vector [ax, ay, az, rx, ry, rz]
        accel = np.concatenate([
            motion_case.acceleration.as_array(),
            motion_case.rotary_acceleration.as_array()
        ])

        env_lc.set_acceleration_field(accel, ref_point)
        self._load_cases.append(env_lc)

    self._generated = True
    return self._load_cases

@property
def load_cases(self) -> List:
    """Get the generated load cases (empty if not yet generated)."""
    return self._load_cases

@property
def gravity_load_case(self):
    """Get the gravity load case (None if not yet generated)."""
    return self._gravity_load_case

@property
def environmental_load_cases(self) -> List:
    """Get only the environmental load cases (excluding gravity)."""
    return self._load_cases[1:] if self._load_cases else []
```

2. Add tests:
   - Verify gravity load case has Permanent type
   - Verify environmental load cases have Environmental type
   - Verify acceleration fields are set correctly
   - Verify RuntimeError if called twice

**Acceptance Criteria:**
- [ ] generate_load_cases creates gravity + environmental load cases
- [ ] Gravity load case has LoadCaseType.Permanent
- [ ] Environmental load cases have LoadCaseType.Environmental
- [ ] Acceleration fields include both translation and rotation
- [ ] Center of rotation is used as reference point

---

## Task 20.5: Define Load Combination Factors

**Requirements:** R-VESSEL-003, R-VESSEL-004
**Dependencies:** Task 20.1
**Difficulty:** Low

**Description:**
Define the load combination factors for LRFD (ULS-a, ULS-b) and ASD methods.

**Steps:**

1. Add load factor definitions to `vessel_motions.py`:

```python
# Load combination factors per design code
# Reference: DNV-GL, API RP 2A, etc.

@dataclass
class LoadFactors:
    """Load factors for a specific design situation.

    Attributes:
        permanent: Factor for permanent (gravity) loads
        environmental: Factor for environmental (motion) loads
        name: Descriptive name for this factor set
    """
    permanent: float
    environmental: float
    name: str


# LRFD Load Factors (based on DNV-GL / offshore standards)
LRFD_FACTORS = {
    # ULS-a: Maximum environmental, reduced permanent
    # Used when environmental load dominates
    ("regular", "ULS-a"): LoadFactors(
        permanent=1.0,
        environmental=1.3,
        name="ULS-a Regular"
    ),
    ("removal", "ULS-a"): LoadFactors(
        permanent=1.0,
        environmental=1.15,  # Reduced for removal operations
        name="ULS-a Removal"
    ),

    # ULS-b: Maximum permanent, reduced environmental
    # Used when permanent load dominates
    ("regular", "ULS-b"): LoadFactors(
        permanent=1.2,
        environmental=0.7,
        name="ULS-b Regular"
    ),
    ("removal", "ULS-b"): LoadFactors(
        permanent=1.2,
        environmental=0.7,
        name="ULS-b Removal"
    ),
}

# ASD Load Factors (unity factors, safety in allowable stress)
ASD_FACTORS = {
    ("regular", "Operating"): LoadFactors(
        permanent=1.0,
        environmental=1.0,
        name="ASD Operating"
    ),
    ("removal", "Operating"): LoadFactors(
        permanent=1.0,
        environmental=1.0,
        name="ASD Removal"
    ),
}


def get_load_factors(
    design_method: DesignMethod,
    operation_type: OperationType
) -> List[LoadFactors]:
    """
    Get applicable load factors for a design method and operation type.

    Args:
        design_method: LRFD or ASD
        operation_type: Regular or Removal

    Returns:
        List of LoadFactors for applicable limit states
    """
    op_key = operation_type.value

    if design_method == DesignMethod.LRFD:
        return [
            LRFD_FACTORS[(op_key, "ULS-a")],
            LRFD_FACTORS[(op_key, "ULS-b")],
        ]
    else:  # ASD
        return [
            ASD_FACTORS[(op_key, "Operating")],
        ]
```

2. Add tests:
   - Verify LRFD returns ULS-a and ULS-b factors
   - Verify ASD returns Operating factors
   - Verify removal factors differ from regular (for ULS-a)

**Acceptance Criteria:**
- [ ] LRFD factors defined for ULS-a and ULS-b
- [ ] ASD factors defined for Operating condition
- [ ] Regular and Removal operation types have appropriate factors
- [ ] get_load_factors returns correct factors based on inputs

---

## Task 20.6: Implement Load Combination Generation

**Requirements:** R-VESSEL-003, R-VESSEL-004
**Dependencies:** Task 20.4, Task 20.5
**Difficulty:** High

**Description:**
Add methods to generate load combinations based on design method and operation type.

**Steps:**

1. Add `generate_load_combinations` method to `VesselMotions`:

```python
def generate_load_combinations(
    self,
    model: "StructuralModel",
    design_method: DesignMethod = DesignMethod.LRFD,
    operation_type: OperationType = OperationType.REGULAR
) -> List:
    """
    Generate load combinations for vessel motion analysis.

    Creates load combinations based on the design method:

    **LRFD (Load and Resistance Factor Design)**:
    - ULS-a: γ_G * Gravity + γ_E * Environmental (for each motion case)
    - ULS-b: γ_G * Gravity + γ_E * Environmental (for each motion case)

    **ASD (Allowable Stress Design)**:
    - Operating: 1.0 * Gravity + 1.0 * Environmental (for each motion case)

    Each environmental motion case produces separate combinations for each
    limit state. For 8 motion cases with LRFD:
    - 8 ULS-a combinations (one per motion case)
    - 8 ULS-b combinations (one per motion case)
    - Total: 16 combinations

    Args:
        model: The StructuralModel to add combinations to
        design_method: LRFD or ASD, default LRFD
        operation_type: Regular or Removal, default Regular

    Returns:
        List of created LoadCombination objects

    Raises:
        RuntimeError: If load cases have not been generated yet

    Example:
        >>> model = StructuralModel(name="Barge")
        >>> motions = VesselMotionsFromNobleDenton(...)
        >>> motions.generate_load_cases(model)
        >>> combinations = motions.generate_load_combinations(
        ...     model,
        ...     design_method=DesignMethod.LRFD,
        ...     operation_type=OperationType.REGULAR
        ... )
        >>> print(f"Created {len(combinations)} load combinations")
        Created 16 load combinations  # 8 ULS-a + 8 ULS-b
    """
    if not self._generated:
        raise RuntimeError(
            "Load cases must be generated before combinations. "
            "Call generate_load_cases() first."
        )

    # Get applicable load factors
    factor_sets = get_load_factors(design_method, operation_type)

    cpp_model = model._model
    combinations = []
    combo_id = 1

    # Get the gravity load case
    gravity_lc = self._gravity_load_case

    # Create combinations for each factor set and each environmental case
    for factors in factor_sets:
        for env_lc in self.environmental_load_cases:
            # Create combination name
            combo_name = f"{self.name}_{factors.name}_{env_lc.name()}"

            # Create the load combination
            combo = cpp_model.create_load_combination(combo_id, combo_name)
            combo_id += 1

            # Add gravity with permanent factor
            combo.add_load_case(gravity_lc, factors.permanent)

            # Add environmental with environmental factor
            combo.add_load_case(env_lc, factors.environmental)

            combinations.append(combo)

    return combinations


def generate_all(
    self,
    model: "StructuralModel",
    design_method: DesignMethod = DesignMethod.LRFD,
    operation_type: OperationType = OperationType.REGULAR
) -> Tuple[List, List]:
    """
    Convenience method to generate both load cases and combinations.

    Args:
        model: The StructuralModel to add load cases and combinations to
        design_method: LRFD or ASD, default LRFD
        operation_type: Regular or Removal, default Regular

    Returns:
        Tuple of (load_cases, load_combinations)

    Example:
        >>> model = StructuralModel(name="Barge")
        >>> motions = VesselMotionsFromNobleDenton(...)
        >>> load_cases, combinations = motions.generate_all(model)
    """
    load_cases = self.generate_load_cases(model)
    combinations = self.generate_load_combinations(
        model, design_method, operation_type
    )
    return load_cases, combinations
```

2. Add comprehensive tests:
   - Test LRFD generates 2 * N combinations (N = number of motion cases)
   - Test ASD generates N combinations
   - Test combination names include limit state and motion case
   - Test factors are correctly applied
   - Test RuntimeError if load cases not generated first

**Acceptance Criteria:**
- [ ] generate_load_combinations creates correct number of combinations
- [ ] Each combination includes gravity and one environmental case
- [ ] Load factors are correctly applied based on design method
- [ ] Operation type affects factors appropriately
- [ ] Combination names are descriptive and unique
- [ ] generate_all convenience method works correctly

---

## Task 20.7: Add DataFrame/Summary Methods

**Requirements:** R-VESSEL-001
**Dependencies:** Task 20.2, Task 20.3
**Difficulty:** Low

**Description:**
Add methods to display acceleration combinations in a readable format.

**Steps:**

1. Add summary methods to `VesselMotions`:

```python
def get_accelerations_dataframe(self):
    """
    Return acceleration combinations as a pandas DataFrame.

    Returns:
        DataFrame with columns: Surge, Sway, Heave, Roll, Pitch, Yaw
        and one row per motion case.

    Example:
        >>> motions = VesselMotionsFromNobleDenton(...)
        >>> df = motions.get_accelerations_dataframe()
        >>> print(df)
              Surge      Sway     Heave      Roll     Pitch  Yaw
        0  0.000000 -2.680097  1.263263 -0.137806  0.000000  0.0
        1  0.000000 -4.030338 -2.446494 -0.137806  0.000000  0.0
        ...
    """
    import pandas as pd

    data = self.acceleration_combinations
    columns = ['Surge', 'Sway', 'Heave', 'Roll', 'Pitch', 'Yaw']
    return pd.DataFrame(data, columns=columns)

def get_summary(self) -> str:
    """
    Return a text summary of the vessel motions configuration.

    Returns:
        Multi-line string describing the motion configuration.
    """
    lines = [
        f"Vessel Motions: {self.name}",
        f"  Center of rotation: {self.center_of_rotation}",
        f"  Gravity: {self.gravity} m/s²",
        f"  Motion cases: {len(self.motion_cases)}",
        "",
        "Motion case summary:",
    ]

    for i, case in enumerate(self.motion_cases):
        a = case.acceleration
        r = case.rotary_acceleration
        lines.append(
            f"  {i+1}. {case.name}: "
            f"a=[{a.ax:.2f}, {a.ay:.2f}, {a.az:.2f}] m/s², "
            f"r=[{r.rx:.3f}, {r.ry:.3f}, {r.rz:.3f}] rad/s²"
        )

    return "\n".join(lines)

def __str__(self) -> str:
    return self.get_summary()
```

2. Add tests for summary methods

**Acceptance Criteria:**
- [ ] get_accelerations_dataframe returns properly formatted DataFrame
- [ ] get_summary returns readable text output
- [ ] __str__ returns summary for print()

---

## Task 20.8: Integration with StructuralModel

**Requirements:** R-VESSEL-001
**Dependencies:** Task 20.6
**Difficulty:** Low

**Description:**
Add convenience methods to StructuralModel for vessel motion workflows.

**Steps:**

1. Add methods to `model_wrapper.py`:

```python
def add_vessel_motions(
    self,
    vessel_motions: "VesselMotions",
    design_method: "DesignMethod" = None,
    operation_type: "OperationType" = None
) -> Tuple[List, List]:
    """
    Add vessel motions and generate load cases/combinations.

    This is a convenience method that:
    1. Generates load cases from the vessel motions
    2. Generates load combinations based on design method

    Args:
        vessel_motions: VesselMotions (or subclass) instance
        design_method: LRFD or ASD (default: LRFD)
        operation_type: Regular or Removal (default: Regular)

    Returns:
        Tuple of (load_cases, load_combinations)

    Example:
        >>> from grillex.core import (
        ...     StructuralModel, VesselMotionsFromNobleDenton,
        ...     DesignMethod, OperationType
        ... )
        >>>
        >>> model = StructuralModel(name="Cargo Barge")
        >>> # ... add structure, cargo, etc. ...
        >>>
        >>> motions = VesselMotionsFromNobleDenton(
        ...     name="Transport",
        ...     center_of_rotation=[50.0, 0.0, 5.0]
        ... )
        >>>
        >>> load_cases, combinations = model.add_vessel_motions(
        ...     motions,
        ...     design_method=DesignMethod.LRFD,
        ...     operation_type=OperationType.REGULAR
        ... )
    """
    from grillex.core.vessel_motions import (
        DesignMethod as DM,
        OperationType as OT
    )

    # Apply defaults
    if design_method is None:
        design_method = DM.LRFD
    if operation_type is None:
        operation_type = OT.REGULAR

    return vessel_motions.generate_all(
        self,
        design_method=design_method,
        operation_type=operation_type
    )
```

2. Export VesselMotions classes from core module

3. Add integration tests

**Acceptance Criteria:**
- [ ] add_vessel_motions is accessible from StructuralModel
- [ ] Default design method is LRFD, default operation is Regular
- [ ] Returns tuple of (load_cases, combinations)
- [ ] Integration with cargo and other model features works

---

## Task 20.9: Comprehensive Tests

**Requirements:** All R-VESSEL requirements
**Dependencies:** Tasks 20.1-20.8
**Difficulty:** Medium

**Description:**
Create comprehensive test suite for vessel motions functionality.

**Test Cases:**

1. **test_vessel_motions_from_amplitudes_basic**:
   - Create VesselMotionsFromAmplitudes with known amplitudes
   - Verify 16 combinations are generated
   - Verify coupling rules are applied

2. **test_vessel_motions_from_noble_denton_basic**:
   - Create VesselMotionsFromNobleDenton with defaults
   - Verify 8 combinations are generated
   - Verify roll/pitch separation

3. **test_noble_denton_accelerations**:
   - Calculate expected accelerations by hand
   - Verify computed accelerations match within tolerance

4. **test_load_case_generation**:
   - Generate load cases for a model
   - Verify gravity is Permanent, environmental is Environmental
   - Verify acceleration fields are set

5. **test_load_combination_lrfd_regular**:
   - Generate LRFD combinations for regular operation
   - Verify 16 combinations (8 ULS-a + 8 ULS-b)
   - Verify factors match LRFD_FACTORS

6. **test_load_combination_lrfd_removal**:
   - Generate LRFD combinations for removal operation
   - Verify ULS-a factors differ from regular

7. **test_load_combination_asd**:
   - Generate ASD combinations
   - Verify 8 combinations (one per motion case)
   - Verify factors are 1.0

8. **test_integration_with_cargo**:
   - Create model with cargo and vessel motions
   - Analyze and verify results make physical sense

9. **test_acceleration_at_point**:
   - Calculate acceleration at various points
   - Verify rotary contributions are correct

10. **test_dataframe_output**:
    - Verify get_accelerations_dataframe format
    - Verify column names and data types

**Acceptance Criteria:**
- [ ] All test cases pass
- [ ] Edge cases are covered (zero amplitudes, extreme values)
- [ ] Integration with other model features verified
- [ ] Coverage for all public methods

---

## Task 20.10: Documentation and Examples

**Requirements:** All R-VESSEL requirements
**Dependencies:** Tasks 20.1-20.9
**Difficulty:** Low

**Description:**
Add documentation and example usage to the module.

**Steps:**

1. Add module-level docstring with complete usage example

2. Update user documentation in `docs/user/`:
   - Add `vessel_motions.rst` with:
     - Coordinate system conventions
     - Motion coupling explanation
     - Example workflows for amplitude and Noble Denton inputs
     - LRFD vs ASD comparison
     - Regular vs Removal operations

3. Add doctest examples to key methods

4. Update `docs/user/analysis_workflow.rst` to include vessel motions step

**Acceptance Criteria:**
- [ ] Module docstring includes complete usage example
- [ ] User documentation explains all concepts
- [ ] Doctest examples pass
- [ ] Analysis workflow documentation updated

---

## Summary

| Task | Description | Difficulty | Dependencies |
|------|-------------|------------|--------------|
| 20.1 | Base class and data structures | Medium | Phase 5, 9 |
| 20.2 | VesselMotionsFromAmplitudes | Medium | 20.1 |
| 20.3 | VesselMotionsFromNobleDenton | Medium | 20.1 |
| 20.4 | Load case generation | Medium | 20.2, 20.3 |
| 20.5 | Load factor definitions | Low | 20.1 |
| 20.6 | Load combination generation | High | 20.4, 20.5 |
| 20.7 | DataFrame/summary methods | Low | 20.2, 20.3 |
| 20.8 | StructuralModel integration | Low | 20.6 |
| 20.9 | Comprehensive tests | Medium | 20.1-20.8 |
| 20.10 | Documentation | Low | 20.1-20.9 |

**Total Acceptance Criteria:** 35
