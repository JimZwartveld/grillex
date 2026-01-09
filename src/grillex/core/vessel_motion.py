"""
Vessel motion support for offshore structural analysis.

This module provides the VesselMotion class for defining vessel motion
accelerations and applying them to load cases. Vessel motions are the 6-DOF
motions of a floating vessel (barge, ship, platform) that induce inertial
loads on the structure and cargo.

The six vessel motion degrees of freedom are:
- Surge (X): Longitudinal translation, positive forward
- Sway (Y): Transverse translation, positive to port
- Heave (Z): Vertical translation, positive upward
- Roll (RX): Rotation about X-axis, positive is starboard down
- Pitch (RY): Rotation about Y-axis, positive is bow down
- Yaw (RZ): Rotation about Z-axis, positive is bow to port

For multi-motion generation, use:
- VesselMotionsFromAmplitudes: Generates +/- motion cases from amplitudes with coupling
- VesselMotionsFromNobleDenton: Generates 6 standard Noble Denton load cases

Example:
    >>> from grillex.core import VesselMotion
    >>> motion = VesselMotion("Design Heave + Roll")
    >>> _ = motion.set_motion_center([50.0, 0.0, 5.0])
    >>> _ = motion.add_heave(2.5)
    >>> _ = motion.add_roll(0.12)
    >>> accel, ref_pt = motion.get_acceleration_field()
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, TYPE_CHECKING
from enum import Enum
from abc import ABC, abstractmethod
import math

import numpy as np

if TYPE_CHECKING:
    from grillex._grillex_cpp import LoadCase


class MotionType(Enum):
    """Vessel motion types in ship coordinate system.

    Attributes:
        SURGE: X translation (fore-aft), positive forward
        SWAY: Y translation (port-starboard), positive to port
        HEAVE: Z translation (vertical), positive upward
        ROLL: Rotation about X-axis (heel), positive starboard down
        PITCH: Rotation about Y-axis (trim), positive bow down
        YAW: Rotation about Z-axis (heading), positive bow to port
    """
    SURGE = "surge"
    SWAY = "sway"
    HEAVE = "heave"
    ROLL = "roll"
    PITCH = "pitch"
    YAW = "yaw"


@dataclass
class MotionComponent:
    """Single motion component with amplitude and phase.

    Attributes:
        motion_type: Type of motion (surge, sway, heave, roll, pitch, yaw)
        amplitude: Acceleration amplitude (m/s² for linear, rad/s² for angular)
        phase: Phase angle in radians for combining motions (default: 0)
    """
    motion_type: MotionType
    amplitude: float
    phase: float = 0.0


@dataclass
class LinkedLoadCase:
    """Reference to a load case linked to a VesselMotion.

    Attributes:
        load_case: The C++ LoadCase object
        sign_multiplier: Sign multiplier for this load case (+1 or -1)
        suffix: Suffix added to name (e.g., " +" or " -")
    """
    load_case: Any  # LoadCase from C++
    sign_multiplier: float = 1.0
    suffix: str = ""


@dataclass
class VesselMotion:
    """
    Vessel motion definition for offshore structural analysis.

    Encapsulates the motion parameters for a floating vessel and provides
    methods to generate acceleration fields for load cases. The motion center
    is the reference point for rotational motions (typically vessel CoG or
    waterline amidships).

    Linked load cases are automatically updated when the vessel motion is
    modified. Load cases linked to a VesselMotion are immutable - their
    acceleration field cannot be modified directly.

    Attributes:
        name: Descriptive name for this motion condition
        motion_center: Reference point for rotational motions [x, y, z] in meters
        components: List of motion components (heave, pitch, roll, etc.)
        description: Optional description of the motion condition

    Example:
        >>> motion = VesselMotion("Design Heave + Pitch")
        >>> _ = motion.set_motion_center([50.0, 0.0, 5.0])  # Midship, waterline
        >>> _ = motion.add_heave(2.5)  # 2.5 m/s² vertical acceleration
        >>> _ = motion.add_pitch(0.08)  # 0.08 rad/s² pitch acceleration
        >>> _ = motion.add_roll(0.12)  # 0.12 rad/s² roll acceleration
    """
    name: str
    motion_center: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    components: List[MotionComponent] = field(default_factory=list)
    description: str = ""

    # Linked load cases (not serialized, managed internally)
    _linked_load_cases: List[LinkedLoadCase] = field(
        default_factory=list, repr=False, compare=False
    )

    def __post_init__(self):
        """Initialize internal state after dataclass creation."""
        # Ensure _linked_load_cases is a list (in case it wasn't set)
        if not hasattr(self, '_linked_load_cases') or self._linked_load_cases is None:
            object.__setattr__(self, '_linked_load_cases', [])

    def _update_linked_load_cases(self) -> None:
        """Update all linked load cases with current acceleration field.

        This is called automatically when the vessel motion is modified.
        """
        for linked in self._linked_load_cases:
            accel_6d, motion_center = self.get_acceleration_field(
                sign_multiplier=linked.sign_multiplier
            )
            linked.load_case.set_acceleration_field(
                np.array(accel_6d),
                np.array(motion_center)
            )

    def link_load_case(
        self,
        load_case: Any,
        sign_multiplier: float = 1.0,
        suffix: str = ""
    ) -> None:
        """Link a load case to this vessel motion.

        The load case's acceleration field will be updated whenever this
        vessel motion is modified. Linked load cases are marked as immutable.

        Args:
            load_case: LoadCase object to link
            sign_multiplier: Sign multiplier (+1 or -1) for this load case
            suffix: Suffix added to distinguish this variant (e.g., " +")
        """
        linked = LinkedLoadCase(load_case, sign_multiplier, suffix)
        self._linked_load_cases.append(linked)

        # Mark the load case as linked to a vessel motion (immutable)
        if hasattr(load_case, '_vessel_motion_linked'):
            load_case._vessel_motion_linked = True

        # Apply current acceleration
        accel_6d, motion_center = self.get_acceleration_field(
            sign_multiplier=sign_multiplier
        )
        load_case.set_acceleration_field(
            np.array(accel_6d),
            np.array(motion_center)
        )

    def unlink_load_case(self, load_case: Any) -> bool:
        """Unlink a load case from this vessel motion.

        Args:
            load_case: LoadCase object to unlink

        Returns:
            True if load case was found and unlinked, False otherwise
        """
        for i, linked in enumerate(self._linked_load_cases):
            if linked.load_case is load_case:
                self._linked_load_cases.pop(i)
                if hasattr(load_case, '_vessel_motion_linked'):
                    load_case._vessel_motion_linked = False
                return True
        return False

    def get_linked_load_cases(self) -> List[Any]:
        """Get all load cases linked to this vessel motion.

        Returns:
            List of LoadCase objects
        """
        return [linked.load_case for linked in self._linked_load_cases]

    def set_motion_center(self, position: List[float]) -> "VesselMotion":
        """Set the motion center (pivot point for rotational motions).

        Args:
            position: [x, y, z] coordinates in meters

        Returns:
            Self for method chaining
        """
        if len(position) != 3:
            raise ValueError("Motion center must be a 3-element list [x, y, z]")
        self.motion_center = list(position)
        self._update_linked_load_cases()
        return self

    def add_component(
        self,
        motion_type: MotionType,
        amplitude: float,
        phase: float = 0.0
    ) -> "VesselMotion":
        """Add a motion component.

        Args:
            motion_type: Type of motion (from MotionType enum)
            amplitude: Acceleration amplitude (m/s² for linear, rad/s² for angular)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        self.components.append(MotionComponent(motion_type, amplitude, phase))
        self._update_linked_load_cases()
        return self

    def add_surge(self, amplitude: float, phase: float = 0.0) -> "VesselMotion":
        """Add surge acceleration (longitudinal).

        Args:
            amplitude: Acceleration in m/s² (positive = forward)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        return self.add_component(MotionType.SURGE, amplitude, phase)

    def add_sway(self, amplitude: float, phase: float = 0.0) -> "VesselMotion":
        """Add sway acceleration (transverse).

        Args:
            amplitude: Acceleration in m/s² (positive = port)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        return self.add_component(MotionType.SWAY, amplitude, phase)

    def add_heave(self, amplitude: float, phase: float = 0.0) -> "VesselMotion":
        """Add heave acceleration (vertical).

        Args:
            amplitude: Acceleration in m/s² (positive = up)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        return self.add_component(MotionType.HEAVE, amplitude, phase)

    def add_roll(self, amplitude: float, phase: float = 0.0) -> "VesselMotion":
        """Add roll angular acceleration (rotation about X-axis).

        Args:
            amplitude: Angular acceleration in rad/s² (positive = starboard down)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        return self.add_component(MotionType.ROLL, amplitude, phase)

    def add_pitch(self, amplitude: float, phase: float = 0.0) -> "VesselMotion":
        """Add pitch angular acceleration (rotation about Y-axis).

        Args:
            amplitude: Angular acceleration in rad/s² (positive = bow down)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        return self.add_component(MotionType.PITCH, amplitude, phase)

    def add_yaw(self, amplitude: float, phase: float = 0.0) -> "VesselMotion":
        """Add yaw angular acceleration (rotation about Z-axis).

        Args:
            amplitude: Angular acceleration in rad/s² (positive = bow to port)
            phase: Phase angle in radians (default: 0)

        Returns:
            Self for method chaining
        """
        return self.add_component(MotionType.YAW, amplitude, phase)

    def clear_components(self) -> "VesselMotion":
        """Remove all motion components.

        Returns:
            Self for method chaining
        """
        self.components.clear()
        self._update_linked_load_cases()
        return self

    def get_acceleration_field(self, sign_multiplier: float = 1.0) -> tuple:
        """
        Compute the 6-component acceleration field from motion components.

        The acceleration field is returned as [ax, ay, az, αx, αy, αz] where:
        - ax, ay, az: Linear accelerations in m/s²
        - αx, αy, αz: Angular accelerations in rad/s²

        Args:
            sign_multiplier: Multiplier applied to all accelerations (default: 1.0).
                Use -1.0 to get the negative (opposite direction) motion.

        Returns:
            Tuple of (acceleration_6d, motion_center) where:
            - acceleration_6d: List of [ax, ay, az, αx, αy, αz]
            - motion_center: List of [x, y, z] reference point in meters

        Note:
            For quasi-static analysis, only the instantaneous acceleration
            magnitude matters. Phase information is used when combining
            results from different motion conditions, not here.
        """
        accel = np.zeros(6)

        for comp in self.components:
            if comp.motion_type == MotionType.SURGE:
                accel[0] += comp.amplitude
            elif comp.motion_type == MotionType.SWAY:
                accel[1] += comp.amplitude
            elif comp.motion_type == MotionType.HEAVE:
                accel[2] += comp.amplitude
            elif comp.motion_type == MotionType.ROLL:
                accel[3] += comp.amplitude
            elif comp.motion_type == MotionType.PITCH:
                accel[4] += comp.amplitude
            elif comp.motion_type == MotionType.YAW:
                accel[5] += comp.amplitude

        # Apply sign multiplier
        accel *= sign_multiplier

        return (accel.tolist(), list(self.motion_center))

    def apply_to_load_case(self, load_case: "LoadCase") -> None:
        """
        Apply this vessel motion to a LoadCase.

        This sets the acceleration field on the load case, which will cause
        inertial loads to be computed on all mass-carrying elements.

        Args:
            load_case: LoadCase object to apply the acceleration field to
        """
        accel, ref_point = self.get_acceleration_field()
        load_case.set_acceleration_field(accel, ref_point)

    def get_component_by_type(self, motion_type: MotionType) -> Optional[MotionComponent]:
        """Get the first component of a specific type.

        Args:
            motion_type: Type of motion to find

        Returns:
            MotionComponent if found, None otherwise
        """
        for comp in self.components:
            if comp.motion_type == motion_type:
                return comp
        return None

    def has_rotational_motion(self) -> bool:
        """Check if this motion includes any rotational components.

        Returns:
            True if roll, pitch, or yaw are defined
        """
        rotational = {MotionType.ROLL, MotionType.PITCH, MotionType.YAW}
        return any(comp.motion_type in rotational for comp in self.components)

    def has_linear_motion(self) -> bool:
        """Check if this motion includes any linear components.

        Returns:
            True if surge, sway, or heave are defined
        """
        linear = {MotionType.SURGE, MotionType.SWAY, MotionType.HEAVE}
        return any(comp.motion_type in linear for comp in self.components)

    def to_dict(self) -> dict:
        """Convert vessel motion to YAML-serializable dictionary.

        The output format matches the YAML input format, allowing round-trip
        serialization.

        Returns:
            Dictionary suitable for YAML serialization with fields:
            - name: Motion name
            - motion_center: [x, y, z] reference point
            - surge, sway, heave, roll, pitch, yaw: acceleration values

        Example:
            >>> motion = VesselMotion("Test").add_heave(2.5).add_roll(0.12)
            >>> d = motion.to_dict()
            >>> d['heave']
            2.5
            >>> d['roll']
            0.12
        """
        result = {
            'name': self.name,
        }

        if self.motion_center != [0.0, 0.0, 0.0]:
            result['motion_center'] = list(self.motion_center)

        # Add each component by type
        for comp in self.components:
            key = comp.motion_type.value  # e.g., "heave", "roll"
            result[key] = comp.amplitude

        return result

    @classmethod
    def from_dict(cls, data: dict) -> "VesselMotion":
        """Create VesselMotion from dictionary.

        Args:
            data: Dictionary with motion data (as from to_dict())

        Returns:
            VesselMotion instance
        """
        motion = cls(name=data.get('name', 'Unnamed'))

        if 'motion_center' in data:
            motion.set_motion_center(data['motion_center'])

        # Add components by type
        component_map = {
            'surge': MotionType.SURGE,
            'sway': MotionType.SWAY,
            'heave': MotionType.HEAVE,
            'roll': MotionType.ROLL,
            'pitch': MotionType.PITCH,
            'yaw': MotionType.YAW,
        }

        for key, motion_type in component_map.items():
            if key in data:
                motion.add_component(motion_type, float(data[key]))

        return motion

    # ===== Factory Methods =====

    @classmethod
    def create_still_water(cls, name: str = "Still Water") -> "VesselMotion":
        """Create still water condition (no vessel motions).

        This represents calm conditions where only gravity acts on the structure.

        Args:
            name: Name for this motion condition (default: "Still Water")

        Returns:
            VesselMotion with no motion components
        """
        return cls(name=name, description="Still water condition - no vessel motions")

    @classmethod
    def create_heave_only(
        cls,
        heave_accel: float,
        motion_center: Optional[List[float]] = None,
        name: Optional[str] = None
    ) -> "VesselMotion":
        """Create a heave-only motion condition.

        Args:
            heave_accel: Heave acceleration in m/s² (positive = up)
            motion_center: Reference point [x, y, z] in meters (default: origin)
            name: Optional name (default: "Heave {value} m/s²")

        Returns:
            VesselMotion with heave component only
        """
        if name is None:
            name = f"Heave {heave_accel:.2f} m/s²"

        motion = cls(name=name)
        if motion_center is not None:
            motion.set_motion_center(motion_center)
        motion.add_heave(heave_accel)
        return motion

    @classmethod
    def create_roll_condition(
        cls,
        roll_angle: float,
        roll_period: float,
        motion_center: Optional[List[float]] = None,
        name: Optional[str] = None
    ) -> "VesselMotion":
        """Create roll motion from angle and period.

        Computes angular acceleration using simple harmonic motion:
        α = (2π/T)² × θ

        where T is the period and θ is the amplitude in radians.

        Args:
            roll_angle: Maximum roll angle in degrees
            roll_period: Roll period in seconds
            motion_center: Reference point [x, y, z] in meters (default: origin)
            name: Optional name (default: "Roll {angle}° @ {period}s")

        Returns:
            VesselMotion with computed roll acceleration
        """
        if name is None:
            name = f"Roll {roll_angle:.1f}° @ {roll_period:.1f}s"

        # Convert angle to radians
        theta_rad = math.radians(roll_angle)

        # Angular acceleration for simple harmonic motion: α = ω² × θ = (2π/T)² × θ
        omega = 2 * math.pi / roll_period
        roll_accel = omega * omega * theta_rad

        motion = cls(name=name, description=f"Roll {roll_angle}° with {roll_period}s period")
        if motion_center is not None:
            motion.set_motion_center(motion_center)
        motion.add_roll(roll_accel)
        return motion

    @classmethod
    def create_pitch_condition(
        cls,
        pitch_angle: float,
        pitch_period: float,
        motion_center: Optional[List[float]] = None,
        name: Optional[str] = None
    ) -> "VesselMotion":
        """Create pitch motion from angle and period.

        Computes angular acceleration using simple harmonic motion:
        α = (2π/T)² × θ

        where T is the period and θ is the amplitude in radians.

        Args:
            pitch_angle: Maximum pitch angle in degrees
            pitch_period: Pitch period in seconds
            motion_center: Reference point [x, y, z] in meters (default: origin)
            name: Optional name (default: "Pitch {angle}° @ {period}s")

        Returns:
            VesselMotion with computed pitch acceleration
        """
        if name is None:
            name = f"Pitch {pitch_angle:.1f}° @ {pitch_period:.1f}s"

        # Convert angle to radians
        theta_rad = math.radians(pitch_angle)

        # Angular acceleration for simple harmonic motion
        omega = 2 * math.pi / pitch_period
        pitch_accel = omega * omega * theta_rad

        motion = cls(name=name, description=f"Pitch {pitch_angle}° with {pitch_period}s period")
        if motion_center is not None:
            motion.set_motion_center(motion_center)
        motion.add_pitch(pitch_accel)
        return motion

    @classmethod
    def create_combined_design_motion(
        cls,
        heave: float = 0.0,
        roll_angle: float = 0.0,
        roll_period: float = 10.0,
        pitch_angle: float = 0.0,
        pitch_period: float = 8.0,
        surge: float = 0.0,
        sway: float = 0.0,
        yaw_angle: float = 0.0,
        yaw_period: float = 10.0,
        motion_center: Optional[List[float]] = None,
        name: str = "Design Motion"
    ) -> "VesselMotion":
        """Create a combined design motion condition.

        This is the most common scenario for offshore design where multiple
        motion components are combined for design verification.

        Linear accelerations (heave, surge, sway) are specified directly.
        Angular accelerations (roll, pitch, yaw) are computed from angle/period
        using simple harmonic motion: α = (2π/T)² × θ

        Args:
            heave: Heave acceleration in m/s² (direct value)
            roll_angle: Roll amplitude in degrees
            roll_period: Roll period in seconds (default: 10s)
            pitch_angle: Pitch amplitude in degrees
            pitch_period: Pitch period in seconds (default: 8s)
            surge: Surge acceleration in m/s² (direct value)
            sway: Sway acceleration in m/s² (direct value)
            yaw_angle: Yaw amplitude in degrees
            yaw_period: Yaw period in seconds (default: 10s)
            motion_center: Reference point [x, y, z] in meters (default: origin)
            name: Condition name (default: "Design Motion")

        Returns:
            VesselMotion with all specified components
        """
        motion = cls(name=name, description="Combined design motion condition")

        if motion_center is not None:
            motion.set_motion_center(motion_center)

        # Add linear accelerations directly
        if surge != 0.0:
            motion.add_surge(surge)
        if sway != 0.0:
            motion.add_sway(sway)
        if heave != 0.0:
            motion.add_heave(heave)

        # Convert angles to angular accelerations
        if roll_angle != 0.0:
            theta_rad = math.radians(roll_angle)
            omega = 2 * math.pi / roll_period
            motion.add_roll(omega * omega * theta_rad)

        if pitch_angle != 0.0:
            theta_rad = math.radians(pitch_angle)
            omega = 2 * math.pi / pitch_period
            motion.add_pitch(omega * omega * theta_rad)

        if yaw_angle != 0.0:
            theta_rad = math.radians(yaw_angle)
            omega = 2 * math.pi / yaw_period
            motion.add_yaw(omega * omega * theta_rad)

        return motion

    def __repr__(self) -> str:
        """String representation for debugging."""
        comp_strs = []
        for c in self.components:
            if c.motion_type in {MotionType.ROLL, MotionType.PITCH, MotionType.YAW}:
                comp_strs.append(f"{c.motion_type.value}={c.amplitude:.4f} rad/s²")
            else:
                comp_strs.append(f"{c.motion_type.value}={c.amplitude:.2f} m/s²")

        components = ", ".join(comp_strs) if comp_strs else "none"
        return f"VesselMotion('{self.name}', center={self.motion_center}, components=[{components}])"


# ============================================================================
# Design Method and Limit State Enums
# ============================================================================


class DesignMethod(Enum):
    """Design method for load combinations.

    Attributes:
        LRFD: Load and Resistance Factor Design - uses ULSa and ULSb factors
        ASD: Allowable Stress Design - uses SLS factors only
    """
    LRFD = "lrfd"
    ASD = "asd"


class LimitState(Enum):
    """Limit states for load combinations.

    Attributes:
        ULSa: Ultimate Limit State a - typically 1.3 * DL + 1.3 * LL + 0.7 * EL
        ULSb: Ultimate Limit State b - typically 1.0 * DL + 1.0 * LL + 1.3 * EL
        SLS: Serviceability Limit State - typically 1.0 * DL + 1.0 * LL + 1.0 * EL
    """
    ULSa = "ulsa"
    ULSb = "ulsb"
    SLS = "sls"


class OperationType(Enum):
    """Type of offshore operation.

    Attributes:
        REMOVAL: Removal/decommissioning condition (uses ULSa_removal factors)
        TRANSPORT_INSTALL: Transportation and Installation (T&I) condition
    """
    REMOVAL = "removal"
    TRANSPORT_INSTALL = "transport_install"


@dataclass
class LoadCombinationFactors:
    """Load combination factors for a specific limit state.

    Attributes:
        dead_load: Factor for permanent/dead loads
        live_load: Factor for variable/live loads
        environmental: Factor for environmental loads (vessel motions)
    """
    dead_load: float
    live_load: float
    environmental: float


# Default load factors per limit state
DEFAULT_LOAD_FACTORS: Dict[LimitState, LoadCombinationFactors] = {
    LimitState.ULSa: LoadCombinationFactors(
        dead_load=1.3,
        live_load=1.3,
        environmental=0.7
    ),
    LimitState.ULSb: LoadCombinationFactors(
        dead_load=1.0,
        live_load=1.0,
        environmental=1.3
    ),
    LimitState.SLS: LoadCombinationFactors(
        dead_load=1.0,
        live_load=1.0,
        environmental=1.0
    ),
}

# Removal operation uses different ULSa factors
DEFAULT_REMOVAL_FACTORS: Dict[LimitState, LoadCombinationFactors] = {
    LimitState.ULSa: LoadCombinationFactors(
        dead_load=1.1,  # Reduced from 1.3 for removal
        live_load=1.1,
        environmental=0.7
    ),
    LimitState.ULSb: LoadCombinationFactors(
        dead_load=1.0,
        live_load=1.0,
        environmental=1.3
    ),
    LimitState.SLS: LoadCombinationFactors(
        dead_load=1.0,
        live_load=1.0,
        environmental=1.0
    ),
}


@dataclass
class AnalysisSettings:
    """Analysis settings including design method and operation type.

    Controls how load combinations are generated and which limit states
    are considered.

    Attributes:
        design_method: LRFD or ASD design approach
        operation_type: Optional operation type (REMOVAL or TRANSPORT_INSTALL).
            None means standard/default factors are used.
        custom_factors: Optional custom load factors (overrides defaults)

    Example:
        >>> settings = AnalysisSettings(design_method=DesignMethod.LRFD)
        >>> settings.get_limit_states()
        [<LimitState.ULSa: 'ulsa'>, <LimitState.ULSb: 'ulsb'>]
    """
    design_method: DesignMethod = DesignMethod.LRFD
    operation_type: Optional[OperationType] = None
    custom_factors: Optional[Dict[LimitState, LoadCombinationFactors]] = None

    def get_limit_states(self) -> List[LimitState]:
        """Get the limit states to consider based on design method.

        Returns:
            List of LimitState enums applicable to this configuration
        """
        if self.design_method == DesignMethod.LRFD:
            return [LimitState.ULSa, LimitState.ULSb]
        else:  # ASD
            return [LimitState.SLS]

    def get_factors(self, limit_state: LimitState) -> LoadCombinationFactors:
        """Get load factors for a specific limit state.

        Args:
            limit_state: The limit state to get factors for

        Returns:
            LoadCombinationFactors for the specified limit state
        """
        # Custom factors override defaults
        if self.custom_factors and limit_state in self.custom_factors:
            return self.custom_factors[limit_state]

        # Use removal factors for removal operation
        if self.operation_type == OperationType.REMOVAL:
            return DEFAULT_REMOVAL_FACTORS.get(
                limit_state,
                DEFAULT_LOAD_FACTORS[limit_state]
            )

        # Default factors
        return DEFAULT_LOAD_FACTORS[limit_state]


# ============================================================================
# VesselMotions Base Class (Plural)
# ============================================================================


class VesselMotions(ABC):
    """Abstract base class for generating multiple vessel motion load cases.

    Subclasses implement specific strategies for generating motion combinations:
    - VesselMotionsFromAmplitudes: From amplitude values with coupling rules
    - VesselMotionsFromNobleDenton: Standard Noble Denton 6-case approach

    The generated motions can be applied to a StructuralModel to create
    load cases with the appropriate acceleration fields.

    Attributes:
        name: Descriptive name for this motion set
        motion_center: Reference point [x, y, z] for rotational motions in meters
    """

    def __init__(
        self,
        name: str,
        motion_center: Optional[List[float]] = None
    ):
        """Initialize vessel motions generator.

        Args:
            name: Name for this motion set
            motion_center: Reference point [x, y, z] in meters (default: origin)
        """
        self.name = name
        self.motion_center = motion_center or [0.0, 0.0, 0.0]
        self._generated_motions: List[VesselMotion] = []

    @abstractmethod
    def generate_motions(self) -> List[VesselMotion]:
        """Generate the list of VesselMotion objects.

        Returns:
            List of VesselMotion objects, each representing a load case
        """
        pass

    def get_motions(self) -> List[VesselMotion]:
        """Get generated motions, generating if not already done.

        Returns:
            List of VesselMotion objects
        """
        if not self._generated_motions:
            self._generated_motions = self.generate_motions()
        return self._generated_motions

    def get_motion_names(self) -> List[str]:
        """Get names of all generated motions.

        Returns:
            List of motion names
        """
        return [m.name for m in self.get_motions()]

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}('{self.name}', n_motions={len(self.get_motions())})"


# ============================================================================
# VesselMotionsFromAmplitudes
# ============================================================================


@dataclass
class MotionAmplitudes:
    """Motion amplitudes for generating +/- load cases.

    All angular values can be specified either as direct accelerations (rad/s²)
    or as angle/period pairs (degrees and seconds).

    Attributes:
        heave: Heave acceleration in m/s²
        roll_accel: Roll angular acceleration in rad/s² (direct)
        roll_angle: Roll amplitude in degrees (with period)
        roll_period: Roll period in seconds
        pitch_accel: Pitch angular acceleration in rad/s² (direct)
        pitch_angle: Pitch amplitude in degrees (with period)
        pitch_period: Pitch period in seconds
        surge: Surge acceleration in m/s² (if not coupled)
        sway: Sway acceleration in m/s² (if not coupled)
    """
    heave: float = 0.0
    roll_accel: Optional[float] = None
    roll_angle: float = 0.0
    roll_period: float = 10.0
    pitch_accel: Optional[float] = None
    pitch_angle: float = 0.0
    pitch_period: float = 8.0
    surge: float = 0.0
    sway: float = 0.0

    def get_roll_acceleration(self) -> float:
        """Get roll acceleration, converting from angle/period if needed."""
        if self.roll_accel is not None:
            return self.roll_accel
        if self.roll_angle != 0.0:
            theta_rad = math.radians(self.roll_angle)
            omega = 2 * math.pi / self.roll_period
            return omega * omega * theta_rad
        return 0.0

    def get_pitch_acceleration(self) -> float:
        """Get pitch acceleration, converting from angle/period if needed."""
        if self.pitch_accel is not None:
            return self.pitch_accel
        if self.pitch_angle != 0.0:
            theta_rad = math.radians(self.pitch_angle)
            omega = 2 * math.pi / self.pitch_period
            return omega * omega * theta_rad
        return 0.0


class VesselMotionsFromAmplitudes(VesselMotions):
    """Generate vessel motion load cases from amplitude values with coupling.

    Coupling rules (based on typical vessel motion behavior):
    - +pitch couples with +surge (bow down -> forward acceleration)
    - +roll couples with -sway (starboard down -> port acceleration)

    Generates +/- variants for each non-zero amplitude.

    Example:
        >>> amplitudes = MotionAmplitudes(
        ...     heave=2.5,
        ...     pitch_angle=5.0,
        ...     pitch_period=8.0,
        ...     roll_angle=10.0,
        ...     roll_period=10.0
        ... )
        >>> motions = VesselMotionsFromAmplitudes(
        ...     name="Design Motions",
        ...     amplitudes=amplitudes,
        ...     motion_center=[50.0, 0.0, 5.0]
        ... )
        >>> len(motions.get_motions())  # heave+/-, pitch+/-, roll+/-
        6
    """

    def __init__(
        self,
        name: str,
        amplitudes: MotionAmplitudes,
        motion_center: Optional[List[float]] = None,
        pitch_surge_coupling: float = 1.0,
        roll_sway_coupling: float = -1.0
    ):
        """Initialize motion generator from amplitudes.

        Args:
            name: Name prefix for generated motions
            amplitudes: Motion amplitudes
            motion_center: Reference point [x, y, z] in meters
            pitch_surge_coupling: Coupling factor for pitch->surge (+1.0 = +pitch gives +surge)
            roll_sway_coupling: Coupling factor for roll->sway (-1.0 = +roll gives -sway)
        """
        super().__init__(name, motion_center)
        self.amplitudes = amplitudes
        self.pitch_surge_coupling = pitch_surge_coupling
        self.roll_sway_coupling = roll_sway_coupling

    def generate_motions(self) -> List[VesselMotion]:
        """Generate +/- motion load cases from amplitudes.

        Returns:
            List of VesselMotion objects
        """
        motions = []

        # Heave +/-
        if self.amplitudes.heave != 0.0:
            heave_pos = VesselMotion(f"{self.name} - Heave+")
            heave_pos.set_motion_center(self.motion_center)
            heave_pos.add_heave(self.amplitudes.heave)
            motions.append(heave_pos)

            heave_neg = VesselMotion(f"{self.name} - Heave-")
            heave_neg.set_motion_center(self.motion_center)
            heave_neg.add_heave(-self.amplitudes.heave)
            motions.append(heave_neg)

        # Pitch +/- with surge coupling
        pitch_accel = self.amplitudes.get_pitch_acceleration()
        if pitch_accel != 0.0:
            # Pitch+ with coupled surge
            pitch_pos = VesselMotion(f"{self.name} - Pitch+")
            pitch_pos.set_motion_center(self.motion_center)
            pitch_pos.add_pitch(pitch_accel)
            surge_coupled = self.amplitudes.surge + (
                pitch_accel * self.pitch_surge_coupling
            )
            if surge_coupled != 0.0:
                pitch_pos.add_surge(surge_coupled)
            motions.append(pitch_pos)

            # Pitch- with coupled surge
            pitch_neg = VesselMotion(f"{self.name} - Pitch-")
            pitch_neg.set_motion_center(self.motion_center)
            pitch_neg.add_pitch(-pitch_accel)
            surge_coupled_neg = self.amplitudes.surge + (
                -pitch_accel * self.pitch_surge_coupling
            )
            if surge_coupled_neg != 0.0:
                pitch_neg.add_surge(surge_coupled_neg)
            motions.append(pitch_neg)

        # Roll +/- with sway coupling
        roll_accel = self.amplitudes.get_roll_acceleration()
        if roll_accel != 0.0:
            # Roll+ with coupled sway
            roll_pos = VesselMotion(f"{self.name} - Roll+")
            roll_pos.set_motion_center(self.motion_center)
            roll_pos.add_roll(roll_accel)
            sway_coupled = self.amplitudes.sway + (
                roll_accel * self.roll_sway_coupling
            )
            if sway_coupled != 0.0:
                roll_pos.add_sway(sway_coupled)
            motions.append(roll_pos)

            # Roll- with coupled sway
            roll_neg = VesselMotion(f"{self.name} - Roll-")
            roll_neg.set_motion_center(self.motion_center)
            roll_neg.add_roll(-roll_accel)
            sway_coupled_neg = self.amplitudes.sway + (
                -roll_accel * self.roll_sway_coupling
            )
            if sway_coupled_neg != 0.0:
                roll_neg.add_sway(sway_coupled_neg)
            motions.append(roll_neg)

        return motions


# ============================================================================
# VesselMotionsFromNobleDenton
# ============================================================================


class VesselMotionsFromNobleDenton(VesselMotions):
    """Generate 6 vessel motion load cases per Noble Denton guidelines.

    Noble Denton approach for heavy lift and transportation operations:
    - 2 heave-only cases (heave+ and heave-)
    - 2 pitch+heave cases (pitch+ with heave, pitch- with heave)
    - 2 roll+heave cases (roll+ with heave, roll- with heave)

    Coupling rules are applied:
    - +pitch = +surge (bow pitching down causes forward acceleration)
    - +roll = -sway (rolling to starboard causes port acceleration)

    Example:
        >>> nd_motions = VesselMotionsFromNobleDenton(
        ...     name="Noble Denton",
        ...     heave=2.5,
        ...     roll_angle=15.0,
        ...     roll_period=10.0,
        ...     pitch_angle=5.0,
        ...     pitch_period=8.0,
        ...     motion_center=[50.0, 0.0, 5.0]
        ... )
        >>> motions = nd_motions.get_motions()
        >>> len(motions)
        6
        >>> [m.name for m in motions]  # doctest: +NORMALIZE_WHITESPACE
        ['Noble Denton - Heave+', 'Noble Denton - Heave-',
         'Noble Denton - Pitch+ Heave', 'Noble Denton - Pitch- Heave',
         'Noble Denton - Roll+ Heave', 'Noble Denton - Roll- Heave']
    """

    def __init__(
        self,
        name: str,
        heave: float,
        roll_angle: float = 0.0,
        roll_period: float = 10.0,
        roll_accel: Optional[float] = None,
        pitch_angle: float = 0.0,
        pitch_period: float = 8.0,
        pitch_accel: Optional[float] = None,
        motion_center: Optional[List[float]] = None,
        pitch_surge_coupling: float = 1.0,
        roll_sway_coupling: float = -1.0
    ):
        """Initialize Noble Denton motion generator.

        Args:
            name: Name prefix for generated motions
            heave: Heave acceleration in m/s² (always applied with roll/pitch)
            roll_angle: Roll amplitude in degrees
            roll_period: Roll period in seconds
            roll_accel: Roll acceleration in rad/s² (overrides angle/period)
            pitch_angle: Pitch amplitude in degrees
            pitch_period: Pitch period in seconds
            pitch_accel: Pitch acceleration in rad/s² (overrides angle/period)
            motion_center: Reference point [x, y, z] in meters
            pitch_surge_coupling: Coupling factor (+1.0 = +pitch gives +surge)
            roll_sway_coupling: Coupling factor (-1.0 = +roll gives -sway)
        """
        super().__init__(name, motion_center)
        self.heave = heave
        self.roll_angle = roll_angle
        self.roll_period = roll_period
        self.roll_accel = roll_accel
        self.pitch_angle = pitch_angle
        self.pitch_period = pitch_period
        self.pitch_accel = pitch_accel
        self.pitch_surge_coupling = pitch_surge_coupling
        self.roll_sway_coupling = roll_sway_coupling

    def _get_roll_acceleration(self) -> float:
        """Get roll acceleration, converting from angle/period if needed."""
        if self.roll_accel is not None:
            return self.roll_accel
        if self.roll_angle != 0.0:
            theta_rad = math.radians(self.roll_angle)
            omega = 2 * math.pi / self.roll_period
            return omega * omega * theta_rad
        return 0.0

    def _get_pitch_acceleration(self) -> float:
        """Get pitch acceleration, converting from angle/period if needed."""
        if self.pitch_accel is not None:
            return self.pitch_accel
        if self.pitch_angle != 0.0:
            theta_rad = math.radians(self.pitch_angle)
            omega = 2 * math.pi / self.pitch_period
            return omega * omega * theta_rad
        return 0.0

    def generate_motions(self) -> List[VesselMotion]:
        """Generate 6 Noble Denton motion load cases.

        Returns:
            List of 6 VesselMotion objects:
            - Heave+, Heave-
            - Pitch+ Heave, Pitch- Heave
            - Roll+ Heave, Roll- Heave
        """
        motions = []

        # 1. Heave+ (heave only, upward)
        heave_pos = VesselMotion(f"{self.name} - Heave+")
        heave_pos.set_motion_center(self.motion_center)
        heave_pos.add_heave(self.heave)
        motions.append(heave_pos)

        # 2. Heave- (heave only, downward)
        heave_neg = VesselMotion(f"{self.name} - Heave-")
        heave_neg.set_motion_center(self.motion_center)
        heave_neg.add_heave(-self.heave)
        motions.append(heave_neg)

        # Get angular accelerations
        pitch_accel = self._get_pitch_acceleration()
        roll_accel = self._get_roll_acceleration()

        # 3. Pitch+ with Heave (and coupled surge)
        pitch_heave_pos = VesselMotion(f"{self.name} - Pitch+ Heave")
        pitch_heave_pos.set_motion_center(self.motion_center)
        pitch_heave_pos.add_heave(self.heave)
        if pitch_accel != 0.0:
            pitch_heave_pos.add_pitch(pitch_accel)
            surge_coupled = pitch_accel * self.pitch_surge_coupling
            if surge_coupled != 0.0:
                pitch_heave_pos.add_surge(surge_coupled)
        motions.append(pitch_heave_pos)

        # 4. Pitch- with Heave (and coupled surge)
        pitch_heave_neg = VesselMotion(f"{self.name} - Pitch- Heave")
        pitch_heave_neg.set_motion_center(self.motion_center)
        pitch_heave_neg.add_heave(self.heave)
        if pitch_accel != 0.0:
            pitch_heave_neg.add_pitch(-pitch_accel)
            surge_coupled_neg = -pitch_accel * self.pitch_surge_coupling
            if surge_coupled_neg != 0.0:
                pitch_heave_neg.add_surge(surge_coupled_neg)
        motions.append(pitch_heave_neg)

        # 5. Roll+ with Heave (and coupled sway)
        roll_heave_pos = VesselMotion(f"{self.name} - Roll+ Heave")
        roll_heave_pos.set_motion_center(self.motion_center)
        roll_heave_pos.add_heave(self.heave)
        if roll_accel != 0.0:
            roll_heave_pos.add_roll(roll_accel)
            sway_coupled = roll_accel * self.roll_sway_coupling
            if sway_coupled != 0.0:
                roll_heave_pos.add_sway(sway_coupled)
        motions.append(roll_heave_pos)

        # 6. Roll- with Heave (and coupled sway)
        roll_heave_neg = VesselMotion(f"{self.name} - Roll- Heave")
        roll_heave_neg.set_motion_center(self.motion_center)
        roll_heave_neg.add_heave(self.heave)
        if roll_accel != 0.0:
            roll_heave_neg.add_roll(-roll_accel)
            sway_coupled_neg = -roll_accel * self.roll_sway_coupling
            if sway_coupled_neg != 0.0:
                roll_heave_neg.add_sway(sway_coupled_neg)
        motions.append(roll_heave_neg)

        return motions


# ============================================================================
# Load Combination Generation
# ============================================================================


@dataclass
class GeneratedLoadCombination:
    """A generated load combination with factors.

    Attributes:
        name: Combination name (e.g., "ULSa - Heave+")
        limit_state: The limit state this combination represents
        vessel_motion: The VesselMotion for environmental load
        dead_load_factor: Factor for permanent loads
        live_load_factor: Factor for variable loads
        environmental_factor: Factor for environmental loads
    """
    name: str
    limit_state: LimitState
    vessel_motion: VesselMotion
    dead_load_factor: float
    live_load_factor: float
    environmental_factor: float


def generate_load_combinations(
    vessel_motions: VesselMotions,
    settings: AnalysisSettings
) -> List[GeneratedLoadCombination]:
    """Generate load combinations from vessel motions and analysis settings.

    Creates combinations for each vessel motion case with each applicable
    limit state based on the design method.

    Args:
        vessel_motions: VesselMotions generator (e.g., VesselMotionsFromNobleDenton)
        settings: AnalysisSettings with design method and operation type

    Returns:
        List of GeneratedLoadCombination objects

    Example:
        >>> nd_motions = VesselMotionsFromNobleDenton(
        ...     "ND", heave=2.5, pitch_angle=5.0, pitch_period=8.0
        ... )
        >>> settings = AnalysisSettings(design_method=DesignMethod.LRFD)
        >>> combinations = generate_load_combinations(nd_motions, settings)
        >>> len(combinations)  # 6 motions * 2 limit states (ULSa, ULSb)
        12
    """
    combinations = []
    limit_states = settings.get_limit_states()
    motions = vessel_motions.get_motions()

    for motion in motions:
        for limit_state in limit_states:
            factors = settings.get_factors(limit_state)
            combo = GeneratedLoadCombination(
                name=f"{limit_state.value.upper()} - {motion.name}",
                limit_state=limit_state,
                vessel_motion=motion,
                dead_load_factor=factors.dead_load,
                live_load_factor=factors.live_load,
                environmental_factor=factors.environmental
            )
            combinations.append(combo)

    return combinations
