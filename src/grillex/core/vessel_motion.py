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
class LoadCaseSpec:
    """Specification for a load case to be generated from a vessel motions generator.

    This represents the configuration for a single load case that will be
    created when the generator is added to a model.

    Attributes:
        name: Load case name (e.g., "Transport - Heave+")
        accelerations: 6-DOF acceleration field [ax, ay, az, αx, αy, αz]
            - ax, ay, az: Linear accelerations in m/s²
            - αx, αy, αz: Angular accelerations in rad/s²
        motion_center: Reference point [x, y, z] for rotational motions in meters
    """
    name: str
    accelerations: List[float]
    motion_center: List[float]


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

    The generator is the single source of truth for motion amplitudes. Load cases
    are generated directly from the generator when added to a model.

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
        self._load_case_specs: Optional[List[LoadCaseSpec]] = None
        # Track linked load cases for auto-update
        self._linked_load_cases: Dict[str, Any] = {}  # name -> LoadCase

    @abstractmethod
    def get_load_case_specs(self) -> List[LoadCaseSpec]:
        """Get specifications for load cases to generate.

        Each spec defines a load case name, acceleration field, and motion center.
        Subclasses implement this to define their specific load case patterns.

        Returns:
            List of LoadCaseSpec objects defining load cases to create
        """
        pass

    @abstractmethod
    def get_combination_specs(self) -> List[List[str]]:
        """Get combinations by load case names.

        Returns a list of combinations, where each combination is a list of
        load case names that should be combined together.

        Returns:
            List of lists of load case names
        """
        pass

    def get_load_case_names(self) -> List[str]:
        """Get names of all load cases that will be generated.

        Returns:
            List of load case names
        """
        return [spec.name for spec in self.get_load_case_specs()]

    def link_load_case(self, name: str, load_case: Any) -> None:
        """Link a load case to this generator for auto-updates.

        When the generator's amplitudes change, linked load cases are
        automatically updated.

        Args:
            name: Load case name (must match a spec name)
            load_case: C++ LoadCase object
        """
        self._linked_load_cases[name] = load_case

    def update_linked_load_cases(self) -> None:
        """Update all linked load cases with current acceleration fields.

        Call this after modifying generator amplitudes.
        """
        import numpy as np
        for spec in self.get_load_case_specs():
            if spec.name in self._linked_load_cases:
                lc = self._linked_load_cases[spec.name]
                lc.set_acceleration_field(
                    np.array(spec.accelerations),
                    np.array(spec.motion_center)
                )

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}('{self.name}', n_load_cases={len(self.get_load_case_specs())})"

    # ===== Backwards compatibility =====
    # These methods maintain compatibility with code that uses the old VesselMotion-based API

    def generate_motions(self) -> List[VesselMotion]:
        """Generate VesselMotion objects (backwards compatibility).

        DEPRECATED: Use get_load_case_specs() for new code.

        Returns:
            List of VesselMotion objects
        """
        motions = []
        for spec in self.get_load_case_specs():
            motion = VesselMotion(spec.name)
            motion.set_motion_center(spec.motion_center)

            # Parse accelerations back to components
            ax, ay, az, rx, ry, rz = spec.accelerations
            if ax != 0.0:
                motion.add_surge(ax)
            if ay != 0.0:
                motion.add_sway(ay)
            if az != 0.0:
                motion.add_heave(az)
            if rx != 0.0:
                motion.add_roll(rx)
            if ry != 0.0:
                motion.add_pitch(ry)
            if rz != 0.0:
                motion.add_yaw(rz)

            motions.append(motion)
        return motions

    def get_motions(self) -> List[VesselMotion]:
        """Get generated motions (backwards compatibility).

        DEPRECATED: Use get_load_case_specs() for new code.

        Returns:
            List of VesselMotion objects
        """
        return self.generate_motions()

    def get_motion_names(self) -> List[str]:
        """Get names of all generated motions (backwards compatibility).

        DEPRECATED: Use get_load_case_names() for new code.

        Returns:
            List of motion names
        """
        return self.get_load_case_names()

    def get_motion_combinations(self) -> List[List[VesselMotion]]:
        """Get motion combinations (backwards compatibility).

        DEPRECATED: Use get_combination_specs() for new code.

        Returns:
            List of motion lists
        """
        motions = self.generate_motions()
        motion_by_name = {m.name: m for m in motions}
        combinations = []
        for combo_names in self.get_combination_specs():
            combo = [motion_by_name[name] for name in combo_names if name in motion_by_name]
            combinations.append(combo)
        return combinations


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
        yaw_accel: Yaw angular acceleration in rad/s² (direct)
        yaw_angle: Yaw amplitude in degrees (with period)
        yaw_period: Yaw period in seconds
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
    yaw_accel: Optional[float] = None
    yaw_angle: float = 0.0
    yaw_period: float = 12.0
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

    def get_yaw_acceleration(self) -> float:
        """Get yaw acceleration, converting from angle/period if needed."""
        if self.yaw_accel is not None:
            return self.yaw_accel
        if self.yaw_angle != 0.0:
            theta_rad = math.radians(self.yaw_angle)
            omega = 2 * math.pi / self.yaw_period
            return omega * omega * theta_rad
        return 0.0


class VesselMotionsFromAmplitudes(VesselMotions):
    """Generate vessel motion load cases from amplitude values with coupling.

    This generator is the single source of truth for motion amplitudes.
    Load cases are generated directly from these amplitudes when added to a model.

    Generates +/- variants for each of the 4 independent motions:
    - Heave+/- (vertical acceleration)
    - Roll+/- (with sway coupling)
    - Pitch+/- (with surge coupling)
    - Yaw+/- (heading change)

    Coupling rules (based on typical vessel motion behavior):
    - +pitch couples with +surge (bow down -> forward acceleration)
    - +roll couples with -sway (starboard down -> port acceleration)

    Example:
        >>> amplitudes = MotionAmplitudes(
        ...     heave=2.5,
        ...     pitch_angle=5.0,
        ...     pitch_period=8.0,
        ...     roll_angle=10.0,
        ...     roll_period=10.0,
        ...     yaw_angle=3.0,
        ...     yaw_period=12.0
        ... )
        >>> motions = VesselMotionsFromAmplitudes(
        ...     name="Design Motions",
        ...     amplitudes=amplitudes,
        ...     motion_center=[50.0, 0.0, 5.0]
        ... )
        >>> len(motions.get_load_case_specs())  # heave+/-, pitch+/-, roll+/-, yaw+/-
        8
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

        The generator stores the amplitudes and can generate load cases directly.

        Args:
            name: Name prefix for generated load cases
            amplitudes: Motion amplitudes (single source of truth)
            motion_center: Reference point [x, y, z] in meters
            pitch_surge_coupling: Coupling factor for pitch->surge (+1.0 = +pitch gives +surge)
            roll_sway_coupling: Coupling factor for roll->sway (-1.0 = +roll gives -sway)
        """
        super().__init__(name, motion_center)
        self._amplitudes = amplitudes
        self._pitch_surge_coupling = pitch_surge_coupling
        self._roll_sway_coupling = roll_sway_coupling

    # ===== Properties for amplitude access and modification =====

    @property
    def amplitudes(self) -> MotionAmplitudes:
        """Motion amplitudes (single source of truth)."""
        return self._amplitudes

    @amplitudes.setter
    def amplitudes(self, value: MotionAmplitudes) -> None:
        self._amplitudes = value
        self.update_linked_load_cases()

    @property
    def pitch_surge_coupling(self) -> float:
        """Coupling factor for pitch->surge."""
        return self._pitch_surge_coupling

    @pitch_surge_coupling.setter
    def pitch_surge_coupling(self, value: float) -> None:
        self._pitch_surge_coupling = value
        self.update_linked_load_cases()

    @property
    def roll_sway_coupling(self) -> float:
        """Coupling factor for roll->sway."""
        return self._roll_sway_coupling

    @roll_sway_coupling.setter
    def roll_sway_coupling(self, value: float) -> None:
        self._roll_sway_coupling = value
        self.update_linked_load_cases()

    # ===== Load Case Specification =====

    def get_load_case_specs(self) -> List[LoadCaseSpec]:
        """Get specifications for load cases (up to 8 total).

        Generates +/- variants for each motion type with non-zero amplitude.

        Returns:
            List of LoadCaseSpec objects
        """
        specs = []

        # Heave +/-
        if self._amplitudes.heave != 0.0:
            # Heave+ (upward)
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Heave+",
                accelerations=[0.0, 0.0, self._amplitudes.heave, 0.0, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ))
            # Heave- (downward)
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Heave-",
                accelerations=[0.0, 0.0, -self._amplitudes.heave, 0.0, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ))

        # Pitch +/- with surge coupling
        pitch_accel = self._amplitudes.get_pitch_acceleration()
        if pitch_accel != 0.0:
            # Pitch+ with coupled surge
            surge_coupled_pos = self._amplitudes.surge + (pitch_accel * self._pitch_surge_coupling)
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Pitch+",
                accelerations=[surge_coupled_pos, 0.0, 0.0, 0.0, pitch_accel, 0.0],
                motion_center=list(self.motion_center)
            ))
            # Pitch- with coupled surge
            surge_coupled_neg = self._amplitudes.surge + (-pitch_accel * self._pitch_surge_coupling)
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Pitch-",
                accelerations=[surge_coupled_neg, 0.0, 0.0, 0.0, -pitch_accel, 0.0],
                motion_center=list(self.motion_center)
            ))

        # Roll +/- with sway coupling
        roll_accel = self._amplitudes.get_roll_acceleration()
        if roll_accel != 0.0:
            # Roll+ with coupled sway
            sway_coupled_pos = self._amplitudes.sway + (roll_accel * self._roll_sway_coupling)
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Roll+",
                accelerations=[0.0, sway_coupled_pos, 0.0, roll_accel, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ))
            # Roll- with coupled sway
            sway_coupled_neg = self._amplitudes.sway + (-roll_accel * self._roll_sway_coupling)
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Roll-",
                accelerations=[0.0, sway_coupled_neg, 0.0, -roll_accel, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ))

        # Yaw +/-
        yaw_accel = self._amplitudes.get_yaw_acceleration()
        if yaw_accel != 0.0:
            # Yaw+
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Yaw+",
                accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, yaw_accel],
                motion_center=list(self.motion_center)
            ))
            # Yaw-
            specs.append(LoadCaseSpec(
                name=f"{self.name} - Yaw-",
                accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, -yaw_accel],
                motion_center=list(self.motion_center)
            ))

        return specs

    def get_combination_specs(self) -> List[List[str]]:
        """Get all load case combinations (up to 16 total).

        Generates all 2^n permutations where n is number of active motion types.
        If any motion type is not defined (amplitude = 0), it's excluded.

        Returns:
            List of load case name lists, each representing one combination
        """
        from itertools import product

        # Build pairs of load case names for each motion type
        name_pairs: List[List[str]] = []

        # Heave
        if self._amplitudes.heave != 0.0:
            name_pairs.append([
                f"{self.name} - Heave+",
                f"{self.name} - Heave-"
            ])

        # Pitch
        if self._amplitudes.get_pitch_acceleration() != 0.0:
            name_pairs.append([
                f"{self.name} - Pitch+",
                f"{self.name} - Pitch-"
            ])

        # Roll
        if self._amplitudes.get_roll_acceleration() != 0.0:
            name_pairs.append([
                f"{self.name} - Roll+",
                f"{self.name} - Roll-"
            ])

        # Yaw
        if self._amplitudes.get_yaw_acceleration() != 0.0:
            name_pairs.append([
                f"{self.name} - Yaw+",
                f"{self.name} - Yaw-"
            ])

        # Generate all combinations (cartesian product)
        if not name_pairs:
            return []

        combinations = []
        for combo in product(*name_pairs):
            combinations.append(list(combo))

        return combinations


# ============================================================================
# VesselMotionsFromNobleDenton
# ============================================================================


class VesselMotionsFromNobleDenton(VesselMotions):
    """Generate 6 vessel motion load cases per Noble Denton guidelines.

    Noble Denton approach for heavy lift and transportation operations:
    - 2 heave-only cases (heave+ and heave-)
    - 2 pitch-only cases (pitch+ and pitch-)
    - 2 roll-only cases (roll+ and roll-)

    This generator is the single source of truth for motion amplitudes.
    Load cases are generated directly from these amplitudes when added to a model.

    Load cases are standalone (not combined). Heave is combined with pitch/roll
    in load combinations, not in the load cases themselves.

    Note:
        Noble Denton does NOT apply surge/sway coupling. Rotations are defined
        at the motion center (rotation center of the barge), so coupling is
        handled implicitly through the motion center reference point.

    Load combinations (8 per limit state):
    - Heave+ & Roll+, Heave+ & Roll-, Heave- & Roll+, Heave- & Roll-
    - Heave+ & Pitch+, Heave+ & Pitch-, Heave- & Pitch+, Heave- & Pitch-

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
        >>> len(nd_motions.get_load_case_specs())
        6
        >>> [spec.name for spec in nd_motions.get_load_case_specs()]  # doctest: +NORMALIZE_WHITESPACE
        ['Noble Denton - Heave+', 'Noble Denton - Heave-',
         'Noble Denton - Pitch+', 'Noble Denton - Pitch-',
         'Noble Denton - Roll+', 'Noble Denton - Roll-']
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
    ):
        """Initialize Noble Denton motion generator.

        The generator stores the amplitudes and can generate load cases directly.

        Args:
            name: Name prefix for generated load cases
            heave: Heave acceleration in m/s²
            roll_angle: Roll amplitude in degrees
            roll_period: Roll period in seconds
            roll_accel: Roll acceleration in rad/s² (overrides angle/period)
            pitch_angle: Pitch amplitude in degrees
            pitch_period: Pitch period in seconds
            pitch_accel: Pitch acceleration in rad/s² (overrides angle/period)
            motion_center: Reference point [x, y, z] in meters (rotation center)
        """
        super().__init__(name, motion_center)
        self._heave = heave
        self._roll_angle = roll_angle
        self._roll_period = roll_period
        self._roll_accel = roll_accel
        self._pitch_angle = pitch_angle
        self._pitch_period = pitch_period
        self._pitch_accel = pitch_accel

    # ===== Properties for amplitude access and modification =====

    @property
    def heave(self) -> float:
        """Heave acceleration in m/s²."""
        return self._heave

    @heave.setter
    def heave(self, value: float) -> None:
        self._heave = value
        self.update_linked_load_cases()

    @property
    def roll_angle(self) -> float:
        """Roll amplitude in degrees."""
        return self._roll_angle

    @roll_angle.setter
    def roll_angle(self, value: float) -> None:
        self._roll_angle = value
        self._roll_accel = None  # Clear direct accel when angle is set
        self.update_linked_load_cases()

    @property
    def roll_period(self) -> float:
        """Roll period in seconds."""
        return self._roll_period

    @roll_period.setter
    def roll_period(self, value: float) -> None:
        self._roll_period = value
        self.update_linked_load_cases()

    @property
    def roll_accel(self) -> Optional[float]:
        """Roll acceleration in rad/s² (overrides angle/period)."""
        return self._roll_accel

    @roll_accel.setter
    def roll_accel(self, value: Optional[float]) -> None:
        self._roll_accel = value
        self.update_linked_load_cases()

    @property
    def pitch_angle(self) -> float:
        """Pitch amplitude in degrees."""
        return self._pitch_angle

    @pitch_angle.setter
    def pitch_angle(self, value: float) -> None:
        self._pitch_angle = value
        self._pitch_accel = None  # Clear direct accel when angle is set
        self.update_linked_load_cases()

    @property
    def pitch_period(self) -> float:
        """Pitch period in seconds."""
        return self._pitch_period

    @pitch_period.setter
    def pitch_period(self, value: float) -> None:
        self._pitch_period = value
        self.update_linked_load_cases()

    @property
    def pitch_accel(self) -> Optional[float]:
        """Pitch acceleration in rad/s² (overrides angle/period)."""
        return self._pitch_accel

    @pitch_accel.setter
    def pitch_accel(self, value: Optional[float]) -> None:
        self._pitch_accel = value
        self.update_linked_load_cases()

    # ===== Computed accelerations =====

    def get_roll_acceleration(self) -> float:
        """Get roll acceleration, converting from angle/period if needed."""
        if self._roll_accel is not None:
            return self._roll_accel
        if self._roll_angle != 0.0:
            theta_rad = math.radians(self._roll_angle)
            omega = 2 * math.pi / self._roll_period
            return omega * omega * theta_rad
        return 0.0

    def get_pitch_acceleration(self) -> float:
        """Get pitch acceleration, converting from angle/period if needed."""
        if self._pitch_accel is not None:
            return self._pitch_accel
        if self._pitch_angle != 0.0:
            theta_rad = math.radians(self._pitch_angle)
            omega = 2 * math.pi / self._pitch_period
            return omega * omega * theta_rad
        return 0.0

    # ===== Load Case Specification =====

    def get_load_case_specs(self) -> List[LoadCaseSpec]:
        """Get specifications for the 6 Noble Denton load cases.

        Returns:
            List of 6 LoadCaseSpec objects:
            - Heave+, Heave- (heave only)
            - Pitch+, Pitch- (pitch only, no surge coupling)
            - Roll+, Roll- (roll only, no sway coupling)
        """
        pitch_accel = self.get_pitch_acceleration()
        roll_accel = self.get_roll_acceleration()

        # Acceleration format: [ax, ay, az, αx, αy, αz]
        specs = [
            # 1. Heave+ (heave only, upward)
            LoadCaseSpec(
                name=f"{self.name} - Heave+",
                accelerations=[0.0, 0.0, self._heave, 0.0, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ),
            # 2. Heave- (heave only, downward)
            LoadCaseSpec(
                name=f"{self.name} - Heave-",
                accelerations=[0.0, 0.0, -self._heave, 0.0, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ),
            # 3. Pitch+ (pitch only, NO heave, NO surge coupling)
            LoadCaseSpec(
                name=f"{self.name} - Pitch+",
                accelerations=[0.0, 0.0, 0.0, 0.0, pitch_accel, 0.0],
                motion_center=list(self.motion_center)
            ),
            # 4. Pitch- (pitch only, NO heave, NO surge coupling)
            LoadCaseSpec(
                name=f"{self.name} - Pitch-",
                accelerations=[0.0, 0.0, 0.0, 0.0, -pitch_accel, 0.0],
                motion_center=list(self.motion_center)
            ),
            # 5. Roll+ (roll only, NO heave, NO sway coupling)
            LoadCaseSpec(
                name=f"{self.name} - Roll+",
                accelerations=[0.0, 0.0, 0.0, roll_accel, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ),
            # 6. Roll- (roll only, NO heave, NO sway coupling)
            LoadCaseSpec(
                name=f"{self.name} - Roll-",
                accelerations=[0.0, 0.0, 0.0, -roll_accel, 0.0, 0.0],
                motion_center=list(self.motion_center)
            ),
        ]
        return specs

    def get_combination_specs(self) -> List[List[str]]:
        """Get Noble Denton combination specifications (8 total).

        Noble Denton combinations pair heave with roll or pitch:
        - Heave+ & Roll+, Heave+ & Roll-, Heave- & Roll+, Heave- & Roll-
        - Heave+ & Pitch+, Heave+ & Pitch-, Heave- & Pitch+, Heave- & Pitch-

        Returns:
            List of 8 pairs of load case names
        """
        heave_pos = f"{self.name} - Heave+"
        heave_neg = f"{self.name} - Heave-"
        pitch_pos = f"{self.name} - Pitch+"
        pitch_neg = f"{self.name} - Pitch-"
        roll_pos = f"{self.name} - Roll+"
        roll_neg = f"{self.name} - Roll-"

        return [
            # Heave × Roll combinations
            [heave_pos, roll_pos],   # Heave+ Roll+
            [heave_pos, roll_neg],   # Heave+ Roll-
            [heave_neg, roll_pos],   # Heave- Roll+
            [heave_neg, roll_neg],   # Heave- Roll-
            # Heave × Pitch combinations
            [heave_pos, pitch_pos],  # Heave+ Pitch+
            [heave_pos, pitch_neg],  # Heave+ Pitch-
            [heave_neg, pitch_pos],  # Heave- Pitch+
            [heave_neg, pitch_neg],  # Heave- Pitch-
        ]

    # ===== Backwards compatibility =====

    def _get_roll_acceleration(self) -> float:
        """Get roll acceleration (backwards compatibility)."""
        return self.get_roll_acceleration()

    def _get_pitch_acceleration(self) -> float:
        """Get pitch acceleration (backwards compatibility)."""
        return self.get_pitch_acceleration()


# ============================================================================
# Load Combination Generation
# ============================================================================


@dataclass
class GeneratedLoadCombination:
    """A generated load combination with factors.

    Attributes:
        name: Combination name (e.g., "ULSa - Heave+ Roll+")
        limit_state: The limit state this combination represents
        vessel_motions: List of VesselMotion objects combined in this load combination
        dead_load_factor: Factor for permanent loads
        live_load_factor: Factor for variable loads
        environmental_factor: Factor for environmental loads
    """
    name: str
    limit_state: LimitState
    vessel_motions: List[VesselMotion]
    dead_load_factor: float
    live_load_factor: float
    environmental_factor: float

    # Backwards compatibility: single vessel_motion property
    @property
    def vessel_motion(self) -> Optional[VesselMotion]:
        """Get first vessel motion (for backwards compatibility)."""
        return self.vessel_motions[0] if self.vessel_motions else None

    def to_load_combination(self, combination_id: int = 0) -> "LoadCombination":
        """Convert to a C++ LoadCombination object with type-based factors.

        Creates a LoadCombination with the correct factors for each load type.
        Load cases must be added separately using add_load_case().

        The factors are mapped as:
        - dead_load_factor → permanent_factor
        - live_load_factor → variable_factor
        - environmental_factor → environmental_factor

        Args:
            combination_id: Unique ID for the combination (default: 0)

        Returns:
            C++ LoadCombination object with factors set, ready for load cases

        Example:
            >>> combo = generated.to_load_combination(combination_id=1)
            >>> combo.add_load_case(dead_load)   # Uses dead_load_factor
            >>> combo.add_load_case(vessel_motion)  # Uses environmental_factor
        """
        from grillex._grillex_cpp import LoadCombination

        return LoadCombination(
            combination_id,
            self.name,
            permanent_factor=self.dead_load_factor,
            variable_factor=self.live_load_factor,
            environmental_factor=self.environmental_factor,
            accidental_factor=1.0
        )


def generate_load_combinations(
    vessel_motions: VesselMotions,
    settings: AnalysisSettings
) -> List[GeneratedLoadCombination]:
    """Generate load combinations from vessel motions and analysis settings.

    Creates combinations using the vessel motion generator's combination pattern.
    For Noble Denton, this creates 8 combinations per limit state (Heave × Roll/Pitch).
    For Amplitudes, this creates up to 16 combinations per limit state (2^4 permutations).

    Args:
        vessel_motions: VesselMotions generator (e.g., VesselMotionsFromNobleDenton)
        settings: AnalysisSettings with design method and operation type

    Returns:
        List of GeneratedLoadCombination objects

    Example:
        >>> nd_motions = VesselMotionsFromNobleDenton(
        ...     "ND", heave=2.5, pitch_angle=5.0, pitch_period=8.0,
        ...     roll_angle=10.0, roll_period=10.0
        ... )
        >>> settings = AnalysisSettings(design_method=DesignMethod.LRFD)
        >>> combinations = generate_load_combinations(nd_motions, settings)
        >>> len(combinations)  # 8 combinations * 2 limit states (ULSa, ULSb)
        16
    """
    combinations = []
    limit_states = settings.get_limit_states()
    motion_combinations = vessel_motions.get_motion_combinations()

    for motion_group in motion_combinations:
        # Build combination name from motion names
        motion_names = []
        for m in motion_group:
            # Extract the suffix (e.g., "Heave+" from "Name - Heave+")
            if " - " in m.name:
                motion_names.append(m.name.split(" - ")[-1])
            else:
                motion_names.append(m.name)
        combo_name = " ".join(motion_names)

        for limit_state in limit_states:
            factors = settings.get_factors(limit_state)
            combo = GeneratedLoadCombination(
                name=f"{limit_state.value.upper()} - {combo_name}",
                limit_state=limit_state,
                vessel_motions=motion_group,
                dead_load_factor=factors.dead_load,
                live_load_factor=factors.live_load,
                environmental_factor=factors.environmental
            )
            combinations.append(combo)

    return combinations


def apply_generated_combinations(
    model: "StructuralModel",
    generated_combinations: List[GeneratedLoadCombination],
    dead_load_cases: Optional[List[str]] = None,
    live_load_cases: Optional[List[str]] = None
) -> List["LoadCombination"]:
    """Apply generated load combinations to a model.

    This function:
    1. Creates vessel motion load cases for each unique VesselMotion
    2. Creates C++ LoadCombination objects with type-based factors
    3. Adds all relevant load cases to each combination
    4. Returns the list of LoadCombinations ready for analysis

    If dead_load_cases or live_load_cases are not provided, they are
    automatically detected from the model's existing load cases by type.

    Args:
        model: StructuralModel to apply combinations to
        generated_combinations: List of GeneratedLoadCombination objects
        dead_load_cases: Names of permanent load cases (auto-detected if None)
        live_load_cases: Names of variable load cases (auto-detected if None)

    Returns:
        List of C++ LoadCombination objects with load cases added

    Example:
        >>> # Generate combinations
        >>> nd = VesselMotionsFromNobleDenton(
        ...     "ND", heave=2.5, pitch_angle=5.0, pitch_period=8.0,
        ...     roll_angle=10.0, roll_period=10.0
        ... )
        >>> settings = AnalysisSettings(design_method=DesignMethod.LRFD)
        >>> generated = generate_load_combinations(nd, settings)
        >>>
        >>> # Apply to model
        >>> combinations = apply_generated_combinations(model, generated)
        >>>
        >>> # Analyze each combination
        >>> for combo in combinations:
        ...     result = model.analyze_load_combination(combo)
    """
    from grillex._grillex_cpp import LoadCaseType

    # Get all load cases from model
    all_load_cases = {lc.name: lc for lc in model.get_load_cases()}

    # Auto-detect load case types if not provided
    if dead_load_cases is None:
        dead_load_cases = [
            lc.name for lc in model.get_load_cases()
            if lc.type == LoadCaseType.Permanent
        ]

    if live_load_cases is None:
        live_load_cases = [
            lc.name for lc in model.get_load_cases()
            if lc.type == LoadCaseType.Variable
        ]

    # Track which vessel motions have load cases created
    motion_load_cases: Dict[str, Any] = {}  # motion name -> LoadCase object

    # Create vessel motion load cases for each unique motion across all combinations
    for gen in generated_combinations:
        for motion in gen.vessel_motions:
            if motion.name not in motion_load_cases:
                # Check if load case already exists
                existing = model.get_vessel_motion(motion.name)
                if existing is not None:
                    # Use existing vessel motion's linked load case
                    motion_load_cases[motion.name] = all_load_cases.get(motion.name)
                else:
                    # Create new vessel motion load case
                    vm = model.add_vessel_motion_load_case(
                        name=motion.name,
                        heave=_get_motion_component(motion, MotionType.HEAVE),
                        pitch=_get_motion_component(motion, MotionType.PITCH),
                        roll=_get_motion_component(motion, MotionType.ROLL),
                        surge=_get_motion_component(motion, MotionType.SURGE),
                        sway=_get_motion_component(motion, MotionType.SWAY),
                        yaw=_get_motion_component(motion, MotionType.YAW),
                        motion_center=motion.motion_center
                    )
                    # Refresh load cases dict and get the new one
                    all_load_cases = {lc.name: lc for lc in model.get_load_cases()}
                    motion_load_cases[motion.name] = all_load_cases.get(motion.name)

    # Convert to C++ LoadCombination objects with load cases added
    result = []
    for i, gen in enumerate(generated_combinations):
        # Create combination with type-based factors
        combo = gen.to_load_combination(combination_id=i + 1)

        # Add dead load cases (uses permanent_factor automatically)
        for lc_name in dead_load_cases:
            lc = all_load_cases.get(lc_name)
            if lc is not None:
                combo.add_load_case(lc)

        # Add live load cases (uses variable_factor automatically)
        for lc_name in live_load_cases:
            lc = all_load_cases.get(lc_name)
            if lc is not None:
                combo.add_load_case(lc)

        # Add all environmental load cases for this combination
        for motion in gen.vessel_motions:
            env_lc = motion_load_cases.get(motion.name)
            if env_lc is not None:
                combo.add_load_case(env_lc)

        result.append(combo)

    return result


def _get_motion_component(motion: VesselMotion, motion_type: MotionType) -> float:
    """Get the amplitude of a specific motion component."""
    comp = motion.get_component_by_type(motion_type)
    return comp.amplitude if comp else 0.0
