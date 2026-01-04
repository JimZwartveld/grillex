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

Example:
    >>> from grillex.core import VesselMotion
    >>> motion = VesselMotion("Design Heave + Roll")
    >>> motion.set_motion_center([50.0, 0.0, 5.0])
    >>> motion.add_heave(2.5)
    >>> motion.add_roll(0.12)
    >>> accel, ref_pt = motion.get_acceleration_field()
"""

from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING
from enum import Enum
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
class VesselMotion:
    """
    Vessel motion definition for offshore structural analysis.

    Encapsulates the motion parameters for a floating vessel and provides
    methods to generate acceleration fields for load cases. The motion center
    is the reference point for rotational motions (typically vessel CoG or
    waterline amidships).

    Structures located away from the motion center experience additional
    tangential accelerations due to rotational motions.

    Attributes:
        name: Descriptive name for this motion condition
        motion_center: Reference point for rotational motions [x, y, z] in meters
        components: List of motion components (heave, pitch, roll, etc.)
        description: Optional description of the motion condition

    Example:
        >>> motion = VesselMotion("Design Heave + Pitch")
        >>> motion.set_motion_center([50.0, 0.0, 5.0])  # Midship, waterline
        >>> motion.add_heave(2.5)  # 2.5 m/s² vertical acceleration
        >>> motion.add_pitch(0.08)  # 0.08 rad/s² pitch acceleration
        >>> motion.add_roll(0.12)  # 0.12 rad/s² roll acceleration
    """
    name: str
    motion_center: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    components: List[MotionComponent] = field(default_factory=list)
    description: str = ""

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
        return self

    def get_acceleration_field(self) -> tuple:
        """
        Compute the 6-component acceleration field from motion components.

        The acceleration field is returned as [ax, ay, az, αx, αy, αz] where:
        - ax, ay, az: Linear accelerations in m/s²
        - αx, αy, αz: Angular accelerations in rad/s²

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
