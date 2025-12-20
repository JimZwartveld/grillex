"""
Cargo modelling abstraction for Grillex.

This module provides a high-level abstraction for modelling cargo on structures
(e.g., equipment, containers on barges). The Cargo class automatically generates
the underlying finite elements (point mass, springs, rigid links) when added to
a model.

The Cargo abstraction follows R-CARGO-001, R-CARGO-002, R-CARGO-003:
- Cargo is a Python-level abstraction (not a C++ element type)
- It composes a point-mass node plus connecting springs and optional rigid links
- The C++ core only sees the constituent elements

Usage:
    from grillex.core import StructuralModel
    from grillex.core.cargo import Cargo

    model = StructuralModel(name="Barge with Cargo")

    # Define cargo
    cargo = Cargo(name="Generator")
    cargo.set_cog([5.0, 2.0, 1.5])  # Center of gravity position
    cargo.set_mass(50.0)  # 50 mT
    cargo.set_inertia(Ixx=100, Iyy=150, Izz=80)  # Rotational inertia

    # Add connections to structure (spring stiffnesses)
    cargo.add_connection(
        structural_position=[5.0, 2.0, 0.0],  # Where cargo connects to deck
        stiffness=[1e6, 1e6, 1e6, 1e4, 1e4, 1e4],  # kx, ky, kz, krx, kry, krz
        cargo_offset=[0.0, 0.0, -1.5]  # Offset from CoG to connection point
    )

    # Add cargo to model (generates elements)
    model.add_cargo(cargo)
"""

from typing import List, Optional, Tuple, TYPE_CHECKING
from dataclasses import dataclass, field
import numpy as np

if TYPE_CHECKING:
    from grillex._grillex_cpp import (
        Model as _CppModel,
        Node,
        PointMass,
        SpringElement,
        RigidLink,
        LoadingCondition as _LoadingCondition
    )


# Valid loading condition values
VALID_LOADING_CONDITIONS = ("all", "static", "dynamic")


@dataclass
class CargoConnection:
    """
    Represents a connection between cargo and structural node.

    A cargo connection defines how the cargo is attached to the structure:
    - The structural position where the connection point is located
    - Spring stiffnesses in 6 DOFs (translations + rotations)
    - Optional offset from cargo CoG to the connection point on the cargo
    - Loading condition to control when the connection is active

    Attributes:
        structural_position: Position [x, y, z] where cargo connects to structure
        stiffness: Spring stiffnesses [kx, ky, kz, krx, kry, krz]
                   Units: translations [kN/m], rotations [kN·m/rad]
        cargo_offset: Offset from cargo CoG to connection point on cargo [dx, dy, dz]
                      If None, connection is directly at CoG
        loading_condition: When this connection is active:
            - "all": Active for all load cases (default, backward compatible)
            - "static": Only active for Permanent load cases (e.g., bearing pads)
            - "dynamic": Only active for Variable/Environmental load cases (e.g., seafastening)

    Note:
        If cargo_offset is specified, a rigid link is created from the cargo CoG
        node to an intermediate node at the offset position, which then connects
        via spring to the structural node.

    Example:
        # Static connection (bearing pad) - only takes gravity load
        CargoConnection([0, 0, 0], [0, 0, 1e9, 0, 0, 0], loading_condition="static")

        # Dynamic connection (seafastening) - only takes environmental loads
        CargoConnection([5, 0, 0], [1e9, 1e9, 0, 0, 0, 0], loading_condition="dynamic")
    """
    structural_position: List[float]
    stiffness: List[float]  # [kx, ky, kz, krx, kry, krz]
    cargo_offset: Optional[List[float]] = None
    loading_condition: str = "all"  # "all", "static", "dynamic"

    # Internal references (set during element generation)
    structural_node: Optional["Node"] = field(default=None, repr=False)
    connection_node: Optional["Node"] = field(default=None, repr=False)
    spring_element: Optional["SpringElement"] = field(default=None, repr=False)
    rigid_link: Optional["RigidLink"] = field(default=None, repr=False)


class Cargo:
    """
    High-level cargo abstraction for structural analysis.

    A Cargo object represents a piece of cargo (equipment, container, etc.)
    that sits on a structure. It is defined by:
    - Center of gravity (CoG) position in global coordinates
    - Total mass and rotational inertia (6-component inertia tensor)
    - Connections to structural nodes via springs

    When generate_elements() is called, the cargo creates:
    1. A node at the CoG position
    2. A point mass element at the CoG node
    3. For each connection:
       - If no offset: spring directly from CoG to structural node
       - If offset: rigid link from CoG to offset node, then spring to structure

    Attributes:
        name: Descriptive name for the cargo
        cog_position: Center of gravity [x, y, z] in global coordinates
        mass: Total mass [mT]
        inertia: Inertia tensor [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] in [mT·m²]
        connections: List of CargoConnection objects

    Example:
        cargo = Cargo("Transformer")
        cargo.set_cog([10.0, 5.0, 2.0])
        cargo.set_mass(100.0)
        cargo.set_inertia(Ixx=500, Iyy=600, Izz=400)
        cargo.add_connection([10.0, 5.0, 0.0], [1e6]*6)
    """

    def __init__(self, name: str):
        """
        Initialize a Cargo object.

        Args:
            name: Descriptive name for the cargo (e.g., "Generator Unit 1")
        """
        self.name = name
        self.cog_position: List[float] = [0.0, 0.0, 0.0]
        self.mass: float = 0.0
        self.inertia: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz
        self.connections: List[CargoConnection] = []

        # Internal references (set during element generation)
        self._cog_node: Optional["Node"] = None
        self._point_mass: Optional["PointMass"] = None
        self._generated: bool = False

    def set_cog(self, position: List[float]) -> "Cargo":
        """
        Set the center of gravity position.

        Args:
            position: CoG coordinates [x, y, z] in global system [m]

        Returns:
            Self for method chaining
        """
        if len(position) != 3:
            raise ValueError("CoG position must be a 3-element list [x, y, z]")
        self.cog_position = list(position)
        return self

    def set_mass(self, mass: float) -> "Cargo":
        """
        Set the total cargo mass.

        Args:
            mass: Total mass [mT] (metric tonnes)

        Returns:
            Self for method chaining
        """
        if mass < 0:
            raise ValueError("Mass must be non-negative")
        self.mass = mass
        return self

    def set_inertia(
        self,
        Ixx: float = 0.0,
        Iyy: float = 0.0,
        Izz: float = 0.0,
        Ixy: float = 0.0,
        Ixz: float = 0.0,
        Iyz: float = 0.0
    ) -> "Cargo":
        """
        Set the rotational inertia tensor.

        The inertia tensor is defined about the CoG in global coordinates.

        Args:
            Ixx: Moment of inertia about X axis [mT·m²]
            Iyy: Moment of inertia about Y axis [mT·m²]
            Izz: Moment of inertia about Z axis [mT·m²]
            Ixy: Product of inertia XY [mT·m²]
            Ixz: Product of inertia XZ [mT·m²]
            Iyz: Product of inertia YZ [mT·m²]

        Returns:
            Self for method chaining
        """
        self.inertia = [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        return self

    def add_connection(
        self,
        structural_position: List[float],
        stiffness: List[float],
        cargo_offset: Optional[List[float]] = None,
        loading_condition: str = "all"
    ) -> "Cargo":
        """
        Add a connection to the structure.

        Each connection represents a support point where the cargo is attached
        to the structural model via springs. If the connection point on the
        cargo is offset from the CoG, a rigid link is used to connect the
        CoG node to the offset point.

        Args:
            structural_position: Position [x, y, z] of structural connection [m]
            stiffness: Spring stiffnesses [kx, ky, kz, krx, kry, krz]
                       Translations in [kN/m], rotations in [kN·m/rad]
            cargo_offset: Optional offset from CoG to cargo connection point [m]
                          If None, connection is directly at CoG
            loading_condition: When this connection is active:
                - "all": Active for all load cases (default)
                - "static": Only active for Permanent load cases (bearing pads)
                - "dynamic": Only active for Variable/Environmental load cases (seafastening)

        Returns:
            Self for method chaining

        Example:
            # Direct connection at CoG
            cargo.add_connection([5.0, 2.0, 0.0], [1e6, 1e6, 1e6, 1e4, 1e4, 1e4])

            # Connection with offset (e.g., cargo footing below CoG)
            cargo.add_connection(
                [5.0, 2.0, 0.0],
                [1e6, 1e6, 1e6, 1e4, 1e4, 1e4],
                cargo_offset=[0.0, 0.0, -1.5]  # 1.5m below CoG
            )

            # Static connection (bearing pad) - only takes gravity load
            cargo.add_connection(
                [0.0, 0.0, 0.0],
                [0, 0, 1e9, 0, 0, 0],
                loading_condition="static"
            )

            # Dynamic connection (seafastening) - only takes environmental loads
            cargo.add_connection(
                [5.0, 0.0, 0.0],
                [1e9, 1e9, 0, 0, 0, 0],
                loading_condition="dynamic"
            )
        """
        if len(structural_position) != 3:
            raise ValueError("Structural position must be a 3-element list [x, y, z]")
        if len(stiffness) != 6:
            raise ValueError("Stiffness must be a 6-element list [kx, ky, kz, krx, kry, krz]")
        if cargo_offset is not None and len(cargo_offset) != 3:
            raise ValueError("Cargo offset must be a 3-element list [dx, dy, dz]")
        if loading_condition not in VALID_LOADING_CONDITIONS:
            raise ValueError(
                f"loading_condition must be one of {VALID_LOADING_CONDITIONS}, "
                f"got '{loading_condition}'"
            )

        connection = CargoConnection(
            structural_position=list(structural_position),
            stiffness=list(stiffness),
            cargo_offset=list(cargo_offset) if cargo_offset else None,
            loading_condition=loading_condition
        )
        self.connections.append(connection)
        return self

    def generate_elements(self, model: "_CppModel") -> None:
        """
        Generate the finite elements for this cargo.

        This method creates the underlying C++ elements:
        1. A node at the CoG position
        2. A point mass element with mass and inertia
        3. For each connection:
           - A spring element connecting to the structure
           - If offset specified: creates spring from offset position to structure
             (Note: rigid link coupling to CoG is planned for future implementation)

        Args:
            model: The C++ Model object to add elements to

        Raises:
            RuntimeError: If elements have already been generated
            ValueError: If no connections are defined
        """
        if self._generated:
            raise RuntimeError(f"Cargo '{self.name}' has already been generated")

        if not self.connections:
            raise ValueError(f"Cargo '{self.name}' has no connections defined")

        # 1. Create node at CoG
        self._cog_node = model.get_or_create_node(
            self.cog_position[0],
            self.cog_position[1],
            self.cog_position[2]
        )

        # 2. Create point mass at CoG
        self._point_mass = model.create_point_mass(self._cog_node)
        self._point_mass.mass = self.mass
        self._point_mass.set_full_inertia(
            self.inertia[0],  # Ixx
            self.inertia[1],  # Iyy
            self.inertia[2],  # Izz
            self.inertia[3],  # Ixy
            self.inertia[4],  # Ixz
            self.inertia[5]   # Iyz
        )

        # 3. Create connections
        for conn in self.connections:
            # Get or create structural node
            conn.structural_node = model.get_or_create_node(
                conn.structural_position[0],
                conn.structural_position[1],
                conn.structural_position[2]
            )

            if conn.cargo_offset is None:
                # Direct connection: spring from CoG to structural node
                conn.connection_node = self._cog_node
                spring = model.create_spring(self._cog_node, conn.structural_node)
            else:
                # Offset connection: use rigid link from CoG to footprint node
                # Calculate footprint node position (CoG + offset)
                offset_pos = [
                    self.cog_position[0] + conn.cargo_offset[0],
                    self.cog_position[1] + conn.cargo_offset[1],
                    self.cog_position[2] + conn.cargo_offset[2]
                ]

                # Force create a separate footprint node (without merging)
                # This ensures cargo footprint is a distinct node from structural node,
                # even when at the same position. Forces transfer via the spring.
                conn.connection_node = model.create_node(
                    offset_pos[0], offset_pos[1], offset_pos[2]
                )

                # Add rigid link from COG (master) to footprint node (slave)
                # The offset vector goes from master to slave in global coords
                offset_vec = np.array(conn.cargo_offset, dtype=float)
                model.add_rigid_link(conn.connection_node, self._cog_node, offset_vec)

                # Create spring from footprint node to structural node
                spring = model.create_spring(conn.connection_node, conn.structural_node)

            # Set spring stiffnesses (if spring was created)
            if spring is not None:
                spring.kx = conn.stiffness[0]
                spring.ky = conn.stiffness[1]
                spring.kz = conn.stiffness[2]
                spring.krx = conn.stiffness[3]
                spring.kry = conn.stiffness[4]
                spring.krz = conn.stiffness[5]

                # Set loading condition on the spring
                # Import here to avoid circular import issues
                from grillex._grillex_cpp import LoadingCondition
                loading_condition_map = {
                    "all": LoadingCondition.All,
                    "static": LoadingCondition.Static,
                    "dynamic": LoadingCondition.Dynamic
                }
                spring.loading_condition = loading_condition_map[conn.loading_condition]

            conn.spring_element = spring

        self._generated = True

    @property
    def cog_node(self) -> Optional["Node"]:
        """Get the CoG node (available after generate_elements)."""
        return self._cog_node

    @property
    def point_mass(self) -> Optional["PointMass"]:
        """Get the point mass element (available after generate_elements)."""
        return self._point_mass

    @property
    def is_generated(self) -> bool:
        """Check if elements have been generated."""
        return self._generated

    def get_weight(self, gravity: float = 9.81) -> float:
        """
        Calculate the weight of the cargo.

        Args:
            gravity: Gravitational acceleration [m/s²], default 9.81

        Returns:
            Weight [kN] (mass * gravity)
        """
        return self.mass * gravity

    def __repr__(self) -> str:
        return (f"Cargo(name='{self.name}', "
                f"cog={self.cog_position}, "
                f"mass={self.mass} mT, "
                f"n_connections={len(self.connections)}, "
                f"generated={self._generated})")
