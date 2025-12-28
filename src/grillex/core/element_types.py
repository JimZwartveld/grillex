"""Element type definitions and factory functions for Grillex.

This module provides infrastructure for selecting between different plate
element formulations, with extensibility for future element types.
"""

from enum import Enum
from typing import Dict, Any, List, Union, TYPE_CHECKING

if TYPE_CHECKING:
    from .data_types import (
        Node, Material, PlateElement, PlateElement8, PlateElement9, PlateElementTri
    )


class PlateElementType(Enum):
    """Available plate element formulations.

    Each type has different characteristics suited for different applications:

    - MITC4: 4-node bilinear Mindlin plate (default). Good general-purpose
      element with MITC treatment to avoid shear locking.

    - MITC8: 8-node serendipity Mindlin plate. Higher accuracy than MITC4,
      especially for curved boundaries.

    - MITC9: 9-node Lagrangian Mindlin plate. Similar to MITC8 but with
      center node for improved stress recovery.

    - DKT: 3-node Discrete Kirchhoff Triangle. Thin plate formulation
      (no transverse shear). Best for triangular meshes and complex geometries.
    """
    MITC4 = "MITC4"   # 4-node Mindlin plate (default)
    MITC8 = "MITC8"   # 8-node serendipity Mindlin plate
    MITC9 = "MITC9"   # 9-node Lagrangian Mindlin plate
    DKT = "DKT"       # 3-node Discrete Kirchhoff Triangle

    # Future element types (not yet implemented)
    # KIRCHHOFF4 = "KIRCHHOFF4"  # 4-node thin plate (no shear)
    # MEMBRANE4 = "MEMBRANE4"     # 4-node membrane (in-plane only)
    # SHELL4 = "SHELL4"           # 4-node shell (bending + membrane)


# Metadata for each element type
ELEMENT_TYPE_INFO: Dict[PlateElementType, Dict[str, Any]] = {
    PlateElementType.MITC4: {
        "n_nodes": 4,
        "dofs_per_node": 6,
        "total_dofs": 24,
        "description": "4-node Mindlin plate with MITC shear locking treatment",
        "supports_shear": True,
        "polynomial_order": 1,
        "quadrature_points": 4,  # 2x2 Gauss
        "geometry": "quad",
    },
    PlateElementType.MITC8: {
        "n_nodes": 8,
        "dofs_per_node": 6,
        "total_dofs": 48,
        "description": "8-node serendipity Mindlin plate (quadratic)",
        "supports_shear": True,
        "polynomial_order": 2,
        "quadrature_points": 9,  # 3x3 Gauss
        "geometry": "quad",
    },
    PlateElementType.MITC9: {
        "n_nodes": 9,
        "dofs_per_node": 6,
        "total_dofs": 54,
        "description": "9-node Lagrangian Mindlin plate (quadratic)",
        "supports_shear": True,
        "polynomial_order": 2,
        "quadrature_points": 9,  # 3x3 Gauss
        "geometry": "quad",
    },
    PlateElementType.DKT: {
        "n_nodes": 3,
        "dofs_per_node": 6,
        "total_dofs": 18,
        "description": "3-node Discrete Kirchhoff Triangle (thin plate)",
        "supports_shear": False,
        "polynomial_order": 2,
        "quadrature_points": 3,  # 3-point triangle rule
        "geometry": "triangle",
    },
}


def get_element_type(name: str) -> PlateElementType:
    """Get element type by name (case-insensitive).

    Args:
        name: Element type name (e.g., "MITC4", "mitc4", "DKT")

    Returns:
        The corresponding PlateElementType enum value.

    Raises:
        ValueError: If the element type is not recognized.

    Example:
        >>> elem_type = get_element_type("mitc4")
        >>> elem_type == PlateElementType.MITC4
        True
    """
    try:
        return PlateElementType(name.upper())
    except ValueError:
        valid = [e.value for e in PlateElementType]
        raise ValueError(f"Unknown element type '{name}'. Valid types: {valid}")


def get_element_info(element_type: Union[str, PlateElementType]) -> Dict[str, Any]:
    """Get metadata about an element type.

    Args:
        element_type: Element type as enum or string name.

    Returns:
        Dictionary with element metadata including n_nodes, description,
        supports_shear, polynomial_order, and geometry.

    Raises:
        ValueError: If the element type is not recognized.

    Example:
        >>> info = get_element_info("MITC4")
        >>> info["n_nodes"]
        4
    """
    if isinstance(element_type, str):
        element_type = get_element_type(element_type)
    return ELEMENT_TYPE_INFO[element_type]


def get_available_element_types() -> List[str]:
    """Get list of available plate element type names.

    Returns:
        List of element type names (e.g., ["MITC4", "MITC8", "MITC9", "DKT"]).
    """
    return [e.value for e in PlateElementType]


def create_plate_element(
    element_type: Union[str, PlateElementType],
    element_id: int,
    nodes: List["Node"],
    thickness: float,
    material: "Material"
) -> Union["PlateElement", "PlateElement8", "PlateElement9", "PlateElementTri"]:
    """Create a plate element of the specified type.

    This factory function creates the appropriate plate element class based
    on the element type specification.

    Args:
        element_type: Type of plate element ("MITC4", "MITC8", "MITC9", "DKT")
            or PlateElementType enum value.
        element_id: Unique element identifier.
        nodes: List of node pointers. Number must match element type:
            - MITC4: 4 nodes
            - MITC8: 8 nodes
            - MITC9: 9 nodes
            - DKT: 3 nodes
        thickness: Plate thickness [m].
        material: Material properties.

    Returns:
        The created plate element object.

    Raises:
        ValueError: If element type is unknown or node count is incorrect.

    Example:
        >>> elem = create_plate_element("MITC4", 1, [n1, n2, n3, n4], 0.01, steel)
        >>> elem.num_dofs()
        24
    """
    from .data_types import PlateElement, PlateElement8, PlateElement9, PlateElementTri

    if isinstance(element_type, str):
        element_type = get_element_type(element_type)

    info = ELEMENT_TYPE_INFO[element_type]
    expected_nodes = info["n_nodes"]

    if len(nodes) != expected_nodes:
        raise ValueError(
            f"Element type {element_type.value} requires {expected_nodes} nodes, "
            f"got {len(nodes)}"
        )

    if element_type == PlateElementType.MITC4:
        return PlateElement(
            element_id,
            nodes[0], nodes[1], nodes[2], nodes[3],
            thickness, material
        )
    elif element_type == PlateElementType.MITC8:
        return PlateElement8(
            element_id,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness, material
        )
    elif element_type == PlateElementType.MITC9:
        return PlateElement9(
            element_id,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness, material
        )
    elif element_type == PlateElementType.DKT:
        return PlateElementTri(
            element_id,
            nodes[0], nodes[1], nodes[2],
            thickness, material
        )
    else:
        raise ValueError(f"Element type {element_type} not yet implemented")


def is_quad_element(element_type: Union[str, PlateElementType]) -> bool:
    """Check if element type uses quadrilateral geometry.

    Args:
        element_type: Element type as enum or string.

    Returns:
        True if element uses quad geometry (MITC4, MITC8, MITC9).
    """
    info = get_element_info(element_type)
    return info["geometry"] == "quad"


def is_triangle_element(element_type: Union[str, PlateElementType]) -> bool:
    """Check if element type uses triangular geometry.

    Args:
        element_type: Element type as enum or string.

    Returns:
        True if element uses triangle geometry (DKT).
    """
    info = get_element_info(element_type)
    return info["geometry"] == "triangle"


def supports_shear_deformation(element_type: Union[str, PlateElementType]) -> bool:
    """Check if element type supports transverse shear deformation.

    Args:
        element_type: Element type as enum or string.

    Returns:
        True if element includes shear deformation (Mindlin theory).
        False for thin plate elements (Kirchhoff theory).
    """
    info = get_element_info(element_type)
    return info["supports_shear"]
