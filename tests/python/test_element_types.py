"""Tests for element type infrastructure.

Tests for PlateElementType enum, element factory, and related utilities.
"""

import pytest
import numpy as np
from grillex.core import (
    Node, Material,
    PlateElement, PlateElement8, PlateElement9, PlateElementTri,
    PlateElementType, ELEMENT_TYPE_INFO,
    get_element_type, get_element_info, get_available_element_types,
    create_plate_element, is_quad_element, is_triangle_element,
    supports_shear_deformation,
)


@pytest.fixture
def steel():
    """Create standard steel material for testing."""
    return Material(1, "Steel", E=210e6, nu=0.3, rho=7.85e-3)


@pytest.fixture
def nodes_4():
    """Create 4 nodes for a unit square plate."""
    return [
        Node(1, 0.0, 0.0, 0.0),
        Node(2, 1.0, 0.0, 0.0),
        Node(3, 1.0, 1.0, 0.0),
        Node(4, 0.0, 1.0, 0.0),
    ]


@pytest.fixture
def nodes_8():
    """Create 8 nodes for serendipity element."""
    return [
        Node(1, 0.0, 0.0, 0.0),   # Corner 1
        Node(2, 1.0, 0.0, 0.0),   # Corner 2
        Node(3, 1.0, 1.0, 0.0),   # Corner 3
        Node(4, 0.0, 1.0, 0.0),   # Corner 4
        Node(5, 0.5, 0.0, 0.0),   # Mid-edge 5
        Node(6, 1.0, 0.5, 0.0),   # Mid-edge 6
        Node(7, 0.5, 1.0, 0.0),   # Mid-edge 7
        Node(8, 0.0, 0.5, 0.0),   # Mid-edge 8
    ]


@pytest.fixture
def nodes_9():
    """Create 9 nodes for Lagrangian element."""
    return [
        Node(1, 0.0, 0.0, 0.0),
        Node(2, 1.0, 0.0, 0.0),
        Node(3, 1.0, 1.0, 0.0),
        Node(4, 0.0, 1.0, 0.0),
        Node(5, 0.5, 0.0, 0.0),
        Node(6, 1.0, 0.5, 0.0),
        Node(7, 0.5, 1.0, 0.0),
        Node(8, 0.0, 0.5, 0.0),
        Node(9, 0.5, 0.5, 0.0),   # Center
    ]


@pytest.fixture
def nodes_3():
    """Create 3 nodes for triangular element."""
    return [
        Node(1, 0.0, 0.0, 0.0),
        Node(2, 1.0, 0.0, 0.0),
        Node(3, 0.5, np.sqrt(3)/2, 0.0),
    ]


class TestPlateElementTypeEnum:
    """Tests for PlateElementType enum."""

    def test_enum_values(self):
        """Test that all expected element types exist."""
        assert PlateElementType.MITC4.value == "MITC4"
        assert PlateElementType.MITC8.value == "MITC8"
        assert PlateElementType.MITC9.value == "MITC9"
        assert PlateElementType.DKT.value == "DKT"

    def test_enum_count(self):
        """Test number of available element types."""
        assert len(PlateElementType) == 4


class TestGetElementType:
    """Tests for get_element_type function."""

    def test_get_element_type_uppercase(self):
        """Test getting element type with uppercase name."""
        assert get_element_type("MITC4") == PlateElementType.MITC4
        assert get_element_type("MITC8") == PlateElementType.MITC8
        assert get_element_type("MITC9") == PlateElementType.MITC9
        assert get_element_type("DKT") == PlateElementType.DKT

    def test_get_element_type_lowercase(self):
        """Test getting element type with lowercase name."""
        assert get_element_type("mitc4") == PlateElementType.MITC4
        assert get_element_type("mitc8") == PlateElementType.MITC8
        assert get_element_type("dkt") == PlateElementType.DKT

    def test_get_element_type_mixed_case(self):
        """Test getting element type with mixed case name."""
        assert get_element_type("Mitc4") == PlateElementType.MITC4
        assert get_element_type("Dkt") == PlateElementType.DKT

    def test_get_element_type_invalid(self):
        """Test that invalid element type raises ValueError."""
        with pytest.raises(ValueError) as exc_info:
            get_element_type("INVALID")
        assert "Unknown element type" in str(exc_info.value)
        assert "MITC4" in str(exc_info.value)  # Should list valid types


class TestElementTypeInfo:
    """Tests for ELEMENT_TYPE_INFO and get_element_info."""

    def test_all_types_have_info(self):
        """Test that all element types have info entries."""
        for elem_type in PlateElementType:
            assert elem_type in ELEMENT_TYPE_INFO

    def test_info_has_required_fields(self):
        """Test that all info entries have required fields."""
        required_fields = [
            "n_nodes", "dofs_per_node", "total_dofs",
            "description", "supports_shear", "polynomial_order", "geometry"
        ]
        for elem_type in PlateElementType:
            info = ELEMENT_TYPE_INFO[elem_type]
            for field in required_fields:
                assert field in info, f"Missing {field} for {elem_type}"

    def test_mitc4_info(self):
        """Test MITC4 info values."""
        info = get_element_info(PlateElementType.MITC4)
        assert info["n_nodes"] == 4
        assert info["total_dofs"] == 24
        assert info["supports_shear"] == True
        assert info["geometry"] == "quad"

    def test_mitc8_info(self):
        """Test MITC8 info values."""
        info = get_element_info("MITC8")
        assert info["n_nodes"] == 8
        assert info["total_dofs"] == 48
        assert info["polynomial_order"] == 2

    def test_mitc9_info(self):
        """Test MITC9 info values."""
        info = get_element_info("mitc9")
        assert info["n_nodes"] == 9
        assert info["total_dofs"] == 54

    def test_dkt_info(self):
        """Test DKT info values."""
        info = get_element_info("DKT")
        assert info["n_nodes"] == 3
        assert info["total_dofs"] == 18
        assert info["supports_shear"] == False
        assert info["geometry"] == "triangle"


class TestGetAvailableElementTypes:
    """Tests for get_available_element_types function."""

    def test_returns_list(self):
        """Test that function returns a list."""
        types = get_available_element_types()
        assert isinstance(types, list)

    def test_contains_all_types(self):
        """Test that all types are returned."""
        types = get_available_element_types()
        assert "MITC4" in types
        assert "MITC8" in types
        assert "MITC9" in types
        assert "DKT" in types
        assert len(types) == 4


class TestCreatePlateElement:
    """Tests for create_plate_element factory function."""

    def test_create_mitc4(self, nodes_4, steel):
        """Test creating MITC4 element."""
        elem = create_plate_element("MITC4", 1, nodes_4, 0.01, steel)
        assert isinstance(elem, PlateElement)
        assert elem.id == 1
        assert elem.thickness == 0.01
        assert elem.num_dofs() == 24

    def test_create_mitc8(self, nodes_8, steel):
        """Test creating MITC8 element."""
        elem = create_plate_element("MITC8", 2, nodes_8, 0.02, steel)
        assert isinstance(elem, PlateElement8)
        assert elem.id == 2
        assert elem.thickness == 0.02
        assert elem.num_dofs() == 48

    def test_create_mitc9(self, nodes_9, steel):
        """Test creating MITC9 element."""
        elem = create_plate_element(PlateElementType.MITC9, 3, nodes_9, 0.015, steel)
        assert isinstance(elem, PlateElement9)
        assert elem.id == 3
        assert elem.num_dofs() == 54

    def test_create_dkt(self, nodes_3, steel):
        """Test creating DKT element."""
        elem = create_plate_element("dkt", 4, nodes_3, 0.01, steel)
        assert isinstance(elem, PlateElementTri)
        assert elem.id == 4
        assert elem.num_dofs() == 18

    def test_wrong_node_count_mitc4(self, nodes_3, steel):
        """Test error when wrong number of nodes for MITC4."""
        with pytest.raises(ValueError) as exc_info:
            create_plate_element("MITC4", 1, nodes_3, 0.01, steel)
        assert "requires 4 nodes" in str(exc_info.value)
        assert "got 3" in str(exc_info.value)

    def test_wrong_node_count_dkt(self, nodes_4, steel):
        """Test error when wrong number of nodes for DKT."""
        with pytest.raises(ValueError) as exc_info:
            create_plate_element("DKT", 1, nodes_4, 0.01, steel)
        assert "requires 3 nodes" in str(exc_info.value)

    def test_invalid_element_type(self, nodes_4, steel):
        """Test error for invalid element type."""
        with pytest.raises(ValueError):
            create_plate_element("INVALID", 1, nodes_4, 0.01, steel)


class TestGeometryHelpers:
    """Tests for geometry helper functions."""

    def test_is_quad_element(self):
        """Test is_quad_element function."""
        assert is_quad_element("MITC4") == True
        assert is_quad_element("MITC8") == True
        assert is_quad_element("MITC9") == True
        assert is_quad_element("DKT") == False

    def test_is_triangle_element(self):
        """Test is_triangle_element function."""
        assert is_triangle_element("MITC4") == False
        assert is_triangle_element("MITC8") == False
        assert is_triangle_element("DKT") == True

    def test_supports_shear_deformation(self):
        """Test supports_shear_deformation function."""
        assert supports_shear_deformation("MITC4") == True
        assert supports_shear_deformation("MITC8") == True
        assert supports_shear_deformation("MITC9") == True
        assert supports_shear_deformation("DKT") == False


class TestCreatedElementsWork:
    """Tests that created elements produce valid matrices."""

    def test_mitc4_stiffness_valid(self, nodes_4, steel):
        """Test that MITC4 element produces valid stiffness matrix."""
        elem = create_plate_element("MITC4", 1, nodes_4, 0.01, steel)
        K = elem.global_stiffness_matrix()
        assert K.shape == (24, 24)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_mitc8_stiffness_valid(self, nodes_8, steel):
        """Test that MITC8 element produces valid stiffness matrix."""
        elem = create_plate_element("MITC8", 1, nodes_8, 0.01, steel)
        K = elem.global_stiffness_matrix()
        assert K.shape == (48, 48)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_mitc9_stiffness_valid(self, nodes_9, steel):
        """Test that MITC9 element produces valid stiffness matrix."""
        elem = create_plate_element("MITC9", 1, nodes_9, 0.01, steel)
        K = elem.global_stiffness_matrix()
        assert K.shape == (54, 54)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_dkt_stiffness_valid(self, nodes_3, steel):
        """Test that DKT element produces valid stiffness matrix."""
        elem = create_plate_element("DKT", 1, nodes_3, 0.01, steel)
        K = elem.global_stiffness_matrix()
        assert K.shape == (18, 18)
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)
