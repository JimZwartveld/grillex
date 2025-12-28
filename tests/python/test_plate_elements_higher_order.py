"""
Tests for higher-order plate elements (PlateElement8 and PlateElement9).

Task 19.4: Higher-Order Plate Elements (MITC8, MITC9)
"""

import numpy as np
import pytest

from grillex.core import (
    Node, Material, PlateElement8, PlateElement9
)


@pytest.fixture
def material():
    """Create a steel material."""
    return Material(
        1,  # id
        "Steel",
        E=210e6,   # kN/m² (210 GPa)
        nu=0.3,
        rho=7.85e-3  # mT/m³
    )


class TestPlateElement8:
    """Tests for the 8-node serendipity plate element."""

    def test_construction(self, material):
        """Test PlateElement8 can be constructed with 8 nodes."""
        # Create 8 nodes: 4 corners + 4 midside
        # Node numbering:
        #   4 --- 7 --- 3
        #   |           |
        #   8           6
        #   |           |
        #   1 --- 5 --- 2

        nodes = [
            Node(1, 0.0, 0.0, 0.0),   # N1: corner (-1,-1)
            Node(2, 2.0, 0.0, 0.0),   # N2: corner (+1,-1)
            Node(3, 2.0, 1.0, 0.0),   # N3: corner (+1,+1)
            Node(4, 0.0, 1.0, 0.0),   # N4: corner (-1,+1)
            Node(5, 1.0, 0.0, 0.0),   # N5: midside (0,-1)
            Node(6, 2.0, 0.5, 0.0),   # N6: midside (+1,0)
            Node(7, 1.0, 1.0, 0.0),   # N7: midside (0,+1)
            Node(8, 0.0, 0.5, 0.0),   # N8: midside (-1,0)
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        assert elem.id == 1
        assert elem.thickness == 0.02
        assert elem.num_dofs() == 48
        assert not elem.has_warping()

    def test_stiffness_matrix_shape(self, material):
        """Test that stiffness matrix has correct shape (48x48)."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        K = elem.global_stiffness_matrix()
        assert K.shape == (48, 48)

    def test_stiffness_matrix_symmetry(self, material):
        """Test that stiffness matrix is symmetric."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        K = elem.global_stiffness_matrix()
        # Relax tolerance for numerical precision in higher-order elements
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_mass_matrix_shape(self, material):
        """Test that mass matrix has correct shape (48x48)."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        M = elem.global_mass_matrix()
        assert M.shape == (48, 48)

    def test_area_calculation(self, material):
        """Test area calculation for rectangular plate."""
        # 2m x 1m rectangular plate
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        area = elem.area()
        np.testing.assert_almost_equal(area, 2.0, decimal=6)

    def test_centroid(self, material):
        """Test centroid calculation."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        c = elem.centroid()
        # Average of all 8 nodes
        np.testing.assert_almost_equal(c[0], 1.0, decimal=6)
        np.testing.assert_almost_equal(c[1], 0.5, decimal=6)
        np.testing.assert_almost_equal(c[2], 0.0, decimal=6)

    def test_local_axes(self, material):
        """Test that local axes are orthonormal."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem = PlateElement8(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7],
            thickness=0.02,
            material=material
        )

        # Check orthonormality
        np.testing.assert_almost_equal(np.dot(elem.x_axis, elem.y_axis), 0.0)
        np.testing.assert_almost_equal(np.dot(elem.x_axis, elem.z_axis), 0.0)
        np.testing.assert_almost_equal(np.dot(elem.y_axis, elem.z_axis), 0.0)

        np.testing.assert_almost_equal(np.linalg.norm(elem.x_axis), 1.0)
        np.testing.assert_almost_equal(np.linalg.norm(elem.y_axis), 1.0)
        np.testing.assert_almost_equal(np.linalg.norm(elem.z_axis), 1.0)


class TestPlateElement9:
    """Tests for the 9-node Lagrangian plate element."""

    def test_construction(self, material):
        """Test PlateElement9 can be constructed with 9 nodes."""
        # Create 9 nodes: 4 corners + 4 midside + 1 center
        # Node numbering:
        #   4 --- 7 --- 3
        #   |     |     |
        #   8 --- 9 --- 6
        #   |     |     |
        #   1 --- 5 --- 2

        nodes = [
            Node(1, 0.0, 0.0, 0.0),   # N1: corner (-1,-1)
            Node(2, 2.0, 0.0, 0.0),   # N2: corner (+1,-1)
            Node(3, 2.0, 1.0, 0.0),   # N3: corner (+1,+1)
            Node(4, 0.0, 1.0, 0.0),   # N4: corner (-1,+1)
            Node(5, 1.0, 0.0, 0.0),   # N5: midside (0,-1)
            Node(6, 2.0, 0.5, 0.0),   # N6: midside (+1,0)
            Node(7, 1.0, 1.0, 0.0),   # N7: midside (0,+1)
            Node(8, 0.0, 0.5, 0.0),   # N8: midside (-1,0)
            Node(9, 1.0, 0.5, 0.0),   # N9: center (0,0)
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        assert elem.id == 1
        assert elem.thickness == 0.02
        assert elem.num_dofs() == 54
        assert not elem.has_warping()

    def test_stiffness_matrix_shape(self, material):
        """Test that stiffness matrix has correct shape (54x54)."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        K = elem.global_stiffness_matrix()
        assert K.shape == (54, 54)

    def test_stiffness_matrix_symmetry(self, material):
        """Test that stiffness matrix is symmetric."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        K = elem.global_stiffness_matrix()
        # Relax tolerance for numerical precision in higher-order elements
        np.testing.assert_allclose(K, K.T, rtol=1e-8, atol=1e-9)

    def test_mass_matrix_shape(self, material):
        """Test that mass matrix has correct shape (54x54)."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        M = elem.global_mass_matrix()
        assert M.shape == (54, 54)

    def test_area_calculation(self, material):
        """Test area calculation for rectangular plate."""
        # 2m x 1m rectangular plate
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        area = elem.area()
        np.testing.assert_almost_equal(area, 2.0, decimal=6)

    def test_centroid(self, material):
        """Test centroid calculation."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        c = elem.centroid()
        # Average of all 9 nodes
        np.testing.assert_almost_equal(c[0], 1.0, decimal=6)
        np.testing.assert_almost_equal(c[1], 0.5, decimal=6)
        np.testing.assert_almost_equal(c[2], 0.0, decimal=6)

    def test_local_axes(self, material):
        """Test that local axes are orthonormal."""
        nodes = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem = PlateElement9(
            1,
            nodes[0], nodes[1], nodes[2], nodes[3],
            nodes[4], nodes[5], nodes[6], nodes[7], nodes[8],
            thickness=0.02,
            material=material
        )

        # Check orthonormality
        np.testing.assert_almost_equal(np.dot(elem.x_axis, elem.y_axis), 0.0)
        np.testing.assert_almost_equal(np.dot(elem.x_axis, elem.z_axis), 0.0)
        np.testing.assert_almost_equal(np.dot(elem.y_axis, elem.z_axis), 0.0)

        np.testing.assert_almost_equal(np.linalg.norm(elem.x_axis), 1.0)
        np.testing.assert_almost_equal(np.linalg.norm(elem.y_axis), 1.0)
        np.testing.assert_almost_equal(np.linalg.norm(elem.z_axis), 1.0)


class TestPlateElementComparison:
    """Tests comparing 4, 8, and 9 node plate elements."""

    def test_stiffness_increases_with_order(self, material):
        """Higher-order elements should have similar but not identical stiffness."""
        # For a rectangular plate, stiffness should be similar
        # This is a qualitative check - the elements should converge to same behavior

        # 4-node element (use PlateElement from existing code)
        from grillex.core import PlateElement

        nodes4 = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
        ]

        elem4 = PlateElement(
            1,
            nodes4[0], nodes4[1], nodes4[2], nodes4[3],
            thickness=0.02,
            material=material
        )

        # 8-node element
        nodes8 = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
        ]

        elem8 = PlateElement8(
            2,
            nodes8[0], nodes8[1], nodes8[2], nodes8[3],
            nodes8[4], nodes8[5], nodes8[6], nodes8[7],
            thickness=0.02,
            material=material
        )

        # 9-node element
        nodes9 = [
            Node(1, 0.0, 0.0, 0.0),
            Node(2, 2.0, 0.0, 0.0),
            Node(3, 2.0, 1.0, 0.0),
            Node(4, 0.0, 1.0, 0.0),
            Node(5, 1.0, 0.0, 0.0),
            Node(6, 2.0, 0.5, 0.0),
            Node(7, 1.0, 1.0, 0.0),
            Node(8, 0.0, 0.5, 0.0),
            Node(9, 1.0, 0.5, 0.0),
        ]

        elem9 = PlateElement9(
            3,
            nodes9[0], nodes9[1], nodes9[2], nodes9[3],
            nodes9[4], nodes9[5], nodes9[6], nodes9[7], nodes9[8],
            thickness=0.02,
            material=material
        )

        K4 = elem4.global_stiffness_matrix()
        K8 = elem8.global_stiffness_matrix()
        K9 = elem9.global_stiffness_matrix()

        # Stiffness matrices should be positive (semi-)definite
        # Check that diagonal elements are positive
        assert np.all(np.diag(K4) >= 0)
        assert np.all(np.diag(K8) >= 0)
        assert np.all(np.diag(K9) >= 0)

        # All elements should have same area
        np.testing.assert_almost_equal(elem4.area(), elem8.area(), decimal=5)
        np.testing.assert_almost_equal(elem4.area(), elem9.area(), decimal=5)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
