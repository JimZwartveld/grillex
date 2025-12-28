"""Tests for Plate integration in StructuralModel.

Tests for Task 19.7: Adding plates to StructuralModel.
"""

import pytest
import numpy as np
from grillex.core import StructuralModel, Plate, EdgeMeshControl


@pytest.fixture
def model_with_material():
    """Create a model with steel material."""
    model = StructuralModel(name="PlateTest")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    return model


class TestAddPlate:
    """Tests for add_plate method."""

    def test_add_simple_quad_plate(self, model_with_material):
        """Test adding a simple 4-corner plate."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            mesh_size=0.5
        )

        assert plate is not None
        assert plate.n_corners == 4
        assert plate.thickness == 0.02
        assert plate.material == "Steel"
        assert plate.mesh_size == 0.5
        assert plate.element_type == "MITC4"
        assert plate.name == "Plate_1"

    def test_add_triangular_plate(self, model_with_material):
        """Test adding a 3-corner triangular plate."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [3, 0, 0], [1.5, 2, 0]],
            thickness=0.015,
            material="Steel",
            element_type="DKT"
        )

        assert plate.n_corners == 3
        assert plate.element_type == "DKT"

    def test_add_plate_with_name(self, model_with_material):
        """Test adding a plate with custom name."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]],
            thickness=0.01,
            material="Steel",
            name="FloorPlate"
        )

        assert plate.name == "FloorPlate"

    def test_add_plate_with_different_element_types(self, model_with_material):
        """Test adding plates with different element types."""
        model = model_with_material

        plate_mitc4 = model.add_plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01, material="Steel", element_type="MITC4"
        )
        assert plate_mitc4.element_type == "MITC4"

        plate_mitc8 = model.add_plate(
            corners=[[2, 0, 0], [3, 0, 0], [3, 1, 0], [2, 1, 0]],
            thickness=0.01, material="Steel", element_type="MITC8"
        )
        assert plate_mitc8.element_type == "MITC8"

        plate_mitc9 = model.add_plate(
            corners=[[4, 0, 0], [5, 0, 0], [5, 1, 0], [4, 1, 0]],
            thickness=0.01, material="Steel", element_type="mitc9"  # lowercase
        )
        assert plate_mitc9.element_type == "mitc9"

    def test_add_plate_missing_material_raises(self):
        """Test that missing material raises ValueError."""
        model = StructuralModel(name="PlateTest")
        # Don't add material

        with pytest.raises(ValueError) as exc_info:
            model.add_plate(
                corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
                thickness=0.01,
                material="Steel"
            )

        assert "Material 'Steel' not found" in str(exc_info.value)

    def test_add_plate_invalid_element_type_raises(self, model_with_material):
        """Test that invalid element type raises ValueError."""
        model = model_with_material

        with pytest.raises(ValueError) as exc_info:
            model.add_plate(
                corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
                thickness=0.01,
                material="Steel",
                element_type="INVALID"
            )

        assert "Unknown element type" in str(exc_info.value)

    def test_add_plate_nonplanar_raises(self, model_with_material):
        """Test that non-planar corners raise ValueError."""
        model = model_with_material

        with pytest.raises(ValueError) as exc_info:
            model.add_plate(
                corners=[
                    [0, 0, 0],
                    [1, 0, 0],
                    [1, 1, 0.5],  # This point is not in plane
                    [0, 1, 0]
                ],
                thickness=0.01,
                material="Steel"
            )

        assert "coplanar" in str(exc_info.value).lower()

    def test_add_plate_too_few_corners_raises(self, model_with_material):
        """Test that fewer than 3 corners raises ValueError."""
        model = model_with_material

        with pytest.raises(ValueError):
            model.add_plate(
                corners=[[0, 0, 0], [1, 0, 0]],  # Only 2 corners
                thickness=0.01,
                material="Steel"
            )


class TestSetEdgeDivisions:
    """Tests for set_edge_divisions method."""

    def test_set_edge_divisions(self, model_with_material):
        """Test setting edge divisions."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        model.set_edge_divisions(plate, 0, 8)
        model.set_edge_divisions(plate, 2, 8)

        assert 0 in plate.edge_controls
        assert plate.edge_controls[0].n_elements == 8
        assert plate.edge_controls[2].n_elements == 8

    def test_set_edge_divisions_all_edges(self, model_with_material):
        """Test setting divisions on all edges."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Set all 4 edges
        for i in range(4):
            model.set_edge_divisions(plate, i, 4 + i)

        for i in range(4):
            assert plate.edge_controls[i].n_elements == 4 + i

    def test_set_edge_divisions_invalid_index_raises(self, model_with_material):
        """Test that invalid edge index raises ValueError."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        with pytest.raises(ValueError) as exc_info:
            model.set_edge_divisions(plate, 4, 8)  # Index 4 doesn't exist

        assert "out of range" in str(exc_info.value)

    def test_set_edge_divisions_negative_index_raises(self, model_with_material):
        """Test that negative edge index raises ValueError."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        with pytest.raises(ValueError):
            model.set_edge_divisions(plate, -1, 8)

    def test_set_edge_divisions_zero_elements_raises(self, model_with_material):
        """Test that zero elements raises ValueError."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        with pytest.raises(ValueError) as exc_info:
            model.set_edge_divisions(plate, 0, 0)

        assert "at least 1" in str(exc_info.value)


class TestGetPlates:
    """Tests for get_plates method."""

    def test_get_plates_empty(self):
        """Test get_plates on empty model."""
        model = StructuralModel(name="EmptyTest")
        plates = model.get_plates()
        assert plates == []

    def test_get_plates_single(self, model_with_material):
        """Test get_plates with single plate."""
        model = model_with_material
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        plates = model.get_plates()
        assert len(plates) == 1
        assert plates[0] is plate

    def test_get_plates_multiple(self, model_with_material):
        """Test get_plates with multiple plates."""
        model = model_with_material

        plate1 = model.add_plate(
            corners=[[0, 0, 0], [2, 0, 0], [2, 1, 0], [0, 1, 0]],
            thickness=0.02,
            material="Steel"
        )

        plate2 = model.add_plate(
            corners=[[2, 0, 0], [4, 0, 0], [4, 1, 0], [2, 1, 0]],
            thickness=0.02,
            material="Steel"
        )

        plate3 = model.add_plate(
            corners=[[0, 1, 0], [2, 1, 0], [1, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="DKT"
        )

        plates = model.get_plates()
        assert len(plates) == 3
        assert plate1 in plates
        assert plate2 in plates
        assert plate3 in plates

    def test_get_plates_returns_copy(self, model_with_material):
        """Test that get_plates returns a copy of the list."""
        model = model_with_material
        model.add_plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01,
            material="Steel"
        )

        plates1 = model.get_plates()
        plates2 = model.get_plates()

        # Should be equal but not the same object
        assert plates1 == plates2
        assert plates1 is not plates2


class TestPlateNaming:
    """Tests for automatic plate naming."""

    def test_auto_naming_sequential(self, model_with_material):
        """Test that plates are named sequentially."""
        model = model_with_material

        plate1 = model.add_plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01, material="Steel"
        )
        plate2 = model.add_plate(
            corners=[[2, 0, 0], [3, 0, 0], [3, 1, 0], [2, 1, 0]],
            thickness=0.01, material="Steel"
        )
        plate3 = model.add_plate(
            corners=[[4, 0, 0], [5, 0, 0], [5, 1, 0], [4, 1, 0]],
            thickness=0.01, material="Steel"
        )

        assert plate1.name == "Plate_1"
        assert plate2.name == "Plate_2"
        assert plate3.name == "Plate_3"

    def test_custom_name_does_not_affect_counter(self, model_with_material):
        """Test that custom names don't affect auto-naming counter."""
        model = model_with_material

        plate1 = model.add_plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.01, material="Steel",
            name="CustomPlate"
        )

        plate2 = model.add_plate(
            corners=[[2, 0, 0], [3, 0, 0], [3, 1, 0], [2, 1, 0]],
            thickness=0.01, material="Steel"
        )

        assert plate1.name == "CustomPlate"
        assert plate2.name == "Plate_2"


class TestPlateIntegration:
    """Integration tests for plates with other model components."""

    def test_plate_and_beams_in_same_model(self, model_with_material):
        """Test that plates and beams can coexist."""
        model = model_with_material
        model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.67e-8)

        # Add a plate
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Add a beam along one edge
        beam = model.add_beam_by_coords(
            [0, 0, 0], [4, 0, 0],
            section_name="IPE200",
            material_name="Steel"
        )

        assert len(model.get_plates()) == 1
        assert len(model.beams) == 1

    def test_multiple_materials_for_plates(self):
        """Test plates with different materials."""
        model = StructuralModel(name="MultiMaterialTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
        model.add_material("Aluminum", E=70e6, nu=0.33, rho=2.7e-3)

        steel_plate = model.add_plate(
            corners=[[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
            thickness=0.02,
            material="Steel"
        )

        alu_plate = model.add_plate(
            corners=[[2, 0, 0], [3, 0, 0], [3, 1, 0], [2, 1, 0]],
            thickness=0.01,
            material="Aluminum"
        )

        assert steel_plate.material == "Steel"
        assert alu_plate.material == "Aluminum"
