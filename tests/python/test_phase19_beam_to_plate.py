"""
Tests for Phase 19.14: Beam to Plate Conversion.

This module tests:
- convert_beam_to_plate() for single beam conversion
- convert_beams_to_plates() for batch conversion
- Different orientations (horizontal, vertical, top, bottom)
- Plate geometry verification
- Error handling
"""

import pytest
import numpy as np
import math

from grillex.core import (
    StructuralModel,
    Plate,
)


# =============================================================================
# Test Fixtures
# =============================================================================

@pytest.fixture
def model_with_material():
    """Create a model with Steel material."""
    model = StructuralModel(name="BeamToPlateTest")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    model.add_section("Flat", A=0.06, Iy=1e-4, Iz=1e-4, J=1e-4)
    return model


@pytest.fixture
def model_with_horizontal_beam(model_with_material):
    """Create a model with a horizontal beam along X-axis."""
    beam = model_with_material.add_beam_by_coords(
        [0, 0, 0], [6, 0, 0], "Flat", "Steel"
    )
    return model_with_material, beam


@pytest.fixture
def model_with_diagonal_beam(model_with_material):
    """Create a model with a diagonal beam in XY plane."""
    beam = model_with_material.add_beam_by_coords(
        [0, 0, 0], [3, 4, 0], "Flat", "Steel"
    )
    return model_with_material, beam


@pytest.fixture
def model_with_vertical_beam(model_with_material):
    """Create a model with a vertical beam along Z-axis."""
    beam = model_with_material.add_beam_by_coords(
        [0, 0, 0], [0, 0, 5], "Flat", "Steel"
    )
    return model_with_material, beam


# =============================================================================
# Task 19.14: Basic Beam to Plate Conversion
# =============================================================================

class TestConvertBeamToPlate:
    """Tests for convert_beam_to_plate() method."""

    def test_basic_conversion_horizontal(self, model_with_horizontal_beam):
        """Test basic horizontal beam to plate conversion."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            orientation="horizontal"
        )

        assert plate is not None
        assert isinstance(plate, Plate)
        assert plate.n_corners == 4
        assert plate.thickness == 0.02
        assert plate.material == "Steel"

    def test_conversion_creates_plate_in_model(self, model_with_horizontal_beam):
        """Test that converted plate is added to model."""
        model, beam = model_with_horizontal_beam

        initial_count = len(model.get_plates())
        plate = model.convert_beam_to_plate(beam, width=0.5, thickness=0.02)
        final_count = len(model.get_plates())

        assert final_count == initial_count + 1
        assert plate in model.get_plates()

    def test_plate_corners_horizontal_beam(self, model_with_horizontal_beam):
        """Test corner positions for horizontal beam to horizontal plate."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=1.0,
            thickness=0.02,
            orientation="horizontal"
        )

        corners = np.array(plate.corners)

        # Beam is from [0,0,0] to [6,0,0]
        # Horizontal plate should extend 0.5m in Y direction (half-width each side)
        # All Z values should be 0

        # Check all Z coordinates are 0
        assert np.allclose(corners[:, 2], 0.0)

        # Check length (X direction) spans beam length
        x_min = corners[:, 0].min()
        x_max = corners[:, 0].max()
        assert np.isclose(x_max - x_min, 6.0)

        # Check width (Y direction) is 1.0m
        y_min = corners[:, 1].min()
        y_max = corners[:, 1].max()
        assert np.isclose(y_max - y_min, 1.0)

    def test_plate_corners_vertical_orientation(self, model_with_horizontal_beam):
        """Test corner positions for vertical plate orientation."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.6,  # Height of the vertical plate
            thickness=0.02,
            orientation="vertical"
        )

        corners = np.array(plate.corners)

        # Beam is from [0,0,0] to [6,0,0]
        # Vertical plate should extend 0.3m up and 0.3m down from Z=0
        # All Y values should be 0

        # Check all Y coordinates are 0
        assert np.allclose(corners[:, 1], 0.0)

        # Check length (X direction) spans beam length
        x_min = corners[:, 0].min()
        x_max = corners[:, 0].max()
        assert np.isclose(x_max - x_min, 6.0)

        # Check height (Z direction) is 0.6m
        z_min = corners[:, 2].min()
        z_max = corners[:, 2].max()
        assert np.isclose(z_max - z_min, 0.6)

    def test_plate_corners_top_orientation(self, model_with_horizontal_beam):
        """Test corner positions for top orientation (offset up)."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.4,
            thickness=0.02,
            orientation="top"
        )

        corners = np.array(plate.corners)

        # Top orientation should shift Z by width/2 = 0.2m
        # So Z should be at 0.2m
        assert np.allclose(corners[:, 2], 0.2)

    def test_plate_corners_bottom_orientation(self, model_with_horizontal_beam):
        """Test corner positions for bottom orientation (offset down)."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.4,
            thickness=0.02,
            orientation="bottom"
        )

        corners = np.array(plate.corners)

        # Bottom orientation should shift Z by -width/2 = -0.2m
        # So Z should be at -0.2m
        assert np.allclose(corners[:, 2], -0.2)

    def test_plate_is_planar(self, model_with_horizontal_beam):
        """Test that created plate is planar."""
        model, beam = model_with_horizontal_beam

        for orientation in ["horizontal", "vertical", "top", "bottom"]:
            plate = model.convert_beam_to_plate(
                beam=beam,
                width=0.5,
                thickness=0.02,
                orientation=orientation
            )
            assert plate.is_planar()


class TestConvertBeamToPlateOrientations:
    """Tests for different beam orientations."""

    def test_diagonal_beam_horizontal_plate(self, model_with_diagonal_beam):
        """Test conversion of diagonal beam to horizontal plate."""
        model, beam = model_with_diagonal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            orientation="horizontal"
        )

        assert plate.is_planar()

        # Normal should be vertical (up)
        normal = plate.get_normal()
        assert np.allclose(np.abs(normal), [0, 0, 1], atol=1e-6)

    def test_diagonal_beam_vertical_plate(self, model_with_diagonal_beam):
        """Test conversion of diagonal beam to vertical plate."""
        model, beam = model_with_diagonal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            orientation="vertical"
        )

        assert plate.is_planar()

        # Plate should be vertical (normal perpendicular to Z)
        normal = plate.get_normal()
        assert np.isclose(normal[2], 0.0, atol=1e-6)

    def test_vertical_beam_horizontal_plate(self, model_with_vertical_beam):
        """Test conversion of vertical beam to horizontal plate.

        For a vertical beam, the "horizontal" orientation means the plate's
        width extends horizontally (in X direction), but the plate itself
        spans vertically along the beam axis. The resulting plate is in the
        XZ plane with normal pointing in Y direction.
        """
        model, beam = model_with_vertical_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            orientation="horizontal"
        )

        assert plate.is_planar()

        # For vertical beam, "horizontal" plate lies in XZ plane (width in X)
        # Normal points in Y direction since plate is in XZ plane
        normal = plate.get_normal()
        assert np.allclose(np.abs(normal), [0, 1, 0], atol=1e-6)

    def test_vertical_beam_vertical_plate_raises_error(self, model_with_vertical_beam):
        """Test that vertical beam with vertical plate orientation raises error."""
        model, beam = model_with_vertical_beam

        with pytest.raises(ValueError, match="Cannot create vertical plate"):
            model.convert_beam_to_plate(
                beam=beam,
                width=0.5,
                thickness=0.02,
                orientation="vertical"
            )


class TestConvertBeamToPlateParameters:
    """Tests for various parameter combinations."""

    def test_custom_mesh_size(self, model_with_horizontal_beam):
        """Test custom mesh_size parameter."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            mesh_size=0.25
        )

        assert plate.mesh_size == 0.25

    def test_custom_element_type(self, model_with_horizontal_beam):
        """Test custom element_type parameter."""
        model, beam = model_with_horizontal_beam

        for elem_type in ["MITC4", "MITC8", "MITC9", "DKT"]:
            plate = model.convert_beam_to_plate(
                beam=beam,
                width=0.5,
                thickness=0.02,
                element_type=elem_type
            )
            assert plate.element_type == elem_type

    def test_custom_name(self, model_with_horizontal_beam):
        """Test custom name parameter."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            name="MyCustomPlate"
        )

        assert plate.name == "MyCustomPlate"

    def test_default_name_includes_beam_id(self, model_with_horizontal_beam):
        """Test default name includes beam ID."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02
        )

        assert f"Plate_from_{beam.beam_id}" in plate.name

    def test_auto_thickness_from_section(self, model_with_horizontal_beam):
        """Test automatic thickness estimation from section area."""
        model, beam = model_with_horizontal_beam

        width = 0.3
        plate = model.convert_beam_to_plate(
            beam=beam,
            width=width,
            # thickness not specified
        )

        # Thickness should be A / width = 0.06 / 0.3 = 0.2
        expected_thickness = beam.section.A / width
        assert np.isclose(plate.thickness, expected_thickness)


class TestConvertBeamToPlateRemoval:
    """Tests for beam removal option."""

    def test_remove_beam_false(self, model_with_horizontal_beam):
        """Test that beam is kept when remove_beam=False."""
        model, beam = model_with_horizontal_beam

        initial_count = len(model.beams)
        model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            remove_beam=False
        )
        final_count = len(model.beams)

        assert final_count == initial_count

    def test_remove_beam_true(self, model_with_horizontal_beam):
        """Test that beam is removed when remove_beam=True."""
        model, beam = model_with_horizontal_beam

        initial_count = len(model.beams)
        model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            remove_beam=True
        )
        final_count = len(model.beams)

        assert final_count == initial_count - 1
        assert beam not in model.beams


class TestConvertBeamToPlateErrors:
    """Tests for error handling."""

    def test_invalid_orientation_raises_error(self, model_with_horizontal_beam):
        """Test that invalid orientation raises ValueError."""
        model, beam = model_with_horizontal_beam

        with pytest.raises(ValueError, match="Invalid orientation"):
            model.convert_beam_to_plate(
                beam=beam,
                width=0.5,
                thickness=0.02,
                orientation="invalid"
            )


# =============================================================================
# Task 19.14: Batch Beam to Plate Conversion
# =============================================================================

class TestConvertBeamsToPlates:
    """Tests for convert_beams_to_plates() method."""

    def test_convert_all_beams(self, model_with_material):
        """Test converting all beams in the model."""
        model = model_with_material

        # Add multiple beams
        model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")
        model.add_beam_by_coords([0, 2, 0], [4, 2, 0], "Flat", "Steel")
        model.add_beam_by_coords([0, 4, 0], [4, 4, 0], "Flat", "Steel")

        plates = model.convert_beams_to_plates(
            width=0.5,
            thickness=0.02
        )

        assert len(plates) == 3
        assert all(isinstance(p, Plate) for p in plates)

    def test_convert_specific_beams(self, model_with_material):
        """Test converting specific beams."""
        model = model_with_material

        beam1 = model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")
        beam2 = model.add_beam_by_coords([0, 2, 0], [4, 2, 0], "Flat", "Steel")
        beam3 = model.add_beam_by_coords([0, 4, 0], [4, 4, 0], "Flat", "Steel")

        # Convert only beam1 and beam3
        plates = model.convert_beams_to_plates(
            beams=[beam1, beam3],
            width=0.5,
            thickness=0.02
        )

        assert len(plates) == 2

    def test_width_function(self, model_with_material):
        """Test using width_function parameter."""
        model = model_with_material

        model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")
        model.add_beam_by_coords([0, 2, 0], [4, 2, 0], "Flat", "Steel")

        # Width based on section area
        plates = model.convert_beams_to_plates(
            width_function=lambda b: b.section.A / 0.1,  # A=0.06 -> width=0.6
            thickness=0.02
        )

        # Each plate should have width based on the function
        for plate in plates:
            # Verify width by checking corner distances
            corners = np.array(plate.corners)
            # For horizontal plate, width is in Y direction
            y_extent = corners[:, 1].max() - corners[:, 1].min()
            assert np.isclose(y_extent, 0.6, atol=1e-6)

    def test_thickness_function(self, model_with_material):
        """Test using thickness_function parameter."""
        model = model_with_material

        model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")

        plates = model.convert_beams_to_plates(
            width=0.5,
            thickness_function=lambda b: b.section.A / 2  # A=0.06 -> t=0.03
        )

        assert plates[0].thickness == 0.03

    def test_remove_beams_option(self, model_with_material):
        """Test remove_beams option."""
        model = model_with_material

        model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")
        model.add_beam_by_coords([0, 2, 0], [4, 2, 0], "Flat", "Steel")

        initial_count = len(model.beams)

        model.convert_beams_to_plates(
            width=0.5,
            thickness=0.02,
            remove_beams=True
        )

        assert len(model.beams) == 0
        assert len(model.get_plates()) == initial_count

    def test_no_width_raises_error(self, model_with_material):
        """Test that missing width and width_function raises error."""
        model = model_with_material
        model.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Flat", "Steel")

        with pytest.raises(ValueError, match="width.*must be provided"):
            model.convert_beams_to_plates(thickness=0.02)


# =============================================================================
# Task 19.14: Plate Geometry Verification
# =============================================================================

class TestPlateGeometry:
    """Tests for plate geometry correctness."""

    def test_plate_area_matches_beam_length_times_width(self, model_with_horizontal_beam):
        """Test that plate area equals beam_length * width."""
        model, beam = model_with_horizontal_beam

        width = 0.8
        plate = model.convert_beam_to_plate(
            beam=beam,
            width=width,
            thickness=0.02
        )

        expected_area = beam.length * width
        assert np.isclose(plate.get_area(), expected_area, rtol=1e-6)

    def test_plate_normal_horizontal(self, model_with_horizontal_beam):
        """Test plate normal for horizontal orientation."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            orientation="horizontal"
        )

        normal = plate.get_normal()
        # Normal should point up (positive Z)
        assert np.allclose(np.abs(normal), [0, 0, 1], atol=1e-6)

    def test_plate_normal_vertical(self, model_with_horizontal_beam):
        """Test plate normal for vertical orientation."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            orientation="vertical"
        )

        normal = plate.get_normal()
        # Normal should be horizontal (Z component = 0)
        assert np.isclose(normal[2], 0.0, atol=1e-6)
        # Normal should be perpendicular to beam direction
        beam_dir = beam.get_direction()
        assert np.isclose(np.abs(np.dot(normal, beam_dir)), 0.0, atol=1e-6)

    def test_plate_edge_lengths(self, model_with_horizontal_beam):
        """Test that plate edge lengths are correct."""
        model, beam = model_with_horizontal_beam

        width = 0.6
        plate = model.convert_beam_to_plate(
            beam=beam,
            width=width,
            thickness=0.02,
            orientation="horizontal"
        )

        # For a horizontal beam along X, plate should have:
        # - Edges 0 and 2: length = beam_length (along beam)
        # - Edges 1 and 3: length = width (perpendicular to beam)

        edge_0_length = plate.get_edge_length(0)
        edge_1_length = plate.get_edge_length(1)
        edge_2_length = plate.get_edge_length(2)
        edge_3_length = plate.get_edge_length(3)

        assert np.isclose(edge_0_length, beam.length, rtol=1e-6)
        assert np.isclose(edge_1_length, width, rtol=1e-6)
        assert np.isclose(edge_2_length, beam.length, rtol=1e-6)
        assert np.isclose(edge_3_length, width, rtol=1e-6)


# =============================================================================
# Task 19.14: Integration Tests
# =============================================================================

class TestBeamToPlateIntegration:
    """Integration tests for beam-to-plate conversion."""

    def test_converted_plate_can_be_meshed(self, model_with_horizontal_beam):
        """Test that converted plate can be meshed."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02,
            mesh_size=1.0
        )

        # Set edge divisions for structured mesh
        model.set_edge_divisions(plate, 0, 6)  # Along beam
        model.set_edge_divisions(plate, 2, 6)  # Opposite edge
        model.set_edge_divisions(plate, 1, 1)  # Width direction
        model.set_edge_divisions(plate, 3, 1)  # Opposite width edge

        # This should not raise an error
        assert plate.edge_controls[0].n_elements == 6
        assert plate.edge_controls[1].n_elements == 1

    def test_converted_plate_can_have_supports(self, model_with_horizontal_beam):
        """Test that converted plate can have support curves."""
        model, beam = model_with_horizontal_beam

        plate = model.convert_beam_to_plate(
            beam=beam,
            width=0.5,
            thickness=0.02
        )

        # Add support curve
        support = model.add_support_curve(
            plate=plate,
            edge_index=0,
            uz=True
        )

        assert support in plate.support_curves
        assert support.uz is True

    def test_converted_plate_can_couple_to_beam(self, model_with_material):
        """Test that converted plate can be coupled to another beam."""
        model = model_with_material

        # Create two beams
        beam1 = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Flat", "Steel")
        beam2 = model.add_beam_by_coords([0, 0, 0], [0, 4, 0], "Flat", "Steel")

        # Convert first beam to plate
        plate = model.convert_beam_to_plate(
            beam=beam1,
            width=0.5,
            thickness=0.02
        )

        # Couple plate edge to second beam
        coupling = model.couple_plate_to_beam(
            plate=plate,
            edge_index=3,  # Start edge
            beam=beam2
        )

        assert coupling in plate.beam_couplings


# =============================================================================
# Run tests
# =============================================================================

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
