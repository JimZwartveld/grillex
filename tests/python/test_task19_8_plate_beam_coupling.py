"""Tests for Plate-Beam Coupling in StructuralModel.

Tests for Task 19.8: Plate-Beam Coupling and Task 19.9: Support Curves.
"""

import pytest
import numpy as np
from grillex.core import StructuralModel, Plate
from grillex.core.plate import PlateBeamCoupling, SupportCurve


@pytest.fixture
def model_with_beam():
    """Create a model with steel material, section, and a beam."""
    model = StructuralModel(name="CouplingTest")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.67e-8)

    # Add a beam along the x-axis
    beam = model.add_beam_by_coords(
        [0, 0, 0], [4, 0, 0],
        section_name="IPE200",
        material_name="Steel"
    )
    return model, beam


class TestCouplePlateToBeam:
    """Tests for couple_plate_to_beam method."""

    def test_basic_coupling(self, model_with_beam):
        """Test basic plate-beam coupling."""
        model, beam = model_with_beam

        # Add a plate with edge along the beam
        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        coupling = model.couple_plate_to_beam(plate, edge_index=0, beam=beam)

        assert coupling is not None
        assert isinstance(coupling, PlateBeamCoupling)
        assert coupling.plate is plate
        assert coupling.edge_index == 0
        assert coupling.beam is beam
        assert coupling.offset == [0.0, 0.0, 0.0]
        assert coupling.releases == {}

    def test_coupling_with_offset(self, model_with_beam):
        """Test coupling with eccentric offset."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0.15], [4, 0, 0.15], [4, 2, 0.15], [0, 2, 0.15]],
            thickness=0.02,
            material="Steel"
        )

        coupling = model.couple_plate_to_beam(
            plate, edge_index=0, beam=beam,
            offset=[0, 0, 0.15]  # Plate 150mm above beam centroid
        )

        assert coupling.offset == [0, 0, 0.15]

    def test_coupling_with_releases(self, model_with_beam):
        """Test coupling with DOF releases."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        coupling = model.couple_plate_to_beam(
            plate, edge_index=0, beam=beam,
            releases={"R_EDGE": True, "UZ": True}
        )

        assert coupling.releases["R_EDGE"] == True
        assert coupling.releases["UZ"] == True

    def test_coupling_stored_in_plate(self, model_with_beam):
        """Test that coupling is stored in plate object."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        assert len(plate.beam_couplings) == 0

        coupling = model.couple_plate_to_beam(plate, edge_index=0, beam=beam)

        assert len(plate.beam_couplings) == 1
        assert plate.beam_couplings[0] is coupling

    def test_multiple_couplings(self, model_with_beam):
        """Test multiple couplings on same plate."""
        model, beam1 = model_with_beam

        # Add second beam
        beam2 = model.add_beam_by_coords(
            [0, 2, 0], [4, 2, 0],
            section_name="IPE200",
            material_name="Steel"
        )

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        coupling1 = model.couple_plate_to_beam(plate, edge_index=0, beam=beam1)
        coupling2 = model.couple_plate_to_beam(plate, edge_index=2, beam=beam2)

        assert len(plate.beam_couplings) == 2
        assert coupling1.edge_index == 0
        assert coupling2.edge_index == 2

    def test_coupling_different_edges(self, model_with_beam):
        """Test coupling different plate edges."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Couple each edge
        for i in range(4):
            coupling = model.couple_plate_to_beam(plate, edge_index=i, beam=beam)
            assert coupling.edge_index == i

    def test_coupling_invalid_edge_index_raises(self, model_with_beam):
        """Test that invalid edge index raises ValueError."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        with pytest.raises(ValueError) as exc_info:
            model.couple_plate_to_beam(plate, edge_index=4, beam=beam)

        assert "out of range" in str(exc_info.value)

    def test_coupling_negative_edge_index_raises(self, model_with_beam):
        """Test that negative edge index raises ValueError."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        with pytest.raises(ValueError):
            model.couple_plate_to_beam(plate, edge_index=-1, beam=beam)

    def test_coupling_triangular_plate(self, model_with_beam):
        """Test coupling with triangular plate."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [3, 0, 0], [1.5, 2, 0]],
            thickness=0.02,
            material="Steel",
            element_type="DKT"
        )

        # Triangular plate has 3 edges
        coupling = model.couple_plate_to_beam(plate, edge_index=0, beam=beam)
        assert coupling.edge_index == 0

        # Edge index 3 should be invalid
        with pytest.raises(ValueError):
            model.couple_plate_to_beam(plate, edge_index=3, beam=beam)


class TestSupportCurve:
    """Tests for add_support_curve method."""

    def test_basic_support(self):
        """Test basic support curve creation."""
        model = StructuralModel(name="SupportTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        support = model.add_support_curve(plate, edge_index=0, uz=True)

        assert support is not None
        assert isinstance(support, SupportCurve)
        assert support.plate is plate
        assert support.edge_index == 0
        assert support.ux == False
        assert support.uy == False
        assert support.uz == True
        assert support.rotation_about_edge == False

    def test_full_support(self):
        """Test fully restrained support."""
        model = StructuralModel(name="SupportTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        support = model.add_support_curve(
            plate, edge_index=0,
            ux=True, uy=True, uz=True, rotation_about_edge=True
        )

        assert support.ux == True
        assert support.uy == True
        assert support.uz == True
        assert support.rotation_about_edge == True

    def test_support_stored_in_plate(self):
        """Test that support is stored in plate object."""
        model = StructuralModel(name="SupportTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        assert len(plate.support_curves) == 0

        support = model.add_support_curve(plate, edge_index=0, uz=True)

        assert len(plate.support_curves) == 1
        assert plate.support_curves[0] is support

    def test_multiple_supports(self):
        """Test multiple supports on same plate."""
        model = StructuralModel(name="SupportTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        support1 = model.add_support_curve(plate, edge_index=0, uz=True)
        support2 = model.add_support_curve(plate, edge_index=2, uz=True)

        assert len(plate.support_curves) == 2
        assert support1.edge_index == 0
        assert support2.edge_index == 2

    def test_support_invalid_edge_index_raises(self):
        """Test that invalid edge index raises ValueError."""
        model = StructuralModel(name="SupportTest")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        with pytest.raises(ValueError) as exc_info:
            model.add_support_curve(plate, edge_index=5, uz=True)

        assert "out of range" in str(exc_info.value)


class TestCouplingAndSupportIntegration:
    """Integration tests for couplings and supports together."""

    def test_plate_with_coupling_and_support(self, model_with_beam):
        """Test plate with both coupling and support."""
        model, beam = model_with_beam

        plate = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel"
        )

        # Edge 0 coupled to beam
        coupling = model.couple_plate_to_beam(plate, edge_index=0, beam=beam)

        # Edge 2 simply supported
        support = model.add_support_curve(plate, edge_index=2, uz=True)

        assert len(plate.beam_couplings) == 1
        assert len(plate.support_curves) == 1
        assert coupling.edge_index == 0
        assert support.edge_index == 2

    def test_multiple_plates_with_connections(self, model_with_beam):
        """Test multiple plates with various connections."""
        model, beam = model_with_beam

        plate1 = model.add_plate(
            corners=[[0, 0, 0], [4, 0, 0], [4, 2, 0], [0, 2, 0]],
            thickness=0.02,
            material="Steel",
            name="Plate1"
        )

        plate2 = model.add_plate(
            corners=[[0, 2, 0], [4, 2, 0], [4, 4, 0], [0, 4, 0]],
            thickness=0.02,
            material="Steel",
            name="Plate2"
        )

        # Plate1: coupled to beam on edge 0, supported on edge 2
        model.couple_plate_to_beam(plate1, edge_index=0, beam=beam)
        model.add_support_curve(plate1, edge_index=2, uz=True)

        # Plate2: supported on all edges
        for i in range(4):
            model.add_support_curve(plate2, edge_index=i, uz=True)

        assert len(plate1.beam_couplings) == 1
        assert len(plate1.support_curves) == 1
        assert len(plate2.beam_couplings) == 0
        assert len(plate2.support_curves) == 4
