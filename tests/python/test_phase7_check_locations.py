"""
Tests for Task 7.3: Implement Check Locations

Verifies that check locations can be defined on beams for design code verification.

Acceptance Criteria:
- [x] Check locations can be added at arbitrary normalized positions
- [x] Standard check locations (0, 0.25, 0.5, 0.75, 1) can be set automatically
- [x] Internal actions are computed correctly at check locations
- [x] Check locations persist across multiple analyses
"""

import numpy as np
import pytest

from grillex.core import StructuralModel, DOFIndex


class TestCheckLocationManagement:
    """Test check location management methods."""

    def test_add_check_location_valid(self):
        """Can add check locations at valid normalized positions."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        # Add valid check locations
        beam.add_check_location(0.0)
        beam.add_check_location(0.5)
        beam.add_check_location(1.0)

        assert 0.0 in beam.check_locations
        assert 0.5 in beam.check_locations
        assert 1.0 in beam.check_locations
        assert len(beam.check_locations) == 3

    def test_add_check_location_invalid(self):
        """Adding check location outside [0, 1] raises ValueError."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        with pytest.raises(ValueError, match="must be in range"):
            beam.add_check_location(-0.1)

        with pytest.raises(ValueError, match="must be in range"):
            beam.add_check_location(1.1)

    def test_add_check_location_sorted(self):
        """Check locations are kept sorted."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        beam.add_check_location(1.0)
        beam.add_check_location(0.25)
        beam.add_check_location(0.0)
        beam.add_check_location(0.75)

        assert beam.check_locations == [0.0, 0.25, 0.75, 1.0]

    def test_add_check_location_no_duplicates(self):
        """Adding duplicate check location doesn't create duplicate."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        beam.add_check_location(0.5)
        beam.add_check_location(0.5)
        beam.add_check_location(0.5)

        assert beam.check_locations.count(0.5) == 1

    def test_set_standard_check_locations(self):
        """Standard check locations are set correctly."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        beam.set_standard_check_locations()

        assert beam.check_locations == [0.0, 0.25, 0.5, 0.75, 1.0]

    def test_clear_check_locations(self):
        """Check locations can be cleared."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        beam.set_standard_check_locations()
        assert len(beam.check_locations) == 5

        beam.clear_check_locations()
        assert len(beam.check_locations) == 0

    def test_check_locations_initially_empty(self):
        """Check locations are initially empty."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")

        assert beam.check_locations == []


class TestInternalActionsAtCheckLocations:
    """Test internal actions at check locations."""

    @pytest.fixture
    def cantilever_model(self):
        """Create a cantilever beam model."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)

        return model, beam

    def test_get_internal_actions_at(self, cantilever_model):
        """get_internal_actions_at returns correct values."""
        model, beam = cantilever_model
        model.analyze()

        # Get internal actions at midspan (x = 3m)
        actions = beam.get_internal_actions_at(3.0, model)

        # Cantilever with tip load: M(x) = P*(L-x) = 10*(6-3) = 30 kNm
        np.testing.assert_almost_equal(abs(actions.Mz), 30.0, decimal=1)

        # Shear should be constant = P = 10 kN
        np.testing.assert_almost_equal(abs(actions.Vy), 10.0, decimal=1)

    def test_get_internal_actions_at_check_locations(self, cantilever_model):
        """get_internal_actions_at_check_locations returns correct results."""
        model, beam = cantilever_model
        beam.set_standard_check_locations()
        model.analyze()

        results = beam.get_internal_actions_at_check_locations(model)

        # Should have 5 results (one per check location)
        assert len(results) == 5

        # Check positions are correct (in meters)
        L = 6.0
        expected_positions = [0.0, 0.25*L, 0.5*L, 0.75*L, L]
        for i, (x, _) in enumerate(results):
            np.testing.assert_almost_equal(x, expected_positions[i], decimal=6)

        # Check moment values
        # M(x) = P*(L-x) for cantilever with tip load
        P = 10.0
        for x, actions in results:
            expected_M = P * (L - x)
            np.testing.assert_almost_equal(abs(actions.Mz), expected_M, decimal=1)

    def test_internal_actions_at_ends(self, cantilever_model):
        """Internal actions are correct at beam ends."""
        model, beam = cantilever_model
        beam.add_check_location(0.0)
        beam.add_check_location(1.0)
        model.analyze()

        results = beam.get_internal_actions_at_check_locations(model)

        # At base (x=0): M = P*L = 60 kNm
        x_base, actions_base = results[0]
        np.testing.assert_almost_equal(abs(actions_base.Mz), 60.0, decimal=1)

        # At tip (x=L): M = 0
        x_tip, actions_tip = results[1]
        np.testing.assert_almost_equal(actions_tip.Mz, 0.0, decimal=1)

    def test_model_not_analyzed_raises(self, cantilever_model):
        """Querying internal actions on unanalyzed model raises ValueError."""
        model, beam = cantilever_model
        # Don't analyze

        with pytest.raises(ValueError, match="must be analyzed"):
            beam.get_internal_actions_at(3.0, model)


class TestCheckLocationsPersistence:
    """Test that check locations persist across analyses."""

    def test_check_locations_persist_after_reanalysis(self):
        """Check locations persist after re-analyzing the model."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)

        # Set check locations before first analysis
        beam.set_standard_check_locations()
        original_locations = beam.check_locations.copy()

        # First analysis
        model.analyze()
        results1 = beam.get_internal_actions_at_check_locations(model)

        # Verify locations are still there
        assert beam.check_locations == original_locations

        # Re-analyze (simulating a model update)
        model.analyze()
        results2 = beam.get_internal_actions_at_check_locations(model)

        # Locations should still be the same
        assert beam.check_locations == original_locations

        # Results should be the same (same model)
        assert len(results1) == len(results2)
        for (x1, a1), (x2, a2) in zip(results1, results2):
            np.testing.assert_almost_equal(x1, x2)
            np.testing.assert_almost_equal(a1.Mz, a2.Mz, decimal=5)

    def test_check_locations_persist_after_adding_loads(self):
        """Check locations persist after adding loads and re-analyzing."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])

        # Set check locations
        beam.set_standard_check_locations()

        # First load and analysis
        model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)
        model.analyze()

        # Verify check locations still present
        assert beam.check_locations == [0.0, 0.25, 0.5, 0.75, 1.0]


class TestMultiElementBeamCheckLocations:
    """Test check locations on multi-element beams."""

    def test_check_locations_multi_element(self):
        """Check locations work correctly on subdivided beams."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        # Create beam with multiple elements
        beam = model.add_beam_by_coords(
            [0, 0, 0], [6, 0, 0], "Test", "Steel",
            subdivisions=3
        )
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)

        # Set check locations
        beam.set_standard_check_locations()
        model.analyze()

        results = beam.get_internal_actions_at_check_locations(model)

        # Should still get correct values
        L = 6.0
        P = 10.0
        for x, actions in results:
            expected_M = P * (L - x)
            np.testing.assert_almost_equal(abs(actions.Mz), expected_M, decimal=0)


class TestArbitraryCheckLocations:
    """Test arbitrary (non-standard) check locations."""

    def test_arbitrary_check_location(self):
        """Can add check location at arbitrary position."""
        model = StructuralModel()
        model.add_material("Steel", 210e6, 0.3, 7.85e-6)
        model.add_section("Test", 0.01, 8e-5, 8e-5, 1e-6)

        beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "Test", "Steel")
        model.fix_node_at([0, 0, 0])
        model.add_point_load([6, 0, 0], DOFIndex.UY, -10.0)

        # Add arbitrary check location at 1/3 span
        beam.add_check_location(1.0/3.0)
        model.analyze()

        results = beam.get_internal_actions_at_check_locations(model)

        assert len(results) == 1
        x, actions = results[0]
        np.testing.assert_almost_equal(x, 2.0, decimal=5)  # 1/3 * 6m = 2m

        # M(2) = 10 * (6-2) = 40 kNm
        np.testing.assert_almost_equal(abs(actions.Mz), 40.0, decimal=1)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
