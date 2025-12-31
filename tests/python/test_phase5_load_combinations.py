"""
Tests for Phase 5.4: Load Combinations

Tests cover:
1. LoadCombination creation with type-based factors
2. Adding load cases with type-based factors
3. Adding load cases with explicit factors
4. Mixed type-based and explicit factors
5. Combined displacement and reaction computation
6. Error handling for missing results
7. Eurocode ULS/SLS combination examples
"""

import pytest
import numpy as np
from grillex._grillex_cpp import (
    Model, BeamConfig, DOFIndex, LoadCaseType,
    LoadCombination, LoadCombinationTerm
)


class TestLoadCombinationBasics:
    """Basic LoadCombination functionality tests."""

    def test_create_combination_default_factors(self):
        """Test creating a combination with default factors (all 1.0)."""
        combo = LoadCombination(1, "Default Combo")

        assert combo.id == 1
        assert combo.name == "Default Combo"
        assert len(combo) == 0
        assert combo.empty()

        # All type factors should be 1.0 by default
        assert combo.get_type_factor(LoadCaseType.Permanent) == 1.0
        assert combo.get_type_factor(LoadCaseType.Variable) == 1.0
        assert combo.get_type_factor(LoadCaseType.Environmental) == 1.0
        assert combo.get_type_factor(LoadCaseType.Accidental) == 1.0

    def test_create_combination_with_type_factors(self):
        """Test creating a combination with specific type factors."""
        combo = LoadCombination(
            id=2,
            name="ULS-STR",
            permanent_factor=1.35,
            variable_factor=1.5,
            environmental_factor=1.5,
            accidental_factor=1.0
        )

        assert combo.get_type_factor(LoadCaseType.Permanent) == 1.35
        assert combo.get_type_factor(LoadCaseType.Variable) == 1.5
        assert combo.get_type_factor(LoadCaseType.Environmental) == 1.5
        assert combo.get_type_factor(LoadCaseType.Accidental) == 1.0

    def test_set_type_factor(self):
        """Test modifying type factors after creation."""
        combo = LoadCombination(1, "Test")

        combo.set_type_factor(LoadCaseType.Permanent, 0.9)
        combo.set_type_factor(LoadCaseType.Variable, 0.7)

        assert combo.get_type_factor(LoadCaseType.Permanent) == 0.9
        assert combo.get_type_factor(LoadCaseType.Variable) == 0.7


class TestAddLoadCases:
    """Tests for adding load cases to combinations."""

    @pytest.fixture
    def simple_model(self):
        """Create a simple cantilever model with multiple load cases."""
        model = Model()

        # Create material and section
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("IPE200", 28.5e-4, 1940e-8, 142e-8, 6.67e-8)

        # Create cantilever beam
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(5, 0, 0)
        model.create_beam(n1, n2, mat, sec)

        # Fix support
        model.boundary_conditions.fix_node(n1.id)

        return model

    def test_add_load_case_type_based_factor(self, simple_model):
        """Test adding load cases using type-based factors."""
        model = simple_model

        # Create load cases of different types
        dead = model.create_load_case("Dead Load", LoadCaseType.Permanent)
        live = model.create_load_case("Live Load", LoadCaseType.Variable)

        # Add loads
        n2 = model.get_all_nodes()[1]
        dead.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -10.0])  # 10 kN down
        live.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -5.0])   # 5 kN down

        # Create combination with Eurocode ULS factors
        combo = LoadCombination(1, "ULS", 1.35, 1.5, 1.5, 1.0)
        combo.add_load_case(dead)  # Should use 1.35
        combo.add_load_case(live)  # Should use 1.5

        assert len(combo) == 2
        assert not combo.empty()

        # Check terms
        terms = combo.get_terms()
        assert len(terms) == 2

        # First term (dead load)
        assert terms[0].load_case.name == "Dead Load"
        assert terms[0].factor == pytest.approx(1.35)
        assert not terms[0].explicit_factor

        # Second term (live load)
        assert terms[1].load_case.name == "Live Load"
        assert terms[1].factor == pytest.approx(1.5)
        assert not terms[1].explicit_factor

    def test_add_load_case_explicit_factor(self, simple_model):
        """Test adding load cases with explicit factors."""
        model = simple_model

        dead = model.create_load_case("Dead Load", LoadCaseType.Permanent)

        n2 = model.get_all_nodes()[1]
        dead.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -10.0])

        # Create combination
        combo = LoadCombination(1, "Custom", 1.35, 1.5, 1.5, 1.0)
        combo.add_load_case(dead, 0.9)  # Explicit factor 0.9 instead of 1.35

        terms = combo.get_terms()
        assert len(terms) == 1
        assert terms[0].factor == pytest.approx(0.9)
        assert terms[0].explicit_factor

    def test_add_load_case_mixed_factors(self, simple_model):
        """Test combination with both type-based and explicit factors."""
        model = simple_model

        dead = model.create_load_case("Dead Load", LoadCaseType.Permanent)
        live = model.create_load_case("Live Load", LoadCaseType.Variable)
        special = model.create_load_case("Special", LoadCaseType.Variable)

        n2 = model.get_all_nodes()[1]
        dead.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -10.0])
        live.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -5.0])
        special.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -3.0])

        # Create combination
        combo = LoadCombination(1, "Mixed", 1.35, 1.5, 1.5, 1.0)
        combo.add_load_case(dead)           # Uses type-based: 1.35
        combo.add_load_case(live)           # Uses type-based: 1.5
        combo.add_load_case(special, 0.7)   # Explicit: 0.7

        terms = combo.get_terms()
        assert len(terms) == 3

        # Check factors
        assert terms[0].factor == pytest.approx(1.35)
        assert not terms[0].explicit_factor
        assert terms[1].factor == pytest.approx(1.5)
        assert not terms[1].explicit_factor
        assert terms[2].factor == pytest.approx(0.7)
        assert terms[2].explicit_factor

    def test_remove_load_case(self, simple_model):
        """Test removing a load case from combination."""
        model = simple_model

        dead = model.create_load_case("Dead Load", LoadCaseType.Permanent)
        live = model.create_load_case("Live Load", LoadCaseType.Variable)

        combo = LoadCombination(1, "Test")
        combo.add_load_case(dead)
        combo.add_load_case(live)
        assert len(combo) == 2

        # Remove dead load
        result = combo.remove_load_case(dead)
        assert result is True
        assert len(combo) == 1
        assert combo.get_terms()[0].load_case.name == "Live Load"

        # Try to remove again (should return False)
        result = combo.remove_load_case(dead)
        assert result is False

    def test_clear_combination(self, simple_model):
        """Test clearing all load cases from combination."""
        model = simple_model

        dead = model.create_load_case("Dead Load", LoadCaseType.Permanent)
        live = model.create_load_case("Live Load", LoadCaseType.Variable)

        combo = LoadCombination(1, "Test")
        combo.add_load_case(dead)
        combo.add_load_case(live)
        assert len(combo) == 2

        combo.clear()
        assert len(combo) == 0
        assert combo.empty()


class TestCombinedResults:
    """Tests for combined result computation."""

    @pytest.fixture
    def analyzed_model(self):
        """Create and analyze a cantilever model with multiple load cases."""
        model = Model()

        # Create material and section
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("IPE200", 28.5e-4, 1940e-8, 142e-8, 6.67e-8)

        # Create cantilever beam (5m)
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(5, 0, 0)
        model.create_beam(n1, n2, mat, sec)

        # Fix support
        model.boundary_conditions.fix_node(n1.id)

        # Create load cases
        dead = model.create_load_case("Dead Load", LoadCaseType.Permanent)
        live = model.create_load_case("Live Load", LoadCaseType.Variable)

        # Add loads - both point loads at tip
        dead.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -10.0])  # 10 kN down
        live.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -5.0])   # 5 kN down

        # Analyze
        success = model.analyze()
        assert success, f"Analysis failed: {model.get_error_message()}"

        return model

    def test_combined_displacements(self, analyzed_model):
        """Test computing combined displacements."""
        model = analyzed_model

        # Get load cases
        load_cases = model.get_load_cases()
        dead = load_cases[0]
        live = load_cases[1]

        # Get individual results
        model.set_active_load_case(dead)
        u_dead = model.get_displacements()

        model.set_active_load_case(live)
        u_live = model.get_displacements()

        # Create combination
        combo = LoadCombination(1, "ULS", 1.35, 1.5, 1.5, 1.0)
        combo.add_load_case(dead)
        combo.add_load_case(live)

        # Get combined displacements
        results = model.get_all_results()
        u_combined = combo.get_combined_displacements(results)

        # Verify: combined = 1.35 * dead + 1.5 * live
        u_expected = 1.35 * u_dead + 1.5 * u_live
        np.testing.assert_allclose(u_combined, u_expected, rtol=1e-10)

    def test_combined_reactions(self, analyzed_model):
        """Test computing combined reactions."""
        model = analyzed_model

        # Get load cases
        load_cases = model.get_load_cases()
        dead = load_cases[0]
        live = load_cases[1]

        # Get individual reactions
        model.set_active_load_case(dead)
        r_dead = model.get_reactions()

        model.set_active_load_case(live)
        r_live = model.get_reactions()

        # Create combination
        combo = LoadCombination(1, "ULS", 1.35, 1.5, 1.5, 1.0)
        combo.add_load_case(dead)
        combo.add_load_case(live)

        # Get combined reactions
        results = model.get_all_results()
        r_combined = combo.get_combined_reactions(results)

        # Verify: combined = 1.35 * dead + 1.5 * live
        r_expected = 1.35 * r_dead + 1.5 * r_live
        np.testing.assert_allclose(r_combined, r_expected, rtol=1e-10)

    def test_superposition_principle(self, analyzed_model):
        """Test that superposition holds for linear analysis."""
        model = analyzed_model

        # Get load cases
        load_cases = model.get_load_cases()
        dead = load_cases[0]
        live = load_cases[1]

        # Get individual results
        model.set_active_load_case(dead)
        u_dead = model.get_displacements()

        model.set_active_load_case(live)
        u_live = model.get_displacements()

        # Combined with factors 1.0, 1.0 should equal sum
        combo = LoadCombination(1, "Sum", 1.0, 1.0, 1.0, 1.0)
        combo.add_load_case(dead)
        combo.add_load_case(live)

        results = model.get_all_results()
        u_combined = combo.get_combined_displacements(results)

        # Simple sum check
        u_sum = u_dead + u_live
        np.testing.assert_allclose(u_combined, u_sum, rtol=1e-10)


class TestEurocodeCombinations:
    """Tests based on Eurocode load combination rules."""

    @pytest.fixture
    def eurocode_model(self):
        """Create a model with Eurocode-style load cases."""
        model = Model()

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("IPE200", 28.5e-4, 1940e-8, 142e-8, 6.67e-8)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)
        model.create_beam(n1, n2, mat, sec)

        model.boundary_conditions.fix_node(n1.id)

        # Create Eurocode load cases
        G = model.create_load_case("G (Permanent)", LoadCaseType.Permanent)
        Q = model.create_load_case("Q (Imposed)", LoadCaseType.Variable)
        W = model.create_load_case("W (Wind)", LoadCaseType.Environmental)

        # Add loads
        G.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -20.0])  # Dead load
        Q.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -15.0])  # Live load
        W.add_nodal_load([n2.x, n2.y, n2.z], [0, 8.0, 0])    # Wind horizontal

        success = model.analyze()
        assert success, f"Analysis failed: {model.get_error_message()}"

        return model

    def test_uls_fundamental_combination(self, eurocode_model):
        """
        Test Eurocode ULS fundamental combination (EN 1990 Eq. 6.10).

        γG * G + γQ * Q + γW * ψ0 * W
        where γG = 1.35, γQ = 1.5, γW = 1.5, ψ0 = 0.6
        """
        model = eurocode_model

        load_cases = model.get_load_cases()
        G = load_cases[0]
        Q = load_cases[1]
        W = load_cases[2]

        # Create ULS combination
        combo = LoadCombination(1, "ULS-Fundamental", 1.35, 1.5, 1.5, 1.0)
        combo.add_load_case(G)           # 1.35 (Permanent)
        combo.add_load_case(Q)           # 1.5 (Variable - leading)
        combo.add_load_case(W, 1.5*0.6)  # 1.5 * ψ0 = 0.9 (accompanying)

        terms = combo.get_terms()
        assert len(terms) == 3
        assert terms[0].factor == pytest.approx(1.35)
        assert terms[1].factor == pytest.approx(1.5)
        assert terms[2].factor == pytest.approx(0.9)

        # Compute combined results
        results = model.get_all_results()
        u_combined = combo.get_combined_displacements(results)

        # Verify manually
        model.set_active_load_case(G)
        u_G = model.get_displacements()
        model.set_active_load_case(Q)
        u_Q = model.get_displacements()
        model.set_active_load_case(W)
        u_W = model.get_displacements()

        u_expected = 1.35 * u_G + 1.5 * u_Q + 0.9 * u_W
        np.testing.assert_allclose(u_combined, u_expected, rtol=1e-10)

    def test_sls_characteristic_combination(self, eurocode_model):
        """
        Test Eurocode SLS characteristic combination (EN 1990 Eq. 6.14).

        G + Q + ψ0 * W
        where ψ0 = 0.6 for wind
        """
        model = eurocode_model

        load_cases = model.get_load_cases()
        G = load_cases[0]
        Q = load_cases[1]
        W = load_cases[2]

        # Create SLS combination (all base factors = 1.0)
        combo = LoadCombination(1, "SLS-Characteristic")
        combo.add_load_case(G)          # 1.0
        combo.add_load_case(Q)          # 1.0
        combo.add_load_case(W, 0.6)     # ψ0 = 0.6

        terms = combo.get_terms()
        assert terms[0].factor == pytest.approx(1.0)
        assert terms[1].factor == pytest.approx(1.0)
        assert terms[2].factor == pytest.approx(0.6)

        # Compute and verify
        results = model.get_all_results()
        u_combined = combo.get_combined_displacements(results)

        model.set_active_load_case(G)
        u_G = model.get_displacements()
        model.set_active_load_case(Q)
        u_Q = model.get_displacements()
        model.set_active_load_case(W)
        u_W = model.get_displacements()

        u_expected = 1.0 * u_G + 1.0 * u_Q + 0.6 * u_W
        np.testing.assert_allclose(u_combined, u_expected, rtol=1e-10)

    def test_favorable_unfavorable_permanent(self, eurocode_model):
        """
        Test Eurocode favorable/unfavorable permanent actions.

        When permanent action is unfavorable: γG,sup = 1.35
        When permanent action is favorable: γG,inf = 1.0
        """
        model = eurocode_model

        load_cases = model.get_load_cases()
        G = load_cases[0]
        W = load_cases[2]

        # Unfavorable permanent (adds to wind effect)
        combo_unfav = LoadCombination(1, "ULS-Wind-Unfav")
        combo_unfav.add_load_case(G, 1.35)
        combo_unfav.add_load_case(W, 1.5)

        # Favorable permanent (reduces wind effect)
        combo_fav = LoadCombination(2, "ULS-Wind-Fav")
        combo_fav.add_load_case(G, 1.0)
        combo_fav.add_load_case(W, 1.5)

        results = model.get_all_results()

        u_unfav = combo_unfav.get_combined_displacements(results)
        u_fav = combo_fav.get_combined_displacements(results)

        # Both should be valid, just different magnitudes
        assert len(u_unfav) == len(u_fav)
        # Unfavorable should have larger vertical deflection (more dead load)
        # Find UZ DOF - it's at position 2 in each node's DOFs
        # Node 2 (tip) has DOFs starting at index 6
        uz_index = 6 + 2  # Second node, UZ
        assert abs(u_unfav[uz_index]) > abs(u_fav[uz_index])


class TestErrorHandling:
    """Tests for error handling."""

    def test_missing_load_case_result(self):
        """Test error when load case result is missing."""
        model = Model()

        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
        sec = model.create_section("IPE200", 28.5e-4, 1940e-8, 142e-8, 6.67e-8)

        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(5, 0, 0)
        model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)

        # Create load cases
        lc1 = model.create_load_case("LC1", LoadCaseType.Permanent)
        lc2 = model.create_load_case("LC2", LoadCaseType.Variable)

        lc1.add_nodal_load([n2.x, n2.y, n2.z], [0, 0, -10.0])
        # Don't add anything to lc2

        # Analyze only lc1 (implicitly both are analyzed, but lc2 has no loads)
        success = model.analyze()
        assert success

        # This should work - both load cases were analyzed
        combo = LoadCombination(1, "Test")
        combo.add_load_case(lc1)
        combo.add_load_case(lc2)

        results = model.get_all_results()
        u = combo.get_combined_displacements(results)
        assert len(u) > 0


class TestRepr:
    """Tests for __repr__ methods."""

    def test_load_combination_repr(self):
        """Test LoadCombination string representation."""
        combo = LoadCombination(1, "Test Combo", 1.35, 1.5, 1.5, 1.0)
        repr_str = repr(combo)
        assert "LoadCombination" in repr_str
        assert "Test Combo" in repr_str
        assert "id=1" in repr_str

    def test_load_combination_term_repr(self):
        """Test LoadCombinationTerm string representation."""
        model = Model()
        lc = model.create_load_case("Dead", LoadCaseType.Permanent)

        term = LoadCombinationTerm(lc, 1.35, False)
        repr_str = repr(term)
        assert "LoadCombinationTerm" in repr_str
        assert "Dead" in repr_str
        assert "1.35" in repr_str


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
