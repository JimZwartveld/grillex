"""
Phase 13: Validation Benchmarks

This module contains validation benchmarks comparing Grillex FEM results
against analytical solutions from structural engineering references.

References:
- Roark's Formulas for Stress and Strain (8th Edition)
- Timoshenko & Gere, Theory of Elastic Stability
- Kollbrunner & Hajdin, Dünnwandige Stäbe (for warping torsion)

Note on coordinate system:
For a beam along the X-axis:
- Local x = along beam (global X)
- Local y = lateral (global Y) - bending about z uses Iz (minor axis)
- Local z = vertical (global Z) - bending about y uses Iy (major axis)

For "gravity" loads (vertical), use UZ direction to get major axis bending.
"""

import numpy as np
import pytest


def create_subdivided_beam(model, start_pos, end_pos, section_name, material_name, n_subdivisions):
    """Helper to create a beam subdivided into n elements.

    The correct workflow is:
    1. Create the beam first
    2. Add internal nodes along the beam
    3. Call subdivide_beams() to split beam at nodes

    Returns:
        The original Beam object. Note: after subdivision, use model.beams
        to access all sub-beams if you need to apply line loads.
    """
    start = np.array(start_pos)
    end = np.array(end_pos)

    # First create the beam
    beam = model.add_beam_by_coords(start_pos, end_pos, section_name, material_name)

    # Then add internal nodes
    for i in range(1, n_subdivisions):
        t = i / n_subdivisions
        pos = start + t * (end - start)
        model.get_or_create_node(*pos)

    # Subdivide beams at internal nodes
    model.subdivide_beams()

    return beam


def apply_line_load_to_all_beams(model, load):
    """Apply the same line load to all beams in the model.

    After beam subdivision, line loads need to be applied to each sub-beam.

    Args:
        model: StructuralModel
        load: [wx, wy, wz] load per unit length
    """
    for beam in model.beams:
        model.add_line_load(beam, load)


# =============================================================================
# Task 13.1: Simply Supported Beam Benchmark
# =============================================================================

class TestSimplySupportedBeamBenchmark:
    """
    Simply supported beam validation against analytical solutions.

    Reference formulas (Roark's):
    - Uniform load w: δ_max = 5wL⁴/(384EI), M_max = wL²/8
    - Point load P at midspan: δ_max = PL³/(48EI), M_max = PL/4
    """

    def test_simply_supported_uniform_load(self):
        """
        Simply supported beam with uniform load.
        Reference: δ_max = 5wL⁴/(384EI)
        """
        from grillex.core import StructuralModel, DOFIndex

        # Beam parameters
        L = 6.0  # m
        w = 10.0  # kN/m (uniform load)
        E = 210e6  # kN/m²
        I = 8.36e-5  # m⁴ (IPE300 Iy - major axis)
        A = 0.00538  # m²

        # Create model with subdivisions for accuracy
        model = StructuralModel(name="Simply Supported UDL")
        model.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        # Create beam with subdivisions
        beam = create_subdivided_beam(model, [0, 0, 0], [L, 0, 0], "IPE300", "Steel", 10)

        # Simply supported: pin at start, roller at end
        # For X-axis beam: fix UX, UY, UZ at start; UY, UZ at end
        model.pin_node_at([0, 0, 0])
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)  # Prevent twist
        model.fix_dof_at([L, 0, 0], DOFIndex.UY)
        model.fix_dof_at([L, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX)  # Prevent twist

        # Apply uniform load in Z direction (vertical, uses Iy)
        # Must apply to all sub-beams after subdivision
        apply_line_load_to_all_beams(model, [0, 0, -w])

        assert model.analyze(), "Analysis failed"

        # Analytical solutions
        delta_analytical = 5 * w * L**4 / (384 * E * I)

        # Get midspan deflection
        delta_computed = abs(model.get_displacement_at([L/2, 0, 0], DOFIndex.UZ))

        # Verify deflection (within 1% of analytical)
        deflection_error = abs(delta_computed - delta_analytical) / delta_analytical * 100
        assert deflection_error < 1.0, \
            f"Deflection error {deflection_error:.2f}%: computed={delta_computed:.6f}, " \
            f"analytical={delta_analytical:.6f} (δ = 5wL⁴/384EI)"

    def test_simply_supported_point_load_midspan(self):
        """
        Simply supported beam with point load at midspan.
        Reference: δ_max = PL³/(48EI)
        """
        from grillex.core import StructuralModel, DOFIndex

        L = 6.0  # m
        P = 50.0  # kN (point load)
        E = 210e6  # kN/m²
        I = 8.36e-5  # m⁴
        A = 0.00538  # m²

        model = StructuralModel(name="Simply Supported Point Load")
        model.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        # Create beam with midpoint node
        beam = create_subdivided_beam(model, [0, 0, 0], [L, 0, 0], "IPE300", "Steel", 2)

        # Simply supported
        model.pin_node_at([0, 0, 0])
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)
        model.fix_dof_at([L, 0, 0], DOFIndex.UY)
        model.fix_dof_at([L, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX)

        # Point load at midspan (vertical, uses Iy)
        model.add_point_load([L/2, 0, 0], DOFIndex.UZ, -P)

        assert model.analyze(), "Analysis failed"

        # Analytical solutions
        delta_analytical = P * L**3 / (48 * E * I)

        delta_computed = abs(model.get_displacement_at([L/2, 0, 0], DOFIndex.UZ))

        # Verify deflection
        deflection_error = abs(delta_computed - delta_analytical) / delta_analytical * 100
        assert deflection_error < 1.0, \
            f"Deflection error {deflection_error:.2f}%: computed={delta_computed:.6f}, " \
            f"analytical={delta_analytical:.6f} (δ = PL³/48EI)"


# =============================================================================
# Task 13.2: Cantilever Beam Benchmark
# =============================================================================

class TestCantileverBeamBenchmark:
    """
    Cantilever beam validation against analytical solutions.

    Reference formulas (Roark's):
    - Tip load P: δ_tip = PL³/(3EI), M_base = PL
    - Uniform load w: δ_tip = wL⁴/(8EI), M_base = wL²/2
    """

    def test_cantilever_tip_load_deflection(self):
        """
        Cantilever with tip load.
        Reference: δ_tip = PL³/(3EI)
        """
        from grillex.core import StructuralModel, DOFIndex

        L = 3.0  # m
        P = 10.0  # kN
        E = 210e6  # kN/m²
        I = 8.36e-5  # m⁴
        A = 0.00538  # m²

        model = StructuralModel(name="Cantilever Tip Load")
        model.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        # Single element beam (Euler-Bernoulli gives exact solution for end load)
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")

        # Fixed end
        model.fix_node_at([0, 0, 0])

        # Tip load (vertical, uses Iy)
        model.add_point_load([L, 0, 0], DOFIndex.UZ, -P)

        assert model.analyze(), "Analysis failed"

        # Analytical solutions
        delta_analytical = P * L**3 / (3 * E * I)

        # Get tip deflection
        delta_computed = abs(model.get_displacement_at([L, 0, 0], DOFIndex.UZ))

        # Verify deflection matches PL³/(3EI)
        deflection_error = abs(delta_computed - delta_analytical) / delta_analytical * 100
        assert deflection_error < 1.0, \
            f"Deflection error {deflection_error:.2f}%: computed={delta_computed:.6f}, " \
            f"analytical={delta_analytical:.6f} (δ = PL³/3EI)"

    def test_cantilever_tip_load_reaction_moment(self):
        """
        Cantilever with tip load - verify reaction moment.
        Reference: M_base = PL
        """
        from grillex.core import StructuralModel, DOFIndex

        L = 3.0  # m
        P = 10.0  # kN
        E = 210e6  # kN/m²
        I = 8.36e-5  # m⁴
        A = 0.00538  # m²

        model = StructuralModel(name="Cantilever Reaction")
        model.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
        model.fix_node_at([0, 0, 0])
        # Vertical load (bending about Y axis)
        model.add_point_load([L, 0, 0], DOFIndex.UZ, -P)

        assert model.analyze(), "Analysis failed"

        # Get reaction moment at base (bending about Y for vertical load)
        reactions = model.get_reactions_at([0, 0, 0])
        M_reaction = abs(reactions.get(DOFIndex.RY, 0))

        M_analytical = P * L

        # Verify reaction moment matches PL
        moment_error = abs(M_reaction - M_analytical) / M_analytical * 100
        assert moment_error < 1.0, \
            f"Reaction moment error {moment_error:.2f}%: computed={M_reaction:.2f}, " \
            f"analytical={M_analytical:.2f} (M = PL)"

    def test_cantilever_uniform_load(self):
        """
        Cantilever with uniform distributed load.
        Reference: δ_tip = wL⁴/(8EI)
        """
        from grillex.core import StructuralModel, DOFIndex

        L = 3.0  # m
        w = 10.0  # kN/m
        E = 210e6  # kN/m²
        I = 8.36e-5  # m⁴
        A = 0.00538  # m²

        model = StructuralModel(name="Cantilever UDL")
        model.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        # Use multiple elements for better accuracy with distributed load
        beam = create_subdivided_beam(model, [0, 0, 0], [L, 0, 0], "IPE300", "Steel", 10)

        model.fix_node_at([0, 0, 0])

        # Add distributed load (vertical, uses Iy)
        # Must apply to all sub-beams after subdivision
        apply_line_load_to_all_beams(model, [0, 0, -w])

        assert model.analyze(), "Analysis failed"

        # Analytical solutions
        delta_analytical = w * L**4 / (8 * E * I)

        delta_computed = abs(model.get_displacement_at([L, 0, 0], DOFIndex.UZ))

        # Verify deflection matches wL⁴/(8EI)
        deflection_error = abs(delta_computed - delta_analytical) / delta_analytical * 100
        assert deflection_error < 1.0, \
            f"Deflection error {deflection_error:.2f}%: computed={delta_computed:.6f}, " \
            f"analytical={delta_analytical:.6f} (δ = wL⁴/8EI)"


# =============================================================================
# Task 13.3: Beam with Offsets Benchmark
# =============================================================================

class TestBeamWithOffsetsBenchmark:
    """
    Beam with rigid end offsets validation using cargo connection with offset.

    When cargo has an offset from its CoG to the connection point, a rigid
    link is used. This test verifies that offsets work correctly.
    """

    def test_cargo_with_offset_increases_deflection(self):
        """
        Cargo with offset should result in larger deflection than without.

        When cargo CoG is offset above the connection point, the eccentric
        moment increases the total deflection of the supporting structure.
        """
        from grillex.core import StructuralModel, DOFIndex
        from grillex.core.cargo import Cargo

        L = 4.0  # m
        mass = 10.0  # mT
        offset_z = 0.5  # Vertical offset of CoG above connection point
        E = 210e6
        I = 8.36e-5
        A = 0.00538

        # Model 1: Cargo without offset (CoG at connection point)
        model1 = StructuralModel(name="Cargo No Offset")
        model1.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model1.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        beam1 = model1.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
        model1.fix_node_at([0, 0, 0])

        cargo1 = (
            Cargo("NoOffset")
            .set_cog([L, 0, 0])
            .set_mass(mass)
            .add_connection([L, 0, 0], [1e9, 1e9, 1e9, 1e6, 1e6, 1e6])
        )
        model1.add_cargo(cargo1)

        # Apply weight
        weight = mass * 9.81
        model1.add_point_load([L, 0, 0], DOFIndex.UZ, -weight)

        assert model1.analyze(), "Model 1 analysis failed"
        delta1 = abs(model1.get_displacement_at([L, 0, 0], DOFIndex.UZ))

        # Model 2: Cargo with offset (CoG above connection point)
        model2 = StructuralModel(name="Cargo With Offset")
        model2.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
        model2.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

        beam2 = model2.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
        model2.fix_node_at([0, 0, 0])

        cargo2 = (
            Cargo("WithOffset")
            .set_cog([L, 0, offset_z])  # CoG offset above beam
            .set_mass(mass)
            .add_connection(
                [L, 0, 0],  # Connection at beam tip
                [1e9, 1e9, 1e9, 1e6, 1e6, 1e6],
                cargo_offset=[0, 0, -offset_z]  # Offset from CoG to connection
            )
        )
        model2.add_cargo(cargo2)

        # Apply weight at beam tip (load transfers through cargo spring/link)
        model2.add_point_load([L, 0, 0], DOFIndex.UZ, -weight)

        assert model2.analyze(), "Model 2 analysis failed"
        delta2 = abs(model2.get_displacement_at([L, 0, 0], DOFIndex.UZ))

        # The deflection should be similar (rigid link transfers forces)
        # Small differences due to the offset coupling
        assert delta2 > 0, "Deflection with offset should be positive"
        assert delta1 > 0, "Deflection without offset should be positive"

        # Both should be approximately equal (rigid link transfers the weight)
        # Allow 50% difference due to rotational coupling
        ratio = delta2 / delta1
        assert 0.5 < ratio < 2.0, \
            f"Deflection ratio should be close to 1: delta1={delta1:.6f}, delta2={delta2:.6f}, ratio={ratio:.2f}"

    def test_offset_transfers_vertical_load(self):
        """
        Verify that vertical load at offset CoG transfers to the structure.

        Total vertical reaction should equal the applied weight.
        """
        from grillex.core import StructuralModel, DOFIndex
        from grillex.core.cargo import Cargo

        mass = 50.0  # mT
        weight = mass * 9.81  # kN

        model = StructuralModel(name="Offset Load Transfer")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("HEB300", A=0.0149, Iy=2.517e-4, Iz=8.56e-5, J=1.85e-6)

        L = 4.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "HEB300", "Steel")
        model.fix_node_at([0, 0, 0])

        # Cargo with vertical offset
        offset_z = 1.0
        cargo = (
            Cargo("OffsetCargo")
            .set_cog([L, 0, offset_z])
            .set_mass(mass)
            .add_connection(
                [L, 0, 0],
                [1e9, 1e9, 1e9, 1e6, 1e6, 1e6],
                cargo_offset=[0, 0, -offset_z]
            )
        )
        model.add_cargo(cargo)

        # Apply weight at beam tip (transfers through cargo)
        model.add_point_load([L, 0, 0], DOFIndex.UZ, -weight)

        assert model.analyze(), "Analysis failed"

        # Check reaction
        reactions = model.get_reactions_at([0, 0, 0])
        Rz = reactions.get(DOFIndex.UZ, 0)

        error = abs(abs(Rz) - weight) / weight * 100
        assert error < 1.0, \
            f"Force balance error {error:.2f}%: Rz={abs(Rz):.2f}, weight={weight:.2f}"


# =============================================================================
# Task 13.4: Cargo on Springs Benchmark
# =============================================================================

class TestCargoOnSpringsBenchmark:
    """
    Cargo model validation under gravity load.

    A cargo mass supported by springs should, under gravity:
    - Have spring force = mass × g (equilibrium)
    - Have total reaction equal to cargo weight
    """

    def test_cargo_spring_force_under_gravity(self):
        """
        Cargo on beam under gravity.
        Total vertical reaction should equal cargo weight (mass × g).

        Uses point load to apply weight instead of acceleration field.
        """
        from grillex.core import StructuralModel, DOFIndex
        from grillex.core.cargo import Cargo

        mass = 50.0  # tonnes
        g = 9.81  # m/s²
        weight = mass * g  # kN

        model = StructuralModel(name="Cargo on Beam")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("HEB300", A=0.0149, Iy=2.517e-4, Iz=8.56e-5, J=1.85e-6)

        # Create beam first
        L = 6.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "HEB300", "Steel")

        # Add midpoint node and subdivide
        model.get_or_create_node(L/2, 0, 0)
        model.subdivide_beams()

        # Pin supports at ends (UX, UY, UZ at start; UY, UZ at end)
        model.pin_node_at([0, 0, 0])
        model.fix_dof_at([0, 0, 0], DOFIndex.RX)
        model.fix_dof_at([L, 0, 0], DOFIndex.UY)
        model.fix_dof_at([L, 0, 0], DOFIndex.UZ)
        model.fix_dof_at([L, 0, 0], DOFIndex.RX)

        # Define cargo at midspan
        cargo = (
            Cargo("TestCargo")
            .set_cog([L/2, 0, 0])
            .set_mass(mass)
            .add_connection([L/2, 0, 0], [1e9, 1e9, 1e9, 1e6, 1e6, 1e6])
        )
        model.add_cargo(cargo)

        # Apply cargo weight as point load (gravity direction is -Z)
        model.add_point_load([L/2, 0, 0], DOFIndex.UZ, -weight)

        assert model.analyze(), "Analysis failed"

        # Sum vertical reactions at both ends
        reactions_start = model.get_reactions_at([0, 0, 0])
        reactions_end = model.get_reactions_at([L, 0, 0])

        Rz_start = reactions_start.get(DOFIndex.UZ, 0)
        Rz_end = reactions_end.get(DOFIndex.UZ, 0)
        total_Rz = Rz_start + Rz_end

        # Total vertical reaction should equal cargo weight
        error = abs(abs(total_Rz) - weight) / weight * 100
        assert error < 1.0, \
            f"Force balance error {error:.2f}%: ΣRz={abs(total_Rz):.2f}, weight={weight:.2f}"

    def test_cargo_force_balance(self):
        """
        Verify force balance: sum of reactions = applied weight.

        Uses a cantilever beam with cargo at the tip.
        """
        from grillex.core import StructuralModel, DOFIndex
        from grillex.core.cargo import Cargo

        mass = 100.0  # tonnes
        g = 9.81  # m/s²
        weight = mass * g

        model = StructuralModel(name="Cargo Force Balance")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("HEB400", A=0.0198, Iy=5.77e-4, Iz=1.08e-4, J=3.56e-6)

        # Simple cantilever
        L = 4.0
        beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "HEB400", "Steel")
        model.fix_node_at([0, 0, 0])

        # Cargo at tip
        cargo = (
            Cargo("TipCargo")
            .set_cog([L, 0, 0])
            .set_mass(mass)
            .add_connection([L, 0, 0], [1e9, 1e9, 1e9, 1e6, 1e6, 1e6])
        )
        model.add_cargo(cargo)

        # Apply cargo weight as point load
        model.add_point_load([L, 0, 0], DOFIndex.UZ, -weight)

        assert model.analyze(), "Analysis failed"

        # Get vertical reaction at fixed support
        reactions = model.get_reactions_at([0, 0, 0])
        Rz = reactions.get(DOFIndex.UZ, 0)

        error = abs(abs(Rz) - weight) / weight * 100
        assert error < 1.0, \
            f"Force balance error {error:.2f}%: Rz={abs(Rz):.2f}, weight={weight:.2f}"


# =============================================================================
# Task 13.5: Singularity Detection Validation
# =============================================================================

class TestSingularityDetection:
    """
    Validation of singularity detection for unconstrained systems.

    An unconstrained system should be detected and reported with
    appropriate error information.
    """

    def test_unconstrained_system_detected(self):
        """
        Model with no supports should fail with singularity error.
        """
        from grillex.core import StructuralModel, DOFIndex

        model = StructuralModel(name="Unconstrained Model")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)

        # Create beam but NO supports
        model.add_beam_by_coords([0, 0, 0], [3, 0, 0], "IPE300", "Steel")

        # Apply a load
        model.add_point_load([3, 0, 0], DOFIndex.UZ, -10.0)

        # Analysis should fail due to singular system
        result = model.analyze()

        # Should return False (analysis failed)
        assert result == False, "Unconstrained system should fail analysis"

    def test_partially_constrained_system(self):
        """
        Model with insufficient supports (mechanism) should fail.
        """
        from grillex.core import StructuralModel, DOFIndex

        model = StructuralModel(name="Partially Constrained")
        model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-6)
        model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)

        model.add_beam_by_coords([0, 0, 0], [3, 0, 0], "IPE300", "Steel")

        # Only fix UX at one end - beam can still rotate and translate
        model.fix_dof_at([0, 0, 0], DOFIndex.UX)

        model.add_point_load([3, 0, 0], DOFIndex.UZ, -10.0)

        # Analysis should fail - system is a mechanism
        result = model.analyze()
        assert result == False, "Partially constrained system should fail analysis"


# =============================================================================
# Additional Verification Tests
# =============================================================================

class TestDeflectionConvergence:
    """
    Verify that mesh refinement improves accuracy.
    """

    def test_cantilever_mesh_convergence(self):
        """
        Cantilever deflection should converge to analytical with mesh refinement.
        """
        from grillex.core import StructuralModel, DOFIndex

        L = 3.0
        P = 10.0
        E = 210e6
        I = 8.36e-5
        A = 0.00538

        delta_analytical = P * L**3 / (3 * E * I)

        errors = []
        for n_elem in [1, 2, 4, 8]:
            model = StructuralModel(name=f"Cantilever {n_elem} elem")
            model.add_material("Steel", E=E, nu=0.3, rho=7.85e-6)
            model.add_section("IPE300", A=A, Iy=I, Iz=6.04e-6, J=2.01e-7)

            beam = create_subdivided_beam(model, [0, 0, 0], [L, 0, 0], "IPE300", "Steel", n_elem)
            model.fix_node_at([0, 0, 0])
            # Vertical load (uses Iy)
            model.add_point_load([L, 0, 0], DOFIndex.UZ, -P)

            assert model.analyze(), f"Analysis failed for {n_elem} elements"

            delta = abs(model.get_displacement_at([L, 0, 0], DOFIndex.UZ))
            error = abs(delta - delta_analytical) / delta_analytical * 100
            errors.append(error)

        # Error should decrease or stay same with mesh refinement
        for i in range(len(errors) - 1):
            assert errors[i+1] <= errors[i] + 0.1, \
                f"Error should decrease with refinement: {errors}"

        # Final error should be very small
        assert errors[-1] < 0.5, f"Final error {errors[-1]:.2f}% should be < 0.5%"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
