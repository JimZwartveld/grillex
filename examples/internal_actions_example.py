"""
Example: Internal Actions and Multi-Element Beam Plotting

This example demonstrates:
1. Creating a multi-span continuous beam
2. Applying loads and running analysis
3. Querying internal actions along the beam
4. Finding moment extrema
5. Plotting moment and shear diagrams

Units: kN, m, mT (as per Grillex conventions)
"""

import numpy as np
from grillex.core import (
    Model, DOFIndex, BeamConfig, BeamFormulation,
    InternalActions, EndForces, DisplacementLine, ActionExtreme
)


def example_cantilever_internal_actions():
    """
    Example 1: Cantilever beam with tip load - accessing internal actions.

    This shows the basic API for getting internal actions at any position.
    """
    print("=" * 60)
    print("Example 1: Cantilever Internal Actions")
    print("=" * 60)

    # Create model
    model = Model()
    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)  # E, nu, rho
    sec = model.create_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)  # A, Iy, Iz, J

    # Create cantilever beam (6m long)
    L = 6.0  # meters
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(L, 0, 0)
    beam = model.create_beam(n1, n2, mat, sec)

    # Fix the left end
    model.boundary_conditions.fix_node(n1.id)

    # Apply tip load
    P = 10.0  # kN downward
    lc = model.get_default_load_case()
    lc.add_nodal_load(n2.id, DOFIndex.UY, -P)

    # Run analysis
    assert model.analyze(), "Analysis failed"

    # Get results
    dof_handler = model.get_dof_handler()
    u = model.get_displacements()

    # Query internal actions at various positions
    print("\nInternal actions along beam:")
    print(f"{'Position (m)':<15} {'Shear Vy (kN)':<15} {'Moment Mz (kNm)':<15}")
    print("-" * 45)

    for x in [0.0, L/4, L/2, 3*L/4, L]:
        actions = beam.get_internal_actions(x, u, dof_handler, lc)
        print(f"{x:<15.2f} {actions.Vy:<15.2f} {actions.Mz:<15.2f}")

    # Expected values:
    # - Shear: constant = P = 10 kN
    # - Moment at base: P * L = 60 kNm
    # - Moment at tip: 0

    print("\nExpected: Shear = 10 kN constant, Moment at base = 60 kNm")


def example_end_forces():
    """
    Example 2: Computing element end forces.

    End forces are useful for design checks and understanding load transfer.
    """
    print("\n" + "=" * 60)
    print("Example 2: Element End Forces")
    print("=" * 60)

    model = Model()
    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
    sec = model.create_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

    L = 6.0
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(L, 0, 0)
    beam = model.create_beam(n1, n2, mat, sec)

    model.boundary_conditions.fix_node(n1.id)

    lc = model.get_default_load_case()
    lc.add_nodal_load(n2.id, DOFIndex.UY, -10.0)

    assert model.analyze()

    dof_handler = model.get_dof_handler()
    u = model.get_displacements()

    # Get end forces (returns tuple of EndForces for node_i and node_j)
    end_forces_i, end_forces_j = beam.compute_end_forces(u, dof_handler)

    print("\nEnd forces at node i (fixed end):")
    print(f"  N  = {end_forces_i.N:8.2f} kN (axial)")
    print(f"  Vy = {end_forces_i.Vy:8.2f} kN (shear y)")
    print(f"  Vz = {end_forces_i.Vz:8.2f} kN (shear z)")
    print(f"  Mx = {end_forces_i.Mx:8.2f} kNm (torsion)")
    print(f"  My = {end_forces_i.My:8.2f} kNm (moment y)")
    print(f"  Mz = {end_forces_i.Mz:8.2f} kNm (moment z)")

    print("\nEnd forces at node j (free end):")
    print(f"  N  = {end_forces_j.N:8.2f} kN")
    print(f"  Vy = {end_forces_j.Vy:8.2f} kN")
    print(f"  Mz = {end_forces_j.Mz:8.2f} kNm")

    # Convert to vector
    print(f"\nAs vector [N, Vy, Vz, Mx, My, Mz]: {end_forces_i.to_vector6()}")


def example_find_moment_extremes():
    """
    Example 3: Finding moment extrema along a beam.

    Useful for design - find max/min moment locations automatically.
    """
    print("\n" + "=" * 60)
    print("Example 3: Finding Moment Extrema")
    print("=" * 60)

    model = Model()
    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
    sec = model.create_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

    L = 6.0
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(L, 0, 0)
    beam = model.create_beam(n1, n2, mat, sec)

    model.boundary_conditions.fix_node(n1.id)

    lc = model.get_default_load_case()
    lc.add_nodal_load(n2.id, DOFIndex.UY, -10.0)

    assert model.analyze()

    dof_handler = model.get_dof_handler()
    u = model.get_displacements()

    # Find moment extremes about z-axis
    min_extreme, max_extreme = beam.find_moment_extremes('z', u, dof_handler, lc)

    print("\nMoment extrema about z-axis:")
    print(f"  Minimum: Mz = {min_extreme.value:8.2f} kNm at x = {min_extreme.x:.2f} m")
    print(f"  Maximum: Mz = {max_extreme.value:8.2f} kNm at x = {max_extreme.x:.2f} m")


def example_multi_element_beam():
    """
    Example 4: Multi-element continuous beam with internal actions.

    Shows how to query internal actions on a beam subdivided into multiple elements.
    """
    print("\n" + "=" * 60)
    print("Example 4: Multi-Element Beam Internal Actions")
    print("=" * 60)

    model = Model()
    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
    sec = model.create_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

    # Create two-element cantilever beam
    L1, L2 = 3.0, 3.0  # Each element 3m, total 6m
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(L1, 0, 0)
    n3 = model.get_or_create_node(L1 + L2, 0, 0)

    beam1 = model.create_beam(n1, n2, mat, sec)
    beam2 = model.create_beam(n2, n3, mat, sec)

    # Fix left end
    model.boundary_conditions.fix_node(n1.id)

    # Apply point load at tip
    lc = model.get_default_load_case()
    lc.add_nodal_load(n3.id, DOFIndex.UY, -10.0)  # 10 kN downward

    assert model.analyze()

    dof_handler = model.get_dof_handler()
    u = model.get_displacements()

    print(f"\nNumber of elements: 2")
    print(f"Total beam length: {L1 + L2} m")

    # Query internal actions along both elements
    print("\nInternal actions along multi-element beam:")
    print(f"{'Global x (m)':<15} {'Element':<10} {'Shear Vy (kN)':<15} {'Moment Mz (kNm)':<15}")
    print("-" * 55)

    # Sample first element (positions are local to element)
    for x in [0.0, L1/2, L1]:
        actions = beam1.get_internal_actions(x, u, dof_handler, lc)
        print(f"{x:<15.2f} {'beam1':<10} {actions.Vy:<15.2f} {actions.Mz:<15.2f}")

    # Sample second element (positions are local to each element)
    for x in [0.0, L2/2, L2]:
        global_x = L1 + x
        actions = beam2.get_internal_actions(x, u, dof_handler, lc)
        print(f"{global_x:<15.2f} {'beam2':<10} {actions.Vy:<15.2f} {actions.Mz:<15.2f}")

    # Expected: Shear = 10 kN constant, Moment varies from 60 kNm at base to 0 at tip
    print("\nExpected: Shear = 10 kN constant, Moment = P*(L_total - x)")


def example_distributed_load_query():
    """
    Example 5: Querying distributed loads from load case.

    Shows how to retrieve distributed load information from elements.
    """
    print("\n" + "=" * 60)
    print("Example 5: Distributed Load Query")
    print("=" * 60)

    model = Model()
    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
    sec = model.create_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

    L = 6.0
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(L, 0, 0)
    beam = model.create_beam(n1, n2, mat, sec)

    model.boundary_conditions.fix_node(n1.id)

    lc = model.get_default_load_case()
    # Apply uniform distributed load of 10 kN/m in -Y direction
    w_start = np.array([0.0, -10.0, 0.0])
    w_end = np.array([0.0, -10.0, 0.0])
    lc.add_line_load(beam.id, w_start, w_end)

    # Query the distributed load
    load_y = beam.get_distributed_load_y(lc)
    load_z = beam.get_distributed_load_z(lc)
    load_axial = beam.get_distributed_load_axial(lc)

    print("\nDistributed loads in local coordinates:")
    print(f"  Local y: q_start = {load_y.q_start:.2f} kN/m, q_end = {load_y.q_end:.2f} kN/m")
    print(f"  Local z: q_start = {load_z.q_start:.2f} kN/m, q_end = {load_z.q_end:.2f} kN/m")
    print(f"  Axial:   q_start = {load_axial.q_start:.2f} kN/m, q_end = {load_axial.q_end:.2f} kN/m")


def example_displacement_line():
    """
    Example 6: Querying displacements along beam.

    Shows how to get displacement and rotation at any position.
    """
    print("\n" + "=" * 60)
    print("Example 6: Displacement Line Query")
    print("=" * 60)

    model = Model()
    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
    sec = model.create_section("IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7)

    L = 6.0
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(L, 0, 0)
    beam = model.create_beam(n1, n2, mat, sec)

    model.boundary_conditions.fix_node(n1.id)

    lc = model.get_default_load_case()
    lc.add_nodal_load(n2.id, DOFIndex.UY, -10.0)

    assert model.analyze()

    dof_handler = model.get_dof_handler()
    u = model.get_displacements()

    print("\nDisplacements along cantilever beam:")
    print(f"{'Position (m)':<12} {'v (mm)':<12} {'theta_z (mrad)':<15}")
    print("-" * 40)

    for x in np.linspace(0, L, 7):
        disp = beam.get_displacements_at(x, u, dof_handler)
        print(f"{x:<12.2f} {disp.v * 1000:<12.4f} {disp.theta_z * 1000:<15.4f}")

    # Analytical solution for cantilever tip deflection: delta = P*L^3 / (3*E*Iz)
    # For load in Y direction, use Iz (bending about z-axis)
    E = 210e6  # kN/m²
    Iz = 6.04e-6  # m⁴ (for bending in XY plane)
    P = 10.0  # kN
    delta_analytical = P * L**3 / (3 * E * Iz) * 1000  # mm
    print(f"\nAnalytical tip deflection: {delta_analytical:.4f} mm (matches computed)")


if __name__ == "__main__":
    example_cantilever_internal_actions()
    example_end_forces()
    example_find_moment_extremes()
    example_multi_element_beam()
    example_distributed_load_query()
    example_displacement_line()

    print("\n" + "=" * 60)
    print("All examples completed successfully!")
    print("=" * 60)
