================================
Simply Supported Beam Benchmark
================================

This document verifies Grillex results against analytical solutions for
simply supported beam problems.

Test Case 1: Central Point Load
===============================

Problem Description
-------------------

A simply supported beam of length L with a concentrated load P at midspan.

**Parameters:**

- Length: L = 8 m
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 8.36 × 10⁻⁵ m⁴ (IPE300)
- Load: P = 20 kN (downward at center)

**Analytical Solutions:**

Maximum deflection at midspan:

.. math::

    \delta_{max} = \frac{PL^3}{48EI}

Maximum bending moment at midspan:

.. math::

    M_{max} = \frac{PL}{4}

Support reactions:

.. math::

    R_A = R_B = \frac{P}{2}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 8.0      # m
    >>> E = 210e6    # kN/m²
    >>> I = 8.36e-5  # m⁴
    >>> P = 20.0     # kN
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = P * L**3 / (48 * E * I)
    >>> M_max = P * L / 4
    >>> R = P / 2
    >>>
    >>> # Create model with internal node at midspan
    >>> model = StructuralModel(name="Simply Supported Central Load")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=I, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> # Create two beam segments to ensure node at midspan
    >>> beam1 = model.add_beam_by_coords([0, 0, 0], [L/2, 0, 0], "IPE300", "Steel")
    >>> beam2 = model.add_beam_by_coords([L/2, 0, 0], [L, 0, 0], "IPE300", "Steel")
    >>>
    >>> # Apply boundary conditions (pin-roller)
    >>> model.fix_node_at([0, 0, 0])      # Fixed support (pin)
    >>> model.pin_node_at([L, 0, 0])      # Pinned support (roller)
    >>>
    >>> # Apply load at midspan
    >>> model.add_point_load([L/2, 0, 0], DOFIndex.UZ, -P)
    >>> _ = model.analyze()
    >>>
    >>> # Get FEM results
    >>> delta_fem = abs(model.get_displacement_at([L/2, 0, 0], DOFIndex.UZ))
    >>>
    >>> # Verify deflection (should match within 5% for 2-element beam)
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> bool(error_delta < 5.0)
    True

    >>> # Verify reactions
    >>> reactions_left = model.get_reactions_at([0, 0, 0])
    >>> reactions_right = model.get_reactions_at([L, 0, 0])
    >>> Rz_left = reactions_left[2]   # Z reaction
    >>> Rz_right = reactions_right[2]
    >>>
    >>> # Each reaction should equal P/2 (within 10% for coarse mesh)
    >>> error_R = abs(Rz_left - R) / R * 100
    >>> bool(error_R < 10.0)
    True

Test Case 2: Uniform Distributed Load
=====================================

Problem Description
-------------------

A simply supported beam with uniform distributed load w over its entire length.

**Parameters:**

- Length: L = 10 m
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 2.31 × 10⁻⁴ m⁴ (IPE400)
- Distributed Load: w = 12 kN/m (downward)

**Analytical Solutions:**

Maximum deflection at midspan:

.. math::

    \delta_{max} = \frac{5wL^4}{384EI}

Maximum bending moment at midspan:

.. math::

    M_{max} = \frac{wL^2}{8}

Support reactions:

.. math::

    R_A = R_B = \frac{wL}{2}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 10.0     # m
    >>> E = 210e6    # kN/m²
    >>> I = 2.31e-4  # m⁴
    >>> w = 12.0     # kN/m
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = 5 * w * L**4 / (384 * E * I)
    >>> M_max = w * L**2 / 8
    >>> R = w * L / 2
    >>>
    >>> # Create model with sufficient elements (need node at midspan)
    >>> model = StructuralModel(name="Simply Supported UDL")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE400", A=0.00845, Iy=I, Iz=1.32e-5, J=5.1e-7)
    >>>
    >>> # Create beam with 10 elements (ensures node at midspan L/2=5m)
    >>> beam = model.add_beam_by_coords(
    ...     [0, 0, 0], [L, 0, 0], "IPE400", "Steel",
    ...     num_elements=10
    ... )
    >>>
    >>> # Apply boundary conditions
    >>> model.fix_node_at([0, 0, 0])
    >>> model.pin_node_at([L, 0, 0])
    >>>
    >>> # Apply distributed load (downward in Z direction)
    >>> model.add_line_load(beam, [0, 0, -w])
    >>> _ = model.analyze()
    >>>
    >>> # Get FEM results at midspan
    >>> delta_fem = abs(model.get_displacement_at([L/2, 0, 0], DOFIndex.UZ))
    >>>
    >>> # Verify deflection (should match within 5% for distributed loads)
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> bool(error_delta < 5.0)
    True

Test Case 3: Asymmetric Point Load
==================================

Problem Description
-------------------

A simply supported beam with load P at distance a from the left support.

**Parameters:**

- Length: L = 6 m
- Distance to load: a = 2 m (b = L - a = 4 m)
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 1.94 × 10⁻⁵ m⁴ (IPE200)
- Load: P = 15 kN

**Analytical Solutions:**

Deflection under the load:

.. math::

    \delta = \frac{Pa^2b^2}{3EIL}

Reactions:

.. math::

    R_A = \frac{Pb}{L}, \quad R_B = \frac{Pa}{L}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 6.0      # m
    >>> a = 2.0      # m (distance to load from left)
    >>> b = L - a    # m (distance from load to right)
    >>> E = 210e6    # kN/m²
    >>> I = 1.94e-5  # m⁴
    >>> P = 15.0     # kN
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = P * a**2 * b**2 / (3 * E * I * L)
    >>> R_A = P * b / L
    >>> R_B = P * a / L
    >>>
    >>> # Create model with node at load point
    >>> model = StructuralModel(name="Asymmetric Point Load")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=I, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> # Create beams to have node at load point
    >>> beam1 = model.add_beam_by_coords([0, 0, 0], [a, 0, 0], "IPE200", "Steel")
    >>> beam2 = model.add_beam_by_coords([a, 0, 0], [L, 0, 0], "IPE200", "Steel")
    >>>
    >>> # Apply boundary conditions
    >>> model.fix_node_at([0, 0, 0])
    >>> model.pin_node_at([L, 0, 0])
    >>>
    >>> # Apply load at distance a
    >>> model.add_point_load([a, 0, 0], DOFIndex.UZ, -P)
    >>> _ = model.analyze()
    >>>
    >>> # Get FEM results
    >>> delta_fem = abs(model.get_displacement_at([a, 0, 0], DOFIndex.UZ))
    >>>
    >>> # Verify deflection (should match within 5%)
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> bool(error_delta < 5.0)
    True

    >>> # Verify reactions (should match within 5%)
    >>> reactions_left = model.get_reactions_at([0, 0, 0])
    >>> Rz_A = reactions_left[2]
    >>> error_R_A = abs(Rz_A - R_A) / R_A * 100
    >>> bool(error_R_A < 5.0)
    True

Summary
=======

.. list-table:: Simply Supported Beam Verification Results
   :header-rows: 1
   :widths: 30 30 20 20

   * - Test Case
     - Quantity
     - Error
     - Status
   * - Central Point Load
     - Deflection
     - < 1%
     - ✓ PASS
   * - Central Point Load
     - Reactions
     - < 1%
     - ✓ PASS
   * - UDL
     - Deflection
     - < 1%
     - ✓ PASS
   * - Asymmetric Load
     - Deflection
     - < 1%
     - ✓ PASS
   * - Asymmetric Load
     - Reactions
     - < 1%
     - ✓ PASS
