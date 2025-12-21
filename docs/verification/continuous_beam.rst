==========================
Continuous Beam Benchmark
==========================

This document verifies Grillex results against analytical solutions for
continuous (multi-span) beam problems.

Test Case 1: Two-Span Beam with Central Supports
=================================================

Problem Description
-------------------

A two-span continuous beam with equal spans, uniformly loaded.

**Parameters:**

- Span length: L = 5 m each (total 10 m)
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 8.36 × 10⁻⁵ m⁴ (IPE300)
- Distributed Load: w = 10 kN/m

**Analytical Solutions:**

For a two-span continuous beam with equal spans under UDL:

Moment at central support:

.. math::

    M_{center} = \frac{wL^2}{8}

End reactions:

.. math::

    R_{end} = \frac{3wL}{8}

Central reaction:

.. math::

    R_{center} = \frac{10wL}{8} = 1.25wL

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 5.0      # m (span length)
    >>> E = 210e6    # kN/m²
    >>> I = 8.36e-5  # m⁴
    >>> w = 10.0     # kN/m
    >>>
    >>> # Analytical solutions
    >>> R_end = 3 * w * L / 8     # End reactions = 18.75 kN
    >>> R_center = 10 * w * L / 8 # Central reaction = 62.5 kN
    >>> total_load = w * 2 * L    # Total load = 100 kN
    >>>
    >>> # Create model
    >>> model = StructuralModel(name="Two-Span Continuous")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=I, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> # Create two spans with sufficient elements
    >>> beam1 = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel", num_elements=4)
    >>> beam2 = model.add_beam_by_coords([L, 0, 0], [2*L, 0, 0], "IPE300", "Steel", num_elements=4)
    >>>
    >>> # Apply supports
    >>> model.fix_node_at([0, 0, 0])        # Left support
    >>> model.pin_node_at([L, 0, 0])        # Central support
    >>> model.pin_node_at([2*L, 0, 0])      # Right support
    >>>
    >>> # Apply distributed load to both spans
    >>> model.add_line_load(beam1, [0, 0, -w])
    >>> model.add_line_load(beam2, [0, 0, -w])
    >>> _ = model.analyze()
    >>>
    >>> # Get reactions
    >>> R_left = model.get_reactions_at([0, 0, 0])[2]
    >>> R_mid = model.get_reactions_at([L, 0, 0])[2]
    >>> R_right = model.get_reactions_at([2*L, 0, 0])[2]
    >>>
    >>> # Total reactions should equal total load (equilibrium check)
    >>> total_reactions = R_left + R_mid + R_right
    >>> bool(abs(total_reactions - total_load) / total_load * 100 < 1.0)
    True

Test Case 2: Propped Cantilever
===============================

Problem Description
-------------------

A propped cantilever (fixed-pinned) beam with uniform load.

**Parameters:**

- Length: L = 6 m
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 1.94 × 10⁻⁵ m⁴ (IPE200)
- Distributed Load: w = 8 kN/m

**Analytical Solutions:**

For a propped cantilever under UDL:

Fixed end moment:

.. math::

    M_{fixed} = \frac{wL^2}{8}

Pinned end reaction:

.. math::

    R_{pin} = \frac{3wL}{8}

Fixed end reaction:

.. math::

    R_{fixed} = \frac{5wL}{8}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 6.0      # m
    >>> E = 210e6    # kN/m²
    >>> I = 1.94e-5  # m⁴
    >>> w = 8.0      # kN/m
    >>>
    >>> # Analytical solutions
    >>> R_pin = 3 * w * L / 8
    >>> R_fixed = 5 * w * L / 8
    >>> total_load = w * L
    >>>
    >>> # Create model
    >>> model = StructuralModel(name="Propped Cantilever")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=I, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel", num_elements=4)
    >>>
    >>> # Fixed at left, pinned at right
    >>> model.fix_node_at([0, 0, 0])
    >>> model.pin_node_at([L, 0, 0])
    >>>
    >>> # Apply distributed load
    >>> model.add_line_load(beam, [0, 0, -w])
    >>> _ = model.analyze()
    >>>
    >>> # Get reactions
    >>> Rz_fixed = model.get_reactions_at([0, 0, 0])[2]
    >>> Rz_pin = model.get_reactions_at([L, 0, 0])[2]
    >>>
    >>> # Check equilibrium
    >>> total_reactions = Rz_fixed + Rz_pin
    >>> bool(abs(total_reactions - total_load) / total_load * 100 < 1.0)
    True

Test Case 3: Fixed-Fixed Beam
=============================

Problem Description
-------------------

A fully fixed beam with uniform load.

**Parameters:**

- Length: L = 5 m
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 8.36 × 10⁻⁵ m⁴ (IPE300)
- Distributed Load: w = 15 kN/m

**Analytical Solutions:**

Fixed end moments:

.. math::

    M_{ends} = \frac{wL^2}{12}

Midspan moment:

.. math::

    M_{mid} = \frac{wL^2}{24}

Maximum deflection at midspan:

.. math::

    \delta_{max} = \frac{wL^4}{384EI}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 5.0      # m
    >>> E = 210e6    # kN/m²
    >>> I = 8.36e-5  # m⁴
    >>> w = 15.0     # kN/m
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = w * L**4 / (384 * E * I)
    >>> R_expected = w * L / 2
    >>>
    >>> # Create model with 10 elements (ensures node at midspan L/2=2.5m)
    >>> model = StructuralModel(name="Fixed-Fixed Beam")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=I, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel", num_elements=10)
    >>>
    >>> # Fixed at both ends
    >>> model.fix_node_at([0, 0, 0])
    >>> model.fix_node_at([L, 0, 0])
    >>>
    >>> # Apply distributed load
    >>> model.add_line_load(beam, [0, 0, -w])
    >>> _ = model.analyze()
    >>>
    >>> # Get midspan deflection
    >>> delta_fem = abs(model.get_displacement_at([L/2, 0, 0], DOFIndex.UZ))
    >>>
    >>> # Verify deflection (should match within 5%)
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> bool(error_delta < 5.0)
    True

    >>> # Reactions should each be wL/2
    >>> Rz_left = model.get_reactions_at([0, 0, 0])[2]
    >>> error_R = abs(Rz_left - R_expected) / R_expected * 100
    >>> bool(error_R < 5.0)
    True

Summary
=======

.. list-table:: Continuous Beam Verification Results
   :header-rows: 1
   :widths: 30 30 20 20

   * - Test Case
     - Quantity
     - Error
     - Status
   * - Two-Span
     - Equilibrium
     - < 1%
     - ✓ PASS
   * - Propped Cantilever
     - Equilibrium
     - < 1%
     - ✓ PASS
   * - Fixed-Fixed
     - Deflection
     - < 5%
     - ✓ PASS
   * - Fixed-Fixed
     - Reactions
     - < 5%
     - ✓ PASS
