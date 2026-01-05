=========================
Cantilever Beam Benchmark
=========================

This document verifies Grillex results against analytical solutions for
cantilever beam problems.

Test Case 1: Point Load at Free End
===================================

Problem Description
-------------------

A cantilever beam of length L with a concentrated vertical load P at the free end.

**Parameters:**

- Length: L = 6 m
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 8.36 × 10⁻⁵ m⁴ (IPE300 about major axis)
- Load: P = 10 kN (downward)

**Analytical Solutions:**

Maximum deflection at free end:

.. math::

    \delta_{max} = \frac{PL^3}{3EI}

Maximum rotation at free end:

.. math::

    \theta_{max} = \frac{PL^2}{2EI}

Support reaction moment:

.. math::

    M_{support} = PL

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 6.0      # m
    >>> E = 210e6    # kN/m²
    >>> I = 8.36e-5  # m⁴
    >>> P = 10.0     # kN
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = P * L**3 / (3 * E * I)
    >>> theta_analytical = P * L**2 / (2 * E * I)
    >>> M_support = P * L
    >>>
    >>> # Create and analyze model
    >>> model = StructuralModel(name="Cantilever Point Load")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=I, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_point_load([L, 0, 0], force=[0, 0, -P])
    >>> _ = model.analyze()
    >>>
    >>> # Get FEM results
    >>> delta_fem = abs(model.get_displacement_at([L, 0, 0], DOFIndex.UZ))
    >>> theta_fem = abs(model.get_displacement_at([L, 0, 0], DOFIndex.RY))
    >>>
    >>> # Verify deflection (should match within 0.1%)
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> error_delta < 0.1
    True

    >>> # Verify rotation (should match within 0.1%)
    >>> error_theta = abs(theta_fem - theta_analytical) / theta_analytical * 100
    >>> error_theta < 0.1
    True

    >>> # Print results for reference
    >>> print(f"Deflection: analytical={delta_analytical:.6f} m, FEM={delta_fem:.6f} m")  # doctest: +ELLIPSIS
    Deflection: analytical=0.041... m, FEM=0.041... m

Test Case 2: Uniform Distributed Load
=====================================

Problem Description
-------------------

A cantilever beam with uniform distributed load w along its entire length.

**Parameters:**

- Length: L = 5 m
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Moment of Inertia: I = 1.94 × 10⁻⁵ m⁴ (IPE200)
- Distributed Load: w = 8 kN/m (downward)

**Analytical Solutions:**

Maximum deflection at free end:

.. math::

    \delta_{max} = \frac{wL^4}{8EI}

Maximum moment at support:

.. math::

    M_{max} = \frac{wL^2}{2}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 5.0      # m
    >>> E = 210e6    # kN/m²
    >>> I = 1.94e-5  # m⁴
    >>> w = 8.0      # kN/m
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = w * L**4 / (8 * E * I)
    >>> M_max = w * L**2 / 2
    >>>
    >>> # Create and analyze model (use multiple elements for distributed load accuracy)
    >>> model = StructuralModel(name="Cantilever UDL")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=I, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> beam = model.add_beam_by_coords(
    ...     [0, 0, 0], [L, 0, 0], "IPE200", "Steel",
    ...     subdivisions=4  # Subdivide for better accuracy
    ... )
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_line_load(beam, [0, 0, -w])
    >>> _ = model.analyze()
    >>>
    >>> # Get FEM results
    >>> delta_fem = abs(model.get_displacement_at([L, 0, 0], DOFIndex.UZ))
    >>>
    >>> # Verify deflection (should match within 1% for 4 elements)
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> error_delta < 1.0
    True

    >>> # Print results for reference
    >>> print(f"Deflection: analytical={delta_analytical:.6f} m, FEM={delta_fem:.6f} m")  # doctest: +ELLIPSIS
    Deflection: analytical=0.153... m, FEM=0.153... m

Test Case 3: Moment at Free End
===============================

Problem Description
-------------------

A cantilever beam with a concentrated moment M₀ at the free end.

**Analytical Solutions:**

Deflection at free end:

.. math::

    \delta = \frac{M_0 L^2}{2EI}

Rotation at free end:

.. math::

    \theta = \frac{M_0 L}{EI}

Verification
------------

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>> import numpy as np
    >>>
    >>> # Problem parameters
    >>> L = 4.0      # m
    >>> E = 210e6    # kN/m²
    >>> I = 8.36e-5  # m⁴
    >>> M0 = 50.0    # kNm (applied moment)
    >>>
    >>> # Analytical solutions
    >>> delta_analytical = M0 * L**2 / (2 * E * I)
    >>> theta_analytical = M0 * L / (E * I)
    >>>
    >>> # Create and analyze model
    >>> model = StructuralModel(name="Cantilever Moment")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=I, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_point_load([L, 0, 0], moment=[0, M0, 0])  # Positive moment about Y
    >>> _ = model.analyze()
    >>>
    >>> # Get FEM results (moment causes upward deflection for positive My)
    >>> delta_fem = abs(model.get_displacement_at([L, 0, 0], DOFIndex.UZ))
    >>> theta_fem = abs(model.get_displacement_at([L, 0, 0], DOFIndex.RY))
    >>>
    >>> # Verify deflection
    >>> error_delta = abs(delta_fem - delta_analytical) / delta_analytical * 100
    >>> error_delta < 0.1
    True

    >>> # Verify rotation
    >>> error_theta = abs(theta_fem - theta_analytical) / theta_analytical * 100
    >>> error_theta < 0.1
    True

Summary
=======

.. list-table:: Cantilever Beam Verification Results
   :header-rows: 1
   :widths: 30 30 20 20

   * - Test Case
     - Quantity
     - Error
     - Status
   * - Point Load
     - Deflection
     - < 0.1%
     - ✓ PASS
   * - Point Load
     - Rotation
     - < 0.1%
     - ✓ PASS
   * - UDL
     - Deflection
     - < 1%
     - ✓ PASS
   * - Moment
     - Deflection
     - < 0.1%
     - ✓ PASS
   * - Moment
     - Rotation
     - < 0.1%
     - ✓ PASS
