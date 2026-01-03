======================================
Warping Torsion Verification (SCI P385)
======================================

This document verifies Grillex warping torsion results against analytical solutions
from SCI P385 "Design of Steel Beams in Torsion" (Steel Construction Institute).

Background
==========

For thin-walled open sections (I-beams, channels), torsion is resisted by two mechanisms:

1. **St. Venant torsion** (uniform torsion): Resisted by the cross-section's torsional stiffness GJ
2. **Warping torsion** (non-uniform torsion): Resisted by warping restraint at supports, characterised by EIw

The total torsion is the sum of both components:

.. math::

    M_x = T_t + T_w

where (using SCI P385 notation):

- :math:`T_t` = St. Venant torsion component [kN·m]
- :math:`T_w` = Warping torsion component [kN·m]
- :math:`M_x` = Total torsion (constant along beam for point torque at tip)

.. note::

    **Sign conventions:**

    - Grillex uses :math:`M_x = M_{x,sv} + M_{x,w}` where :math:`M_{x,w} = -T_w`
    - Grillex uses :math:`B = -EI_w \cdot d^2\phi/dx^2` (opposite to SCI P385)

    The verification code accounts for these sign differences when comparing results.

Test Case: SCI P385 Figure 2.7 - Cantilever Under End Torque
============================================================

Problem Description
-------------------

A cantilever I-beam with:

- Fixed end (warping restrained) at x = 0
- Free end at x = L
- Torque T applied at free end

.. figure:: /_static/sci_p385_fig27.png
   :alt: Cantilever beam with end torque
   :width: 400px
   :align: center

   SCI P385 Figure 2.7: Cantilever with warping restrained at fixed end

**Parameters:**

- Length: L = 6 m
- Section: IPE300
- Young's Modulus: E = 210 × 10⁶ kN/m²
- Shear Modulus: G = 80.77 × 10⁶ kN/m² (calculated from E/(2(1+ν)))
- Torsional Constant: J = 2.01 × 10⁻⁷ m⁴
- Warping Constant: Iw = 1.26 × 10⁻⁷ m⁶
- Applied Torque: T = 1.0 kNm

Analytical Solutions (SCI P385)
-------------------------------

The torsion parameter :math:`a` is defined as:

.. math::

    a = \sqrt{\frac{EI_w}{GJ}}

For the test case:

.. math::

    a = \sqrt{\frac{210 \times 10^6 \times 1.26 \times 10^{-7}}{80.77 \times 10^6 \times 2.01 \times 10^{-7}}} = 1.277 \text{ m}

    \frac{L}{a} = \frac{6}{1.277} = 4.70

For the cantilever with warping restraint at the fixed end (case "Fixed-Free" in SCI P385
Table 2.2), the analytical solutions are:

**St. Venant Torsion:**

.. math::

    T_t(x) = T \cdot \left( \tanh\frac{L}{a} \cdot \sinh\frac{x}{a} - \cosh\frac{x}{a} + 1 \right)

**Warping Torsion:**

.. math::

    T_w(x) = T \cdot \left( \tanh\frac{L}{a} \cdot \sinh\frac{x}{a} - \cosh\frac{x}{a} \right)

**Bimoment:**

.. math::

    B(x) = Ta \cdot \left( \tanh\frac{L}{a} \cdot \cosh\frac{x}{a} - \sinh\frac{x}{a} \right)

**Total Torsion:**

.. math::

    M_x(x) = T_t(x) - T_w(x) = T

.. note::

    SCI P385 uses the convention :math:`M_x = T_t - T_w` (note the minus sign).
    The total torsion equals the applied torque T at all positions along the beam.

Key Observations
----------------

At the **fixed end** (x = 0):

- Warping is fully restrained (no warping displacement)
- 100% of the torque is carried by warping torsion: :math:`T_w = -T`
- St. Venant component is zero: :math:`T_t = 0`
- Maximum bimoment: :math:`B_{max} = Ta \cdot \tanh(L/a) \approx Ta` (for large L/a)

At the **free end** (x = L):

- Warping is free (bimoment = 0)
- Nearly all torsion is carried by St. Venant (for large L/a)
- Zero bimoment: :math:`B = 0`

Verification: Torsion Split Comparison
--------------------------------------

The following doctest creates a Grillex model and compares the FEM results
with the SCI P385 analytical solutions at 10% intervals along the beam:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>> import numpy as np
    >>>
    >>> # Problem parameters (SCI P385 Figure 2.7)
    >>> L = 6.0            # m
    >>> E = 210e6          # kN/m²
    >>> nu = 0.3
    >>> G = E / (2 * (1 + nu))  # kN/m²
    >>> J = 2.01e-7        # m⁴
    >>> Iw = 1.26e-7       # m⁶
    >>> T = 1.0            # kNm
    >>>
    >>> # Torsion parameter (SCI P385 notation)
    >>> a = np.sqrt(E * Iw / (G * J))
    >>> La = L / a
    >>> tanh_La = np.tanh(La)
    >>>
    >>> print(f"Torsion parameter a = {a:.4f} m")
    Torsion parameter a = 1.2767 m
    >>> print(f"L/a = {La:.2f}")
    L/a = 4.70
    >>>
    >>> # Create and analyze Grillex model
    >>> model = StructuralModel(name="SCI P385 Cantilever Torsion")
    >>> _ = model.add_material("Steel", E=E, nu=nu, rho=7.85e-6)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=J, Iw=Iw)
    >>>
    >>> beam = model.add_beam_by_coords(
    ...     [0, 0, 0], [L, 0, 0], "IPE300", "Steel",
    ...     subdivisions=20,
    ...     warping_enabled=True
    ... )
    >>> model.fix_node_at([0, 0, 0], include_warping=True)
    >>> model.add_point_load([L, 0, 0], moment=[T, 0, 0])
    >>> _ = model.analyze()
    >>>
    >>> # Get internals for warping actions
    >>> dof_handler = model._cpp_model.get_dof_handler()
    >>> load_case = model._cpp_model.get_default_load_case()
    >>> displacements = model.get_all_displacements(load_case)
    >>>
    >>> # Print comparison table header
    >>> print()
    <BLANKLINE>
    >>> print("="*95)
    ===============================================================================================
    >>> print("TORSION SPLIT COMPARISON: SCI P385 Figure 2.7 vs Grillex FEM")
    TORSION SPLIT COMPARISON: SCI P385 Figure 2.7 vs Grillex FEM
    >>> print("="*95)
    ===============================================================================================
    >>> print(f"{'x/L':>5} | {'Tt_ana':>7} {'Tt_fem':>7} {'err':>5} | {'|Tw|_ana':>8} {'|Tw|_fem':>8} {'err':>5} | {'B_ana':>7} {'B_fem':>7} {'err':>5}")
      x/L |  Tt_ana  Tt_fem   err | |Tw|_ana |Tw|_fem   err |   B_ana   B_fem   err
    >>> print("-"*95)
    -----------------------------------------------------------------------------------------------
    >>>
    >>> max_Tt_err = 0.0
    >>> max_Tw_err = 0.0
    >>> max_B_err = 0.0
    >>>
    >>> for pct in range(0, 101, 10):  # doctest: +NORMALIZE_WHITESPACE
    ...     x = L * pct / 100.0
    ...     xa = x / a
    ...     sinh_xa, cosh_xa = np.sinh(xa), np.cosh(xa)
    ...
    ...     # Analytical (SCI P385) - note: B sign flipped for Grillex convention
    ...     Tt_ana = T * (tanh_La * sinh_xa - cosh_xa + 1)
    ...     Tw_ana = T * (tanh_La * sinh_xa - cosh_xa)
    ...     B_ana = -T * a * (tanh_La * cosh_xa - sinh_xa)  # Negative for Grillex
    ...
    ...     # FEM (Grillex)
    ...     x_query = min(x, L - 0.001) if pct == 100 else max(x, 0.001)
    ...     element, local_x = beam._find_element_at_position(x_query)
    ...     actions = element.get_warping_internal_actions(local_x, displacements, dof_handler)
    ...
    ...     Tt_fem = actions.Mx_sv
    ...     Tw_fem = -actions.Mx_w  # Grillex: Mx_w = -Tw
    ...     B_fem = actions.B
    ...
    ...     # Errors (% of T for torsion, % of Ta for bimoment)
    ...     Tt_err = abs(Tt_fem - Tt_ana) / T * 100
    ...     Tw_err = abs(abs(Tw_fem) - abs(Tw_ana)) / T * 100
    ...     B_err = abs(B_fem - B_ana) / abs(T * a) * 100 if abs(B_ana) > 1e-6 else 0.0
    ...
    ...     max_Tt_err = max(max_Tt_err, Tt_err)
    ...     max_Tw_err = max(max_Tw_err, Tw_err)
    ...     max_B_err = max(max_B_err, B_err)
    ...
    ...     print(f"{pct:>4}% | {Tt_ana:>7.3f} {Tt_fem:>7.3f} {Tt_err:>4.1f}% | {abs(Tw_ana):>8.3f} {abs(Tw_fem):>8.3f} {Tw_err:>4.1f}% | {B_ana:>7.4f} {B_fem:>7.4f} {B_err:>4.1f}%")
       0% |   0.000   0.001  0.1% |    1.000    1.000  0.0% | -1.2764 -1.2779  0.1%
      10% |   0.375   0.376  0.1% |    0.625    0.625  0.0% | -0.7977 -0.7990  0.1%
      20% |   0.609   0.610  0.1% |    0.391    0.390  0.0% | -0.4984 -0.4987  0.0%
      30% |   0.756   0.756  0.1% |    0.244    0.244  0.1% | -0.3112 -0.3111  0.0%
      40% |   0.847   0.848  0.1% |    0.153    0.153  0.1% | -0.1941 -0.1938  0.0%
      50% |   0.904   0.904  0.1% |    0.096    0.096  0.0% | -0.1206 -0.1203  0.0%
      60% |   0.939   0.939  0.0% |    0.061    0.061  0.0% | -0.0743 -0.0741  0.0%
      70% |   0.961   0.961  0.0% |    0.039    0.039  0.0% | -0.0447 -0.0445  0.0%
      80% |   0.973   0.973  0.0% |    0.027    0.027  0.0% | -0.0252 -0.0251  0.0%
      90% |   0.980   0.980  0.0% |    0.020    0.020  0.0% | -0.0113 -0.0113  0.0%
     100% |   0.982   0.982  0.0% |    0.018    0.018  0.0% | -0.0000 -0.0000  0.0%
    >>>
    >>> print()
    <BLANKLINE>
    >>> print("="*95)
    ===============================================================================================
    >>> print("MAXIMUM ERRORS:")
    MAXIMUM ERRORS:
    >>> print(f"  St. Venant Torsion Tt: {max_Tt_err:.2f}%")  # doctest: +ELLIPSIS
      St. Venant Torsion Tt: ...%
    >>> print(f"  Warping Torsion |Tw|:  {max_Tw_err:.2f}%")  # doctest: +ELLIPSIS
      Warping Torsion |Tw|:  ...%
    >>> print(f"  Bimoment B:            {max_B_err:.2f}%")  # doctest: +ELLIPSIS
      Bimoment B:            ...%
    >>>
    >>> # Verify errors are within tolerance
    >>> assert max_Tt_err < 10.0, f"St. Venant torsion error {max_Tt_err:.2f}% exceeds 10%"
    >>> assert max_Tw_err < 10.0, f"Warping torsion error {max_Tw_err:.2f}% exceeds 10%"
    >>> assert max_B_err < 10.0, f"Bimoment error {max_B_err:.2f}% exceeds 10%"
    >>> print("\nAll verification checks PASSED!")
    <BLANKLINE>
    All verification checks PASSED!

Physical Interpretation
-----------------------

The results confirm the expected behaviour:

1. **At the fixed end** (x = 0):

   - Warping is fully restrained (φ' = 0)
   - 100% of the torque is carried by warping torsion
   - Maximum bimoment occurs here

2. **At the free end** (x = L):

   - Warping is free (bimoment = 0)
   - Nearly all torsion is carried by St. Venant torsion
   - The section can freely warp

3. **Along the beam**:

   - Gradual transition from warping to St. Venant dominance
   - The transition rate depends on the parameter L/a
   - Larger L/a means faster transition (longer beam or lower warping stiffness)
   - For L/a = 4.7, St. Venant already dominates at 20% of the span (61% Tt)

C++ Test Reference
------------------

The corresponding C++ tests are located in:

``tests/cpp/test_internal_actions.cpp``

- ``SCI P385 Figure 2.7 - Torsion split at 10% intervals``
- ``SCI P385 - Torsion split boundary conditions verification``

These tests verify:

1. Bimoment matches analytical solution within 10% at all points
2. Warping torsion matches within 10%
3. St. Venant torsion matches within 10%
4. Total torsion matches within 5% (more accurate due to equilibrium)
5. At fixed end: ~100% warping torsion (within 2%)
6. At free end: ~95-100% St. Venant torsion (within 5%)

Summary
=======

.. list-table:: Warping Torsion Verification Results
   :header-rows: 1
   :widths: 40 30 20

   * - Quantity
     - Maximum Error
     - Status
   * - Bimoment B(x)
     - 0.1%
     - PASS
   * - Warping Torsion |Tw|(x)
     - 0.1%
     - PASS
   * - St. Venant Torsion Tt(x)
     - 0.1%
     - PASS
   * - Total torsion equilibrium
     - Maintained
     - PASS

References
==========

1. SCI P385 "Design of Steel Beams in Torsion", Steel Construction Institute, 2011
2. Vlasov, V.Z. "Thin-Walled Elastic Beams", 2nd Edition, 1961
3. Trahair, N.S. "Flexural-Torsional Buckling of Structures", CRC Press, 1993
