======================
Verification Summary
======================

This page summarizes all verification tests and their results.

Overview
========

Grillex has been verified against analytical solutions for a range of
standard structural engineering problems. All tests are implemented as
doctests and are executed automatically during documentation builds.

Verification Methodology
========================

Each verification case follows this methodology:

1. **Problem Definition**: Clear statement of geometry, materials, loads
2. **Analytical Solution**: Reference to textbook formulas
3. **FEM Model**: Grillex model matching the analytical problem
4. **Comparison**: Quantitative comparison with error tolerance
5. **Pass Criteria**: Explicit error thresholds

Error Tolerances
----------------

Different tolerances are used based on problem complexity:

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Problem Type
     - Tolerance
     - Rationale
   * - Single element, exact formulation
     - < 0.1%
     - Should match exactly
   * - Distributed loads (multiple elements)
     - < 1%
     - Discretization error expected
   * - Continuous beams
     - < 2%
     - Complex stress redistribution
   * - Warping torsion (components)
     - < 10%
     - Rapid spatial variation in warping
   * - Warping torsion (total)
     - < 5%
     - Equilibrium constraint improves accuracy

Test Summary by Category
========================

Cantilever Beams
----------------

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Test Case
     - Verified Quantities
     - Status
   * - Point load at tip
     - Deflection, rotation
     - ✓ PASS
   * - Uniform distributed load
     - Deflection
     - ✓ PASS
   * - Moment at tip
     - Deflection, rotation
     - ✓ PASS

See :doc:`cantilever_beam` for detailed results.

Simply Supported Beams
----------------------

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Test Case
     - Verified Quantities
     - Status
   * - Central point load
     - Deflection, reactions
     - ✓ PASS
   * - Uniform distributed load
     - Deflection
     - ✓ PASS
   * - Asymmetric point load
     - Deflection, reactions
     - ✓ PASS

See :doc:`simply_supported_beam` for detailed results.

Continuous Beams
----------------

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Test Case
     - Verified Quantities
     - Status
   * - Two-span with UDL
     - Reactions
     - ✓ PASS
   * - Propped cantilever
     - Reactions
     - ✓ PASS
   * - Fixed-fixed beam
     - Deflection, reactions
     - ✓ PASS

See :doc:`continuous_beam` for detailed results.

Warping Torsion (SCI P385)
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 40 30 30

   * - Test Case
     - Verified Quantities
     - Status
   * - Cantilever with end torque
     - Bimoment B(x)
     - PASS
   * - Cantilever with end torque
     - Warping torsion Mx,w(x)
     - PASS
   * - Cantilever with end torque
     - St. Venant torsion Mx,sv(x)
     - PASS
   * - Cantilever with end torque
     - Total torsion Mx(x)
     - PASS
   * - Boundary conditions
     - 100% warping at fixed end
     - PASS
   * - Boundary conditions
     - 100% St. Venant at free end
     - PASS

See :doc:`warping_torsion` for detailed results.

Running Verification Tests
==========================

The verification tests can be run using Sphinx's doctest builder:

.. code-block:: bash

    cd docs
    make doctest

This executes all doctests in the documentation and reports any failures.

Alternatively, run from the project root:

.. code-block:: bash

    sphinx-build -b doctest docs docs/_build/doctest

Expected output for passing tests:

.. code-block:: text

    Doctest summary
    ===============
    X tests
    0 failures in tests
    0 failures in setup code
    0 failures in cleanup code

Continuous Integration
----------------------

These verification tests are run automatically in the CI/CD pipeline
to ensure code changes don't break verified behavior.

References
==========

The analytical solutions used for verification are based on standard
structural engineering references:

1. Timoshenko, S.P. & Gere, J.M. (1961). *Theory of Elastic Stability*
2. Ghali, A. & Neville, A.M. (2017). *Structural Analysis*
3. Hibbeler, R.C. (2018). *Structural Analysis*
4. Roark, R.J. & Young, W.C. (2012). *Roark's Formulas for Stress and Strain*
5. SCI P385 (2011). *Design of Steel Beams in Torsion*, Steel Construction Institute
6. Vlasov, V.Z. (1961). *Thin-Walled Elastic Beams*, 2nd Edition

Document History
================

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Date
     - Change
     - Author
   * - 2025-12-21
     - Initial verification documentation
     - Grillex Development Team

Adding New Verification Cases
=============================

When adding new features to Grillex, corresponding verification cases
should be added following these guidelines:

1. Create a new .rst file in ``docs/verification/``
2. Include problem description with all parameters
3. Document analytical formulas with LaTeX math
4. Implement as doctests with clear error checks
5. Add summary to this page
6. Update the toctree in ``docs/verification/index.rst``
