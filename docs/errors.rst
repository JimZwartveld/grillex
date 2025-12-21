=============================
Error and Warning Reference
=============================

This chapter provides a comprehensive reference for all error codes and warnings in Grillex. These structured error messages help both human users and LLM agents diagnose and fix modeling issues.

Overview
========

Grillex uses a structured error handling system with:

- **Error codes**: Machine-readable codes for programmatic handling
- **Human-readable messages**: Clear descriptions of what went wrong
- **Diagnostic context**: Involved nodes, elements, and DOFs
- **Suggestions**: Recommended fixes for common issues

Errors indicate failures that prevent analysis from completing, while warnings indicate potential issues that may affect result accuracy.

Error Codes
===========

Errors are returned when analysis cannot proceed. Each error has a unique code number for programmatic handling.

Structural/Constraint Errors (100-199)
--------------------------------------

These errors indicate problems with the structural system's constraints or stability.

.. list-table::
   :header-rows: 1
   :widths: 10 25 65

   * - Code
     - Name
     - Description
   * - 100
     - ``UNCONSTRAINED_SYSTEM``
     - System has unconstrained degrees of freedom (rigid body modes). The model can move freely without resistance. **Fix**: Add boundary conditions to prevent rigid body motion.
   * - 101
     - ``SINGULAR_MATRIX``
     - Stiffness matrix is singular and cannot be inverted. Often caused by insufficient constraints or zero-stiffness elements. **Fix**: Check boundary conditions and element properties.
   * - 102
     - ``INSUFFICIENT_CONSTRAINTS``
     - Not enough boundary conditions to prevent all rigid body modes. **Fix**: Ensure at least 6 DOFs are constrained (or 3 for 2D problems).
   * - 103
     - ``REDUNDANT_CONSTRAINTS``
     - Conflicting or over-constraining boundary conditions. **Fix**: Review constraints for conflicts (e.g., same DOF fixed to different values).

Element Errors (200-299)
------------------------

These errors indicate problems with element definitions.

.. list-table::
   :header-rows: 1
   :widths: 10 25 65

   * - Code
     - Name
     - Description
   * - 200
     - ``INVALID_ELEMENT``
     - Invalid element definition (e.g., zero-length beam). **Fix**: Check element geometry.
   * - 201
     - ``INVALID_MATERIAL``
     - Element has invalid or missing material. **Fix**: Assign a valid material with positive properties.
   * - 202
     - ``INVALID_SECTION``
     - Element has invalid or missing section. **Fix**: Assign a valid section with positive properties.
   * - 203
     - ``INVALID_NODE_REFERENCE``
     - Element references a non-existent node. **Fix**: Ensure all nodes exist before creating elements.
   * - 204
     - ``INVALID_PROPERTY``
     - Element property is invalid (e.g., negative area). **Fix**: Check property values are positive.
   * - 205
     - ``INVALID_ELEMENT_STIFFNESS``
     - Element stiffness matrix is not positive definite. **Fix**: Check material/section properties.

Load Errors (300-399)
---------------------

These errors indicate problems with load definitions.

.. list-table::
   :header-rows: 1
   :widths: 10 25 65

   * - Code
     - Name
     - Description
   * - 300
     - ``INVALID_LOAD_NODE``
     - Load references a non-existent node. **Fix**: Apply loads only to existing nodes.
   * - 301
     - ``INVALID_LOAD_ELEMENT``
     - Load references a non-existent element. **Fix**: Apply distributed loads only to existing elements.
   * - 302
     - ``EMPTY_LOAD_CASE``
     - Load case contains no loads. **Fix**: Add at least one load to the load case.
   * - 303
     - ``INVALID_LOAD_COMBINATION``
     - Load combination references a non-existent load case. **Fix**: Ensure all referenced load cases exist.

Model Errors (400-499)
----------------------

These errors indicate problems with the overall model structure.

.. list-table::
   :header-rows: 1
   :widths: 10 25 65

   * - Code
     - Name
     - Description
   * - 400
     - ``EMPTY_MODEL``
     - Model has no elements. **Fix**: Add at least one element (beam, spring, plate).
   * - 401
     - ``NO_NODES``
     - Model has no nodes. **Fix**: Create nodes before building elements.
   * - 402
     - ``DISCONNECTED_MODEL``
     - Model has disconnected parts. **Fix**: Connect all parts or analyze separately.
   * - 403
     - ``NOT_ANALYZED``
     - Results queried before running analysis. **Fix**: Call ``model.analyze()`` first.

Solver Errors (500-599)
-----------------------

These errors indicate problems during the solution process.

.. list-table::
   :header-rows: 1
   :widths: 10 25 65

   * - Code
     - Name
     - Description
   * - 500
     - ``SOLVER_CONVERGENCE_FAILED``
     - Iterative solver failed to converge. **Fix**: Try a different solver or refine the mesh.
   * - 501
     - ``NUMERICAL_OVERFLOW``
     - Numerical overflow during computation. **Fix**: Check for extreme property values or loads.
   * - 502
     - ``OUT_OF_MEMORY``
     - Insufficient memory for analysis. **Fix**: Reduce model size or use sparse solvers.


Warning Codes
=============

Warnings don't prevent analysis but indicate potential issues that may affect result accuracy.

Warning Severity Levels
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 15 85

   * - Severity
     - Meaning
   * - **Low**
     - Minor issue, likely acceptable. Review if unexpected.
   * - **Medium**
     - Potentially problematic. Review recommended.
   * - **High**
     - Likely indicates a modeling error. Investigation strongly recommended.

Geometry Warnings (100-199)
---------------------------

.. list-table::
   :header-rows: 1
   :widths: 10 25 15 50

   * - Code
     - Name
     - Severity
     - Description
   * - 100
     - ``EXTREME_ASPECT_RATIO``
     - Medium
     - Beam has extreme aspect ratio (L/h > 100 or < 2). Very slender beams may need Euler-Bernoulli formulation; very deep beams need Timoshenko.
   * - 101
     - ``SMALL_ELEMENT``
     - Medium
     - Very short element (may cause numerical issues). Consider merging with adjacent element.
   * - 102
     - ``LARGE_ELEMENT``
     - Low
     - Very long element. May need subdivision for accurate internal force distribution.
   * - 103
     - ``NON_COLLINEAR_WARPING``
     - Low
     - Non-collinear beams share warping DOF. May be intentional for complex connections.

Stiffness Warnings (200-299)
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 10 25 15 50

   * - Code
     - Name
     - Severity
     - Description
   * - 200
     - ``STIFFNESS_CONTRAST``
     - High
     - Large stiffness contrast between adjacent elements (ratio > 1e6). May cause ill-conditioning.
   * - 201
     - ``NEAR_SINGULARITY``
     - High
     - Stiffness matrix is poorly conditioned (condition number > 1e12). Results may be inaccurate.
   * - 202
     - ``VERY_STIFF_SPRING``
     - Medium
     - Spring stiffness is very high. May cause numerical issues similar to rigid links.
   * - 203
     - ``VERY_SOFT_SPRING``
     - Low
     - Spring stiffness is very low. May not provide effective restraint.

Property Warnings (300-399)
---------------------------

.. list-table::
   :header-rows: 1
   :widths: 10 25 15 50

   * - Code
     - Name
     - Severity
     - Description
   * - 300
     - ``NEAR_ZERO_PROPERTY``
     - High
     - Near-zero property value (e.g., very small area). May indicate modeling error or wrong units.
   * - 301
     - ``POSSIBLE_UNIT_ERROR``
     - High
     - Material property may be in wrong units. E.g., E=210000 instead of E=210e6 for steel in kN/m².
   * - 302
     - ``INCONSISTENT_SECTION``
     - Medium
     - Section properties are physically inconsistent. E.g., Iy > A² is geometrically impossible.

Load Warnings (400-499)
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 10 25 15 50

   * - Code
     - Name
     - Severity
     - Description
   * - 400
     - ``LARGE_LOAD``
     - Medium
     - Very large load magnitude. May indicate unit error (e.g., N instead of kN).
   * - 401
     - ``LOAD_AT_FREE_NODE``
     - Low
     - Load applied at unsupported node. Intentional for tip loads, but verify if unexpected.
   * - 402
     - ``ACCELERATION_WITHOUT_MASS``
     - Medium
     - Acceleration loads applied but elements have no mass. Add mass for inertia effects.

Analysis Warnings (500-599)
---------------------------

.. list-table::
   :header-rows: 1
   :widths: 10 25 15 50

   * - Code
     - Name
     - Severity
     - Description
   * - 500
     - ``LARGE_DISPLACEMENT``
     - Medium
     - Large displacements detected (>1% of span). Geometric nonlinear analysis may be needed.
   * - 501
     - ``HIGH_STRESS``
     - Medium
     - High stress detected (may exceed yield). Material nonlinearity may be needed.
   * - 502
     - ``SOLVER_REFINEMENT``
     - Low
     - Solver used iterative refinement. Results are valid but took extra computation.


Using Errors and Warnings in Python
===================================

Checking Error Status
---------------------

.. code-block:: python

    from grillex.core import GrillexError, ErrorCode

    # Create an error (typically returned by analysis)
    error = GrillexError()  # Default is OK status

    # Check status
    if error.is_ok():
        print("Analysis successful")
    else:
        print(f"Error: {error.message}")
        print(f"Code: {error.code_string()}")

    # Access diagnostic information
    if error.involved_nodes:
        print(f"Problem nodes: {error.involved_nodes}")
    if error.involved_dofs:
        print(f"Problem DOFs: {error.involved_dofs}")
    if error.suggestion:
        print(f"Suggestion: {error.suggestion}")

Working with Warnings
---------------------

.. code-block:: python

    from grillex.core import WarningList, WarningSeverity

    # Get warnings from model validation
    warnings = WarningList()

    # Check for warnings
    if warnings.has_warnings():
        print(f"Found {warnings.count()} warnings")
        print(warnings.summary())

        # Filter by severity
        high_severity = warnings.get_by_min_severity(WarningSeverity.High)
        for warn in high_severity:
            print(warn.to_string())


Common Error Scenarios
======================

Singular Matrix
---------------

The most common analysis failure. Typical causes:

1. **Missing supports**: Model has rigid body modes

   .. code-block:: python

      # Wrong: beam floating in space
      model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")

      # Fix: add supports
      model.fix_node_at([0, 0, 0])

2. **Missing torsional restraint**: 3D beams need twist restraint

   .. code-block:: python

      # Even with vertical restraints, beams can spin about their axis
      model.add_fixed_dof([0, 0, 0], DOFIndex.RX, 0.0)  # Prevent twist

3. **Disconnected elements**: Parts not connected

   .. code-block:: python

      # Check that beam endpoints match exactly (within tolerance)

Not Analyzed
------------

Trying to get results before running analysis:

.. code-block:: python

    model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    model.fix_node_at([0, 0, 0])

    # Wrong: query results immediately
    # disp = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)  # Error!

    # Fix: run analysis first
    model.analyze()
    disp = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)  # OK

Empty Model
-----------

Analysis attempted with no elements:

.. code-block:: python

    model = StructuralModel("Empty")
    # model.analyze()  # Error: no elements

    # Fix: add elements first
    model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")


LLM Agent Integration
=====================

Error messages are designed for parsing by LLM agents:

1. **Structured format**: Error code + message + context
2. **Suggestions**: Actionable fix recommendations
3. **Diagnostic data**: Node/element/DOF lists for targeted fixes

Example workflow:

.. code-block:: python

    result = model.analyze()

    if result.is_error():
        # Parse error for automated fixing
        if result.code == ErrorCode.UNCONSTRAINED_SYSTEM:
            for dof in result.involved_dofs:
                # Suggest adding constraint at this DOF
                pass

        # Or display suggestion to user
        print(result.suggestion)
