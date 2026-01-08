=================
Analysis Workflow
=================

This guide describes the complete analysis workflow in Grillex.

Overview
========

The typical analysis workflow consists of:

1. Create model and define materials/sections
2. Create beam elements
3. Apply boundary conditions
4. Apply loads
5. Run analysis
6. Extract and interpret results

Complete Workflow Example
=========================

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>>
    >>> # Step 1: Create model
    >>> model = StructuralModel(name="Complete Workflow Example")
    >>>
    >>> # Step 2: Define materials and sections
    >>> _ = model.add_material("S355", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("HEA200", A=0.00538, Iy=3.69e-5, Iz=1.34e-5, J=2.1e-7)
    >>>
    >>> # Step 3: Create beam elements
    >>> beam1 = model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "HEA200", "S355")
    >>> beam2 = model.add_beam_by_coords([5, 0, 0], [10, 0, 0], "HEA200", "S355")
    >>>
    >>> # Step 4: Apply boundary conditions
    >>> model.fix_node_at([0, 0, 0])   # Fixed support at left end
    >>> model.pin_node_at([10, 0, 0])  # Pinned support at right end
    >>>
    >>> # Step 5: Apply loads
    >>> model.add_point_load([5, 0, 0], force=[0, 0, -50.0])  # 50 kN at midspan
    >>>
    >>> # Step 6: Run analysis
    >>> _ = model.analyze()
    >>>
    >>> # Step 7: Extract results
    >>> disp = model.get_displacement_at([5, 0, 0], DOFIndex.UZ)
    >>> disp < 0  # Should be negative (downward)
    True

Pre-Analysis Checks
===================

Before running the analysis, ensure:

Model Completeness
------------------

- At least one material is defined
- At least one section is defined
- At least one element exists
- Boundary conditions prevent rigid body motion

.. doctest::

    >>> model = StructuralModel(name="Check Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    >>> model.fix_node_at([0, 0, 0])  # Prevent rigid body motion
    >>> _ = model.analyze()  # Should succeed

Sufficient Constraints
----------------------

A 3D beam model requires at minimum:

- 3 translational constraints (UX, UY, UZ)
- 3 rotational constraints (RX, RY, RZ)

For a single beam, fixing one end provides all 6 constraints.

Running the Analysis
====================

The ``analyze()`` method performs these steps internally:

1. **DOF Numbering**: Assigns global DOF numbers to all nodes
2. **Assembly**: Assembles global stiffness matrix and force vector
3. **BC Application**: Applies boundary conditions
4. **Solving**: Solves the linear system Ku = F
5. **Post-processing**: Computes reactions and internal forces

Analysis Options
----------------

The analysis can be customized with options:

.. doctest::

    >>> model = StructuralModel(name="Options Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>> model.add_point_load([6, 0, 0], force=[0, 0, -10.0])
    >>>
    >>> # Run analysis (default options)
    >>> _ = model.analyze()

Error Handling
==============

If the analysis fails, Grillex provides structured error messages:

Singular Matrix
---------------

A singular matrix usually indicates insufficient constraints:

.. code-block:: python

    # This would fail - no boundary conditions
    model = StructuralModel(name="Unconstrained")
    model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    beam = model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "IPE200", "Steel")
    model.analyze()  # Raises GrillexError with SINGULAR_MATRIX code

Invalid References
------------------

Using undefined materials or sections:

.. code-block:: python

    model.add_beam_by_coords([0, 0, 0], [5, 0, 0], "Unknown", "Steel")
    # Raises GrillexError with INVALID_SECTION code

Re-Analysis
===========

You can modify the model and re-analyze:

.. doctest::

    >>> model = StructuralModel(name="Re-analysis Example")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel")
    >>> model.fix_node_at([0, 0, 0])
    >>>
    >>> # First analysis with 10 kN load
    >>> model.add_point_load([6, 0, 0], force=[0, 0, -10.0])
    >>> _ = model.analyze()
    >>> disp1 = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
    >>>
    >>> # Add another 10 kN load
    >>> model.add_point_load([6, 0, 0], force=[0, 0, -10.0])
    >>> _ = model.analyze()
    >>> disp2 = model.get_displacement_at([6, 0, 0], DOFIndex.UZ)
    >>>
    >>> # Displacement should approximately double
    >>> abs(disp2 / disp1 - 2.0) < 0.01
    True

Performance Considerations
==========================

For large models:

- **Mesh Density**: Use appropriate element subdivision
- **Sparse Matrices**: Grillex uses sparse matrix storage automatically
- **Constraint Handling**: Penalty method is used for boundary conditions

Typical Analysis Times
----------------------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Model Size (DOFs)
     - Typical Time
     - Notes
   * - < 1,000
     - < 0.1 seconds
     - Interactive analysis
   * - 1,000 - 10,000
     - 0.1 - 1 second
     - Standard models
   * - > 10,000
     - > 1 second
     - Large grillage models

See Also
========

- :doc:`results_and_postprocessing` for extracting results
- :doc:`loads_and_boundary_conditions` for load application
