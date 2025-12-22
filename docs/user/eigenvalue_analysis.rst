===================
Eigenvalue Analysis
===================

This guide describes how to perform eigenvalue (modal) analysis in Grillex
to determine natural frequencies, mode shapes, and participation factors.

When to Use Modal Analysis
==========================

Modal analysis is essential for understanding dynamic behavior:

- **Resonance check**: Verify natural frequencies are away from excitation frequencies
- **Dynamic response**: Prepare for response spectrum or time-history analysis
- **Design verification**: Check that fundamental frequency meets code requirements
- **Vibration problems**: Diagnose and solve vibration issues

Running Eigenvalue Analysis
===========================

Basic Usage
-----------

Use the ``analyze_modes()`` method to perform eigenvalue analysis:

.. doctest::

    >>> from grillex.core import StructuralModel
    >>>
    >>> # Create a simple cantilever beam
    >>> model = StructuralModel(name="Cantilever Modal Analysis")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE300", A=0.00538, Iy=8.36e-5, Iz=6.04e-6, J=2.01e-7)
    >>>
    >>> # Create beam with subdivisions for accurate higher modes
    >>> beam = model.add_beam_by_coords([0, 0, 0], [6, 0, 0], "IPE300", "Steel",
    ...                                  subdivisions=10)
    >>>
    >>> # Fix the left end (cantilever support)
    >>> model.fix_node_at([0, 0, 0])
    >>>
    >>> # Run eigenvalue analysis
    >>> success = model.analyze_modes(n_modes=5)
    >>> success
    True

Getting Results
---------------

After analysis, retrieve frequencies, periods, and mode shapes:

.. doctest::

    >>> # Get natural frequencies [Hz]
    >>> frequencies = model.get_natural_frequencies()
    >>> len(frequencies) >= 3
    True
    >>> frequencies[0] > 0  # First positive frequency
    True

.. doctest::

    >>> # Get periods [seconds]
    >>> periods = model.get_periods()
    >>> len(periods) == len(frequencies)
    True

.. doctest::

    >>> # Get mode shape at a specific location
    >>> mode_disp = model.get_mode_displacement_at(1, [6, 0, 0])  # Mode 1, free end
    >>> 'UY' in mode_disp and 'UZ' in mode_disp
    True

Interpreting Results
====================

Natural Frequencies
-------------------

Natural frequencies are returned in Hz, sorted from lowest to highest.
The first mode is the **fundamental frequency**, which typically dominates
the dynamic response.

- **Low frequency** (< 1 Hz): Flexible structure, may be sensitive to wind/waves
- **Medium frequency** (1-10 Hz): Common for offshore structures
- **High frequency** (> 10 Hz): Stiff structure, less sensitive to low-frequency excitation

Mode Shapes
-----------

Mode shapes describe the deformation pattern at each natural frequency.
Each mode is independent - the structure can vibrate in any combination of modes.

.. doctest::

    >>> # Check first mode shape at fixed and free ends
    >>> fixed_end = model.get_mode_displacement_at(1, [0, 0, 0])
    >>> free_end = model.get_mode_displacement_at(1, [6, 0, 0])
    >>>
    >>> # Fixed end should have zero displacement
    >>> bool(abs(fixed_end['UY']) < 1e-10)
    True
    >>> bool(abs(fixed_end['UZ']) < 1e-10)
    True
    >>>
    >>> # Free end should have non-zero displacement
    >>> bool(max(abs(free_end['UY']), abs(free_end['UZ'])) > 0)
    True

Participation Factors
---------------------

Participation factors indicate how much each mode contributes to the total
response in each direction (X, Y, Z). Higher participation means the mode
is more important for that direction.

**Effective modal mass** is the portion of total mass that participates
in each mode. Design codes often require capturing 90% of effective mass.

Example: Cantilever Beam
========================

This example shows a complete modal analysis of a cantilever beam with
verification against analytical results.

.. doctest::

    >>> import math
    >>> from grillex.core import StructuralModel
    >>>
    >>> # Beam parameters
    >>> L = 2.0    # Length [m]
    >>> E = 210e6  # Young's modulus [kN/m²]
    >>> I = 1e-4   # Second moment of area [m⁴]
    >>> A = 0.01   # Cross-section area [m²]
    >>> rho = 7.85e-3  # Density [mT/m³]
    >>>
    >>> # Analytical first frequency (Euler-Bernoulli)
    >>> lambda_1 = 1.8751  # First eigenvalue for cantilever
    >>> f1_analytical = (lambda_1**2) * math.sqrt(E * I / (rho * A * L**4)) / (2 * math.pi)
    >>>
    >>> # Create model
    >>> model = StructuralModel(name="Cantilever Verification")
    >>> _ = model.add_material("Steel", E=E, nu=0.3, rho=rho)
    >>> _ = model.add_section("Test", A=A, Iy=I, Iz=I, J=1e-6)
    >>> beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "Test", "Steel",
    ...                                  subdivisions=10)
    >>> model.fix_node_at([0, 0, 0])
    >>>
    >>> # Run analysis
    >>> _ = model.analyze_modes(n_modes=5)
    >>> frequencies = model.get_natural_frequencies()
    >>>
    >>> # Compare with analytical (within 5% tolerance)
    >>> f1_computed = frequencies[0]
    >>> error_pct = abs(f1_computed - f1_analytical) / f1_analytical * 100
    >>> error_pct < 5.0
    True

Example: Simply Supported Beam
==============================

Modal analysis of a simply supported beam:

.. doctest::

    >>> from grillex.core import StructuralModel, DOFIndex
    >>>
    >>> model = StructuralModel(name="Simply Supported Modal")
    >>> _ = model.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model.add_section("IPE200", A=0.00285, Iy=1.94e-5, Iz=1.42e-6, J=6.9e-8)
    >>>
    >>> # Create beam
    >>> L = 6.0
    >>> beam = model.add_beam_by_coords([0, 0, 0], [L, 0, 0], "IPE200", "Steel",
    ...                                  subdivisions=10)
    >>>
    >>> # Simply supported: pin at both ends
    >>> model.fix_dof_at([0, 0, 0], DOFIndex.UX)
    >>> model.fix_dof_at([0, 0, 0], DOFIndex.UY)
    >>> model.fix_dof_at([0, 0, 0], DOFIndex.UZ)
    >>> model.fix_dof_at([0, 0, 0], DOFIndex.RX)
    >>> model.fix_dof_at([0, 0, 0], DOFIndex.RZ)
    >>>
    >>> model.fix_dof_at([L, 0, 0], DOFIndex.UY)
    >>> model.fix_dof_at([L, 0, 0], DOFIndex.UZ)
    >>> model.fix_dof_at([L, 0, 0], DOFIndex.RX)
    >>> model.fix_dof_at([L, 0, 0], DOFIndex.RZ)
    >>>
    >>> # Run analysis
    >>> _ = model.analyze_modes(n_modes=5)
    >>> frequencies = model.get_natural_frequencies()
    >>> len(frequencies) >= 3
    True

Troubleshooting
===============

Common Issues
-------------

**Singular mass matrix**

This occurs when DOFs have zero mass. Ensure:

- Material density (rho) is non-zero
- Section area (A) is non-zero
- Point masses are added where needed

**Convergence failure**

For large models or closely-spaced eigenvalues:

- Increase the number of iterations
- Use subspace iteration method for large models
- Check for nearly-zero stiffness (unstable elements)

**Unexpected rigid body modes**

If frequencies near zero appear:

- Check boundary conditions are sufficient
- Verify all DOFs are properly constrained
- Add torsional restraints (RX) if missing

Mesh Refinement
---------------

Accurate higher modes require sufficient mesh refinement:

.. doctest::

    >>> # Coarse mesh - fewer elements
    >>> model1 = StructuralModel(name="Coarse")
    >>> _ = model1.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model1.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)
    >>> beam1 = model1.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Test", "Steel",
    ...                                    subdivisions=2)
    >>> model1.fix_node_at([0, 0, 0])
    >>> _ = model1.analyze_modes(n_modes=3)
    >>> f1_coarse = model1.get_natural_frequencies()[0]

.. doctest::

    >>> # Fine mesh - more elements for accuracy
    >>> model2 = StructuralModel(name="Fine")
    >>> _ = model2.add_material("Steel", E=210e6, nu=0.3, rho=7.85e-3)
    >>> _ = model2.add_section("Test", A=0.01, Iy=1e-4, Iz=1e-4, J=1e-5)
    >>> beam2 = model2.add_beam_by_coords([0, 0, 0], [4, 0, 0], "Test", "Steel",
    ...                                    subdivisions=10)
    >>> model2.fix_node_at([0, 0, 0])
    >>> _ = model2.analyze_modes(n_modes=3)
    >>> f1_fine = model2.get_natural_frequencies()[0]

Technical Reference
===================

Solver Methods
--------------

Grillex supports multiple eigenvalue solver methods:

- **Dense**: Full matrix decomposition, best for small models (< 500 DOFs)
- **Subspace iteration**: Iterative method for large models
- **Shift-invert**: For computing modes near a target frequency

The default method is automatically selected based on model size.

Mass Matrix
-----------

Grillex uses a consistent mass matrix formulation, which provides:

- Higher accuracy than lumped mass
- Better convergence for higher modes
- Correct coupling between translational and rotational DOFs

Point masses can be added at nodes to model concentrated equipment:

.. doctest::

    >>> from grillex._grillex_cpp import Model
    >>>
    >>> model = Model()
    >>> n1 = model.get_or_create_node(0, 0, 0)
    >>> n2 = model.get_or_create_node(2, 0, 0)
    >>>
    >>> mat = model.create_material("Steel", 210e6, 0.3, 7.85e-3)
    >>> sec = model.create_section("Beam", 0.01, 1e-4, 1e-4, 1e-5)
    >>> beam = model.create_beam(n1, n2, mat, sec)
    >>>
    >>> # Add point mass at tip
    >>> pm = model.create_point_mass(n2)
    >>> pm.mass = 10.0  # 10 mT
    >>>
    >>> model.boundary_conditions.fix_node(n1.id)
    >>>
    >>> from grillex._grillex_cpp import EigensolverSettings
    >>> settings = EigensolverSettings()
    >>> settings.n_modes = 3
    >>> _ = model.analyze_eigenvalues(settings)
    >>> frequencies = model.get_natural_frequencies()
    >>> len(frequencies) >= 2
    True

Normalization
-------------

Mode shapes are mass-normalized by default, meaning:

φᵀ M φ = 1

This ensures participation factors and effective masses are correctly computed.

Related Topics
==============

- :doc:`analysis_workflow` - Static analysis workflow
- :doc:`loads_and_boundary_conditions` - Defining constraints
- :doc:`results_and_postprocessing` - Extracting analysis results
