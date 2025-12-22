================
Nonlinear Springs
================

Grillex supports nonlinear springs that model contact behavior such as bearing
pads, seafastening, and gap connections. These springs have state-dependent
stiffness that requires iterative solution.

.. contents:: Contents
   :local:
   :depth: 2

Spring Behavior Types
====================

Springs can have one of three behavior types per DOF:

**Linear** (default)
    Standard two-way spring: active in both tension and compression.
    ``F = k × δ`` where δ is the relative deformation.

**TensionOnly**
    Spring that only resists tension (positive deformation).
    Used for modeling cables, tie-downs, and seafastening.
    When compressed, the spring becomes inactive (zero stiffness).

**CompressionOnly**
    Spring that only resists compression (negative deformation).
    Used for modeling bearing pads, contact surfaces, and supports.
    When in tension, the spring becomes inactive (liftoff).

Setting Spring Behavior
=======================

Using the C++ Model API
-----------------------

.. code-block:: python

    from grillex.core import Model, SpringBehavior, DOFIndex

    model = Model()
    n1 = model.get_or_create_node(0, 0, 0)
    n2 = model.get_or_create_node(1, 0, 0)

    # Create spring
    spring = model.create_spring(n1, n2)
    spring.kz = 10000.0  # 10 MN/m

    # Set compression-only behavior in Z
    spring.set_behavior(2, SpringBehavior.CompressionOnly)

    # Or set all DOFs at once
    spring.set_all_behavior(SpringBehavior.TensionOnly)

Gap Springs
===========

Springs can have a gap value that defines a dead zone before the spring
engages. This is useful for modeling:

- **Contact with clearance**: Gaps between cargo and deck
- **Slack cables**: Tie-downs with initial slack
- **Preloaded connections**: Springs that only activate after preload

Gap Values
----------

For each DOF, you can set a gap value (in meters for translation DOFs,
radians for rotation DOFs):

.. code-block:: python

    spring = model.create_spring(n1, n2)
    spring.kz = 10000.0
    spring.set_behavior(2, SpringBehavior.CompressionOnly)
    spring.set_gap(2, 0.01)  # 10mm gap in Z

    # The spring only activates when |δ| > gap

Force-Displacement Relationship
-------------------------------

For a gap spring:

- **Inactive**: When deformation hasn't exceeded the gap, ``F = 0``
- **Active**: When gap is closed, ``F = k × (δ - gap)``

This offset ensures the spring carries no force at the gap threshold.

Nonlinear Analysis
==================

Running Nonlinear Analysis
--------------------------

.. code-block:: python

    model = Model()
    # ... create model with nonlinear springs ...

    # Check if model requires nonlinear analysis
    if model.has_nonlinear_springs():
        success = model.analyze_nonlinear()
    else:
        success = model.analyze()

    if success:
        print("Analysis converged")
    else:
        print(f"Error: {model.get_error_message()}")

Solver Settings
---------------

The nonlinear solver has several configurable settings:

.. code-block:: python

    from grillex.core import NonlinearSolverSettings

    settings = NonlinearSolverSettings()
    settings.max_iterations = 50           # Maximum iterations
    settings.displacement_tolerance = 1e-8  # Convergence tolerance
    settings.gap_tolerance = 1e-10          # Gap evaluation tolerance

    # Convergence enhancements
    settings.enable_oscillation_damping = True
    settings.oscillation_damping_factor = 0.5
    settings.hysteresis_band = 0.001  # 1mm hysteresis
    settings.use_partial_stiffness = True

Querying Spring States
----------------------

After analysis, you can check the state of each spring:

.. code-block:: python

    success = model.analyze_nonlinear()

    for spring in model.springs:
        # Check if spring is active in Z direction
        if spring.is_active[2]:
            print(f"Spring {spring.id}: Active, deformation={spring.deformation[2]:.4f} m")
        else:
            print(f"Spring {spring.id}: Inactive (lifted off)")

        # Get spring forces
        forces = spring.compute_forces()
        print(f"  Force Z: {forces[2]:.2f} kN")

Practical Examples
==================

Example 1: Cargo on Bearing Pads
--------------------------------

A cargo item resting on four bearing pads. Under horizontal force or moment,
some pads may lift off:

.. code-block:: python

    from grillex.core import Model, SpringBehavior, DOFIndex, BeamConfig

    model = Model()

    # Cargo corners (simplified as single beam for example)
    n1 = model.get_or_create_node(0, 0, 1.0)  # 1m above deck
    n2 = model.get_or_create_node(2, 0, 1.0)

    # Deck connection points
    n1_deck = model.get_or_create_node(0, 0, 0)
    n2_deck = model.get_or_create_node(2, 0, 0)

    mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
    sec = model.create_section("Cargo", 0.01, 1e-4, 1e-4, 1e-5)

    config = BeamConfig()
    model.create_beam(n1, n2, mat, sec, config)

    # Add bearing pads (compression-only springs)
    pad_stiffness = 50000.0  # 50 MN/m

    pad1 = model.create_spring(n1, n1_deck)
    pad1.kz = pad_stiffness
    pad1.set_behavior(2, SpringBehavior.CompressionOnly)

    pad2 = model.create_spring(n2, n2_deck)
    pad2.kz = pad_stiffness
    pad2.set_behavior(2, SpringBehavior.CompressionOnly)

    # Fix deck
    model.boundary_conditions.fix_node(n1_deck.id)
    model.boundary_conditions.fix_node(n2_deck.id)

    # Apply loads
    lc = model.get_default_load_case()
    lc.add_nodal_load(n1.id, DOFIndex.UZ, -50.0)  # Gravity
    lc.add_nodal_load(n2.id, DOFIndex.UZ, -50.0)
    lc.add_nodal_load(n2.id, DOFIndex.UZ, 80.0)   # Overturning force

    # Analyze
    success = model.analyze_nonlinear()

    # Check for liftoff
    if pad2.is_active[2]:
        print("Right pad in contact")
    else:
        print("Right pad lifted off!")

Example 2: Tie-Down with Slack
------------------------------

A tension-only spring (cable) with initial slack:

.. code-block:: python

    # Create tie-down spring
    tiedown = model.create_spring(cargo_node, anchor_node)
    tiedown.kz = 10000.0  # Cable axial stiffness

    # Set as tension-only with 20mm slack
    tiedown.set_behavior(2, SpringBehavior.TensionOnly)
    tiedown.set_gap(2, 0.02)  # 20mm slack

    # The tie-down only carries load after the cargo moves up 20mm

Load Combinations
=================

With nonlinear springs, load case superposition is not valid. Each load
combination must be solved directly:

.. code-block:: python

    from grillex.core import LoadCombination, LoadCaseType

    # Create load cases
    lc_dead = model.create_load_case("Dead", LoadCaseType.Permanent)
    lc_wind = model.create_load_case("Wind", LoadCaseType.Environmental)

    # Add loads to each case
    lc_dead.add_nodal_load(node.id, DOFIndex.UZ, -100.0)
    lc_wind.add_nodal_load(node.id, DOFIndex.UZ, 30.0)

    # Create combination
    combo = LoadCombination(1, "ULS", 1.35, 1.5, 1.5, 1.0)
    combo.add_load_case(lc_dead)  # Uses 1.35 factor
    combo.add_load_case(lc_wind)  # Uses 1.5 factor

    # Analyze combination directly
    result = model.analyze_combination(combo)

    if result.converged:
        print(f"Converged in {result.iterations} iterations")
    else:
        print(f"Failed: {result.message}")

Static-to-Dynamic Sequencing
----------------------------

For correct physical behavior, permanent loads should establish the baseline
contact pattern before dynamic loads are applied. The solver automatically
handles this when using ``analyze_combination()``:

1. Permanent (gravity) loads are solved first
2. The resulting spring states become the initial state
3. The full combination is solved starting from this state

This ensures cargo properly "settles" under gravity before dynamic effects.

Convergence Issues
==================

If the solver fails to converge, try these strategies:

1. **Increase max_iterations**

   .. code-block:: python

      settings.max_iterations = 100

2. **Enable oscillation damping** (default: enabled)

   .. code-block:: python

      settings.enable_oscillation_damping = True
      settings.oscillation_damping_factor = 0.5

3. **Add hysteresis band** to prevent chattering near thresholds

   .. code-block:: python

      settings.hysteresis_band = 0.001  # 1mm

4. **Use partial stiffness** for oscillating springs

   .. code-block:: python

      settings.use_partial_stiffness = True

5. **Check model stability**: Ensure the model isn't unstable when springs
   lift off. Add minimal restraints if necessary.

Technical Reference
===================

Iteration Algorithm
-------------------

The nonlinear solver uses a state-based iteration approach:

1. Start with all springs active (or from initial state)
2. Assemble stiffness matrix with current active springs
3. Compute gap forces for springs with gaps
4. Solve: ``K × u = F + F_gap``
5. Update spring states based on new deformations
6. If no state changes and displacement converged: done
7. Otherwise: repeat from step 2

Oscillation Detection
---------------------

The solver tracks state history to detect oscillating springs (e.g., a spring
that alternates between active and inactive). When detected:

- Solution is damped: ``u_new = α × u_new + (1-α) × u_prev``
- If ``use_partial_stiffness`` is enabled, oscillating springs use 50% stiffness

Hysteresis Band
---------------

The hysteresis band creates different thresholds for activation and deactivation:

- **Activation threshold**: ``gap + hysteresis``
- **Deactivation threshold**: ``gap - hysteresis``

This prevents rapid state switching when deformation is near the gap value.
