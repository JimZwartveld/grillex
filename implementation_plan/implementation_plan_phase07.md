## Phase 7: Internal Actions & Results

**Overview:**
This phase implements computation of internal actions (forces, moments, shears, torsion) along beam elements using **analytical closed-form solutions derived from differential equations**. This approach, similar to pystructeng, provides exact results that properly account for element end releases, distributed loads, and different beam theories (Euler-Bernoulli vs Timoshenko).

**Key Features:**
- Differential equation-based internal action computation
- Release-specific formulas for each boundary condition combination
- Support for both Euler-Bernoulli and Timoshenko beam theories
- Multi-element beam plotting with continuous section lines
- Warping/bimoment results for 14-DOF elements
- Displacement and rotation lines along elements

---

## Background: Differential Equation Approach

### Governing Equations

Internal actions along beam elements are governed by equilibrium differential equations:

**Axial:**
```
dN/dx + q_x = 0
N = EA × du/dx
```

**Bending (Euler-Bernoulli):**
```
dV/dx + q = 0
dM/dx - V = 0
M = EI × d²w/dx²
```

**Bending (Timoshenko):**
```
dV/dx + q = 0
dM/dx - V = 0
M = EI × dφ/dx          (φ = bending rotation)
V = kAG × (dw/dx - φ)   (shear deformation)
```

**Torsion:**
```
dM_x/dx + m_x = 0
M_x = GJ × dθ/dx
```

**Warping (for 14-DOF elements):**
```
d²B/dx² = -T_w           (T_w = warping torsion component)
B = EI_w × d²φ'/dx²      (φ' = warping parameter)
```

### Solution Methodology

Given element end conditions (displacements, rotations) and distributed loads, we:
1. Solve the differential equations analytically for each release combination
2. Apply boundary conditions to determine integration constants
3. Obtain closed-form expressions for N(x), V(x), M(x), u(x), w(x), θ(x)

**Advantages over simple interpolation:**
- Exact for polynomial distributed loads
- Properly handles end releases (hinge, roller, etc.)
- Accounts for shear deformation (Timoshenko)
- Can find extrema analytically (not just numerically)

---

## Task 7.1: Implement Element End Forces

**Requirements:** R-RES-001
**Dependencies:** Task 3.5, (optionally Task 5.2 for distributed loads)
**Difficulty:** Medium

### Description
Compute element end forces from global displacements using the element stiffness matrix approach.

### C++ Data Structures

```cpp
namespace grillex {

/**
 * @brief Element end forces in local coordinates
 */
struct EndForces {
    double N;   // Axial force [kN]
    double Vy;  // Shear force in local y [kN]
    double Vz;  // Shear force in local z [kN]
    double Mx;  // Torsion moment [kN·m]
    double My;  // Bending moment about local y [kN·m]
    double Mz;  // Bending moment about local z [kN·m]

    // For 14-DOF warping elements:
    double B = 0.0;   // Bimoment [kN·m²] (zero for 12-DOF elements)
};

} // namespace grillex
```

### Implementation Steps

1. **Add helper method to extract element DOFs:**
   ```cpp
   class BeamElement {
   public:
       /**
        * @brief Extract element displacements from global solution
        * @param global_displacements Full displacement vector
        * @param dof_handler DOF numbering manager
        * @return Element DOFs in local coordinates (12 or 14 length)
        */
       Eigen::VectorXd get_element_displacements_local(
           const Eigen::VectorXd& global_displacements,
           const DOFHandler& dof_handler) const;
   };
   ```

2. **Implement end force computation:**
   ```cpp
   class BeamElement {
   public:
       /**
        * @brief Compute element end forces from displacements
        * @param global_displacements Full displacement vector
        * @param dof_handler DOF numbering manager
        * @return Pair of (end_i_forces, end_j_forces)
        */
       std::pair<EndForces, EndForces> compute_end_forces(
           const Eigen::VectorXd& global_displacements,
           const DOFHandler& dof_handler) const;
   };
   ```

3. **Algorithm:**
   ```
   1. Extract element DOFs from global vector using dof_handler
   2. Transform to local coordinates:
      u_local = T_transformation.transpose() * u_global_element
   3. Compute local forces:
      f_local = K_local * u_local
   4. If distributed loads present (Phase 5 complete):
      f_local -= f_fixed_end  (subtract fixed-end forces)
   5. If element has releases:
      Zero out forces at released DOFs (condensed DOFs)
   6. Split f_local into (f_i, f_j) and populate EndForces structs
   7. Apply sign convention per R-COORD-004
   ```

### Sign Conventions

Following standard structural engineering conventions:
- **Axial:** Tension positive, compression negative
- **Shear:** Positive per right-hand rule about local axes
- **Moments:** Positive per right-hand rule about local axes
- **Bimoment:** Sign follows warping stress convention

### Dependencies

**Without Phase 5 (distributed loads):**
- Can compute end forces for point loads only
- Fixed-end forces are zero
- Limited accuracy for actual beam loading

**With Phase 5 (distributed loads):**
- Full computation including fixed-end forces from distributed loads
- Accurate results for real-world loading scenarios

### Acceptance Criteria
- [ ] End forces match support reactions for simple cases (cantilever, simply supported)
- [ ] Sign convention is consistent across all force/moment components
- [ ] Forces satisfy equilibrium: sum(F) = 0, sum(M) = 0
- [ ] End releases properly zero out forces at released DOFs
- [ ] Works for both 12-DOF and 14-DOF (warping) elements

---

## Task 7.2: Implement Internal Action Functions Along Beam

**Requirements:** R-RES-002, R-RES-003, R-RES-004, R-LOAD-008, R-LOAD-009
**Dependencies:** Task 7.1, Phase 5 (for distributed loads)
**Difficulty:** High

### Description
Compute internal actions (N, V, M) at any position along the beam element using **analytical closed-form solutions from differential equations**, accounting for element end releases, distributed loads, and beam theory (Euler-Bernoulli vs Timoshenko).

### C++ Data Structures

```cpp
namespace grillex {

/**
 * @brief Internal actions at a position along the beam
 */
struct InternalActions {
    double x;    // Position along beam [0, L] in meters
    double N;    // Axial force [kN]
    double Vy;   // Shear force in y [kN]
    double Vz;   // Shear force in z [kN]
    double Mx;   // Torsion moment [kN·m]
    double My;   // Moment about y [kN·m]
    double Mz;   // Moment about z [kN·m]
};

/**
 * @brief Extremum location and value
 */
struct ActionExtreme {
    double x;      // Position along beam [m]
    double value;  // Value at extremum
};

/**
 * @brief Distributed load (trapezoidal)
 */
struct DistributedLoad {
    double q_start;  // Load intensity at element start [kN/m]
    double q_end;    // Load intensity at element end [kN/m]

    // Linear variation: q(x) = q_start + (q_end - q_start) * x / L
    // For uniform load: q_start == q_end
};

/**
 * @brief Release combinations for bending (4-DOF)
 */
enum class ReleaseCombo4DOF {
    FIXED_FIXED_FIXED_FIXED,  // w1, φ1, w2, φ2 all fixed
    FIXED_FIXED_FREE_FIXED,   // w1, φ1, φ2 fixed; w2 free
    FIXED_FIXED_FIXED_FREE,   // w1, φ1, w2 fixed; φ2 free
    FIXED_FIXED_FREE_FREE,    // w1, φ1 fixed; w2, φ2 free
    FIXED_FREE_FIXED_FIXED,   // w1, w2, φ2 fixed; φ1 free
    FIXED_FREE_FREE_FIXED,    // w1, φ2 fixed; φ1, w2 free
    FIXED_FREE_FIXED_FREE,    // w1, w2 fixed; φ1, φ2 free
    FIXED_FREE_FREE_FREE,     // w1 fixed; φ1, w2, φ2 free
    FREE_FIXED_FIXED_FIXED,   // φ1, w2, φ2 fixed; w1 free
    FREE_FIXED_FREE_FIXED,    // φ1, φ2 fixed; w1, w2 free
    FREE_FIXED_FIXED_FREE,    // φ1, w2 fixed; w1, φ2 free
    FREE_FIXED_FREE_FREE,     // φ1 fixed; w1, w2, φ2 free
    FREE_FREE_FIXED_FIXED,    // w2, φ2 fixed; w1, φ1 free
    FREE_FREE_FREE_FIXED,     // φ2 fixed; w1, φ1, w2 free
    FREE_FREE_FIXED_FREE,     // w2 fixed; w1, φ1, φ2 free
    FREE_FREE_FREE_FREE       // All free (rigid body motion)
};

/**
 * @brief Release combinations for axial/torsion (2-DOF)
 */
enum class ReleaseCombo2DOF {
    FIXED_FIXED,  // Both ends fixed
    FIXED_FREE,   // Start fixed, end free
    FREE_FIXED,   // Start free, end fixed
    FREE_FREE     // Both ends free (rigid body motion)
};

} // namespace grillex
```

### Implementation Architecture

Use **strategy pattern** to organize formulas by component and beam theory:

```cpp
cpp/include/grillex/
├── internal_actions.hpp              // Structs defined above
└── internal_actions_computer.hpp     // Computer classes

cpp/src/
├── internal_actions_axial.cpp               // Axial force formulas
├── internal_actions_bending_euler.cpp       // Euler-Bernoulli bending
├── internal_actions_bending_timoshenko.cpp  // Timoshenko bending
├── internal_actions_torsion.cpp             // Torsion formulas
└── internal_actions_warping.cpp             // Warping/bimoment (Task 7.2b)
```

### BeamElement Public Interface

```cpp
class BeamElement {
public:
    /**
     * @brief Get internal actions at position x along element
     * @param x Position [0, L] in meters
     * @param global_displacements Full displacement vector
     * @param dof_handler DOF numbering manager
     * @return Internal actions (N, Vy, Vz, Mx, My, Mz)
     */
    InternalActions get_internal_actions(
        double x,
        const Eigen::VectorXd& global_displacements,
        const DOFHandler& dof_handler) const;

    /**
     * @brief Find moment extrema along element
     * @param axis 'y' or 'z' for bending plane
     * @param global_displacements Full displacement vector
     * @param dof_handler DOF numbering manager
     * @return Pair of (min, max) extrema with positions and values
     */
    std::pair<ActionExtreme, ActionExtreme> find_moment_extremes(
        char axis,
        const Eigen::VectorXd& global_displacements,
        const DOFHandler& dof_handler) const;

private:
    // Release detection
    ReleaseCombo4DOF detect_release_combination_bending_y() const;
    ReleaseCombo4DOF detect_release_combination_bending_z() const;
    ReleaseCombo2DOF detect_release_combination_axial() const;
    ReleaseCombo2DOF detect_release_combination_torsion() const;

    // Component computations (delegate to computer classes)
    double compute_axial_force(double x, const Eigen::VectorXd& u_local,
                               const DistributedLoad& q_x, ReleaseCombo2DOF release) const;
    double compute_shear_y(double x, const Eigen::VectorXd& u_local,
                          const DistributedLoad& q_y, ReleaseCombo4DOF release) const;
    double compute_moment_z(double x, const Eigen::VectorXd& u_local,
                           const DistributedLoad& q_y, ReleaseCombo4DOF release) const;
    // ... similar for Vy, Mz, My, Mx
};
```

### Implementation: Axial Component (Example)

File: `cpp/src/internal_actions_axial.cpp`

```cpp
namespace grillex {

/**
 * @brief Compute axial force at position x
 *
 * Differential equation: dN/dx + q_x = 0, N = EA * du/dx
 * Solution: N(x) depends on end displacements u1, u2 and distributed load q_x
 */
class AxialForceComputer {
public:
    AxialForceComputer(double L, double EA, double u1, double u2,
                      double q1, double q2)
        : L_(L), EA_(EA), u1_(u1), u2_(u2), q1_(q1), q2_(q2) {}

    double compute(double x, ReleaseCombo2DOF release) const {
        switch (release) {
            case ReleaseCombo2DOF::FIXED_FIXED:
                return fixed_fixed(x);
            case ReleaseCombo2DOF::FIXED_FREE:
                return fixed_free(x);
            case ReleaseCombo2DOF::FREE_FIXED:
                return free_fixed(x);
            case ReleaseCombo2DOF::FREE_FREE:
                return 0.0;  // Rigid body motion, no internal force
        }
    }

private:
    double L_, EA_, u1_, u2_, q1_, q2_;

    // Analytical formulas derived from differential equations:

    double fixed_fixed(double x) const {
        // Both ends restrained axially
        // From pystructeng: N(x) = (6*EA*(-u1 + u2) + L*(2*L*q1 + L*q2 - 6*q1*x)
        //                           + 3*x^2*(q1 - q2)) / (6*L)
        return (6.0 * EA_ * (-u1_ + u2_)
                + L_ * (2.0 * L_ * q1_ + L_ * q2_ - 6.0 * q1_ * x)
                + 3.0 * x * x * (q1_ - q2_)) / (6.0 * L_);
    }

    double fixed_free(double x) const {
        // Start fixed, end free (cantilever-like for axial)
        // From pystructeng: N(x) = (L*(L*(q1 + q2) - 2*q1*x) + x^2*(q1 - q2)) / (2*L)
        return (L_ * (L_ * (q1_ + q2_) - 2.0 * q1_ * x) + x * x * (q1_ - q2_)) / (2.0 * L_);
    }

    double free_fixed(double x) const {
        // Start free, end fixed
        // From pystructeng: N(x) = x*(-2*L*q1 + x*(q1 - q2)) / (2*L)
        return x * (-2.0 * L_ * q1_ + x * (q1_ - q2_)) / (2.0 * L_);
    }
};

} // namespace grillex
```

### Implementation: Bending - Euler-Bernoulli (Example)

File: `cpp/src/internal_actions_bending_euler.cpp`

```cpp
namespace grillex {

/**
 * @brief Compute moment about z-axis (bending in x-y plane) - Euler-Bernoulli
 *
 * Differential equations:
 *   dV/dx + q = 0
 *   dM/dx - V = 0
 *   M = EI * d²w/dx²
 *
 * End conditions: w1, φ1 (slope), w2, φ2
 * Distributed load: q(x) = q1 + (q2 - q1) * x / L (trapezoidal)
 */
class MomentZEulerComputer {
public:
    MomentZEulerComputer(double L, double EI, double w1, double phi1,
                        double w2, double phi2, double q1, double q2)
        : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2),
          q1_(q1), q2_(q2) {}

    double compute(double x, ReleaseCombo4DOF release) const {
        switch (release) {
            case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
                return fixed_fixed_fixed_fixed(x);
            case ReleaseCombo4DOF::FIXED_FIXED_FREE_FIXED:
                return fixed_fixed_free_fixed(x);
            case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FREE:
                return fixed_fixed_fixed_free(x);
            // ... all 16 cases
            default:
                throw std::runtime_error("Release combination not implemented");
        }
    }

private:
    double L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    // Analytical formulas from differential equations
    // (Derived using symbolic math or from pystructeng reference)

    double fixed_fixed_fixed_fixed(double x) const {
        // Both displacement and rotation fixed at both ends
        // From pystructeng lines.py:5798
        return (-120.0 * EI_ * L_ * L_ * (2.0 * phi1_ + phi2_)
                + 360.0 * EI_ * L_ * (-w1_ + w2_)
                + L_ * L_ * L_ * (3.0 * L_ * L_ * q1_ + 2.0 * L_ * L_ * q2_ + 30.0 * q1_ * x * x)
                + 10.0 * L_ * L_ * x * x * x * (-q1_ + q2_)
                + 3.0 * x * (120.0 * EI_ * L_ * (phi1_ + phi2_) + 240.0 * EI_ * (w1_ - w2_)
                            - L_ * L_ * L_ * L_ * (7.0 * q1_ + 3.0 * q2_)))
               / (60.0 * L_ * L_ * L_);
    }

    double fixed_fixed_free_fixed(double x) const {
        // Displacement fixed at both ends, rotation fixed at i, free at j (hinge at j)
        // From pystructeng lines.py:5832
        return (-24.0 * EI_ * phi1_ + 24.0 * EI_ * phi2_
                + 3.0 * L_ * L_ * L_ * q1_ + 5.0 * L_ * L_ * L_ * q2_
                - 12.0 * L_ * x * (L_ * (q1_ + q2_) - q1_ * x)
                - 4.0 * x * x * x * (q1_ - q2_))
               / (24.0 * L_);
    }

    // ... Continue with all 16 release combinations
    // (See pystructeng lines.py:5749-6400 for complete set)
};

/**
 * @brief Compute shear force in y direction - Euler-Bernoulli
 *
 * V(x) = dM/dx (derivative of moment)
 */
class ShearYEulerComputer {
public:
    ShearYEulerComputer(double L, double EI, double w1, double phi1,
                       double w2, double phi2, double q1, double q2)
        : L_(L), EI_(EI), w1_(w1), phi1_(phi1), w2_(w2), phi2_(phi2),
          q1_(q1), q2_(q2) {}

    double compute(double x, ReleaseCombo4DOF release) const {
        // Shear is derivative of moment
        // Can compute analytically or use finite difference of moment formula
        // From pystructeng lines.py:4105-4700

        switch (release) {
            case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
                return fixed_fixed_fixed_fixed(x);
            // ... all 16 cases
            default:
                throw std::runtime_error("Release combination not implemented");
        }
    }

private:
    double L_, EI_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    double fixed_fixed_fixed_fixed(double x) const {
        // From pystructeng lines.py:4140
        return (-80.0 * EI_ * L_ * L_ * q1_ - 40.0 * EI_ * L_ * L_ * q2_
                + 120.0 * EI_ * L_ * (phi1_ + phi2_)
                + 240.0 * EI_ * (w1_ - w2_)
                - 7.0 * L_ * L_ * L_ * L_ * q1_ - 3.0 * L_ * L_ * L_ * L_ * q2_
                + 20.0 * L_ * q1_ * x * (12.0 * EI_ + L_ * L_)
                + 10.0 * x * x * (12.0 * EI_ + L_ * L_) * (-q1_ + q2_))
               / (20.0 * L_ * (12.0 * EI_ + L_ * L_));
    }

    // ... Continue with all 16 cases
};

} // namespace grillex
```

### Implementation: Bending - Timoshenko (Example)

File: `cpp/src/internal_actions_bending_timoshenko.cpp`

```cpp
namespace grillex {

/**
 * @brief Compute moment about z - Timoshenko beam theory
 *
 * Includes shear deformation effects. Formulas modified from Euler-Bernoulli
 * by including kAG (shear stiffness) term: (12*EI + L²*kAG)
 */
class MomentZTimoshenkoComputer {
public:
    MomentZTimoshenkoComputer(double L, double EI, double kAG,
                             double w1, double phi1, double w2, double phi2,
                             double q1, double q2)
        : L_(L), EI_(EI), kAG_(kAG), w1_(w1), phi1_(phi1),
          w2_(w2), phi2_(phi2), q1_(q1), q2_(q2) {}

    double compute(double x, ReleaseCombo4DOF release) const {
        // Similar to Euler, but with kAG terms
        // From pystructeng lines.py (Timoshenko variants)

        switch (release) {
            case ReleaseCombo4DOF::FIXED_FIXED_FIXED_FIXED:
                return fixed_fixed_fixed_fixed(x);
            // ... all 16 cases
            default:
                throw std::runtime_error("Release combination not implemented");
        }
    }

private:
    double L_, EI_, kAG_, w1_, phi1_, w2_, phi2_, q1_, q2_;

    double fixed_fixed_fixed_fixed(double x) const {
        // Timoshenko version includes kAG effects
        // Formula similar to Euler but denominators change
        // (Not directly in pystructeng - would need to derive or use similar approach)

        // Simplified version (full derivation needed):
        double denominator = 12.0 * EI_ + L_ * L_ * kAG_;

        // This is a placeholder - actual formula would follow from
        // Timoshenko differential equations with shear deformation
        return 0.0;  // TODO: Implement full Timoshenko formulas
    }

    // ... Continue with all 16 cases
};

} // namespace grillex
```

### Finding Moment Extrema

```cpp
std::pair<ActionExtreme, ActionExtreme> BeamElement::find_moment_extremes(
    char axis,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    // For bending with distributed load q(x), moment extremum occurs where V(x) = 0
    //
    // For parabolic moment distribution (linear load):
    //   M(x) = a + b*x + c*x²
    //   V(x) = dM/dx = b + 2*c*x
    //   Extremum at x = -b / (2*c)
    //
    // Algorithm:
    // 1. Compute shear V(x) symbolically or numerically
    // 2. Find roots of V(x) = 0 in domain [0, L]
    // 3. Evaluate M(x) at roots and endpoints
    // 4. Return min and max

    std::vector<double> critical_points;
    critical_points.push_back(0.0);    // Start
    critical_points.push_back(length_); // End

    // Find interior extrema (where V = 0)
    // For polynomial loads, this is analytical
    // For general loads, use numerical root finding

    // ... root finding logic ...

    ActionExtreme min_extremum, max_extremum;
    double M_min = std::numeric_limits<double>::infinity();
    double M_max = -std::numeric_limits<double>::infinity();

    for (double x : critical_points) {
        InternalActions actions = get_internal_actions(x, global_displacements, dof_handler);
        double M = (axis == 'y') ? actions.My : actions.Mz;

        if (M < M_min) {
            M_min = M;
            min_extremum = {x, M};
        }
        if (M > M_max) {
            M_max = M;
            max_extremum = {x, M};
        }
    }

    return {min_extremum, max_extremum};
}
```

### Integration with Model

The `Model` class needs to provide distributed loads to elements:

```cpp
class Model {
public:
    // ... existing methods ...

    /**
     * @brief Get distributed loads for an element (requires Phase 5)
     * @param element_id Element ID
     * @return Distributed loads in 3 directions
     */
    struct ElementLoads {
        DistributedLoad qx;  // Axial
        DistributedLoad qy;  // Lateral y
        DistributedLoad qz;  // Lateral z
    };

    ElementLoads get_element_distributed_loads(int element_id) const;

private:
    // Storage for distributed loads (from Phase 5)
    std::map<int, ElementLoads> element_loads_;
};
```

### Acceptance Criteria
- [ ] Simply supported beam with UDL: M_max = wL²/8 at midspan (within 0.1%)
- [ ] Cantilever with tip load: M_max = PL at support (exact)
- [ ] Cantilever with UDL: M_max = wL²/2 at support, M(L/2) = wL²/8 (within 0.1%)
- [ ] Fixed-fixed beam with UDL: M_ends = wL²/12, M_mid = wL²/24 (within 0.1%)
- [ ] All 16 release combinations produce physically correct results
- [ ] Shear and moment satisfy dM/dx = V at all points
- [ ] Extrema are found correctly (analytical vs numerical agreement)
- [ ] Euler-Bernoulli and Timoshenko results agree for slender beams (L/h > 20)
- [ ] Timoshenko shows increased deflection for short, deep beams

---

## Task 7.2b: Implement Warping Results (Bimoments)

**Requirements:** R-RES-006, R-ELEM-007
**Dependencies:** Task 7.1, Task 2.7 (warping DOF implementation)
**Difficulty:** High

### Description
Compute warping-related results for 14-DOF elements: bimoment, warping torsion, warping stress.

### Background

For thin-walled open sections (I-beams, channels) under torsion:
- **St. Venant torsion:** Mx_sv = GJ × dθ/dx (uniform shear stress)
- **Warping torsion:** Mx_w = -EIw × d³φ'/dx³ (non-uniform normal stress)
- **Total torsion:** Mx = Mx_sv + Mx_w
- **Bimoment:** B = EIw × d²φ'/dx² (generalized force conjugate to warping)
- **Warping normal stress:** σ_w = -B × ω / Iw (ω = sectorial coordinate)

### C++ Data Structures

```cpp
namespace grillex {

/**
 * @brief Warping-specific internal actions (extends InternalActions)
 */
struct WarpingInternalActions : InternalActions {
    double B;            // Bimoment [kN·m²]
    double Mx_sv;        // St. Venant torsion component [kN·m]
    double Mx_w;         // Warping torsion component [kN·m]
    double sigma_w_max;  // Maximum warping normal stress [kN/m²]
};

/**
 * @brief Release combinations for warping (2-DOF: φ'_i, φ'_j)
 */
enum class ReleaseComboWarping {
    FIXED_FIXED,  // Both ends restrained against warping
    FIXED_FREE,   // Start restrained, end free to warp
    FREE_FIXED,   // Start free, end restrained
    FREE_FREE     // Both ends free (pure St. Venant torsion)
};

} // namespace grillex
```

### Implementation

```cpp
class BeamElement {
public:
    /**
     * @brief Get warping internal actions at position x
     * @param x Position [0, L] in meters
     * @param global_displacements Full displacement vector
     * @param dof_handler DOF numbering manager
     * @return Warping-specific internal actions including bimoment
     * @throws std::runtime_error if element is not 14-DOF warping type
     */
    WarpingInternalActions get_warping_internal_actions(
        double x,
        const Eigen::VectorXd& global_displacements,
        const DOFHandler& dof_handler) const;

    /**
     * @brief Compute warping normal stress at position x and sectorial coordinate ω
     * @param x Position along beam [m]
     * @param omega Sectorial coordinate at point of interest [m²]
     * @param global_displacements Full displacement vector
     * @param dof_handler DOF numbering manager
     * @return Warping normal stress σ_w [kN/m²]
     */
    double compute_warping_stress(
        double x,
        double omega,
        const Eigen::VectorXd& global_displacements,
        const DOFHandler& dof_handler) const;

private:
    ReleaseComboWarping detect_release_combination_warping() const;

    double compute_bimoment(double x, const Eigen::VectorXd& u_local,
                           ReleaseComboWarping release) const;
};
```

File: `cpp/src/internal_actions_warping.cpp`

```cpp
namespace grillex {

/**
 * @brief Compute bimoment distribution B(x)
 *
 * Differential equation: d²B/dx² = -Mx_w = EIw * d⁴φ'/dx⁴
 *
 * For 14-DOF element with warping parameters φ'_i, φ'_j at ends:
 *   B(x) = analytical expression based on end conditions
 */
class BimomentComputer {
public:
    BimomentComputer(double L, double EIw, double GJ,
                    double theta_i, double phi_prime_i,
                    double theta_j, double phi_prime_j)
        : L_(L), EIw_(EIw), GJ_(GJ),
          theta_i_(theta_i), phi_prime_i_(phi_prime_i),
          theta_j_(theta_j), phi_prime_j_(phi_prime_j) {}

    double compute(double x, ReleaseComboWarping release) const {
        switch (release) {
            case ReleaseComboWarping::FIXED_FIXED:
                return fixed_fixed(x);
            case ReleaseComboWarping::FIXED_FREE:
                return fixed_free(x);
            case ReleaseComboWarping::FREE_FIXED:
                return free_fixed(x);
            case ReleaseComboWarping::FREE_FREE:
                return 0.0;  // Pure St. Venant, no warping restraint
        }
    }

private:
    double L_, EIw_, GJ_;
    double theta_i_, phi_prime_i_, theta_j_, phi_prime_j_;

    double fixed_fixed(double x) const {
        // Both ends restrained against warping
        // Bimoment varies cubically along length
        // From element stiffness matrix: B = EIw * d²φ'/dx²

        // Using shape functions for φ':
        // φ'(x) = N1*φ'_i + N2*dφ'/dx|_i + N3*φ'_j + N4*dφ'/dx|_j
        // where N1, N2, N3, N4 are cubic Hermite functions

        double xi = x / L_;  // Normalized position [0, 1]

        // Second derivatives of Hermite functions:
        double N1_dd = (6.0 - 12.0 * xi) / (L_ * L_);
        double N2_dd = (4.0 - 6.0 * xi) / L_;
        double N3_dd = (-6.0 + 12.0 * xi) / (L_ * L_);
        double N4_dd = (2.0 - 6.0 * xi) / L_;

        // From element equations, dφ'/dx|_i and dφ'/dx|_j are related to θ and φ'
        // (Need to extract from 14x14 stiffness matrix)

        // Simplified (requires full derivation):
        double phi_prime_dd = N1_dd * phi_prime_i_ + N3_dd * phi_prime_j_;

        return EIw_ * phi_prime_dd;
    }

    double fixed_free(double x) const {
        // Start restrained, end free to warp
        // Bimoment decreases from fixed end, zero at free end

        double xi = x / L_;
        return EIw_ * phi_prime_i_ * (1.0 - xi) * (1.0 - xi) / (L_ * L_);
    }

    double free_fixed(double x) const {
        // Start free, end restrained
        double xi = x / L_;
        return EIw_ * phi_prime_j_ * xi * xi / (L_ * L_);
    }
};

} // namespace grillex
```

### Warping Stress Computation

```cpp
double BeamElement::compute_warping_stress(
    double x,
    double omega,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    if (num_dofs() != 14) {
        throw std::runtime_error("Warping stress only for 14-DOF elements");
    }

    WarpingInternalActions actions = get_warping_internal_actions(
        x, global_displacements, dof_handler);

    // σ_w = -B * ω / Iw
    // omega is sectorial coordinate at point of interest (from section geometry)
    // For I-section: omega varies linearly across flange width

    double Iw = section_->Iw;
    if (Iw < 1e-12) {
        return 0.0;  // No warping stiffness, no warping stress
    }

    return -actions.B * omega / Iw;
}
```

### Acceptance Criteria
- [ ] Bimoment at warping-restrained end matches analytical solution for uniform torsion
- [ ] Warping-free end has B ≈ 0 (within numerical tolerance)
- [ ] For two-span continuous beam under torsion, bimoment is continuous at support
- [ ] Total normal stress σ_total = σ_axial + σ_bending + σ_warping is computed correctly
- [ ] For section with Iw = 0 (closed sections), bimoment results are zero
- [ ] Sign convention consistent with standard references (Kollbrunner & Hajdin)
- [ ] Comparison with analytical solution for cantilever I-beam under torsion

---

## Task 7.2c: Implement Displacement/Rotation Lines

**Requirements:** R-RES-002
**Dependencies:** Task 7.2
**Difficulty:** Medium

### Description
Provide methods to query displacements and rotations at any position along the element, similar to internal actions. Uses the same differential equation approach.

### C++ Data Structures

```cpp
namespace grillex {

/**
 * @brief Displacements and rotations at a position along the beam
 */
struct DisplacementLine {
    double x;    // Position along beam [m]
    double u;    // Axial displacement [m]
    double v;    // Lateral displacement in y [m]
    double w;    // Lateral displacement in z [m]
    double θx;   // Twist rotation (torsion) [rad]
    double θy;   // Bending rotation about y [rad]
    double θz;   // Bending rotation about z [rad]

    // For 14-DOF elements:
    double φ_prime = 0.0;  // Warping parameter [rad]
};

} // namespace grillex
```

### Implementation

```cpp
class BeamElement {
public:
    /**
     * @brief Get displacements/rotations at position x
     * @param x Position [0, L] in meters
     * @param global_displacements Full displacement vector
     * @param dof_handler DOF numbering manager
     * @return Displacements and rotations at x
     */
    DisplacementLine get_displacements_at(
        double x,
        const Eigen::VectorXd& global_displacements,
        const DOFHandler& dof_handler) const;
};
```

The implementation uses Hermite shape functions (already implicit in stiffness matrix):

```cpp
DisplacementLine BeamElement::get_displacements_at(
    double x,
    const Eigen::VectorXd& global_displacements,
    const DOFHandler& dof_handler) const
{
    // 1. Extract element DOFs in local coordinates
    Eigen::VectorXd u_local = get_element_displacements_local(global_displacements, dof_handler);

    // 2. Use shape functions to interpolate
    double xi = x / length_;  // Normalized position [0, 1]

    DisplacementLine result;
    result.x = x;

    // Axial (linear interpolation):
    result.u = (1.0 - xi) * u_local[0] + xi * u_local[6];

    // Lateral and bending (cubic Hermite for bending, linear for twist):
    // ... shape function interpolation ...

    return result;
}
```

### Acceptance Criteria
- [ ] Displacements at element ends match nodal values exactly
- [ ] Deflection shape for cantilever with tip load matches analytical curve
- [ ] Rotation φ_z = dw/dy for Euler-Bernoulli beams
- [ ] For Timoshenko, φ_z ≠ dw/dy (shear deformation included)

---

## Task 7.2f: Multi-Element Beam Plotting and Continuous Lines

**Requirements:** R-RES-002, R-MOD-006
**Dependencies:** Task 7.2, Phase 4 (Python front-end)
**Difficulty:** Medium

### Description
Enable plotting of continuous internal action diagrams across beams consisting of multiple `BeamElement`s. This is a **Python-level** feature that aggregates results from individual elements.

### Architecture

**Key distinction:**
- **BeamElement** (C++): Single finite element, provides `get_internal_actions(x_local, ...)`
- **Beam** (Python): Aggregation of multiple `BeamElement`s, provides continuous lines for plotting

### Python Implementation

File: `src/grillex/beam.py` (new file in Phase 4)

```python
"""
Beam class - Python-level abstraction for multi-element beams
"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

from grillex.core import BeamElement, Model, InternalActions, DOFIndex


@dataclass
class BeamResultLine:
    """Results along a beam for plotting"""
    x_positions: np.ndarray  # Positions along beam [m]
    values: np.ndarray       # Values (moment, shear, etc.)
    component: str           # 'Mz', 'My', 'Vy', 'Vz', 'N', 'Mx'
    units: str              # 'kN⋅m', 'kN', etc.

    # Metadata for enhanced plotting
    extrema: List[Tuple[float, float]]  # [(x, value), ...] for max/min
    element_boundaries: List[float]     # x-positions of element joints
    discontinuities: List[float]        # x-positions of jumps (concentrated loads)

    def plot(self, ax=None, **kwargs):
        """Convenience method to plot this line"""
        import matplotlib.pyplot as plt

        if ax is None:
            fig, ax = plt.subplots()

        ax.plot(self.x_positions, self.values, label=self.component, **kwargs)

        # Mark extrema
        for x_ext, val_ext in self.extrema:
            ax.plot(x_ext, val_ext, 'ro', markersize=8)
            ax.annotate(f'{val_ext:.2f}', xy=(x_ext, val_ext),
                       xytext=(5, 5), textcoords='offset points')

        # Mark element boundaries
        for x_bound in self.element_boundaries:
            ax.axvline(x_bound, color='gray', linestyle=':', alpha=0.5)

        ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('Position along beam [m]')
        ax.set_ylabel(f'{self.component} [{self.units}]')

        return ax


class Beam:
    """
    Python-level beam consisting of multiple BeamElements

    A Beam aggregates multiple connected BeamElements to provide:
    - Continuous internal action diagrams across element boundaries
    - Convenient plotting methods
    - Extrema finding across the entire beam
    """

    def __init__(self, elements: List[BeamElement]):
        """
        Create a Beam from a list of BeamElements

        Parameters:
            elements: List of connected BeamElements (must share nodes end-to-end)
        """
        self.elements = elements
        self.length = sum(elem.length() for elem in elements)
        self._validate_connectivity()

    def _validate_connectivity(self):
        """Check that elements are connected end-to-end"""
        for i in range(len(self.elements) - 1):
            if self.elements[i].node_j.id != self.elements[i+1].node_i.id:
                raise ValueError(f"Elements {i} and {i+1} are not connected")

    def _find_element_at_position(self, x_global: float) -> Tuple[BeamElement, float]:
        """
        Find which element contains x_global

        Returns:
            (element, x_local): Element and local position within that element
        """
        if x_global < 0 or x_global > self.length:
            raise ValueError(f"Position {x_global} outside beam length {self.length}")

        cumulative_length = 0.0

        for element in self.elements:
            elem_length = element.length()
            if x_global <= cumulative_length + elem_length + 1e-10:  # tolerance
                x_local = x_global - cumulative_length
                return element, x_local
            cumulative_length += elem_length

        # Edge case: x_global == total length
        return self.elements[-1], self.elements[-1].length()

    def get_internal_actions_at(
        self,
        x_global: float,
        model: Model
    ) -> InternalActions:
        """
        Query internal actions at any position along the entire beam

        Parameters:
            x_global: Position along beam [0, L_total]
            model: Model object (for accessing displacements and dof_handler)

        Returns:
            Internal actions (N, Vy, Vz, Mx, My, Mz) at x_global
        """
        element, x_local = self._find_element_at_position(x_global)

        return element.get_internal_actions(
            x_local,
            model.get_displacements(),
            model.get_dof_handler()
        )

    def get_moment_line(
        self,
        axis: str,
        model: Model,
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get moment diagram for plotting

        Parameters:
            axis: 'y' or 'z' for bending plane
            model: Model object
            num_points: Number of points to sample along beam

        Returns:
            (x_positions, moments): Arrays for plotting
        """
        x_positions = np.linspace(0, self.length, num_points)
        moments = np.array([
            self.get_internal_actions_at(x, model).My if axis == 'y'
            else self.get_internal_actions_at(x, model).Mz
            for x in x_positions
        ])

        return x_positions, moments

    def get_shear_line(
        self,
        axis: str,
        model: Model,
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get shear diagram for plotting"""
        x_positions = np.linspace(0, self.length, num_points)
        shears = np.array([
            self.get_internal_actions_at(x, model).Vy if axis == 'y'
            else self.get_internal_actions_at(x, model).Vz
            for x in x_positions
        ])

        return x_positions, shears

    def get_axial_force_line(
        self,
        model: Model,
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get axial force diagram for plotting"""
        x_positions = np.linspace(0, self.length, num_points)
        axial_forces = np.array([
            self.get_internal_actions_at(x, model).N
            for x in x_positions
        ])

        return x_positions, axial_forces

    def get_deflection_line(
        self,
        axis: str,
        model: Model,
        num_points: int = 100
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get deflection diagram for plotting"""
        x_positions = np.linspace(0, self.length, num_points)

        deflections = []
        cumulative_length = 0.0

        for element in self.elements:
            elem_length = element.length()
            # Sample this element
            elem_points = [x for x in x_positions
                          if cumulative_length <= x <= cumulative_length + elem_length]

            for x in elem_points:
                x_local = x - cumulative_length
                disp_line = element.get_displacements_at(
                    x_local,
                    model.get_displacements(),
                    model.get_dof_handler()
                )
                deflections.append(disp_line.v if axis == 'y' else disp_line.w)

            cumulative_length += elem_length

        return x_positions, np.array(deflections)

    def find_moment_extrema(
        self,
        axis: str,
        model: Model
    ) -> List[Tuple[float, float]]:
        """
        Find all moment extrema across entire beam

        Returns:
            List of (x_global, moment_value) for all local max/min
        """
        extrema = []
        cumulative_length = 0.0

        for element in self.elements:
            # Find extrema within this element
            elem_extrema = element.find_moment_extremes(
                axis,
                model.get_displacements(),
                model.get_dof_handler()
            )

            # Convert to global coordinates
            extrema.append((cumulative_length + elem_extrema[0].x, elem_extrema[0].value))
            extrema.append((cumulative_length + elem_extrema[1].x, elem_extrema[1].value))

            cumulative_length += element.length()

        # Also check element boundaries (concentrated loads can create extrema)
        cumulative_length = 0.0
        for i, element in enumerate(self.elements):
            if i > 0:  # Interior boundaries
                actions = self.get_internal_actions_at(cumulative_length, model)
                M = actions.My if axis == 'y' else actions.Mz
                extrema.append((cumulative_length, M))
            cumulative_length += element.length()

        # Filter for actual extrema (not all critical points are extrema)
        return self._filter_extrema(extrema)

    def _filter_extrema(self, critical_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Filter critical points to keep only local max/min"""
        if len(critical_points) < 3:
            return critical_points

        extrema = []
        critical_points.sort(key=lambda p: p[0])  # Sort by x

        for i in range(len(critical_points)):
            if i == 0 or i == len(critical_points) - 1:
                # Endpoints are always included
                extrema.append(critical_points[i])
            else:
                # Check if local extremum
                val_prev = critical_points[i-1][1]
                val_curr = critical_points[i][1]
                val_next = critical_points[i+1][1]

                if (val_curr > val_prev and val_curr > val_next) or \
                   (val_curr < val_prev and val_curr < val_next):
                    extrema.append(critical_points[i])

        return extrema

    def plot_internal_actions(
        self,
        model: Model,
        components: List[str] = ['Mz', 'Vy', 'N'],
        figsize: Tuple[float, float] = (12, 8)
    ):
        """
        Create matplotlib plots of internal actions

        Parameters:
            model: Model object (must be analyzed)
            components: List of components to plot ('Mz', 'My', 'Vy', 'Vz', 'N', 'Mx')
            figsize: Figure size (width, height)

        Returns:
            Figure object
        """
        import matplotlib.pyplot as plt

        num_plots = len(components)
        fig, axes = plt.subplots(num_plots, 1, figsize=figsize, squeeze=False)
        axes = axes.flatten()

        for ax, component in zip(axes, components):
            # Get line data
            if component == 'Mz':
                x, values = self.get_moment_line('z', model)
                units = 'kN⋅m'
            elif component == 'My':
                x, values = self.get_moment_line('y', model)
                units = 'kN⋅m'
            elif component == 'Vy':
                x, values = self.get_shear_line('y', model)
                units = 'kN'
            elif component == 'Vz':
                x, values = self.get_shear_line('z', model)
                units = 'kN'
            elif component == 'N':
                x, values = self.get_axial_force_line(model)
                units = 'kN'
            elif component == 'Mx':
                x_positions = np.linspace(0, self.length, 100)
                values = np.array([
                    self.get_internal_actions_at(x, model).Mx
                    for x in x_positions
                ])
                x = x_positions
                units = 'kN⋅m'
            else:
                raise ValueError(f"Unknown component: {component}")

            # Plot
            ax.plot(x, values, 'b-', linewidth=2)
            ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
            ax.grid(True, alpha=0.3)
            ax.set_ylabel(f'{component} [{units}]')
            ax.set_title(f'{component} diagram')

            # Mark element boundaries
            cumulative_length = 0.0
            for i, element in enumerate(self.elements[:-1]):
                cumulative_length += element.length()
                ax.axvline(cumulative_length, color='gray', linestyle=':', alpha=0.5,
                          label='Element boundary' if i == 0 else '')

            # Mark extrema for moment diagrams
            if component in ['Mz', 'My']:
                axis_char = component[1].lower()
                extrema = self.find_moment_extrema(axis_char, model)
                for x_ext, val_ext in extrema:
                    ax.plot(x_ext, val_ext, 'ro', markersize=6)
                    ax.annotate(f'{val_ext:.2f}', xy=(x_ext, val_ext),
                               xytext=(5, 5), textcoords='offset points',
                               fontsize=8)

        axes[-1].set_xlabel('Position along beam [m]')
        plt.tight_layout()

        return fig
```

### Handling Discontinuities

For concentrated loads at element joints, the shear force diagram will have jumps. To capture this:

```python
def get_moment_line_with_discontinuities(
    self,
    axis: str,
    model: Model,
    num_points_per_element: int = 50
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get moment line accounting for discontinuities

    Samples each element separately to properly capture jumps
    in shear at concentrated loads
    """
    x_all = []
    M_all = []

    cumulative_length = 0.0

    for i, element in enumerate(self.elements):
        L_elem = element.length()

        # Sample this element
        x_local = np.linspace(0, L_elem, num_points_per_element)
        x_global = x_local + cumulative_length

        M_local = []
        for x_l in x_local:
            actions = element.get_internal_actions(
                x_l,
                model.get_displacements(),
                model.get_dof_handler()
            )
            M_local.append(actions.My if axis == 'y' else actions.Mz)

        x_all.extend(x_global)
        M_all.extend(M_local)

        cumulative_length += L_elem

    return np.array(x_all), np.array(M_all)
```

### Usage Example

```python
from grillex import Model, DOFIndex
from grillex.beam import Beam
import matplotlib.pyplot as plt

# Create model with multi-element beam
model = Model()

# Create 3 elements forming one logical beam
nodes = [model.get_or_create_node(i*2.0, 0, 0) for i in range(4)]
material = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
section = model.create_section("IPE200", 0.01, 1e-5, 2e-5, 1.5e-5)

# Create 3 beam elements
beam_elements = []
for i in range(3):
    elem = model.create_beam(nodes[i], nodes[i+1], material, section)
    beam_elements.append(elem)

# Apply BCs and loads
model.boundary_conditions.fix_node(nodes[0].id)
model.boundary_conditions.pin_node(nodes[3].id)
model.add_nodal_load(nodes[1].id, DOFIndex.UY, -10.0)  # Concentrated load
model.add_nodal_load(nodes[2].id, DOFIndex.UY, -5.0)

# Analyze
model.analyze()

# Create Beam abstraction (aggregates the 3 elements)
beam = Beam(beam_elements)

# Plot all diagrams
fig = beam.plot_internal_actions(model, components=['Mz', 'Vy', 'N'])
plt.show()

# Or manually plot with customization
x, Mz = beam.get_moment_line('z', model, num_points=200)
plt.figure()
plt.plot(x, Mz, 'b-', linewidth=2)
plt.xlabel('Position [m]')
plt.ylabel('Moment Mz [kN⋅m]')
plt.title('Bending Moment Diagram')
plt.grid(True)
plt.show()
```

### Acceptance Criteria
- [ ] Continuous moment diagram across 3-element beam matches hand calculation
- [ ] Element boundaries are clearly marked in plots
- [ ] Concentrated loads cause visible shear discontinuities
- [ ] Extrema are found and marked correctly across element boundaries
- [ ] Deflection diagram is smooth and continuous
- [ ] Works with beams of varying element counts (2 to 10+ elements)

---

## Task 7.3: Implement Check Locations

**Requirements:** R-RES-005, R-CODE-003
**Dependencies:** Task 7.2
**Difficulty:** Low

### Description
Support user-defined check locations on beams for design code verification.

### Python Implementation

```python
class Beam:
    """Beam class (from Task 7.2f)"""

    def __init__(self, elements: List[BeamElement]):
        self.elements = elements
        self.length = sum(elem.length() for elem in elements)
        self.check_locations: List[float] = []  # Normalized positions [0, 1]

    def add_check_location(self, x_normalized: float) -> None:
        """
        Add a check location at normalized position

        Parameters:
            x_normalized: Position (0=start, 1=end)
        """
        if not 0.0 <= x_normalized <= 1.0:
            raise ValueError("Check location must be in range [0, 1]")
        self.check_locations.append(x_normalized)

    def set_standard_check_locations(self) -> None:
        """Set standard check locations: ends, midspan, and quarter points"""
        self.check_locations = [0.0, 0.25, 0.5, 0.75, 1.0]

    def get_internal_actions_at_check_locations(
        self,
        model: Model
    ) -> List[Tuple[float, InternalActions]]:
        """
        Get internal actions at all check locations

        Returns:
            List of (x_position, internal_actions)
        """
        results = []
        for x_norm in self.check_locations:
            x_global = x_norm * self.length
            actions = self.get_internal_actions_at(x_global, model)
            results.append((x_global, actions))
        return results
```

### Acceptance Criteria
- [ ] Check locations can be added at arbitrary normalized positions
- [ ] Standard check locations (0, 0.25, 0.5, 0.75, 1) can be set automatically
- [ ] Internal actions are computed correctly at check locations
- [ ] Check locations persist across multiple analyses

---

## Testing Strategy

### Unit Tests for Release Combinations

Create comprehensive tests for all formula combinations:

**Test file:** `tests/python/test_phase7_internal_actions.py`

```python
import pytest
import numpy as np
from grillex.core import Model, Material, Section, DOFIndex, BeamFormulation


class TestAxialForceFormulas:
    """Test all axial force release combinations"""

    def test_axial_fixed_fixed_uniform_load(self):
        """Axial: both ends fixed, uniform distributed load"""
        # Setup model with known solution
        # ...
        # Verify N(x) matches analytical formula
        pass

    def test_axial_cantilever(self):
        """Axial: fixed-free (cantilever)"""
        # ...
        pass


class TestBendingEulerBernoulli:
    """Test Euler-Bernoulli bending formulas"""

    def test_cantilever_tip_load(self):
        """Cantilever with point load at tip"""
        model = Model()
        n1 = model.get_or_create_node(0, 0, 0)
        n2 = model.get_or_create_node(6, 0, 0)
        mat = model.create_material("Steel", 210e6, 0.3, 7.85e-6)
        sec = model.create_section("Test", 0.01, 1e-5, 1e-5, 1e-5)

        elem = model.create_beam(n1, n2, mat, sec)
        model.boundary_conditions.fix_node(n1.id)
        model.add_nodal_load(n2.id, DOFIndex.UY, -10.0)

        model.analyze()

        # Check moment at support: M = P * L = 10 * 6 = 60 kN⋅m
        actions_support = elem.get_internal_actions(0, model.get_displacements(),
                                                    model.get_dof_handler())
        assert abs(actions_support.Mz - 60.0) < 0.01

        # Check moment at tip: M = 0
        actions_tip = elem.get_internal_actions(6, model.get_displacements(),
                                                model.get_dof_handler())
        assert abs(actions_tip.Mz) < 1e-6

    def test_simply_supported_udl(self):
        """Simply supported beam with uniform distributed load"""
        # Analytical: M_max = w * L² / 8 at midspan
        # ...
        pass

    def test_fixed_fixed_udl(self):
        """Fixed-fixed beam with UDL"""
        # Analytical: M_support = -w*L²/12, M_midspan = w*L²/24
        # ...
        pass


class TestBendingTimoshenko:
    """Test Timoshenko bending formulas"""

    def test_short_deep_beam_shear_deformation(self):
        """Short beam shows increased deflection due to shear"""
        # Create two models: one Euler, one Timoshenko
        # Compare deflections - Timoshenko should be larger
        # ...
        pass


class TestMomentExtrema:
    """Test finding moment extrema"""

    def test_simply_supported_point_load_at_midspan(self):
        """Extremum at load point for simply supported beam"""
        # ...
        pass


class TestMultiElementBeam:
    """Test continuous lines across multiple elements"""

    def test_three_element_beam_moment_continuity(self):
        """Moment is continuous across element boundaries"""
        # ...
        pass
```

### Comparison with Analytical Solutions

Create validation tests comparing with known solutions:

```python
class TestAnalyticalValidation:
    """Compare with classical beam theory solutions"""

    @pytest.mark.parametrize("L,P,EI,expected_delta", [
        (1.0, 1.0, 210e6*1e-5, -1.0**3/(3*210e6*1e-5)),  # Cantilever tip load
        (2.0, 5.0, 210e6*2e-5, -5.0*2.0**3/(3*210e6*2e-5)),
    ])
    def test_cantilever_tip_deflection(self, L, P, EI, expected_delta):
        """Cantilever tip deflection: δ = P*L³/(3*EI)"""
        # ...
        assert abs(computed_delta - expected_delta) / abs(expected_delta) < 0.01
```

### Performance Tests

```python
class TestPerformance:
    """Test computational efficiency"""

    def test_internal_actions_query_performance(self):
        """Query internal actions at 1000 points should be fast"""
        import time
        # ...
        assert elapsed_time < 0.1  # Should complete in < 100ms
```

---

## Formula Reference Tables

### Axial Force Formulas (2-DOF)

| Release Combo | Formula N(x) |
|---------------|--------------|
| Fixed-Fixed | `(6*EA*(-u1 + u2) + L*(2*L*q1 + L*q2 - 6*q1*x) + 3*x²*(q1 - q2)) / (6*L)` |
| Fixed-Free | `(L*(L*(q1 + q2) - 2*q1*x) + x²*(q1 - q2)) / (2*L)` |
| Free-Fixed | `x*(-2*L*q1 + x*(q1 - q2)) / (2*L)` |
| Free-Free | `0` (rigid body) |

### Bending Moment Formulas - Euler-Bernoulli (4-DOF)

*Complete formulas for all 16 combinations referenced from pystructeng `lines.py:5749-6400`*

**Example: Fixed-Fixed-Fixed-Fixed**
```
Mz(x) = (-120*EI*L²*(2*φ1 + φ2) + 360*EI*L*(-w1 + w2)
         + L³*(3*L²*q1 + 2*L²*q2 + 30*q1*x²)
         + 10*L²*x³*(-q1 + q2)
         + 3*x*(120*EI*L*(φ1 + φ2) + 240*EI*(w1 - w2) - L⁴*(7*q1 + 3*q2)))
       / (60*L³)
```

*(Full table would include all 16 formulas - omitted for brevity)*

### Shear Force Formulas - Euler-Bernoulli

*Shear formulas are derivatives of moment formulas: V(x) = dM/dx*

### Timoshenko Corrections

For Timoshenko beams, denominators include shear term:
- Replace `EI` with `EI / (1 + 12*EI/(kAG*L²))`
- Or equivalently, use factor `(12*EI + L²*kAG)`

---

## Documentation Requirements

### User Guide Section

Add to user documentation:

```markdown
# Internal Actions and Results

## Querying Internal Actions

After running `model.analyze()`, you can query internal actions at any position along beam elements:

```python
# Single element
elem = model.elements[0]
actions = elem.get_internal_actions(
    x=3.0,  # Position 3m from element start
    global_displacements=model.get_displacements(),
    dof_handler=model.get_dof_handler()
)

print(f"Axial force: {actions.N} kN")
print(f"Shear Vy: {actions.Vy} kN")
print(f"Moment Mz: {actions.Mz} kN⋅m")
```

## Multi-Element Beams and Plotting

For beams consisting of multiple elements:

```python
from grillex.beam import Beam

# Create Beam from elements
beam = Beam([elem1, elem2, elem3])

# Plot internal actions
fig = beam.plot_internal_actions(model, components=['Mz', 'Vy', 'N'])
plt.show()

# Find extrema
extrema = beam.find_moment_extrema('z', model)
for x, M in extrema:
    print(f"Extremum at x={x}m: M={M} kN⋅m")
```

## Design Code Checks

Use check locations for design verification:

```python
beam.set_standard_check_locations()  # 0, 0.25, 0.5, 0.75, 1.0
results = beam.get_internal_actions_at_check_locations(model)

for x, actions in results:
    # Perform code checks at each location
    unity_check = verify_eurocode3(actions, section, material)
    print(f"x={x}m: Unity check = {unity_check}")
```
```

### API Reference

Document all new classes and methods in API reference:

```markdown
## BeamElement Methods

### get_internal_actions

```python
BeamElement.get_internal_actions(
    x: float,
    global_displacements: VectorXd,
    dof_handler: DOFHandler
) -> InternalActions
```

Get internal actions at position `x` along element.

**Parameters:**
- `x`: Position [0, L] in meters
- `global_displacements`: Global displacement vector from analysis
- `dof_handler`: DOF numbering manager

**Returns:**
- `InternalActions`: Struct with N, Vy, Vz, Mx, My, Mz

**Raises:**
- `RuntimeError`: If model not analyzed

### find_moment_extremes

```python
BeamElement.find_moment_extremes(
    axis: str,
    global_displacements: VectorXd,
    dof_handler: DOFHandler
) -> Tuple[ActionExtreme, ActionExtreme]
```

Find minimum and maximum moment along element.

**Parameters:**
- `axis`: 'y' or 'z' for bending plane
- ...

**Returns:**
- `(min_extreme, max_extreme)`: Pair of extrema with positions and values
```

---

## Implementation Priority

### Phased Approach

**Phase 7a (MVP - Can implement NOW):**
- Task 7.1: Element end forces using basic `f = K*u`
- Simple linear interpolation for internal actions (no distributed loads)
- **Deliverable**: Can compute and plot element forces for point loads

**Phase 7b (Enhanced - requires Phase 5 complete):**
- Implement Phase 5 distributed load infrastructure
- Update Task 7.1 to handle fixed-end forces
- **Deliverable**: Accurate end forces with distributed loads

**Phase 7c (Full differential equations - recommended):**
- Implement all release combination formulas (Euler-Bernoulli)
- Task 7.2: Complete internal action computation
- Task 7.2f: Multi-element beam plotting
- **Deliverable**: Production-ready internal actions

**Phase 7d (Timoshenko - optional):**
- Implement Timoshenko formulations
- **Deliverable**: Accurate for short, deep beams

**Phase 7e (Warping - optional):**
- Task 7.2b: Bimoment computation
- **Deliverable**: Full 14-DOF warping analysis

---

## Summary

Phase 7 implements a comprehensive internal action computation system using analytical solutions from differential equations. This provides:

✅ **Accuracy**: Exact for polynomial distributed loads
✅ **Flexibility**: Handles all release combinations
✅ **Completeness**: Both Euler-Bernoulli and Timoshenko
✅ **Usability**: Convenient Python API with plotting
✅ **Extensibility**: Ready for warping/bimoment analysis

The differential equation approach is more complex to implement than simple interpolation, but provides superior accuracy and properly accounts for the underlying beam physics.
