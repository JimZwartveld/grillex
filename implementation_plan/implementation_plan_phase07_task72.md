# Phase 7: Task 7.2 - Internal Action Functions

**Extracted from:** `implementation_plan_phase07.md`
**Purpose:** Detailed implementation plan for internal action computation along beam elements

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
- **Warping torsion:** Mx_w = -EIw × d³θ/dx³ (non-uniform normal stress)
- **Total torsion:** Mx = Mx_sv + Mx_w
- **Bimoment:** B = -EIw × d²θ/dx² (generalized force conjugate to warping)
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
 * @brief Release combinations for warping (4-DOF: θ₁, φ₁, θ₂, φ₂)
 *
 * θ = twist angle (rotation DOF)
 * φ = rate of twist / warping DOF (dθ/dx at restrained end)
 */
enum class ReleaseComboWarping {
    // All 16 combinations of fixed/free for (θ₁, φ₁, θ₂, φ₂)
    FIXED_FIXED_FIXED_FIXED,  // θ₁, φ₁, θ₂, φ₂ all fixed
    FIXED_FIXED_FIXED_FREE,   // θ₁, φ₁, θ₂ fixed; φ₂ free
    FIXED_FIXED_FREE_FIXED,   // θ₁, φ₁, φ₂ fixed; θ₂ free
    FIXED_FIXED_FREE_FREE,    // θ₁, φ₁ fixed; θ₂, φ₂ free
    FIXED_FREE_FIXED_FIXED,   // θ₁, θ₂, φ₂ fixed; φ₁ free
    FIXED_FREE_FIXED_FREE,    // θ₁, θ₂ fixed; φ₁, φ₂ free (pure St. Venant)
    FIXED_FREE_FREE_FIXED,    // θ₁, φ₂ fixed; φ₁, θ₂ free
    FIXED_FREE_FREE_FREE,     // θ₁ fixed; φ₁, θ₂, φ₂ free
    FREE_FIXED_FIXED_FIXED,   // φ₁, θ₂, φ₂ fixed; θ₁ free
    FREE_FIXED_FIXED_FREE,    // φ₁, θ₂ fixed; θ₁, φ₂ free
    FREE_FIXED_FREE_FIXED,    // φ₁, φ₂ fixed; θ₁, θ₂ free
    FREE_FIXED_FREE_FREE,     // φ₁ fixed; θ₁, θ₂, φ₂ free
    FREE_FREE_FIXED_FIXED,    // θ₂, φ₂ fixed; θ₁, φ₁ free
    FREE_FREE_FIXED_FREE,     // θ₂ fixed; θ₁, φ₁, φ₂ free
    FREE_FREE_FREE_FIXED,     // φ₂ fixed; θ₁, φ₁, θ₂ free
    FREE_FREE_FREE_FREE       // All free (rigid body motion)
};

} // namespace grillex
```

### Governing Differential Equation

The warping torsion is governed by the 4th order differential equation:

$$EI_\omega \frac{d^4\theta}{dx^4} - GJ \frac{d^2\theta}{dx^2} = m_x(x)$$

For the homogeneous case (concentrated end moments/rotations only):

$$\frac{d^4\theta}{dx^4} - k^2 \frac{d^2\theta}{dx^2} = 0$$

where the **warping parameter** is:

$$k = \sqrt{\frac{GJ}{EI_\omega}}$$

The **general solution** is:

$$\theta(x) = C_1 + C_2 x + C_3\cosh(kx) + C_4\sinh(kx)$$

### Boundary Conditions

| Condition | Mathematical Form | Physical Meaning |
|-----------|-------------------|------------------|
| Fixed rotation (θ) | θ = θ_prescribed | Twist angle prescribed |
| Fixed warping (φ) | dθ/dx = φ_prescribed | Rate of twist / warping restrained |
| Free warping | B = -EIω d²θ/dx² = 0 | Cross-section free to warp |

### Analytical Solutions - All 16 Boundary Condition Combinations

**Notation:**
- θ₁, θ₂ = twist angles at x=0 and x=L
- φ₁, φ₂ = rates of twist (dθ/dx) at x=0 and x=L
- k = √(GJ/EIω) = warping parameter
- L = beam length
- EIω = warping stiffness
- GJ = St. Venant torsional stiffness

**Derived quantities:**
- θ(x) = twist angle distribution
- φ(x) = dθ/dx = rate of twist
- Ts = GJ·φ = St. Venant torsion
- Tw = -EIω·k²·d²θ/dx² (excluding linear part) = Warping torsion
- T = Ts + Tw = Total torsion
- Bw = -EIω·d²θ/dx² = Bimoment

---

#### Case 1: θ₁=fixed, φ₁=fixed, θ₂=fixed, φ₂=fixed

**Common denominator:** Δ = L·k²·sinh(kL) - k·sinh²(kL) + k·cosh²(kL) - 2k·cosh(kL) + k

Note: Using identity cosh²(x) - sinh²(x) = 1, this simplifies to:
Δ = k·(L·k·sinh(kL) - 2·cosh(kL) + 2)

**Twist angle θ(x):**
```
θ(x) = [L·k²·θ₁·sinh(kL) + L·k·φ₁·cosh(kL) - L·k·φ₂
        - k·θ₁·sinh²(kL) + k·θ₁·cosh²(kL) - k·θ₁·cosh(kL)
        - k·θ₂·cosh(kL) + k·θ₂ - φ₁·sinh(kL) + φ₂·sinh(kL)] / Δ
      + x·[-k·θ₁·sinh(kL) + k·θ₂·sinh(kL) - φ₁·sinh²(kL) + φ₁·cosh²(kL)
           - φ₁·cosh(kL) - φ₂·cosh(kL) + φ₂] / Δ'
      + sinh(kx)·[L·k·φ₁·sinh(kL) + k·θ₁·sinh(kL) - k·θ₂·sinh(kL)
                  - φ₁·cosh(kL) + φ₁ + φ₂·cosh(kL) - φ₂] / Δ
      + cosh(kx)·[-L·k·φ₁·cosh(kL) + L·k·φ₂ - k·θ₁·cosh(kL) + k·θ₁
                  + k·θ₂·cosh(kL) - k·θ₂ + φ₁·sinh(kL) - φ₂·sinh(kL)] / Δ
```

where Δ' = L·k·sinh(kL) - sinh²(kL) + cosh²(kL) - 2·cosh(kL) + 1

**Rate of twist φ(x) = dθ/dx:**
```
φ(x) = [-k·θ₁·sinh(kL) + k·θ₂·sinh(kL) - φ₁·sinh²(kL) + φ₁·cosh²(kL)
        - φ₁·cosh(kL) - φ₂·cosh(kL) + φ₂] / Δ'
      + k·cosh(kx)·[L·k·φ₁·sinh(kL) + k·θ₁·sinh(kL) - k·θ₂·sinh(kL)
                    - φ₁·cosh(kL) + φ₁ + φ₂·cosh(kL) - φ₂] / Δ
      + k·sinh(kx)·[-L·k·φ₁·cosh(kL) + L·k·φ₂ - k·θ₁·cosh(kL) + k·θ₁
                    + k·θ₂·cosh(kL) - k·θ₂ + φ₁·sinh(kL) - φ₂·sinh(kL)] / Δ
```

**St. Venant torsion Ts(x) = GJ·φ(x)**

**Warping torsion Tw(x) = -EIω·k²·(k·C₃·sinh(kx) + k·C₄·cosh(kx))**

**Total torsion T(x) = Ts(x) + Tw(x)**

**Bimoment Bw(x) = -EIω·k²·(C₃·cosh(kx) + C₄·sinh(kx))**

---

#### Case 2: θ₁=fixed, φ₁=fixed, θ₂=fixed, φ₂=free

**Common denominator:** Δ = L·k·cosh(kL) - sinh(kL)

**Twist angle θ(x):**
```
θ(x) = L·k·θ₁·cosh(kL)/Δ + L·φ₁·sinh(kL)/Δ - θ₂·sinh(kL)/Δ
      + x·[-k·θ₁·cosh(kL) + k·θ₂·cosh(kL) - φ₁·sinh(kL)] / Δ
      + cosh(kx)·[-L·φ₁·sinh(kL) - θ₁·sinh(kL) + θ₂·sinh(kL)] / Δ
      + sinh(kx)·[L·φ₁·cosh(kL) + θ₁·cosh(kL) - θ₂·cosh(kL)] / Δ
```

**Rate of twist φ(x):**
```
φ(x) = [-k·θ₁·cosh(kL) + k·θ₂·cosh(kL) - φ₁·sinh(kL)] / Δ
      + k·sinh(kx)·[-L·φ₁·sinh(kL) - θ₁·sinh(kL) + θ₂·sinh(kL)] / Δ
      + k·cosh(kx)·[L·φ₁·cosh(kL) + θ₁·cosh(kL) - θ₂·cosh(kL)] / Δ
```

**Bimoment Bw(x):**
```
Bw(x) = -EIω·k²·[cosh(kx)·(-L·φ₁·sinh(kL) - θ₁·sinh(kL) + θ₂·sinh(kL))/Δ
               + sinh(kx)·(L·φ₁·cosh(kL) + θ₁·cosh(kL) - θ₂·cosh(kL))/Δ]
```

---

#### Case 3: θ₁=fixed, φ₁=fixed, θ₂=free, φ₂=fixed

**Twist angle θ(x):**
```
θ(x) = θ₁ + x·(φ₂ - GJ·φ₂/(EIω·k²))
      + sinh(kx)·(φ₁/k - φ₂/k + GJ·φ₂/(EIω·k³))
      + cosh(kx)·[-φ₁·cosh(kL)/(k·sinh(kL)) + φ₂·cosh(kL)/(k·sinh(kL))
                  - GJ·φ₂·cosh(kL)/(EIω·k³·sinh(kL)) + GJ·φ₂/(EIω·k³·sinh(kL))]
      + φ₁·cosh(kL)/(k·sinh(kL)) - φ₂·cosh(kL)/(k·sinh(kL))
      + GJ·φ₂·cosh(kL)/(EIω·k³·sinh(kL)) - GJ·φ₂/(EIω·k³·sinh(kL))
```

**Rate of twist φ(x):**
```
φ(x) = φ₂ - GJ·φ₂/(EIω·k²)
      + k·cosh(kx)·(φ₁/k - φ₂/k + GJ·φ₂/(EIω·k³))
      + k·sinh(kx)·[-φ₁·cosh(kL)/(k·sinh(kL)) + φ₂·cosh(kL)/(k·sinh(kL))
                    - GJ·φ₂·cosh(kL)/(EIω·k³·sinh(kL)) + GJ·φ₂/(EIω·k³·sinh(kL))]
```

---

#### Case 4: θ₁=fixed, φ₁=fixed, θ₂=free, φ₂=free

**Common denominator:** Δ = -EIω·k³·sinh²(kL) + EIω·k³·cosh²(kL) + GJ·k·sinh²(kL) - GJ·k·cosh²(kL) + GJ·k·cosh(kL)

Using identity: Δ = EIω·k³ - GJ·k + GJ·k·cosh(kL) = k·(EIω·k² - GJ + GJ·cosh(kL))

**Twist angle θ(x):**
```
θ(x) = θ₁·[-EIω·k³·sinh²(kL) + EIω·k³·cosh²(kL) + GJ·k·sinh²(kL)
           - GJ·k·cosh²(kL) + GJ·k·cosh(kL)] / Δ
      + x·[-EIω·k²·φ₁·sinh²(kL) + EIω·k²·φ₁·cosh²(kL) + GJ·φ₁·sinh²(kL)
           - GJ·φ₁·cosh²(kL)] / Δ'
      + cosh(kx)·[-GJ·φ₁·sinh(kL)] / Δ
      + sinh(kx)·[GJ·φ₁·cosh(kL)] / Δ
      + GJ·φ₁·sinh(kL)/Δ
```

where Δ' = -EIω·k²·sinh²(kL) + EIω·k²·cosh²(kL) + GJ·sinh²(kL) - GJ·cosh²(kL) + GJ·cosh(kL)

---

#### Case 5: θ₁=fixed, φ₁=free, θ₂=fixed, φ₂=fixed

**Common denominator:** Δ = L·k·cosh(kL) - sinh(kL)

**Twist angle θ(x):**
```
θ(x) = θ₁ + x·[-k·θ₁·cosh(kL) + k·θ₂·cosh(kL) - φ₂·sinh(kL)] / Δ
      + sinh(kx)·[L·φ₂ + θ₁ - θ₂] / Δ
```

**Rate of twist φ(x):**
```
φ(x) = [-k·θ₁·cosh(kL) + k·θ₂·cosh(kL) - φ₂·sinh(kL)] / Δ
      + k·cosh(kx)·[L·φ₂ + θ₁ - θ₂] / Δ
```

**Bimoment Bw(x):**
```
Bw(x) = -EIω·k²·sinh(kx)·(L·φ₂ + θ₁ - θ₂) / Δ
```

---

#### Case 6: θ₁=fixed, φ₁=free, θ₂=fixed, φ₂=free (Pure St. Venant)

**This is the pure St. Venant torsion case (no warping restraint)**

**Twist angle θ(x):**
```
θ(x) = θ₁ + x·(θ₂ - θ₁)/L
```

**Rate of twist φ(x):**
```
φ(x) = (θ₂ - θ₁)/L = constant
```

**St. Venant torsion Ts(x):**
```
Ts(x) = GJ·(θ₂ - θ₁)/L
```

**Warping torsion Tw(x):**
```
Tw(x) = 0
```

**Total torsion T(x):**
```
T(x) = GJ·(θ₂ - θ₁)/L
```

**Bimoment Bw(x):**
```
Bw(x) = 0
```

---

#### Case 7: θ₁=fixed, φ₁=free, θ₂=free, φ₂=fixed

**Twist angle θ(x):**
```
θ(x) = θ₁ + x·(φ₂ - GJ·φ₂/(EIω·k²)) + GJ·φ₂·sinh(kx)/(EIω·k³·cosh(kL))
```

**Rate of twist φ(x):**
```
φ(x) = φ₂ - GJ·φ₂/(EIω·k²) + GJ·φ₂·cosh(kx)/(EIω·k²·cosh(kL))
```

**Warping torsion Tw(x):**
```
Tw(x) = -GJ·φ₂·cosh(kx)/cosh(kL)
```

**Bimoment Bw(x):**
```
Bw(x) = -GJ·φ₂·sinh(kx)/(k·cosh(kL))
```

---

#### Case 8: θ₁=fixed, φ₁=free, θ₂=free, φ₂=free

**This is the free end case - rigid body rotation**

**Twist angle θ(x):**
```
θ(x) = θ₁
```

**Rate of twist φ(x):**
```
φ(x) = 0
```

**All internal actions:**
```
Ts(x) = 0
Tw(x) = 0
T(x) = 0
Bw(x) = 0
```

---

#### Case 9: θ₁=free, φ₁=fixed, θ₂=fixed, φ₂=fixed

**Twist angle θ(x):**
```
θ(x) = -L·φ₁ + θ₂ + x·(φ₁ - GJ·φ₁/(EIω·k²))
      + cosh(kx)·[-φ₁/(k·sinh(kL)) + φ₂/(k·sinh(kL))
                  - GJ·φ₁·cosh(kL)/(EIω·k³·sinh(kL)) + GJ·φ₁/(EIω·k³·sinh(kL))]
      + GJ·L·φ₁/(EIω·k²) - GJ·φ₁·sinh(kL)/(EIω·k³) + GJ·φ₁·sinh(kx)/(EIω·k³)
      + GJ·φ₁·cosh²(kL)/(EIω·k³·sinh(kL)) - GJ·φ₁·cosh(kL)/(EIω·k³·sinh(kL))
      + φ₁·cosh(kL)/(k·sinh(kL)) - φ₂·cosh(kL)/(k·sinh(kL))
```

**Rate of twist φ(x):**
```
φ(x) = φ₁ - GJ·φ₁/(EIω·k²) + GJ·φ₁·cosh(kx)/(EIω·k²)
      + k·sinh(kx)·[-φ₁/(k·sinh(kL)) + φ₂/(k·sinh(kL))
                    - GJ·φ₁·cosh(kL)/(EIω·k³·sinh(kL)) + GJ·φ₁/(EIω·k³·sinh(kL))]
```

---

#### Case 10: θ₁=free, φ₁=fixed, θ₂=fixed, φ₂=free

**Twist angle θ(x):**
```
θ(x) = -L·φ₁ + θ₂ + x·(φ₁ - GJ·φ₁/(EIω·k²))
      + GJ·L·φ₁/(EIω·k²) - GJ·φ₁·sinh(kL)·cosh(kx)/(EIω·k³·cosh(kL))
      + GJ·φ₁·sinh(kx)/(EIω·k³)
```

**Rate of twist φ(x):**
```
φ(x) = φ₁ - GJ·φ₁·sinh(kL)·sinh(kx)/(EIω·k²·cosh(kL))
      + GJ·φ₁·cosh(kx)/(EIω·k²) - GJ·φ₁/(EIω·k²)
```

---

#### Case 11: θ₁=free, φ₁=fixed, θ₂=free, φ₂=fixed

**General solution with undetermined constants:**

**Twist angle θ(x):**
```
θ(x) = C₁ + C₂·x + C₃·cosh(kx) + C₄·sinh(kx)
```

**Rate of twist φ(x):**
```
φ(x) = C₂ + C₃·k·sinh(kx) + C₄·k·cosh(kx)
```

**St. Venant torsion Ts(x):**
```
Ts(x) = GJ·(C₂ + C₃·k·sinh(kx) + C₄·k·cosh(kx))
```

**Warping torsion Tw(x):**
```
Tw(x) = -EIω·k²·(C₃·k·sinh(kx) + C₄·k·cosh(kx))
```

**Total torsion T(x):**
```
T(x) = -EIω·k³·(C₃·sinh(kx) + C₄·cosh(kx)) + GJ·(C₂ + C₃·k·sinh(kx) + C₄·k·cosh(kx))
```

**Bimoment Bw(x):**
```
Bw(x) = -EIω·k²·(C₃·cosh(kx) + C₄·sinh(kx))
```

Note: Constants C₁, C₂, C₃, C₄ are determined from boundary conditions φ₁ and φ₂.

---

#### Case 12: θ₁=free, φ₁=fixed, θ₂=free, φ₂=free

**Same general form as Case 11**

---

#### Case 13: θ₁=free, φ₁=free, θ₂=fixed, φ₂=fixed

**Common denominator:** Δ = EIω·k³ + GJ·k·cosh(kL) - GJ·k

**Twist angle θ(x):**
```
θ(x) = [-EIω·L·k³·φ₂ + EIω·k³·θ₂ + GJ·L·k·φ₂ + GJ·k·θ₂·cosh(kL) - GJ·k·θ₂
        - GJ·φ₂·sinh(kL) + GJ·φ₂·sinh(kx)] / Δ
      + x·[EIω·k²·φ₂ - GJ·φ₂] / (EIω·k² + GJ·cosh(kL) - GJ)
```

**Rate of twist φ(x):**
```
φ(x) = [EIω·k²·φ₂ + GJ·k·φ₂·cosh(kx) - GJ·φ₂] / (EIω·k² + GJ·cosh(kL) - GJ)
```

**Warping torsion Tw(x):**
```
Tw(x) = -EIω·GJ·k³·φ₂·cosh(kx) / Δ
```

**Bimoment Bw(x):**
```
Bw(x) = -EIω·GJ·k²·φ₂·sinh(kx) / Δ
```

---

#### Case 14: θ₁=free, φ₁=free, θ₂=fixed, φ₂=free

**This is rigid body rotation to match θ₂**

**Twist angle θ(x):**
```
θ(x) = θ₂
```

**All internal actions:**
```
φ(x) = 0
Ts(x) = 0
Tw(x) = 0
T(x) = 0
Bw(x) = 0
```

---

#### Case 15: θ₁=free, φ₁=free, θ₂=free, φ₂=fixed

**Same general form as Case 11**

---

#### Case 16: θ₁=free, φ₁=free, θ₂=free, φ₂=free

**Rigid body motion - constant rotation**

**Twist angle θ(x):**
```
θ(x) = C₁ (arbitrary constant)
```

**All internal actions:**
```
φ(x) = 0
Ts(x) = 0
Tw(x) = 0
T(x) = 0
Bw(x) = 0
```

---

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
 * @brief Compute warping results using analytical solutions
 *
 * Governing differential equation: EIω d⁴θ/dx⁴ - GJ d²θ/dx² = 0
 * General solution: θ(x) = C₁ + C₂·x + C₃·cosh(kx) + C₄·sinh(kx)
 * where k = sqrt(GJ/EIω) is the warping parameter
 *
 * Variables:
 * - θ₁, θ₂ = twist angles at x=0 and x=L
 * - φ₁, φ₂ = rates of twist (dθ/dx) at x=0 and x=L
 * - Results: θ(x), φ(x), Ts(x), Tw(x), T(x), Bw(x)
 */
class WarpingComputer {
public:
    WarpingComputer(double L, double EIw, double GJ,
                   double theta1, double phi1, double theta2, double phi2)
        : L_(L), EIw_(EIw), GJ_(GJ),
          theta1_(theta1), phi1_(phi1), theta2_(theta2), phi2_(phi2),
          k_(std::sqrt(GJ / EIw)),
          kL_(k_ * L) {}

    WarpingInternalActions compute(double x, ReleaseComboWarping release) const {
        WarpingInternalActions result;
        result.x = x;

        switch (release) {
            case ReleaseComboWarping::FIXED_FIXED_FIXED_FIXED:
                compute_fixed_fixed_fixed_fixed(x, result);
                break;
            case ReleaseComboWarping::FIXED_FIXED_FIXED_FREE:
                compute_fixed_fixed_fixed_free(x, result);
                break;
            case ReleaseComboWarping::FIXED_FREE_FIXED_FREE:
                compute_pure_st_venant(x, result);
                break;
            // ... all 16 cases
            default:
                throw std::runtime_error("Release combination not implemented");
        }

        return result;
    }

private:
    double L_, EIw_, GJ_;
    double theta1_, phi1_, theta2_, phi2_;
    double k_, kL_;

    // Helper: Common denominator for fixed-fixed-fixed-fixed
    double Delta_FFFF() const {
        return k_ * (L_ * k_ * std::sinh(kL_) - 2.0 * std::cosh(kL_) + 2.0);
    }

    // Helper: Common denominator for fixed-fixed-fixed-free / free-fixed-fixed-fixed
    double Delta_prime() const {
        return L_ * k_ * std::cosh(kL_) - std::sinh(kL_);
    }

    void compute_fixed_fixed_fixed_fixed(double x, WarpingInternalActions& result) const {
        double Delta = Delta_FFFF();
        double sinhkL = std::sinh(kL_);
        double coshkL = std::cosh(kL_);
        double sinhkx = std::sinh(k_ * x);
        double coshkx = std::cosh(k_ * x);

        // Coefficients C3 and C4 for hyperbolic terms
        double C3 = (L_ * k_ * phi1_ * sinhkL + k_ * theta1_ * sinhkL - k_ * theta2_ * sinhkL
                    - phi1_ * coshkL + phi1_ + phi2_ * coshkL - phi2_) / Delta;
        double C4 = (-L_ * k_ * phi1_ * coshkL + L_ * k_ * phi2_ - k_ * theta1_ * coshkL + k_ * theta1_
                    + k_ * theta2_ * coshkL - k_ * theta2_ + phi1_ * sinhkL - phi2_ * sinhkL) / Delta;

        // Rate of twist (phi = dtheta/dx)
        double phi_linear = (-k_ * theta1_ * sinhkL + k_ * theta2_ * sinhkL
                            - phi1_ * (sinhkL * sinhkL - coshkL * coshkL + coshkL)
                            - phi2_ * coshkL + phi2_) / (L_ * k_ * sinhkL - sinhkL * sinhkL
                            + coshkL * coshkL - 2.0 * coshkL + 1.0);
        double phi = phi_linear + k_ * C3 * coshkx + k_ * C4 * sinhkx;

        // St. Venant torsion
        result.Mx_sv = GJ_ * phi;

        // Warping torsion: Tw = -EIw * k² * (k*C3*sinh(kx) + k*C4*cosh(kx))
        result.Mx_w = -EIw_ * k_ * k_ * (k_ * C3 * sinhkx + k_ * C4 * coshkx);

        // Total torsion
        result.Mx = result.Mx_sv + result.Mx_w;

        // Bimoment: Bw = -EIw * k² * (C3*cosh(kx) + C4*sinh(kx))
        result.B = -EIw_ * k_ * k_ * (C3 * coshkx + C4 * sinhkx);
    }

    void compute_pure_st_venant(double x, WarpingInternalActions& result) const {
        // Case 6: θ₁=fixed, φ₁=free, θ₂=fixed, φ₂=free
        // Pure St. Venant torsion - linear twist, no warping
        double phi = (theta2_ - theta1_) / L_;

        result.Mx_sv = GJ_ * phi;
        result.Mx_w = 0.0;
        result.Mx = result.Mx_sv;
        result.B = 0.0;
    }

    void compute_fixed_fixed_fixed_free(double x, WarpingInternalActions& result) const {
        double Delta = Delta_prime();
        double sinhkL = std::sinh(kL_);
        double coshkL = std::cosh(kL_);
        double sinhkx = std::sinh(k_ * x);
        double coshkx = std::cosh(k_ * x);

        // Coefficients for Case 2
        double C3 = (-L_ * phi1_ * sinhkL - theta1_ * sinhkL + theta2_ * sinhkL) / Delta;
        double C4 = (L_ * phi1_ * coshkL + theta1_ * coshkL - theta2_ * coshkL) / Delta;

        double phi_linear = (-k_ * theta1_ * coshkL + k_ * theta2_ * coshkL - phi1_ * sinhkL) / Delta;
        double phi = phi_linear + k_ * C3 * sinhkx + k_ * C4 * coshkx;

        result.Mx_sv = GJ_ * phi;
        result.Mx_w = -EIw_ * k_ * k_ * (k_ * C3 * sinhkx + k_ * C4 * coshkx);
        result.Mx = result.Mx_sv + result.Mx_w;
        result.B = -EIw_ * k_ * k_ * (C3 * coshkx + C4 * sinhkx);
    }

    // ... implement remaining 13 cases following the analytical solutions above
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
- [ ] All 16 boundary condition combinations produce correct results

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
