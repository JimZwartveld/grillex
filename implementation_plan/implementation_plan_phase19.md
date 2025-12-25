# Phase 19: Spectral Loads (Response Spectrum Analysis)

## Overview

This phase implements response spectrum analysis (RSA) capabilities for seismic and dynamic design. RSA is a modal superposition method that combines eigenvalue analysis results with design response spectra to compute peak structural responses without time-history integration.

**Key Features:**
- **Design Response Spectra** - Eurocode 8, user-defined tabular, and analytical spectra
- **Modal Combination Rules** - SRSS, CQC, and Absolute Sum methods
- **Directional Combination** - 100-30-30 rule, SRSS of directions
- **Complete Results** - Spectral displacements, internal forces, reactions, base shear
- **Code Compliance** - 90% mass participation check, missing mass correction

**Use Cases:**
- Seismic design per Eurocode 8, ASCE 7, or other codes
- Equipment qualification for dynamic loads
- Offshore structures under wave-induced vibrations
- Industrial structures with rotating machinery excitation

**Requirements Reference:** R-LOAD (extended), R-ASM-006 (dynamic analysis capability)

**Dependencies:**
- Phase 16 complete (Eigenvalue Analysis) - **REQUIRED**
- Phase 5 complete (Loads & Load Cases) - **REQUIRED**
- Phase 7 complete (Internal Actions) - **REQUIRED** for spectral forces

**Difficulty:** High

---

## Mathematical Background

### Response Spectrum Concept

A **response spectrum** represents the maximum response of a family of single-degree-of-freedom (SDOF) oscillators to a given ground motion. For each natural period T, the spectrum gives:

- **Sa(T)** - Spectral acceleration [m/s² or g]
- **Sv(T)** - Spectral velocity [m/s]
- **Sd(T)** - Spectral displacement [m]

These are related by:
```
Sd = Sa × (T/2π)² = Sa / ω²
Sv = Sa × (T/2π) = Sa / ω
```

### Modal Response Spectrum Analysis

For a structure with n computed modes, the response spectrum method computes:

**1. Spectral Acceleration per Mode:**
```
Sa,n = S(Tn, ξn)
```
where Tn is the period of mode n and ξn is the damping ratio.

**2. Modal Displacement Response:**
```
un = Γn × φn × Sd,n = Γn × φn × Sa,n / ωn²
```
where:
- Γn = participation factor for mode n in direction d
- φn = mode shape (mass-normalized)
- ωn = natural circular frequency [rad/s]

**3. Modal Force Response:**
```
fn = K × un = Γn × K × φn × Sa,n / ωn²
   = Γn × ωn² × M × φn × Sa,n / ωn²
   = Γn × M × φn × Sa,n
```

**4. Modal Base Shear:**
```
Vb,n = Γn × Meff,n × Sa,n
```
where Meff,n = Γn² (for mass-normalized modes) is the effective modal mass.

### Modal Combination Rules

Since modal maxima don't occur simultaneously, responses must be combined statistically:

**SRSS (Square Root of Sum of Squares):**
```
R = √(Σ Rn²)
```
Valid when modes are well-separated (|Ti - Tj| / Ti > 0.1).

**CQC (Complete Quadratic Combination):**
```
R = √(Σi Σj ρij × Ri × Rj)
```
where ρij is the correlation coefficient:
```
ρij = 8 × √(ξi × ξj) × (ξi + r × ξj) × r^1.5 /
      [(1 - r²)² + 4 × ξi × ξj × r × (1 + r²) + 4 × (ξi² + ξj²) × r²]
```
with r = ωi / ωj. Required for closely-spaced modes.

**Absolute Sum:**
```
R = Σ |Rn|
```
Conservative upper bound, sometimes required by codes.

### Directional Combination

When spectra are applied in multiple directions (X, Y, Z):

**100-30-30 Rule (Eurocode 8):**
```
E = max(Ex + 0.3Ey + 0.3Ez, 0.3Ex + Ey + 0.3Ez, 0.3Ex + 0.3Ey + Ez)
```

**SRSS of Directions:**
```
E = √(Ex² + Ey² + Ez²)
```

### Missing Mass Correction

If cumulative modal mass participation is less than 100%, a static correction can be applied:

**Residual Mode Method:**
```
u_residual = (I - Σ φn × φnᵀ × M) × M⁻¹ × r × Sa,ZPA
```
where Sa,ZPA is the zero-period acceleration (high-frequency plateau of spectrum).

**Simplified Approach:**
Add a static response using the ZPA for the "missing mass" portion.

---

## Implementation Tasks

### Task 19.1: Response Spectrum Data Structures (C++)

**File:** `cpp/include/grillex/response_spectrum.hpp`

**Description:** Create data structures for response spectrum definition and spectral analysis settings.

**Structures:**

```cpp
enum class SpectrumType {
    UserDefined,      // Tabular (T, Sa) pairs
    Eurocode8Type1,   // EC8 Type 1 elastic spectrum
    Eurocode8Type2,   // EC8 Type 2 elastic spectrum
    ASCE7,            // ASCE 7 design spectrum
    Constant          // Constant acceleration (for testing)
};

enum class CombinationRule {
    SRSS,             // Square root of sum of squares
    CQC,              // Complete quadratic combination
    AbsoluteSum       // Absolute sum (conservative)
};

enum class DirectionalCombination {
    SRSS,             // √(Ex² + Ey² + Ez²)
    Rule100_30_30,    // EC8: 100-30-30 rule
    Maximum           // max(Ex, Ey, Ez)
};

struct SpectrumPoint {
    double period;        // [s]
    double acceleration;  // [m/s²]
};

class ResponseSpectrum {
public:
    ResponseSpectrum();

    // User-defined spectrum from (T, Sa) pairs
    ResponseSpectrum(const std::vector<SpectrumPoint>& points);

    // Eurocode 8 spectrum
    static ResponseSpectrum eurocode8_type1(
        double ag,           // Design ground acceleration [m/s²]
        double S,            // Soil factor
        double TB,           // Corner period TB [s]
        double TC,           // Corner period TC [s]
        double TD,           // Corner period TD [s]
        double eta = 1.0     // Damping correction factor (η = 1 for 5% damping)
    );

    static ResponseSpectrum eurocode8_type2(
        double ag, double S, double TB, double TC, double TD, double eta = 1.0
    );

    // Constant spectrum (for testing)
    static ResponseSpectrum constant(double acceleration);

    // Query spectrum
    double get_acceleration(double period) const;  // Sa(T) [m/s²]
    double get_displacement(double period) const;  // Sd(T) [m]
    double get_velocity(double period) const;      // Sv(T) [m/s]

    // Properties
    SpectrumType type() const;
    double peak_acceleration() const;  // Maximum Sa (ZPA for high-freq)
    double zpa() const;                // Zero Period Acceleration

    // Interpolation settings
    void set_log_interpolation(bool use_log);  // Log-log interpolation

private:
    SpectrumType type_;
    std::vector<SpectrumPoint> points_;
    bool use_log_interpolation_ = true;

    // EC8 parameters (if applicable)
    double ag_, S_, TB_, TC_, TD_, eta_;

    double interpolate(double period) const;
    double eurocode8_formula(double period) const;
};

struct SpectralAnalysisSettings {
    int n_modes = 0;                   // 0 = use all computed modes
    double damping_ratio = 0.05;       // Default 5% critical damping
    CombinationRule combination = CombinationRule::CQC;
    DirectionalCombination dir_combination = DirectionalCombination::SRSS;
    bool include_missing_mass = true;  // Apply missing mass correction
    double mass_participation_threshold = 0.90;  // Warn if < 90%
};
```

**Acceptance Criteria:**
- [ ] ResponseSpectrum can be created from tabular (T, Sa) points
- [ ] Eurocode 8 Type 1 spectrum matches code formulas (within 0.1%)
- [ ] Eurocode 8 Type 2 spectrum matches code formulas (within 0.1%)
- [ ] get_acceleration() interpolates correctly between points
- [ ] Log-log interpolation available for smoother spectra
- [ ] ZPA (zero period acceleration) correctly identified
- [ ] SpectralAnalysisSettings has all required configuration options
- [ ] Default 5% damping ratio applied

---

### Task 19.2: Eurocode 8 Spectrum Implementation

**File:** `cpp/src/response_spectrum.cpp`

**Description:** Implement Eurocode 8 elastic response spectrum formulas.

**Eurocode 8 Type 1 Elastic Spectrum:**

```
0 ≤ T ≤ TB:     Se(T) = ag × S × [1 + T/TB × (η × 2.5 - 1)]
TB ≤ T ≤ TC:    Se(T) = ag × S × η × 2.5
TC ≤ T ≤ TD:    Se(T) = ag × S × η × 2.5 × (TC/T)
TD ≤ T:         Se(T) = ag × S × η × 2.5 × (TC × TD/T²)
```

**Recommended EC8 Parameters (Table 3.2, 3.3):**

| Ground Type | S | TB | TC | TD |
|-------------|---|----|----|-----|
| A | 1.0 | 0.15 | 0.4 | 2.0 |
| B | 1.2 | 0.15 | 0.5 | 2.0 |
| C | 1.15 | 0.20 | 0.6 | 2.0 |
| D | 1.35 | 0.20 | 0.8 | 2.0 |
| E | 1.4 | 0.15 | 0.5 | 2.0 |

**Damping Correction Factor:**
```
η = √(10 / (5 + ξ)) ≥ 0.55
```
where ξ is damping in percent.

**Implementation:**

```cpp
ResponseSpectrum ResponseSpectrum::eurocode8_type1(
    double ag, double S, double TB, double TC, double TD, double eta)
{
    ResponseSpectrum spectrum;
    spectrum.type_ = SpectrumType::Eurocode8Type1;
    spectrum.ag_ = ag;
    spectrum.S_ = S;
    spectrum.TB_ = TB;
    spectrum.TC_ = TC;
    spectrum.TD_ = TD;
    spectrum.eta_ = eta;

    // Generate points for common periods
    std::vector<double> periods = {
        0.0, 0.05, 0.1, TB, 0.3, TC, 0.8, 1.0, TD, 3.0, 4.0
    };

    for (double T : periods) {
        double Sa = spectrum.eurocode8_formula(T);
        spectrum.points_.push_back({T, Sa});
    }

    return spectrum;
}

double ResponseSpectrum::eurocode8_formula(double T) const {
    if (T <= 0) return ag_ * S_;  // ZPA

    if (T <= TB_) {
        return ag_ * S_ * (1 + T / TB_ * (eta_ * 2.5 - 1));
    } else if (T <= TC_) {
        return ag_ * S_ * eta_ * 2.5;
    } else if (T <= TD_) {
        return ag_ * S_ * eta_ * 2.5 * (TC_ / T);
    } else {
        return ag_ * S_ * eta_ * 2.5 * (TC_ * TD_ / (T * T));
    }
}
```

**Acceptance Criteria:**
- [ ] EC8 Type 1 spectrum for Ground Type A matches tabulated values
- [ ] EC8 Type 1 spectrum for Ground Type C matches tabulated values
- [ ] Damping correction factor η computed correctly
- [ ] Spectrum is continuous at corner periods (TB, TC, TD)
- [ ] ZPA equals ag × S at T = 0
- [ ] Peak plateau equals ag × S × η × 2.5

---

### Task 19.3: Modal Response Calculation (C++)

**File:** `cpp/include/grillex/spectral_analysis.hpp`, `cpp/src/spectral_analysis.cpp`

**Description:** Compute modal responses using response spectrum.

**Class:**

```cpp
struct ModalResponse {
    int mode_number;
    double period;              // [s]
    double spectral_accel;      // Sa(T) [m/s²]
    double spectral_disp;       // Sd(T) [m]
    Eigen::VectorXd displacement;  // Modal displacement response
    Eigen::VectorXd acceleration;  // Modal acceleration response
    double base_shear;          // Modal base shear contribution
};

struct SpectralDirection {
    std::string name;           // "X", "Y", "Z"
    ResponseSpectrum spectrum;
    Eigen::Vector3d direction;  // Unit vector [1,0,0], [0,1,0], [0,0,1]
};

class SpectralAnalyzer {
public:
    SpectralAnalyzer(const EigensolverResult& modes,
                     const Eigen::SparseMatrix<double>& M,
                     const Eigen::SparseMatrix<double>& K);

    // Compute modal responses for single direction
    std::vector<ModalResponse> compute_modal_responses(
        const ResponseSpectrum& spectrum,
        const Eigen::Vector3d& direction,
        double damping_ratio = 0.05
    ) const;

    // Combine modal responses
    Eigen::VectorXd combine_displacements(
        const std::vector<ModalResponse>& responses,
        CombinationRule rule,
        double damping_ratio = 0.05
    ) const;

    // CQC correlation coefficient
    static double cqc_correlation(
        double omega_i, double omega_j,
        double xi_i, double xi_j
    );

    // Base shear computation
    double compute_base_shear(
        const std::vector<ModalResponse>& responses,
        CombinationRule rule,
        double damping_ratio = 0.05
    ) const;

private:
    const EigensolverResult& modes_;
    const Eigen::SparseMatrix<double>& M_;
    const Eigen::SparseMatrix<double>& K_;
};
```

**Implementation Notes:**

```cpp
std::vector<ModalResponse> SpectralAnalyzer::compute_modal_responses(
    const ResponseSpectrum& spectrum,
    const Eigen::Vector3d& direction,
    double damping_ratio) const
{
    std::vector<ModalResponse> responses;

    for (const auto& mode : modes_.modes) {
        if (mode.is_rigid_body_mode) continue;

        ModalResponse resp;
        resp.mode_number = mode.mode_number;
        resp.period = mode.period_s;

        // Get spectral values
        resp.spectral_accel = spectrum.get_acceleration(mode.period_s);
        resp.spectral_disp = resp.spectral_accel / (mode.omega * mode.omega);

        // Participation factor for this direction
        double gamma = 0.0;
        if (std::abs(direction.x()) > 0.5) gamma = mode.participation_x;
        else if (std::abs(direction.y()) > 0.5) gamma = mode.participation_y;
        else if (std::abs(direction.z()) > 0.5) gamma = mode.participation_z;

        // Modal displacement: u_n = Γ × φ × Sd
        resp.displacement = gamma * mode.mode_shape * resp.spectral_disp;

        // Modal acceleration: a_n = Γ × φ × Sa
        resp.acceleration = gamma * mode.mode_shape * resp.spectral_accel;

        // Base shear: Vb = Γ² × Sa (using Meff = Γ²)
        resp.base_shear = gamma * gamma * resp.spectral_accel;

        responses.push_back(resp);
    }

    return responses;
}
```

**Acceptance Criteria:**
- [ ] Modal displacement computed as Γ × φ × Sd
- [ ] Modal acceleration computed as Γ × φ × Sa
- [ ] Spectral displacement Sd = Sa / ω² relationship correct
- [ ] Base shear per mode equals Γ² × Sa (effective mass × acceleration)
- [ ] Rigid body modes (ω ≈ 0) are excluded from spectral analysis
- [ ] Participation factor correctly selected based on direction

---

### Task 19.4: Modal Combination Implementation (C++)

**File:** `cpp/src/spectral_analysis.cpp`

**Description:** Implement SRSS, CQC, and Absolute Sum combination rules.

**Implementation:**

```cpp
double SpectralAnalyzer::cqc_correlation(
    double omega_i, double omega_j,
    double xi_i, double xi_j)
{
    double r = omega_j / omega_i;  // Frequency ratio
    if (r > 1.0) r = 1.0 / r;      // Ensure r ≤ 1

    double xi_avg = std::sqrt(xi_i * xi_j);

    double numerator = 8.0 * xi_avg * (xi_i + r * xi_j) * std::pow(r, 1.5);
    double denom = std::pow(1.0 - r * r, 2) +
                   4.0 * xi_i * xi_j * r * (1.0 + r * r) +
                   4.0 * (xi_i * xi_i + xi_j * xi_j) * r * r;

    if (denom < 1e-12) return 1.0;  // Same frequency
    return numerator / denom;
}

Eigen::VectorXd SpectralAnalyzer::combine_displacements(
    const std::vector<ModalResponse>& responses,
    CombinationRule rule,
    double damping_ratio) const
{
    if (responses.empty()) return Eigen::VectorXd();

    int n_dofs = responses[0].displacement.size();
    Eigen::VectorXd combined = Eigen::VectorXd::Zero(n_dofs);

    switch (rule) {
        case CombinationRule::SRSS: {
            // R = √(Σ Rn²)
            for (const auto& resp : responses) {
                combined += resp.displacement.cwiseProduct(resp.displacement);
            }
            combined = combined.cwiseSqrt();
            break;
        }

        case CombinationRule::CQC: {
            // R = √(Σi Σj ρij × Ri × Rj)
            int n_modes = responses.size();
            for (int dof = 0; dof < n_dofs; ++dof) {
                double sum = 0.0;
                for (int i = 0; i < n_modes; ++i) {
                    for (int j = 0; j < n_modes; ++j) {
                        double rho = cqc_correlation(
                            2.0 * M_PI / responses[i].period,
                            2.0 * M_PI / responses[j].period,
                            damping_ratio, damping_ratio
                        );
                        sum += rho * responses[i].displacement(dof)
                                   * responses[j].displacement(dof);
                    }
                }
                combined(dof) = std::sqrt(std::abs(sum));
            }
            break;
        }

        case CombinationRule::AbsoluteSum: {
            // R = Σ |Rn|
            for (const auto& resp : responses) {
                combined += resp.displacement.cwiseAbs();
            }
            break;
        }
    }

    return combined;
}
```

**Acceptance Criteria:**
- [ ] SRSS combination: R = √(Σ Rn²) computed correctly
- [ ] CQC correlation coefficient matches published formulas (within 0.1%)
- [ ] CQC for identical modes (r = 1) gives ρ = 1.0
- [ ] CQC for well-separated modes approaches SRSS result
- [ ] Absolute Sum is conservative (larger than SRSS/CQC)
- [ ] Combination preserves DOF structure (result has correct size)

---

### Task 19.5: Directional Combination (C++)

**File:** `cpp/src/spectral_analysis.cpp`

**Description:** Combine responses from multiple seismic directions.

**Implementation:**

```cpp
struct DirectionalResponses {
    Eigen::VectorXd response_x;
    Eigen::VectorXd response_y;
    Eigen::VectorXd response_z;
};

Eigen::VectorXd SpectralAnalyzer::combine_directions(
    const DirectionalResponses& dir_resp,
    DirectionalCombination rule) const
{
    int n_dofs = dir_resp.response_x.size();
    Eigen::VectorXd combined = Eigen::VectorXd::Zero(n_dofs);

    switch (rule) {
        case DirectionalCombination::SRSS: {
            // E = √(Ex² + Ey² + Ez²)
            combined = (dir_resp.response_x.cwiseProduct(dir_resp.response_x) +
                       dir_resp.response_y.cwiseProduct(dir_resp.response_y) +
                       dir_resp.response_z.cwiseProduct(dir_resp.response_z)).cwiseSqrt();
            break;
        }

        case DirectionalCombination::Rule100_30_30: {
            // E = max of three combinations
            Eigen::VectorXd combo1 = dir_resp.response_x +
                                     0.3 * dir_resp.response_y +
                                     0.3 * dir_resp.response_z;
            Eigen::VectorXd combo2 = 0.3 * dir_resp.response_x +
                                     dir_resp.response_y +
                                     0.3 * dir_resp.response_z;
            Eigen::VectorXd combo3 = 0.3 * dir_resp.response_x +
                                     0.3 * dir_resp.response_y +
                                     dir_resp.response_z;

            for (int i = 0; i < n_dofs; ++i) {
                combined(i) = std::max({combo1(i), combo2(i), combo3(i)});
            }
            break;
        }

        case DirectionalCombination::Maximum: {
            // E = max(Ex, Ey, Ez)
            for (int i = 0; i < n_dofs; ++i) {
                combined(i) = std::max({dir_resp.response_x(i),
                                       dir_resp.response_y(i),
                                       dir_resp.response_z(i)});
            }
            break;
        }
    }

    return combined;
}
```

**Acceptance Criteria:**
- [ ] SRSS directional combination: E = √(Ex² + Ey² + Ez²)
- [ ] 100-30-30 rule correctly evaluates all three combinations
- [ ] 100-30-30 takes maximum of the three combinations
- [ ] Single-direction analysis (Y=Z=0) gives same result as X alone
- [ ] Symmetric structure with equal X/Y spectra gives symmetric response

---

### Task 19.6: Missing Mass Correction (C++)

**File:** `cpp/src/spectral_analysis.cpp`

**Description:** Implement missing mass correction for truncated modal analysis.

**Implementation:**

```cpp
struct MissingMassResult {
    double missing_mass_x;      // [mT] or [%]
    double missing_mass_y;
    double missing_mass_z;
    Eigen::VectorXd correction; // Static correction vector
    bool correction_applied;
};

MissingMassResult SpectralAnalyzer::compute_missing_mass_correction(
    const std::vector<ModalResponse>& responses,
    const ResponseSpectrum& spectrum,
    const Eigen::Vector3d& direction,
    double mass_threshold) const
{
    MissingMassResult result;

    // Compute participating mass from included modes
    double participating_mass = 0.0;
    for (const auto& resp : responses) {
        const auto& mode = modes_.modes[resp.mode_number - 1];
        if (std::abs(direction.x()) > 0.5)
            participating_mass += mode.effective_mass_x;
        else if (std::abs(direction.y()) > 0.5)
            participating_mass += mode.effective_mass_y;
        else
            participating_mass += mode.effective_mass_z;
    }

    // Total mass in this direction
    double total_mass = 0.0;
    if (std::abs(direction.x()) > 0.5) total_mass = modes_.total_mass_x;
    else if (std::abs(direction.y()) > 0.5) total_mass = modes_.total_mass_y;
    else total_mass = modes_.total_mass_z;

    double participation_ratio = participating_mass / total_mass;

    // Missing mass
    if (std::abs(direction.x()) > 0.5) {
        result.missing_mass_x = total_mass - participating_mass;
        result.missing_mass_y = 0;
        result.missing_mass_z = 0;
    }
    // ... similar for Y, Z

    // Apply correction if below threshold
    if (participation_ratio < mass_threshold) {
        // Use ZPA (zero-period acceleration) for static correction
        double zpa = spectrum.zpa();

        // Build influence vector
        Eigen::VectorXd r = Eigen::VectorXd::Zero(M_.rows());
        // Fill with 1.0 at translation DOFs in the specified direction
        // ...

        // Residual force = (1 - Σ Meff_n/M_total) × M × r × ZPA
        double residual_factor = 1.0 - participation_ratio;
        Eigen::VectorXd F_residual = residual_factor * M_ * r * zpa;

        // Solve K × u_residual = F_residual
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(K_);
        result.correction = solver.solve(F_residual);
        result.correction_applied = true;
    } else {
        result.correction = Eigen::VectorXd::Zero(M_.rows());
        result.correction_applied = false;
    }

    return result;
}
```

**Acceptance Criteria:**
- [ ] Missing mass computed as (total - sum of effective modal masses)
- [ ] Warning issued when cumulative mass < 90% threshold
- [ ] ZPA (zero-period acceleration) correctly identified from spectrum
- [ ] Static correction uses K⁻¹ × M × r × ZPA formulation
- [ ] Combined response includes missing mass correction (via SRSS with modal)
- [ ] Correction can be disabled via settings

---

### Task 19.7: Spectral Internal Forces (C++)

**File:** `cpp/src/spectral_analysis.cpp`

**Description:** Compute spectral internal forces in elements from modal responses.

**Implementation:**

```cpp
struct SpectralElementForces {
    int element_id;

    // End forces (combined from all modes)
    Eigen::Vector<double, 6> forces_i;  // [Fx, Fy, Fz, Mx, My, Mz] at node i
    Eigen::Vector<double, 6> forces_j;  // [Fx, Fy, Fz, Mx, My, Mz] at node j

    // Internal action extrema along element
    double max_axial;
    double max_shear_y;
    double max_shear_z;
    double max_moment_y;
    double max_moment_z;
    double max_torsion;
};

std::vector<SpectralElementForces> SpectralAnalyzer::compute_element_forces(
    const Eigen::VectorXd& combined_displacements,
    const std::vector<BeamElement*>& elements,
    const DOFHandler& dof_handler) const
{
    std::vector<SpectralElementForces> results;

    for (const auto* elem : elements) {
        SpectralElementForces forces;
        forces.element_id = elem->id;

        // Extract element displacements from global vector
        std::vector<int> loc = dof_handler.get_location_array(*elem);
        Eigen::VectorXd u_elem(12);
        for (int i = 0; i < 12; ++i) {
            u_elem(i) = (loc[i] >= 0) ? combined_displacements(loc[i]) : 0.0;
        }

        // Compute end forces: f = K_local × T × u_global
        Eigen::MatrixXd K_local = elem->local_stiffness_matrix();
        Eigen::MatrixXd T = elem->transformation_matrix();
        Eigen::VectorXd u_local = T * u_elem;
        Eigen::VectorXd f_local = K_local * u_local;

        forces.forces_i = f_local.head<6>();
        forces.forces_j = f_local.tail<6>();

        // Extract extrema
        forces.max_axial = std::max(std::abs(f_local(0)), std::abs(f_local(6)));
        forces.max_shear_y = std::max(std::abs(f_local(1)), std::abs(f_local(7)));
        forces.max_shear_z = std::max(std::abs(f_local(2)), std::abs(f_local(8)));
        forces.max_torsion = std::max(std::abs(f_local(3)), std::abs(f_local(9)));
        forces.max_moment_y = std::max(std::abs(f_local(4)), std::abs(f_local(10)));
        forces.max_moment_z = std::max(std::abs(f_local(5)), std::abs(f_local(11)));

        results.push_back(forces);
    }

    return results;
}
```

**Acceptance Criteria:**
- [ ] Element end forces computed from f = K × u
- [ ] Local/global transformation applied correctly
- [ ] Maximum internal actions (axial, shear, moment, torsion) extracted
- [ ] Forces at both ends (i and j) available
- [ ] Works with 12-DOF and 14-DOF beam elements
- [ ] Sign convention consistent with Phase 7 internal actions

---

### Task 19.8: Spectral Reactions (C++)

**File:** `cpp/src/spectral_analysis.cpp`

**Description:** Compute spectral reactions at supports.

**Implementation:**

```cpp
struct SpectralReaction {
    int node_id;
    Eigen::Vector<double, 6> forces;  // [Fx, Fy, Fz, Mx, My, Mz]
};

std::vector<SpectralReaction> SpectralAnalyzer::compute_reactions(
    const Eigen::VectorXd& combined_displacements,
    const BCHandler& bc_handler,
    const DOFHandler& dof_handler) const
{
    // R = K × u for DOFs with prescribed BCs
    Eigen::VectorXd full_reactions = K_ * combined_displacements;

    std::vector<SpectralReaction> results;

    // Group reactions by node
    std::map<int, SpectralReaction> node_reactions;

    for (const auto& fixed_dof : bc_handler.get_fixed_dofs()) {
        int node_id = fixed_dof.node_id;
        int local_dof = fixed_dof.local_dof;
        int global_dof = dof_handler.global_dof(node_id, local_dof);

        if (global_dof < 0) continue;

        if (node_reactions.find(node_id) == node_reactions.end()) {
            node_reactions[node_id] = {node_id, Eigen::Vector<double, 6>::Zero()};
        }

        node_reactions[node_id].forces(local_dof) = full_reactions(global_dof);
    }

    for (const auto& [node_id, reaction] : node_reactions) {
        results.push_back(reaction);
    }

    return results;
}
```

**Acceptance Criteria:**
- [ ] Reactions computed at all fixed DOFs
- [ ] Reactions grouped by node for convenience
- [ ] Total base shear equals sum of horizontal reactions
- [ ] Reactions satisfy equilibrium with applied spectral forces
- [ ] Works with partial fixity (some DOFs free, some fixed)

---

### Task 19.9: Model Integration (C++)

**File:** `cpp/include/grillex/model.hpp`, `cpp/src/model.cpp`

**Description:** Add spectral analysis methods to Model class.

**API:**

```cpp
class Model {
public:
    // Existing...
    bool analyze_eigenvalues(const EigensolverSettings& settings = {});

    // New: Spectral analysis
    bool analyze_spectral(
        const ResponseSpectrum& spectrum_x,
        const ResponseSpectrum& spectrum_y,
        const ResponseSpectrum& spectrum_z,
        const SpectralAnalysisSettings& settings = {}
    );

    // Single-direction variant
    bool analyze_spectral(
        const ResponseSpectrum& spectrum,
        const Eigen::Vector3d& direction,
        const SpectralAnalysisSettings& settings = {}
    );

    // Query spectral results
    bool has_spectral_results() const;
    const SpectralAnalysisResult& get_spectral_result() const;

    // Convenience methods
    Eigen::VectorXd get_spectral_displacements() const;
    std::vector<SpectralElementForces> get_spectral_forces() const;
    std::vector<SpectralReaction> get_spectral_reactions() const;
    double get_spectral_base_shear() const;

private:
    std::unique_ptr<SpectralAnalysisResult> spectral_result_;
};

struct SpectralAnalysisResult {
    bool success = false;
    std::string message;

    // Settings used
    SpectralAnalysisSettings settings;

    // Modal information
    int n_modes_used;
    double mass_participation_x;  // Cumulative [%]
    double mass_participation_y;
    double mass_participation_z;
    bool missing_mass_applied;

    // Combined responses
    Eigen::VectorXd displacements;
    std::vector<SpectralElementForces> element_forces;
    std::vector<SpectralReaction> reactions;

    // Summary values
    double base_shear_x;
    double base_shear_y;
    double base_shear_z;
    double max_displacement;
};
```

**Workflow:**
1. Check eigenvalue results exist (run if not)
2. Create SpectralAnalyzer with modes, K, M
3. Compute modal responses for each direction
4. Apply modal combination (SRSS/CQC)
5. Apply directional combination
6. Compute missing mass correction if needed
7. Compute element forces and reactions
8. Store results

**Acceptance Criteria:**
- [ ] analyze_spectral() returns true on success
- [ ] Automatically runs eigenvalue analysis if not done
- [ ] Uses all computed modes by default (or n_modes from settings)
- [ ] Results accessible via get_spectral_result()
- [ ] Error handling for missing mass warning
- [ ] Single-direction and multi-direction APIs both work
- [ ] Results cleared when model is modified

---

### Task 19.10: Python Bindings

**File:** `cpp/bindings/bindings.cpp`

**Description:** Expose spectral analysis to Python.

**Bindings:**

```cpp
// Enums
py::enum_<SpectrumType>(m, "SpectrumType")
    .value("UserDefined", SpectrumType::UserDefined)
    .value("Eurocode8Type1", SpectrumType::Eurocode8Type1)
    .value("Eurocode8Type2", SpectrumType::Eurocode8Type2)
    .value("ASCE7", SpectrumType::ASCE7)
    .value("Constant", SpectrumType::Constant);

py::enum_<CombinationRule>(m, "CombinationRule")
    .value("SRSS", CombinationRule::SRSS)
    .value("CQC", CombinationRule::CQC)
    .value("AbsoluteSum", CombinationRule::AbsoluteSum);

py::enum_<DirectionalCombination>(m, "DirectionalCombination")
    .value("SRSS", DirectionalCombination::SRSS)
    .value("Rule100_30_30", DirectionalCombination::Rule100_30_30)
    .value("Maximum", DirectionalCombination::Maximum);

// ResponseSpectrum
py::class_<ResponseSpectrum>(m, "ResponseSpectrum")
    .def(py::init<>())
    .def(py::init<const std::vector<SpectrumPoint>&>())
    .def_static("eurocode8_type1", &ResponseSpectrum::eurocode8_type1,
        py::arg("ag"), py::arg("S"), py::arg("TB"), py::arg("TC"), py::arg("TD"),
        py::arg("eta") = 1.0)
    .def_static("eurocode8_type2", &ResponseSpectrum::eurocode8_type2)
    .def_static("constant", &ResponseSpectrum::constant)
    .def("get_acceleration", &ResponseSpectrum::get_acceleration)
    .def("get_displacement", &ResponseSpectrum::get_displacement)
    .def("zpa", &ResponseSpectrum::zpa)
    .def_property_readonly("type", &ResponseSpectrum::type);

// SpectralAnalysisSettings
py::class_<SpectralAnalysisSettings>(m, "SpectralAnalysisSettings")
    .def(py::init<>())
    .def_readwrite("n_modes", &SpectralAnalysisSettings::n_modes)
    .def_readwrite("damping_ratio", &SpectralAnalysisSettings::damping_ratio)
    .def_readwrite("combination", &SpectralAnalysisSettings::combination)
    .def_readwrite("dir_combination", &SpectralAnalysisSettings::dir_combination)
    .def_readwrite("include_missing_mass", &SpectralAnalysisSettings::include_missing_mass);

// SpectralAnalysisResult
py::class_<SpectralAnalysisResult>(m, "SpectralAnalysisResult")
    .def_readonly("success", &SpectralAnalysisResult::success)
    .def_readonly("message", &SpectralAnalysisResult::message)
    .def_readonly("n_modes_used", &SpectralAnalysisResult::n_modes_used)
    .def_readonly("mass_participation_x", &SpectralAnalysisResult::mass_participation_x)
    // ... etc

// Model methods
py::class_<Model>(m, "Model")
    // ... existing
    .def("analyze_spectral",
         py::overload_cast<const ResponseSpectrum&, const ResponseSpectrum&,
                          const ResponseSpectrum&, const SpectralAnalysisSettings&>(
             &Model::analyze_spectral),
         py::arg("spectrum_x"), py::arg("spectrum_y"), py::arg("spectrum_z"),
         py::arg("settings") = SpectralAnalysisSettings{})
    .def("has_spectral_results", &Model::has_spectral_results)
    .def("get_spectral_result", &Model::get_spectral_result)
    .def("get_spectral_displacements", &Model::get_spectral_displacements)
    .def("get_spectral_base_shear", &Model::get_spectral_base_shear);
```

**Acceptance Criteria:**
- [ ] All C++ types exported to Python
- [ ] ResponseSpectrum factory methods accessible (eurocode8_type1, etc.)
- [ ] SpectralAnalysisSettings configurable from Python
- [ ] Results accessible with all fields
- [ ] Displacements returned as numpy arrays
- [ ] Element forces accessible as list of structs

---

### Task 19.11: Python API Wrapper

**File:** `src/grillex/core/model_wrapper.py`

**Description:** Add high-level Python API for spectral analysis.

**API:**

```python
class StructuralModel:
    def analyze_spectral(
        self,
        spectrum_x: 'ResponseSpectrum',
        spectrum_y: Optional['ResponseSpectrum'] = None,
        spectrum_z: Optional['ResponseSpectrum'] = None,
        damping_ratio: float = 0.05,
        combination: str = "CQC",
        dir_combination: str = "SRSS",
        n_modes: int = 0,
        include_missing_mass: bool = True
    ) -> bool:
        """
        Perform response spectrum analysis.

        Args:
            spectrum_x: Response spectrum for X direction
            spectrum_y: Response spectrum for Y direction (optional)
            spectrum_z: Response spectrum for Z direction (optional)
            damping_ratio: Modal damping ratio (default 5%)
            combination: Modal combination rule ("SRSS", "CQC", "AbsoluteSum")
            dir_combination: Directional combination ("SRSS", "100-30-30", "Maximum")
            n_modes: Number of modes (0 = all computed modes)
            include_missing_mass: Apply missing mass correction if needed

        Returns:
            True if analysis succeeded

        Example:
            >>> spectrum = ResponseSpectrum.eurocode8_type1(
            ...     ag=0.25*9.81, S=1.15, TB=0.20, TC=0.60, TD=2.0
            ... )
            >>> model.analyze_spectral(spectrum)
            True
        """

    def create_eurocode8_spectrum(
        self,
        ag: float,
        ground_type: str = "C",
        spectrum_type: int = 1,
        damping_pct: float = 5.0
    ) -> 'ResponseSpectrum':
        """
        Create Eurocode 8 response spectrum.

        Args:
            ag: Design ground acceleration [m/s²]
            ground_type: Ground type ("A", "B", "C", "D", "E")
            spectrum_type: 1 or 2
            damping_pct: Damping percentage (5% gives η=1)

        Returns:
            ResponseSpectrum object
        """

    def get_spectral_displacements(self) -> Dict[str, np.ndarray]:
        """
        Get spectral displacement results.

        Returns:
            Dictionary with node_id keys and displacement arrays
        """

    def get_spectral_displacement_at(
        self,
        position: List[float],
        dof: 'DOFIndex'
    ) -> float:
        """Get spectral displacement at position for specified DOF."""

    def get_spectral_forces_dataframe(self) -> 'pd.DataFrame':
        """
        Get spectral element forces as DataFrame.

        Columns: element_id, max_axial, max_shear_y, max_shear_z,
                 max_moment_y, max_moment_z, max_torsion
        """

    def get_spectral_reactions_dataframe(self) -> 'pd.DataFrame':
        """
        Get spectral reactions as DataFrame.

        Columns: node_id, Fx, Fy, Fz, Mx, My, Mz
        """

    def get_spectral_summary(self) -> Dict:
        """
        Get summary of spectral analysis.

        Returns:
            Dictionary with base_shear, max_displacement, n_modes_used,
            mass_participation, missing_mass_applied
        """
```

**Acceptance Criteria:**
- [ ] analyze_spectral() provides clean interface
- [ ] Helper method for Eurocode 8 spectrum creation
- [ ] Displacement queries work at arbitrary positions
- [ ] DataFrame outputs for forces and reactions
- [ ] Summary method for quick overview
- [ ] Docstrings complete with examples and units
- [ ] Type hints for all public methods

---

### Task 19.12: Validation Tests - Analytical Benchmarks

**File:** `tests/python/test_phase19_spectral.py`

**Description:** Validate spectral analysis against hand calculations.

**Test Cases:**

```python
class TestSpectralAnalytical:
    """Analytical benchmark tests for spectral analysis."""

    def test_sdof_spectral_response(self):
        """
        SDOF system with known spectral response.

        f = 2 Hz, Sa(0.5s) = 5.0 m/s²
        Expected: u = Sa / ω² = 5.0 / (2π×2)² = 0.0316 m
        """

    def test_cantilever_spectral_tip_displacement(self):
        """
        Cantilever beam with constant spectrum.

        First mode participation ~ 0.783, Sd = Sa / ω₁²
        u_tip ≈ Γ₁ × Sd × φ₁(tip)
        """

    def test_base_shear_equals_effective_mass_times_sa(self):
        """
        Base shear = Σ Meff,n × Sa,n (for SRSS combination)
        """

    def test_srss_vs_cqc_well_separated_modes(self):
        """
        For well-separated modes, SRSS ≈ CQC (within 5%)
        """

    def test_cqc_correlation_coefficient(self):
        """
        Verify CQC ρij against published values.

        ω₁ = ω₂: ρ = 1.0
        ω₁/ω₂ = 0.5, ξ = 5%: ρ ≈ 0.05
        """

    def test_eurocode8_spectrum_values(self):
        """
        EC8 Type 1, Ground C, ag=0.25g

        T=0: Sa = ag×S = 0.25×9.81×1.15 = 2.82 m/s²
        T=0.4s (plateau): Sa = ag×S×η×2.5 = 7.05 m/s²
        T=1.0s: Sa = ag×S×η×2.5×TC/T = 4.23 m/s²
        """

    def test_100_30_30_vs_srss_symmetric_structure(self):
        """
        For symmetric structure with equal X/Y spectra:
        100-30-30 gives √(1² + 0.3² + 0.3²) = 1.086 times single direction
        """

    def test_missing_mass_correction_effect(self):
        """
        With only 1 mode (< 90% mass), missing mass correction
        should increase response.
        """
```

**Acceptance Criteria:**
- [ ] SDOF spectral response within 1% of analytical
- [ ] Cantilever tip displacement within 5% of hand calculation
- [ ] Base shear equals effective mass × spectral acceleration
- [ ] SRSS ≈ CQC for well-separated modes (within 5%)
- [ ] CQC correlation matches published values
- [ ] Eurocode 8 spectrum matches code tables (within 0.5%)
- [ ] Missing mass correction increases conservative response

---

### Task 19.13: Validation Tests - Code Compliance

**File:** `tests/python/test_phase19_spectral.py`

**Description:** Test code compliance features.

**Test Cases:**

```python
class TestCodeCompliance:
    """Tests for building code compliance features."""

    def test_90_percent_mass_participation_warning(self):
        """Warn when cumulative mass < 90%."""

    def test_missing_mass_applied_when_below_threshold(self):
        """Missing mass correction applied automatically."""

    def test_no_missing_mass_when_above_threshold(self):
        """No correction when participation > threshold."""

    def test_n_modes_limit_respected(self):
        """Only requested number of modes used."""

    def test_rigid_body_modes_excluded(self):
        """Zero-frequency modes not included in spectral response."""

    def test_damping_ratio_affects_results(self):
        """Different damping gives different spectral values."""
```

**Acceptance Criteria:**
- [ ] Warning issued when mass participation < 90%
- [ ] Missing mass correction automatically applied when needed
- [ ] Correction skipped when participation is sufficient
- [ ] Mode count limit properly enforced
- [ ] Rigid body modes excluded from spectral analysis
- [ ] Damping ratio correctly affects spectrum interpolation

---

### Task 19.14: Integration Tests

**File:** `tests/python/test_phase19_spectral.py`

**Description:** Integration tests for real-world scenarios.

**Test Cases:**

```python
class TestSpectralIntegration:
    """Integration tests for spectral analysis."""

    def test_multistory_frame_spectral(self):
        """Multi-story frame building under seismic load."""

    def test_grillage_deck_spectral(self):
        """Bridge deck grillage under vertical spectrum."""

    def test_combined_static_and_spectral(self):
        """
        Spectral results can be combined with static load cases
        using load combination factors.
        """

    def test_spectral_with_point_masses(self):
        """Equipment masses on structure."""

    def test_spectral_yaml_workflow(self):
        """Load model from YAML, run spectral analysis."""

    def test_large_model_performance(self):
        """500+ DOF model completes in reasonable time."""

    def test_multi_direction_earthquake(self):
        """X, Y, Z spectra applied simultaneously."""
```

**Acceptance Criteria:**
- [ ] Multi-story frame produces reasonable story drifts
- [ ] Grillage works with vertical spectrum
- [ ] Static + spectral load combination supported
- [ ] Point masses included in modal participation
- [ ] YAML workflow end-to-end verified
- [ ] Performance acceptable for 500+ DOF models
- [ ] Three-directional analysis produces correct combined response

---

### Task 19.15: Documentation

**File:** `docs/user/spectral_analysis.rst`

**Description:** User documentation for spectral analysis.

**Contents:**
1. Introduction to response spectrum analysis
2. When to use spectral vs time-history analysis
3. Creating response spectra (EC8, user-defined)
4. Running spectral analysis
5. Interpreting results (displacements, forces, base shear)
6. Modal combination rules (SRSS vs CQC)
7. Directional combination
8. Missing mass correction
9. Example: seismic design of a frame
10. Example: equipment qualification
11. Troubleshooting
12. Technical reference

**Acceptance Criteria:**
- [ ] Clear explanation of response spectrum method
- [ ] Step-by-step examples with code
- [ ] Eurocode 8 spectrum creation guide
- [ ] Interpretation guide for results
- [ ] Troubleshooting section
- [ ] All examples are doctests that pass

---

### Task 19.16: LLM Tool Schema

**File:** `src/grillex/llm/tools.py`

**Description:** Add tool schemas for LLM-driven spectral analysis.

**Tools:**

```python
{
    "name": "create_eurocode8_spectrum",
    "description": "Create Eurocode 8 response spectrum for seismic analysis.",
    "input_schema": {
        "type": "object",
        "properties": {
            "ag": {
                "type": "number",
                "description": "Design ground acceleration [m/s²]"
            },
            "ground_type": {
                "type": "string",
                "enum": ["A", "B", "C", "D", "E"],
                "description": "Ground type per EC8 Table 3.1"
            },
            "spectrum_type": {
                "type": "integer",
                "enum": [1, 2],
                "description": "Type 1 (high seismicity) or Type 2 (low)"
            }
        },
        "required": ["ag"]
    }
},
{
    "name": "analyze_spectral",
    "description": "Run response spectrum analysis for seismic design.",
    "input_schema": {
        "type": "object",
        "properties": {
            "spectrum_id": {
                "type": "string",
                "description": "ID of spectrum to use (created with create_eurocode8_spectrum)"
            },
            "directions": {
                "type": "array",
                "items": {"type": "string", "enum": ["X", "Y", "Z"]},
                "description": "Directions to apply spectrum"
            },
            "combination_rule": {
                "type": "string",
                "enum": ["SRSS", "CQC", "AbsoluteSum"],
                "default": "CQC"
            }
        }
    }
},
{
    "name": "get_spectral_summary",
    "description": "Get summary of spectral analysis including base shear and max displacement.",
    "input_schema": {
        "type": "object",
        "properties": {}
    }
},
{
    "name": "check_seismic_drift",
    "description": "Check if spectral displacements meet drift limits.",
    "input_schema": {
        "type": "object",
        "properties": {
            "drift_limit": {
                "type": "number",
                "description": "Maximum allowed drift ratio (e.g., 0.01 for 1%)"
            },
            "story_height": {
                "type": "number",
                "description": "Story height for drift calculation [m]"
            }
        },
        "required": ["drift_limit", "story_height"]
    }
}
```

**Acceptance Criteria:**
- [ ] Tool schema for create_eurocode8_spectrum
- [ ] Tool schema for analyze_spectral
- [ ] Tool schema for get_spectral_summary
- [ ] Tool schema for check_seismic_drift
- [ ] Handler implementations in ToolExecutor
- [ ] Error suggestions for common spectral analysis issues

---

## Acceptance Criteria Summary

| Task | Description | Criteria Count |
|------|-------------|----------------|
| 19.1 | Response Spectrum Data Structures | 8 |
| 19.2 | Eurocode 8 Spectrum Implementation | 6 |
| 19.3 | Modal Response Calculation | 6 |
| 19.4 | Modal Combination Implementation | 6 |
| 19.5 | Directional Combination | 5 |
| 19.6 | Missing Mass Correction | 6 |
| 19.7 | Spectral Internal Forces | 6 |
| 19.8 | Spectral Reactions | 5 |
| 19.9 | Model Integration | 7 |
| 19.10 | Python Bindings | 6 |
| 19.11 | Python API Wrapper | 7 |
| 19.12 | Analytical Benchmarks | 7 |
| 19.13 | Code Compliance Tests | 6 |
| 19.14 | Integration Tests | 7 |
| 19.15 | Documentation | 6 |
| 19.16 | LLM Tool Schema | 6 |
| **Total** | | **100** |

---

## Implementation Order

**Recommended sequence:**

1. **Task 19.1** - Data structures (foundation)
2. **Task 19.2** - Eurocode 8 spectrum (needed for testing)
3. **Task 19.3** - Modal response calculation
4. **Task 19.4** - Modal combination (SRSS, CQC)
5. **Task 19.12** - Analytical benchmarks (validate 19.3-19.4)
6. **Task 19.5** - Directional combination
7. **Task 19.6** - Missing mass correction
8. **Task 19.7-19.8** - Forces and reactions
9. **Task 19.9** - Model integration
10. **Task 19.10-19.11** - Python bindings and wrapper
11. **Task 19.13-19.14** - Remaining tests
12. **Task 19.15-19.16** - Documentation and LLM tools

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| CQC formula numerical issues for very close modes | Low | Medium | Add epsilon to denominator; validate against references |
| Missing mass correction overshoots | Low | Low | Compare with full-mode analysis; add cap |
| Performance for many modes | Medium | Low | Cache spectral values; use sparse operations |
| Sign convention confusion in combined forces | Medium | High | Extensive testing; clear documentation |
| EC8 parameter confusion (ag vs agR) | Medium | Medium | Clear documentation with examples |

---

## Dependencies on Other Phases

| Phase | Dependency Type | Status |
|-------|-----------------|--------|
| Phase 16 (Eigenvalue) | **Required** - modes, participation factors | ✅ 96% Complete |
| Phase 5 (Loads) | **Required** - load case infrastructure | ✅ 100% Complete |
| Phase 7 (Internal Actions) | **Required** - force computation | ✅ 100% Complete |
| Phase 3 (Assembly) | **Required** - K, M matrices | ✅ 100% Complete |
| Phase 10 (Design Codes) | Optional - seismic code checks | 🔄 13% Complete |

---

## Future Extensions

After Phase 19 is complete, the following can be added:

1. **Time-History Analysis** - Direct integration for earthquake records
2. **Pushover Analysis** - Nonlinear static for capacity assessment
3. **Soil-Structure Interaction** - Foundation impedance functions
4. **Multi-Support Excitation** - Different spectra at different supports
5. **Floor Response Spectra** - For equipment design
6. **ASCE 7 Spectrum** - US code design spectrum
7. **IBC Spectrum** - International Building Code
