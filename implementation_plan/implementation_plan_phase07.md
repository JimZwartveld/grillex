## Phase 7: Internal Actions & Results

### Task 7.1: Implement Element End Forces
**Requirements:** R-RES-001
**Dependencies:** Task 3.5
**Difficulty:** Medium

**Description:**
Compute element end forces from displacements.

**Steps:**
1. Add to BeamElement:
   ```cpp
   struct EndForces {
       double N;   // Axial force
       double Vy;  // Shear in y
       double Vz;  // Shear in z
       double Mx;  // Torsion
       double My;  // Bending about y
       double Mz;  // Bending about z
   };

   std::pair<EndForces, EndForces> compute_end_forces(
       const Eigen::VectorXd& global_displacements,
       const DOFHandler& dof_handler) const;
   ```

2. Algorithm:
   ```
   1. Extract element displacements from global vector
   2. Transform to local coordinates: u_local = T * u_global
   3. Compute local forces: f_local = K_local * u_local
   4. Subtract fixed-end forces if applicable
   5. Apply sign convention per R-COORD-004
   ```

**Acceptance Criteria:**
- [ ] End forces match reactions for simple cases
- [ ] Sign convention is consistent with requirements
- [ ] Forces satisfy equilibrium

---

### Task 7.2: Implement Internal Action Functions Along Beam
**Requirements:** R-RES-002, R-RES-003, R-RES-004, R-LOAD-008, R-LOAD-009
**Dependencies:** Task 7.1
**Difficulty:** High

**Description:**
Compute N, V, M along the beam length using shape functions.

**Steps:**
1. Add to BeamElement:
   ```cpp
   struct InternalActions {
       double x;   // Position along beam
       double N;   // Axial
       double Vy;  // Shear y
       double Vz;  // Shear z
       double Mx;  // Torsion
       double My;  // Moment about y
       double Mz;  // Moment about z
   };

   InternalActions get_internal_actions(double x) const;

   // Find extremes
   struct ActionExtreme {
       double x;
       double value;
   };
   std::pair<ActionExtreme, ActionExtreme> get_moment_extremes(char axis) const;
   ```

2. For bending with distributed load, use closed-form:
   ```
   M(x) = M_i + V_i * x - w * x² / 2  (for uniform load w)
   V(x) = V_i - w * x
   ```

3. For no distributed load, linear interpolation:
   ```
   M(x) = M_i + V_i * x
   V(x) = V_i (constant)
   ```

**Acceptance Criteria:**
- [ ] Simply supported beam with UDL: M_max = wL²/8 at midspan
- [ ] Cantilever with tip load: M_max = PL at support
- [ ] Extreme values are found correctly

---

### Task 7.2b: Implement Warping Results (Bimoments)
**Requirements:** R-RES-006 (new), R-ELEM-007
**Dependencies:** Task 7.1, Task 2.7
**Difficulty:** High

**Description:**
Compute warping-related results for elements with 7th DOF: bimoment, warping stress, warping displacement.

**Background:**
For thin-walled open sections under torsion with warping restraint:
- **Bimoment (B):** The generalized force conjugate to warping displacement
- **Warping normal stress:** σ_w = -B × ω / Iω (where ω is sectorial coordinate)
- **Total normal stress:** σ = N/A ± My×z/Iy ± Mz×y/Iz ± B×ω/Iω

**Steps:**
1. Extend EndForces struct for warping elements:
   ```cpp
   struct EndForces {
       double N;   // Axial force
       double Vy;  // Shear in y
       double Vz;  // Shear in z
       double Mx;  // Torsion (St. Venant)
       double My;  // Bending about y
       double Mz;  // Bending about z
       double B;   // Bimoment [kN·m²] - NEW for warping elements
   };
   ```

2. Add warping-specific internal actions:
   ```cpp
   struct WarpingInternalActions : InternalActions {
       double B;           // Bimoment [kN·m²]
       double Tw;          // Warping torsion component
       double sigma_w_max; // Maximum warping stress [kN/m²]
   };

   WarpingInternalActions get_warping_internal_actions(double x) const;
   ```

3. Implement bimoment calculation:
   ```
   For warping element with DOFs [θx_i, φ'_i, θx_j, φ'_j]:

   Local displacements from global:
   u_local = T_warping * u_global  (14×1 vector)

   Bimoment at ends:
   B_i = EIw × (φ''_i)
   B_j = EIw × (φ''_j)

   From element equations:
   [Mx_i]   [GJ/L + 12EIw/L³   6EIw/L²    ...] [θx_i ]
   [B_i ] = [6EIw/L²          4EIw/L     ...] [φ'_i ]
   [...]    [...                              ] [...]
   ```

4. Warping stress calculation:
   ```cpp
   double compute_warping_stress(double x, double omega) const {
       // omega = sectorial coordinate at point of interest
       // For I-sections: omega varies across the flange width
       WarpingInternalActions actions = get_warping_internal_actions(x);
       return -actions.B * omega / section->Iw;
   }
   ```

**Acceptance Criteria:**
- [ ] Bimoment at warping-restrained end matches analytical solution
- [ ] Warping-free end has B = 0
- [ ] Total normal stress = axial + bending + warping components
- [ ] For Iw = 0, bimoment results are zero (degenerates correctly)
- [ ] Sign convention consistent with standard references

---

### Task 7.3: Implement Check Locations
**Requirements:** R-RES-005, R-CODE-003
**Dependencies:** Task 7.2
**Difficulty:** Low

**Description:**
Support user-defined check locations on beams.

**Steps:**
1. Add to Beam Python class:
   ```python
   class Beam:
       check_locations: list[float] = []  # Normalized positions [0, 1]

       def add_check_location(self, x_normalized: float) -> None:
           """Add a check location at normalized position (0=start, 1=end)."""
           self.check_locations.append(x_normalized)

       def set_standard_check_locations(self) -> None:
           """Set standard check locations: ends and midspan."""
           self.check_locations = [0.0, 0.5, 1.0]
   ```

2. Results should include internal actions at all check locations

**Acceptance Criteria:**
- [ ] Check locations can be defined
- [ ] Internal actions are computed at check locations
- [ ] Default check locations work

---

