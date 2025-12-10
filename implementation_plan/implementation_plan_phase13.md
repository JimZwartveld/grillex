## Phase 13: Validation & Benchmarks

### Task 13.1: Create Simply Supported Beam Benchmark
**Requirements:** R-VAL-001, R-VAL-002
**Dependencies:** Phase 5 complete
**Difficulty:** Low

**Description:**
Create validation benchmark for simply supported beam.

**Steps:**
1. Create `tests/benchmarks/test_simply_supported_beam.py`:
   ```python
   def test_simply_supported_beam_uniform_load():
       """
       Simply supported beam with uniform load.
       Reference: M_max = wL²/8, δ_max = 5wL⁴/(384EI)
       """
       model = Model()
       # Create 6m beam with UDL of 10 kN/m
       # Check midspan moment = 10 * 6² / 8 = 45 kNm
       # Check midspan deflection against formula
   ```

2. Include comparison with analytical solution
3. Assert results within tolerance (e.g., 0.1%)

**Acceptance Criteria:**
- [ ] Test passes
- [ ] Results within tolerance of analytical
- [ ] Clear reference to formula

---

### Task 13.2: Create Cantilever Benchmark
**Requirements:** R-VAL-001, R-VAL-002
**Dependencies:** Phase 5 complete
**Difficulty:** Low

**Description:**
Create cantilever beam benchmark.

**Steps:**
1. Create benchmark test:
   ```python
   def test_cantilever_tip_load():
       """
       Cantilever with tip load.
       Reference: δ_tip = PL³/(3EI), M_base = PL
       """
       # Test deflection and reaction moment
   ```

2. Test both tip load and distributed load cases

**Acceptance Criteria:**
- [ ] Tests pass
- [ ] Deflection matches PL³/(3EI)
- [ ] Reaction moment matches PL

---

### Task 13.3: Create Beam with Offsets Benchmark
**Requirements:** R-VAL-002
**Dependencies:** Task 2.4 complete
**Difficulty:** Medium

**Description:**
Validate beams with end offsets.

**Steps:**
1. Create benchmark with known offset solution
2. Compare with reference software if available

**Acceptance Criteria:**
- [ ] Offset beam gives correct deflection
- [ ] Forces transfer correctly through offset

---

### Task 13.4: Create Cargo on Springs Benchmark
**Requirements:** R-VAL-002
**Dependencies:** Phase 9 complete
**Difficulty:** Medium

**Description:**
Validate cargo model under acceleration.

**Steps:**
1. Create simple cargo on springs model
2. Apply gravity acceleration
3. Verify spring forces equal cargo weight

**Acceptance Criteria:**
- [ ] Spring force = mass × g
- [ ] Forces balance correctly

---

### Task 13.5: Create Unconstrained Model Test
**Requirements:** R-VAL-003
**Dependencies:** Task 11.3 complete
**Difficulty:** Low

**Description:**
Test that unconstrained models are properly detected.

**Steps:**
1. Create model with no supports
2. Run analysis
3. Verify error is returned with correct code

**Acceptance Criteria:**
- [ ] UNCONSTRAINED_SYSTEM error returned
- [ ] Unconstrained DOFs identified

---

