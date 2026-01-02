/**
 * @file test_internal_actions.cpp
 * @brief C++ tests for internal actions (bimoment calculations)
 *
 * These tests were moved from Python (test_phase7_internal_actions.py) because
 * pybind11 cannot handle Eigen::SparseMatrix as input parameters.
 *
 * Tests include:
 * - Bimoment continuity at internal supports
 * - Analytical comparison for cantilever I-beam under torsion
 * - Bimoment distribution along beam
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "grillex/node_registry.hpp"
#include "grillex/material.hpp"
#include "grillex/section.hpp"
#include "grillex/beam_element.hpp"
#include "grillex/dof_handler.hpp"
#include "grillex/assembler.hpp"
#include "grillex/boundary_condition.hpp"
#include "grillex/solver.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>
#include <vector>

using namespace grillex;
using Catch::Matchers::WithinAbs;

// =============================================================================
// Task 7.2b: Bimoment Continuity Tests
// =============================================================================

TEST_CASE("Bimoment continuous at internal support", "[InternalActions][bimoment][continuity]") {
    /**
     * Test bimoment continuity at internal support
     * (from TestTask72bBimomentContinuity.test_bimoment_continuous_at_internal_support)
     *
     * Two collinear warping elements with torque at internal support.
     * Bimoment must be continuous at the shared node.
     */
    double L = 6.0;  // Element length

    NodeRegistry registry(1e-6);
    Node* node1 = registry.get_or_create_node(0.0, 0.0, 0.0);
    Node* node2 = registry.get_or_create_node(L, 0.0, 0.0);     // Internal support
    Node* node3 = registry.get_or_create_node(2 * L, 0.0, 0.0);

    node1->enable_warping_dof();
    node2->enable_warping_dof();
    node3->enable_warping_dof();

    // IPE300 section with warping constant
    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);
    sec.Iw = 1.26e-7;  // Warping constant [m^6]

    BeamConfig config;
    config.include_warping = true;

    BeamElement elem1(1, node1, node2, &mat, &sec, config);
    BeamElement elem2(2, node2, node3, &mat, &sec, config);

    DOFHandler dof_handler;
    std::vector<BeamElement*> elements = {&elem1, &elem2};
    dof_handler.number_dofs_with_elements(registry, elements);

    // Verify shared warping DOF at internal node
    int warp_dof_node2_elem1 = dof_handler.get_warping_dof(elem1.id, node2->id);
    int warp_dof_node2_elem2 = dof_handler.get_warping_dof(elem2.id, node2->id);
    REQUIRE(warp_dof_node2_elem1 == warp_dof_node2_elem2);

    // Assemble system
    Assembler assembler(dof_handler);
    auto K = assembler.assemble_stiffness(elements);
    int n_dofs = dof_handler.total_dofs();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_dofs);

    // Apply torque at internal support (node2)
    int rx_node2 = dof_handler.get_global_dof(node2->id, DOFIndex::RX);
    double T = 10.0;  // kNm torque
    F[rx_node2] = T;

    // Fixed ends with warping restraint
    BCHandler bc;
    bc.fix_node_with_warping(node1->id, elem1.id);
    bc.fix_node_with_warping(node3->id, elem2.id);

    // Apply BCs and solve
    auto [K_mod, F_mod] = bc.apply_to_system(K, F, dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u = solver.solve(K_mod, F_mod);
    REQUIRE_FALSE(solver.is_singular());

    // Use get_warping_internal_actions to compute bimoment at internal node
    // Element 1: bimoment at x=L (node2)
    auto warping_actions1 = elem1.get_warping_internal_actions(L, u, dof_handler);
    double B1_at_node2 = warping_actions1.B;

    // Element 2: bimoment at x=0 (node2)
    auto warping_actions2 = elem2.get_warping_internal_actions(0.0, u, dof_handler);
    double B2_at_node2 = warping_actions2.B;

    // Bimoment must be continuous at internal support
    // Allow some numerical tolerance
    REQUIRE_THAT(B1_at_node2, WithinAbs(B2_at_node2, 1e-3));
}

// =============================================================================
// Task 7.2b: Analytical Comparison Tests
// =============================================================================

TEST_CASE("Cantilever bimoment matches analytical solution", "[InternalActions][bimoment][analytical]") {
    /**
     * Compare cantilever I-beam bimoment with analytical solution (Vlasov theory)
     * (from TestCantileverIBeamAnalyticalComparison.test_cantilever_bimoment_analytical)
     *
     * Analytical solution for cantilever under end torque T with warping restrained
     * at fixed end:
     *     k = sqrt(GJ / EIw)
     *     B(x) = T/k * sinh(k*(L-x)) / cosh(k*L)
     *
     * At fixed end (x=0): B(0) = T/k * tanh(kL)
     * At free end (x=L):  B(L) = 0
     */
    double L = 3.0;      // Beam length [m]
    double T = 10.0;     // Applied torque [kNm]
    int n_elem = 10;     // Use finer mesh for better accuracy

    NodeRegistry registry(1e-6);
    double elem_len = L / n_elem;
    std::vector<Node*> nodes;
    for (int i = 0; i <= n_elem; ++i) {
        double x = i * elem_len;
        Node* node = registry.get_or_create_node(x, 0.0, 0.0);
        node->enable_warping_dof();
        nodes.push_back(node);
    }

    // IPE300 section
    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);
    sec.Iw = 1.26e-7;  // Warping constant [m^6]

    BeamConfig config;
    config.include_warping = true;

    std::vector<std::unique_ptr<BeamElement>> element_storage;
    std::vector<BeamElement*> elements;
    for (int i = 0; i < n_elem; ++i) {
        auto elem = std::make_unique<BeamElement>(i + 1, nodes[i], nodes[i + 1], &mat, &sec, config);
        elements.push_back(elem.get());
        element_storage.push_back(std::move(elem));
    }

    // Number DOFs
    DOFHandler dof_handler;
    dof_handler.number_dofs_with_elements(registry, elements);

    // Assemble
    Assembler assembler(dof_handler);
    auto K = assembler.assemble_stiffness(elements);
    int n_dofs = dof_handler.total_dofs();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_dofs);

    // Apply torque at free end (last node)
    int rx_end = dof_handler.get_global_dof(nodes.back()->id, DOFIndex::RX);
    F[rx_end] = T;

    // Fixed end with warping restraint at node 0, free at other end
    BCHandler bc;
    bc.fix_node_with_warping(nodes[0]->id, elements[0]->id);
    // Free end: only pin for stability (translations fixed, rotation/warping free)
    bc.pin_node(nodes.back()->id);

    // Apply BCs and solve
    auto [K_mod, F_mod] = bc.apply_to_system(K, F, dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u = solver.solve(K_mod, F_mod);
    REQUIRE_FALSE(solver.is_singular());

    // Material properties
    double E = mat.E;
    double G = mat.G;
    double GJ = G * sec.J;
    double EIw = E * sec.Iw;

    // Analytical solution parameters
    double k = std::sqrt(GJ / EIw);
    double kL = k * L;

    // Compute bimoment at fixed end (x=0) using first element
    auto warping_actions_0 = elements[0]->get_warping_internal_actions(0.0, u, dof_handler);
    double B_computed_0 = warping_actions_0.B;

    // Analytical bimoment at fixed end: B(0) = T/k * tanh(kL)
    double B_analytical_0 = T / k * std::tanh(kL);

    // Compare magnitudes - FEM and analytical may use different sign conventions
    // Allow 10% tolerance for FEM approximation
    double error_percent = std::abs(std::abs(B_computed_0) - std::abs(B_analytical_0)) / std::abs(B_analytical_0) * 100.0;
    REQUIRE(error_percent < 10.0);

    // Verify non-zero bimoment at fixed end (warping is restrained)
    REQUIRE(std::abs(B_computed_0) > 0.1 * std::abs(B_analytical_0));

    // Compute bimoment at free end (x=L) - should be ~0
    auto warping_actions_L = elements.back()->get_warping_internal_actions(elem_len, u, dof_handler);
    double B_computed_L = warping_actions_L.B;

    // At free end, bimoment magnitude should be much smaller than at fixed end
    REQUIRE(std::abs(B_computed_L) < 0.2 * std::abs(B_computed_0));
}

TEST_CASE("Bimoment distribution matches analytical along beam", "[InternalActions][bimoment][distribution]") {
    /**
     * Verify bimoment distribution matches analytical along entire beam
     * (from TestCantileverIBeamAnalyticalComparison.test_bimoment_distribution_along_beam)
     */
    double L = 3.0;
    double T = 10.0;
    int n_elem = 10;  // Use finer mesh for better accuracy

    NodeRegistry registry(1e-6);
    double elem_len = L / n_elem;
    std::vector<Node*> nodes;
    for (int i = 0; i <= n_elem; ++i) {
        double x = i * elem_len;
        Node* node = registry.get_or_create_node(x, 0.0, 0.0);
        node->enable_warping_dof();
        nodes.push_back(node);
    }

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);
    sec.Iw = 1.26e-7;

    BeamConfig config;
    config.include_warping = true;

    std::vector<std::unique_ptr<BeamElement>> element_storage;
    std::vector<BeamElement*> elements;
    for (int i = 0; i < n_elem; ++i) {
        auto elem = std::make_unique<BeamElement>(i + 1, nodes[i], nodes[i + 1], &mat, &sec, config);
        elements.push_back(elem.get());
        element_storage.push_back(std::move(elem));
    }

    DOFHandler dof_handler;
    dof_handler.number_dofs_with_elements(registry, elements);

    Assembler assembler(dof_handler);
    auto K = assembler.assemble_stiffness(elements);
    int n_dofs = dof_handler.total_dofs();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_dofs);

    int rx_end = dof_handler.get_global_dof(nodes.back()->id, DOFIndex::RX);
    F[rx_end] = T;

    BCHandler bc;
    bc.fix_node_with_warping(nodes[0]->id, elements[0]->id);
    bc.pin_node(nodes.back()->id);

    auto [K_mod, F_mod] = bc.apply_to_system(K, F, dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u = solver.solve(K_mod, F_mod);

    double E = mat.E;
    double G = mat.G;
    double GJ = G * sec.J;
    double EIw = E * sec.Iw;
    double k = std::sqrt(GJ / EIw);

    // Check bimoment at nodal locations along the beam
    // Compare magnitudes to account for possible sign convention differences
    double max_error = 0.0;

    for (int i = 0; i < n_elem; ++i) {
        // Check at start of element
        double x_global = i * elem_len;
        auto warping_actions = elements[i]->get_warping_internal_actions(0.0, u, dof_handler);
        double B_computed = warping_actions.B;

        // Analytical: B(x) = T/k * sinh(k*(L-x)) / cosh(k*L)
        double B_analytical = T / k * std::sinh(k * (L - x_global)) / std::cosh(k * L);

        if (std::abs(B_analytical) > 1e-6) {
            // Compare magnitudes - FEM and analytical may use different sign conventions
            double error = std::abs(std::abs(B_computed) - std::abs(B_analytical)) / std::abs(B_analytical) * 100.0;
            max_error = std::max(max_error, error);
        }
    }

    // Maximum error should be within 15% for FEM approximation
    REQUIRE(max_error < 15.0);
}

// =============================================================================
// SCI P385 Figure 2.7: Torsion Split Verification
// =============================================================================

TEST_CASE("SCI P385 Figure 2.7 - Torsion split at 10% intervals", "[InternalActions][torsion][SCIP385]") {
    /**
     * Comprehensive test matching SCI P385 Figure 2.7 analytical formulas.
     *
     * For a cantilever beam with:
     * - Fixed end (warping restrained) at x=0
     * - Free end at x=L
     * - Torque T applied at free end
     *
     * Analytical solutions (Vlasov theory):
     *     k = sqrt(GJ / EIw)              - torsion parameter
     *
     *     B(x) = T/k * sinh(k*(L-x)) / cosh(k*L)    - bimoment
     *
     *     Mx_w(x) = T * cosh(k*(L-x)) / cosh(k*L)   - warping torsion
     *
     *     Mx_sv(x) = T * [1 - cosh(k*(L-x)) / cosh(k*L)]  - St. Venant torsion
     *
     *     Mx(x) = T = Mx_sv(x) + Mx_w(x)            - total torsion (constant)
     *
     * This test verifies all three components at 0%, 10%, 20%, ..., 100% of beam length.
     */
    double L = 6.0;      // Beam length [m]
    double T = 1.0;      // Applied torque [kNm] - unit load for easy verification
    int n_elem = 20;     // Fine mesh for accuracy

    NodeRegistry registry(1e-6);
    double elem_len = L / n_elem;
    std::vector<Node*> nodes;
    for (int i = 0; i <= n_elem; ++i) {
        double x = i * elem_len;
        Node* node = registry.get_or_create_node(x, 0.0, 0.0);
        node->enable_warping_dof();
        nodes.push_back(node);
    }

    // IPE300 section with warping constant
    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);
    sec.Iw = 1.26e-7;  // Warping constant [m^6]

    BeamConfig config;
    config.include_warping = true;

    std::vector<std::unique_ptr<BeamElement>> element_storage;
    std::vector<BeamElement*> elements;
    for (int i = 0; i < n_elem; ++i) {
        auto elem = std::make_unique<BeamElement>(i + 1, nodes[i], nodes[i + 1], &mat, &sec, config);
        elements.push_back(elem.get());
        element_storage.push_back(std::move(elem));
    }

    // Number DOFs
    DOFHandler dof_handler;
    dof_handler.number_dofs_with_elements(registry, elements);

    // Assemble
    Assembler assembler(dof_handler);
    auto K = assembler.assemble_stiffness(elements);
    int n_dofs = dof_handler.total_dofs();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_dofs);

    // Apply torque at free end (last node)
    int rx_end = dof_handler.get_global_dof(nodes.back()->id, DOFIndex::RX);
    F[rx_end] = T;

    // Fixed end with warping restraint at node 0, free at other end
    BCHandler bc;
    bc.fix_node_with_warping(nodes[0]->id, elements[0]->id);
    // Free end: only pin for stability (translations fixed, rotation/warping free)
    bc.pin_node(nodes.back()->id);

    // Apply BCs and solve
    auto [K_mod, F_mod] = bc.apply_to_system(K, F, dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u = solver.solve(K_mod, F_mod);
    REQUIRE_FALSE(solver.is_singular());

    // Material properties
    double E = mat.E;
    double G = mat.G;
    double GJ = G * sec.J;
    double EIw = E * sec.Iw;

    // SCI P385 analytical solution parameters
    double k = std::sqrt(GJ / EIw);
    double kL = k * L;
    double cosh_kL = std::cosh(kL);

    // Test at 10% intervals (0%, 10%, 20%, ..., 100%)
    double max_B_error = 0.0;
    double max_Mx_w_error = 0.0;
    double max_Mx_sv_error = 0.0;
    double max_Mx_total_error = 0.0;

    for (int pct = 0; pct <= 100; pct += 10) {
        double x_global = L * pct / 100.0;

        // Find element containing x_global and compute local x
        int elem_idx = std::min(static_cast<int>(x_global / elem_len), n_elem - 1);
        double x_local = x_global - elem_idx * elem_len;

        // Clamp x_local to element bounds
        if (x_local < 0) x_local = 0;
        if (x_local > elem_len) x_local = elem_len;

        // Get warping internal actions from FEM
        auto actions = elements[elem_idx]->get_warping_internal_actions(x_local, u, dof_handler);

        // SCI P385 analytical solutions
        double sinh_k_Lmx = std::sinh(k * (L - x_global));
        double cosh_k_Lmx = std::cosh(k * (L - x_global));

        double B_analytical = T / k * sinh_k_Lmx / cosh_kL;
        double Mx_w_analytical = T * cosh_k_Lmx / cosh_kL;
        double Mx_sv_analytical = T * (1.0 - cosh_k_Lmx / cosh_kL);
        double Mx_total_analytical = T;  // Total is constant

        // Compute errors (compare magnitudes for sign convention independence)
        if (std::abs(B_analytical) > 1e-6) {
            double B_error = std::abs(std::abs(actions.B) - std::abs(B_analytical)) / std::abs(B_analytical) * 100.0;
            max_B_error = std::max(max_B_error, B_error);
        }

        if (std::abs(Mx_w_analytical) > 1e-6) {
            double Mx_w_error = std::abs(std::abs(actions.Mx_w) - std::abs(Mx_w_analytical)) / std::abs(Mx_w_analytical) * 100.0;
            max_Mx_w_error = std::max(max_Mx_w_error, Mx_w_error);
        }

        if (std::abs(Mx_sv_analytical) > 1e-6) {
            double Mx_sv_error = std::abs(std::abs(actions.Mx_sv) - std::abs(Mx_sv_analytical)) / std::abs(Mx_sv_analytical) * 100.0;
            max_Mx_sv_error = std::max(max_Mx_sv_error, Mx_sv_error);
        }

        double Mx_total_error = std::abs(std::abs(actions.Mx) - std::abs(Mx_total_analytical)) / std::abs(Mx_total_analytical) * 100.0;
        max_Mx_total_error = std::max(max_Mx_total_error, Mx_total_error);
    }

    // Verify all components are within acceptable tolerance
    // Allow up to 10% error for FEM approximation
    REQUIRE(max_B_error < 10.0);
    REQUIRE(max_Mx_w_error < 10.0);
    REQUIRE(max_Mx_sv_error < 10.0);
    REQUIRE(max_Mx_total_error < 5.0);  // Total should be more accurate
}

TEST_CASE("SCI P385 - Torsion split boundary conditions verification", "[InternalActions][torsion][SCIP385][boundary]") {
    /**
     * Verify SCI P385 boundary conditions are satisfied:
     * - At fixed end (x=0): Mx_w = T (100% warping), Mx_sv = 0 (0% St. Venant)
     * - At free end (x=L): Mx_w → 0, Mx_sv → T (100% St. Venant), B = 0
     */
    double L = 6.0;
    double T = 10.0;
    int n_elem = 20;

    NodeRegistry registry(1e-6);
    double elem_len = L / n_elem;
    std::vector<Node*> nodes;
    for (int i = 0; i <= n_elem; ++i) {
        double x = i * elem_len;
        Node* node = registry.get_or_create_node(x, 0.0, 0.0);
        node->enable_warping_dof();
        nodes.push_back(node);
    }

    Material mat(1, "Steel", 210e6, 0.3, 7.85e-6);
    Section sec(1, "IPE300", 0.00538, 8.36e-5, 6.04e-6, 2.01e-7);
    sec.Iw = 1.26e-7;

    BeamConfig config;
    config.include_warping = true;

    std::vector<std::unique_ptr<BeamElement>> element_storage;
    std::vector<BeamElement*> elements;
    for (int i = 0; i < n_elem; ++i) {
        auto elem = std::make_unique<BeamElement>(i + 1, nodes[i], nodes[i + 1], &mat, &sec, config);
        elements.push_back(elem.get());
        element_storage.push_back(std::move(elem));
    }

    DOFHandler dof_handler;
    dof_handler.number_dofs_with_elements(registry, elements);

    Assembler assembler(dof_handler);
    auto K = assembler.assemble_stiffness(elements);
    int n_dofs = dof_handler.total_dofs();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_dofs);

    int rx_end = dof_handler.get_global_dof(nodes.back()->id, DOFIndex::RX);
    F[rx_end] = T;

    BCHandler bc;
    bc.fix_node_with_warping(nodes[0]->id, elements[0]->id);
    bc.pin_node(nodes.back()->id);

    auto [K_mod, F_mod] = bc.apply_to_system(K, F, dof_handler);
    LinearSolver solver;
    Eigen::VectorXd u = solver.solve(K_mod, F_mod);
    REQUIRE_FALSE(solver.is_singular());

    // Check fixed end (x=0): 100% warping torsion
    auto actions_fixed = elements[0]->get_warping_internal_actions(0.0, u, dof_handler);

    // At fixed end, warping torsion should dominate
    double warping_ratio_fixed = std::abs(actions_fixed.Mx_w) / std::abs(actions_fixed.Mx);
    REQUIRE(warping_ratio_fixed > 0.95);  // At least 95% warping at fixed end

    // St. Venant should be near zero at fixed end
    double sv_ratio_fixed = std::abs(actions_fixed.Mx_sv) / std::abs(actions_fixed.Mx);
    REQUIRE(sv_ratio_fixed < 0.05);  // Less than 5% St. Venant at fixed end

    // Check free end (x=L): Bimoment should be ~0
    auto actions_free = elements.back()->get_warping_internal_actions(elem_len, u, dof_handler);

    // Bimoment at free end should be near zero
    REQUIRE(std::abs(actions_free.B) < 0.1 * std::abs(actions_fixed.B));

    // At free end, St. Venant should dominate
    double sv_ratio_free = std::abs(actions_free.Mx_sv) / std::abs(actions_free.Mx);
    REQUIRE(sv_ratio_free > 0.90);  // At least 90% St. Venant at free end

    // Total torsion should be constant (= T) throughout
    REQUIRE_THAT(std::abs(actions_fixed.Mx), WithinAbs(T, 0.1 * T));
    REQUIRE_THAT(std::abs(actions_free.Mx), WithinAbs(T, 0.1 * T));
}
