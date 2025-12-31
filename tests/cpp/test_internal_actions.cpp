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
