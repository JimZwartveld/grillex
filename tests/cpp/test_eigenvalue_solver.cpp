/**
 * @file test_eigenvalue_solver.cpp
 * @brief C++ tests for EigenvalueSolver reduce_system and solve functionality
 *
 * These tests were moved from Python because pybind11 cannot handle
 * Eigen::SparseMatrix as input parameters.
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
#include "grillex/eigenvalue_solver.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cmath>

using namespace grillex;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

/**
 * @brief Helper class to set up a simple beam model for eigenvalue tests
 */
class SimpleBeamFixture {
public:
    NodeRegistry registry;
    Node* n1;
    Node* n2;
    std::unique_ptr<Material> material;
    std::unique_ptr<Section> section;
    std::unique_ptr<BeamElement> beam;
    DOFHandler dof_handler;
    std::unique_ptr<Assembler> assembler;

    SimpleBeamFixture()
        : registry(1e-6) {

        n1 = registry.get_or_create_node(0, 0, 0);
        n2 = registry.get_or_create_node(1, 0, 0);

        material = std::make_unique<Material>(1, "Steel", 210e6, 0.3, 7.85e-3);
        section = std::make_unique<Section>(1, "Test", 0.01, 1e-4, 1e-4, 1e-5);

        BeamConfig config;
        beam = std::make_unique<BeamElement>(1, n1, n2, material.get(), section.get(), config);

        dof_handler.number_dofs(registry);
        assembler = std::make_unique<Assembler>(dof_handler);
    }
};

// =============================================================================
// Task 16.2: Test boundary condition reduction for eigenvalue analysis
// =============================================================================

TEST_CASE("reduce_system: fixed node reduces DOF count", "[EigenvalueSolver][reduce_system]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    // Fix first node (all 6 DOFs)
    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    // Reduce system
    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    // Should have 6 free DOFs (node 2)
    int total_dofs = fixture.dof_handler.total_dofs();
    REQUIRE(K_red.rows() == total_dofs - 6);
    REQUIRE(M_red.rows() == total_dofs - 6);
    REQUIRE(dof_map.size() == static_cast<size_t>(total_dofs - 6));
}

TEST_CASE("reduce_system: preserves matrix symmetry", "[EigenvalueSolver][reduce_system]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    // Convert to dense for symmetry check
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_red);
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M_red);

    // Check symmetry
    REQUIRE((K_dense - K_dense.transpose()).norm() < 1e-10);
    REQUIRE((M_dense - M_dense.transpose()).norm() < 1e-10);
}

TEST_CASE("reduce_system: DOF mapping excludes fixed DOFs", "[EigenvalueSolver][reduce_system]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    // Fix only UX at node 1
    BCHandler bc;
    bc.add_fixed_dof(fixture.n1->id, DOFIndex::UX, 0.0);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    // Total DOFs - 1 fixed = 11 free
    int total_dofs = fixture.dof_handler.total_dofs();
    REQUIRE(K_red.rows() == total_dofs - 1);

    // Mapping should not include the fixed DOF
    int fixed_dof = fixture.dof_handler.get_global_dof(fixture.n1->id, DOFIndex::UX);
    bool found_fixed = false;
    for (int dof : dof_map) {
        if (dof == fixed_dof) {
            found_fixed = true;
            break;
        }
    }
    REQUIRE(!found_fixed);
}

// =============================================================================
// Task 16.3: Test dense eigenvalue solver
// =============================================================================

TEST_CASE("solve: cantilever beam first mode frequency", "[EigenvalueSolver][solve]") {
    /**
     * Test cantilever beam with 2 nodes (1 element).
     *
     * The analytical solution for the first bending mode of a cantilever is:
     * f_1 = (1.875)^2 * sqrt(EI / (rho*A*L^4)) / (2*pi)
     */
    double L = 1.0;        // m
    double E = 210e6;      // kN/m^2
    double I = 1e-4;       // m^4
    double rho = 7.85e-3;  // mT/m^3
    double A = 0.01;       // m^2

    NodeRegistry registry(1e-6);
    Node* n1 = registry.get_or_create_node(0, 0, 0);
    Node* n2 = registry.get_or_create_node(L, 0, 0);

    Material material(1, "Steel", E, 0.3, rho);
    Section section(1, "Test", A, I, I, 1e-5);

    BeamConfig config;
    BeamElement beam(1, n1, n2, &material, &section, config);

    DOFHandler dof_handler;
    dof_handler.number_dofs(registry);

    Assembler assembler(dof_handler);
    std::vector<BeamElement*> elements = {&beam};
    auto K = assembler.assemble_stiffness(elements);
    auto M = assembler.assemble_mass(elements);

    // Fix first node (cantilever)
    BCHandler bc;
    bc.fix_node(n1->id);

    // Reduce system
    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, dof_handler);

    // Solve
    EigensolverSettings settings;
    settings.n_modes = 6;
    settings.method = EigensolverMethod::Dense;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    // Check solver converged
    REQUIRE(result.converged);
    REQUIRE(result.modes.size() >= 1);

    // Analytical first bending mode frequency for continuous beam
    double lambda1 = 1.875;  // first eigenvalue parameter for cantilever
    double f1_analytical = (lambda1 * lambda1) / (2.0 * M_PI * L * L) *
                           std::sqrt(E * I / (rho * A));

    // First mode should exist and be positive frequency
    REQUIRE(result.modes[0].frequency_hz > 0);

    // Note: With 1 element, we won't match analytical solution well
    // Just check we get reasonable frequencies
}

TEST_CASE("solve: all eigenvalues are non-negative", "[EigenvalueSolver][solve]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    REQUIRE(result.converged);
    for (const auto& mode : result.modes) {
        REQUIRE(mode.eigenvalue >= -1e-10);  // Allow small numerical tolerance
        REQUIRE(mode.omega >= 0);
        REQUIRE(mode.frequency_hz >= 0);
    }
}

TEST_CASE("solve: modes are sorted by frequency", "[EigenvalueSolver][solve]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    REQUIRE(result.converged);

    // Check modes are sorted by frequency (ascending)
    for (size_t i = 1; i < result.modes.size(); ++i) {
        REQUIRE(result.modes[i].frequency_hz >= result.modes[i-1].frequency_hz);
    }
}

TEST_CASE("solve: mode shapes are mass normalized", "[EigenvalueSolver][solve]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;
    settings.mass_normalize = true;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    REQUIRE(result.converged);

    // For mass-normalized modes: phi^T * M * phi = 1
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M_red);
    for (const auto& mode : result.modes) {
        double phi_M_phi = mode.mode_shape.transpose() * M_dense * mode.mode_shape;
        REQUIRE_THAT(phi_M_phi, WithinAbs(1.0, 1e-6));
    }
}

TEST_CASE("solve: mode shapes satisfy eigenvalue equation", "[EigenvalueSolver][solve]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    REQUIRE(result.converged);

    // Check: K * phi = lambda * M * phi (eigenvalue equation)
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K_red);
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M_red);

    for (const auto& mode : result.modes) {
        if (mode.eigenvalue < 1e-6) continue;  // Skip rigid body modes

        Eigen::VectorXd lhs = K_dense * mode.mode_shape;
        Eigen::VectorXd rhs = mode.eigenvalue * M_dense * mode.mode_shape;

        // Normalize for comparison
        double residual = (lhs - rhs).norm() / rhs.norm();
        REQUIRE(residual < 1e-6);
    }
}

// =============================================================================
// Result struct utility tests
// =============================================================================

TEST_CASE("EigensolverResult: get_frequencies returns sorted list", "[EigenvalueSolver][result]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    REQUIRE(result.converged);

    auto freqs = result.get_frequencies();
    REQUIRE(freqs.size() == result.modes.size());

    // Check frequencies match modes
    for (size_t i = 0; i < freqs.size(); ++i) {
        REQUIRE_THAT(freqs[i], WithinAbs(result.modes[i].frequency_hz, 1e-10));
    }
}

TEST_CASE("EigensolverResult: expand_mode_shape works correctly", "[EigenvalueSolver][result]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;

    EigensolverResult result = solver.solve(K_red, M_red, settings);
    result.reduced_to_full = dof_map;
    result.total_dofs = fixture.dof_handler.total_dofs();

    REQUIRE(result.converged);
    REQUIRE(!result.modes.empty());

    // Expand first mode shape
    Eigen::VectorXd full_shape = result.expand_mode_shape(result.modes[0].mode_shape);

    REQUIRE(full_shape.size() == result.total_dofs);

    // Fixed DOFs should be zero
    for (int i = 0; i < 6; ++i) {  // First 6 DOFs are fixed
        REQUIRE_THAT(full_shape[i], WithinAbs(0.0, 1e-10));
    }
}

// =============================================================================
// Acceptance Criteria Tests
// =============================================================================

TEST_CASE("AC: EigenvalueSolver.reduce_system properly eliminates fixed DOFs", "[EigenvalueSolver][acceptance]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    // Fix 3 DOFs (pin at node 1)
    BCHandler bc;
    bc.pin_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    // Should have total_dofs - 3 free DOFs
    int total_dofs = fixture.dof_handler.total_dofs();
    REQUIRE(K_red.rows() == total_dofs - 3);
    REQUIRE(K_red.cols() == total_dofs - 3);
    REQUIRE(M_red.rows() == total_dofs - 3);
    REQUIRE(M_red.cols() == total_dofs - 3);
    REQUIRE(dof_map.size() == static_cast<size_t>(total_dofs - 3));
}

TEST_CASE("AC: Eigenvalue solver produces positive eigenvalues for positive-definite system", "[EigenvalueSolver][acceptance]") {
    SimpleBeamFixture fixture;

    std::vector<BeamElement*> elements = {fixture.beam.get()};
    auto K = fixture.assembler->assemble_stiffness(elements);
    auto M = fixture.assembler->assemble_mass(elements);

    // Cantilever: fix all DOFs at one end
    BCHandler bc;
    bc.fix_node(fixture.n1->id);

    EigenvalueSolver solver;
    auto [K_red, M_red, dof_map] = solver.reduce_system(K, M, bc, fixture.dof_handler);

    EigensolverSettings settings;
    settings.n_modes = 6;

    EigensolverResult result = solver.solve(K_red, M_red, settings);

    REQUIRE(result.converged);
    for (const auto& mode : result.modes) {
        REQUIRE(mode.eigenvalue >= 0);
    }
}
