#include "grillex/nonlinear_solver.hpp"
#include <cmath>
#include <sstream>
#include <algorithm>
#include <set>

namespace grillex {

NonlinearSolver::NonlinearSolver(const NonlinearSolverSettings& settings)
    : settings_(settings)
    , linear_solver_(settings.linear_method)
{}

NonlinearSolverResult NonlinearSolver::solve(
    const Eigen::SparseMatrix<double>& base_K,
    const Eigen::VectorXd& F,
    std::vector<SpringElement*>& springs,
    const DOFHandler& dof_handler,
    const NonlinearInitialState& initial_state)
{
    NonlinearSolverResult result;
    const int n = base_K.rows();

    // =========================================================================
    // Step 1: Check if we actually need nonlinear iteration
    // =========================================================================
    bool has_nonlinear = false;
    bool has_gaps = false;

    for (const auto* spring : springs) {
        if (spring->is_nonlinear()) {
            has_nonlinear = true;
        }
        if (spring->has_gap()) {
            has_gaps = true;
        }
        if (has_nonlinear && has_gaps) break;
    }

    // Fast path: purely linear springs with no gaps
    if (!has_nonlinear && !has_gaps) {
        // All springs are always active, assemble once and solve
        for (auto* spring : springs) {
            spring->is_active.fill(true);
            spring->deformation.fill(0.0);
        }

        Eigen::SparseMatrix<double> K_springs =
            assemble_spring_stiffness(springs, dof_handler, n);
        Eigen::SparseMatrix<double> K_total = base_K + K_springs;

        result.displacements = linear_solver_.solve(K_total, F);
        result.converged = !linear_solver_.is_singular();
        result.iterations = 1;
        result.message = result.converged
            ? "Linear solve (no nonlinear springs)"
            : "Linear solve failed: " + linear_solver_.get_error_message();

        // Update spring deformations for reporting
        if (result.converged) {
            for (auto* spring : springs) {
                spring->update_state(result.displacements, dof_handler);
                result.spring_states.emplace_back(spring->id, spring->is_active);
                result.spring_forces.emplace_back(spring->id, spring->compute_forces());
            }
        }
        return result;
    }

    // =========================================================================
    // Step 2: Initialize for iterative solve
    // =========================================================================

    Eigen::VectorXd u;
    Eigen::VectorXd u_prev;

    if (initial_state.has_initial_state()) {
        // Starting from provided initial state (e.g., static solution)
        // This is essential for static→dynamic sequencing where the static
        // (gravity) solution establishes the baseline contact pattern

        u = initial_state.displacement;
        u_prev = u;

        // Apply initial spring states from previous solve
        if (!initial_state.spring_states.empty()) {
            // Create a map for quick lookup
            std::map<int, std::array<bool, 6>> state_map;
            for (const auto& [spring_id, states] : initial_state.spring_states) {
                state_map[spring_id] = states;
            }

            // Apply states to springs
            for (auto* spring : springs) {
                auto it = state_map.find(spring->id);
                if (it != state_map.end()) {
                    spring->is_active = it->second;
                } else {
                    // Spring not in initial state - start as active
                    spring->is_active.fill(true);
                }
                // Update deformations from displacement
                spring->update_state(u, dof_handler);
            }
        } else {
            // No spring states provided - update from initial displacement
            for (auto* spring : springs) {
                spring->update_state(u, dof_handler);
            }
        }
    } else {
        // No initial state - start from zero with all springs active
        // (optimistic initial guess)
        u = Eigen::VectorXd::Zero(n);
        u_prev = u;

        for (auto* spring : springs) {
            spring->is_active.fill(true);
            spring->deformation.fill(0.0);
        }
    }

    // State history for oscillation detection
    std::vector<std::vector<bool>> state_history;

    // =========================================================================
    // Step 3: Iteration loop
    // =========================================================================
    for (int iter = 0; iter < settings_.max_iterations; ++iter) {
        result.iterations = iter + 1;

        // ---------------------------------------------------------------------
        // 3a. Assemble stiffness matrix with current active springs
        // ---------------------------------------------------------------------
        Eigen::SparseMatrix<double> K_springs =
            assemble_spring_stiffness(springs, dof_handler, n);
        Eigen::SparseMatrix<double> K_total = base_K + K_springs;

        // ---------------------------------------------------------------------
        // 3b. Compute gap forces for active gap springs
        // ---------------------------------------------------------------------
        Eigen::VectorXd F_gap = assemble_gap_forces(springs, dof_handler, n);
        Eigen::VectorXd F_total = F + F_gap;

        // ---------------------------------------------------------------------
        // 3c. Solve linear system: K * u = F + F_gap
        // ---------------------------------------------------------------------
        Eigen::VectorXd u_new = linear_solver_.solve(K_total, F_total);

        if (linear_solver_.is_singular()) {
            result.converged = false;
            result.displacements = u;
            std::ostringstream oss;
            oss << "Singular system at iteration " << (iter + 1)
                << ": " << linear_solver_.get_error_message();
            result.message = oss.str();

            // Store current spring states for diagnostics
            for (auto* spring : springs) {
                result.spring_states.emplace_back(spring->id, spring->is_active);
            }
            return result;
        }

        // ---------------------------------------------------------------------
        // 3c2. Apply line search damping if enabled
        // ---------------------------------------------------------------------
        if (settings_.enable_line_search && iter > 0) {
            // Use previous iteration's state changes to determine step size
            int prev_changes = result.state_changes_per_iteration.back();
            if (prev_changes > 0) {
                // Reduce step size proportional to number of state changes
                // step_factor = 1 / (1 + factor * changes)
                double step_factor = 1.0 / (1.0 + settings_.line_search_factor * prev_changes);
                u_new = u + step_factor * (u_new - u);
            }
        }

        // ---------------------------------------------------------------------
        // 3d. Update spring states based on new displacements
        // ---------------------------------------------------------------------
        int num_state_changes = 0;
        for (auto* spring : springs) {
            // Use hysteresis band if configured
            if (settings_.hysteresis_band > 0.0) {
                spring->update_state_with_hysteresis(u_new, dof_handler, settings_.hysteresis_band);
            } else {
                spring->update_state(u_new, dof_handler);
            }
            if (spring->state_changed()) {
                num_state_changes++;
            }
        }

        result.state_changes_per_iteration.push_back(num_state_changes);

        // Progress callback
        if (settings_.progress_callback) {
            double u_norm = u_new.norm();
            settings_.progress_callback(iter + 1, num_state_changes, u_norm);
        }

        // ---------------------------------------------------------------------
        // 3e. Check convergence: no state changes
        // ---------------------------------------------------------------------
        if (num_state_changes == 0) {
            // Also check displacement convergence as secondary criterion
            double du_norm = (u_new - u).norm();
            double u_norm = u_new.norm();
            bool displacement_converged =
                (u_norm < 1e-15) || (du_norm / u_norm < settings_.displacement_tolerance);

            if (displacement_converged) {
                result.displacements = u_new;
                result.converged = true;

                std::ostringstream oss;
                oss << "Converged after " << (iter + 1) << " iteration"
                    << ((iter + 1) > 1 ? "s" : "");
                result.message = oss.str();

                // Store final spring states and forces
                for (auto* spring : springs) {
                    result.spring_states.emplace_back(spring->id, spring->is_active);
                    result.spring_forces.emplace_back(spring->id, spring->compute_forces());
                }

                // Warning if all nonlinear springs are inactive
                if (!settings_.allow_all_springs_inactive) {
                    bool any_nonlinear_active = false;
                    for (const auto* spring : springs) {
                        if (!spring->is_nonlinear()) continue;
                        for (int d = 0; d < 6; ++d) {
                            if (spring->behavior[d] != SpringBehavior::Linear &&
                                spring->is_active[d]) {
                                any_nonlinear_active = true;
                                break;
                            }
                        }
                        if (any_nonlinear_active) break;
                    }
                    if (!any_nonlinear_active && has_nonlinear) {
                        result.message += " (WARNING: all nonlinear springs inactive)";
                    }
                }

                return result;
            }
        }

        // ---------------------------------------------------------------------
        // 3f. Oscillation detection and damping
        // ---------------------------------------------------------------------
        if (settings_.enable_oscillation_damping) {
            state_history.push_back(encode_spring_states(springs));

            // Keep history bounded
            if (static_cast<int>(state_history.size()) > settings_.oscillation_history_size) {
                state_history.erase(state_history.begin());
            }

            if (detect_oscillation(state_history)) {
                // Mark oscillating springs (those whose state changed in recent iterations)
                if (settings_.use_partial_stiffness && state_history.size() >= 2) {
                    const auto& prev_state = state_history[state_history.size() - 2];
                    const auto& curr_state = state_history.back();

                    size_t idx = 0;
                    for (auto* spring : springs) {
                        bool spring_oscillating = false;
                        for (int d = 0; d < 6; ++d) {
                            if (idx + d < prev_state.size() && idx + d < curr_state.size()) {
                                if (prev_state[idx + d] != curr_state[idx + d]) {
                                    spring_oscillating = true;
                                    break;
                                }
                            }
                        }
                        spring->set_oscillating(spring_oscillating);
                        idx += 6;
                    }
                }

                // Apply damping: blend with previous solution
                double alpha = settings_.oscillation_damping_factor;
                u_new = alpha * u_new + (1.0 - alpha) * u_prev;

                // Re-update spring states with damped displacement
                for (auto* spring : springs) {
                    if (settings_.hysteresis_band > 0.0) {
                        spring->update_state_with_hysteresis(u_new, dof_handler, settings_.hysteresis_band);
                    } else {
                        spring->update_state(u_new, dof_handler);
                    }
                }
            } else {
                // Clear oscillating flag when not oscillating
                for (auto* spring : springs) {
                    spring->set_oscillating(false);
                }
            }
        }

        u_prev = u;
        u = u_new;
    }

    // =========================================================================
    // Step 4: Failed to converge
    // =========================================================================
    result.displacements = u;
    result.converged = false;

    std::ostringstream oss;
    oss << "Failed to converge after " << settings_.max_iterations << " iterations";

    // Analyze why: still oscillating?
    if (!result.state_changes_per_iteration.empty()) {
        int last_changes = result.state_changes_per_iteration.back();
        if (last_changes > 0) {
            oss << " (still " << last_changes << " spring state changes)";
        }
    }

    result.message = oss.str();

    // Store final states anyway for diagnostics
    for (auto* spring : springs) {
        result.spring_states.emplace_back(spring->id, spring->is_active);
        result.spring_forces.emplace_back(spring->id, spring->compute_forces());
    }

    return result;
}

Eigen::SparseMatrix<double> NonlinearSolver::assemble_spring_stiffness(
    const std::vector<SpringElement*>& springs,
    const DOFHandler& dof_handler,
    int total_dofs) const
{
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(springs.size() * 24);  // Up to 12*2 entries per spring

    for (const auto* spring : springs) {
        // Get element stiffness (respects is_active flags)
        Eigen::Matrix<double, 12, 12> K_elem = spring->current_stiffness_matrix();

        // Apply partial stiffness factor for oscillating springs
        // This helps stabilize convergence for marginally-active springs
        double stiffness_factor = 1.0;
        if (settings_.use_partial_stiffness && spring->is_oscillating()) {
            stiffness_factor = 0.5;  // Use half stiffness for oscillating springs
        }

        // Build location array
        std::array<int, 12> loc;
        for (int d = 0; d < 6; ++d) {
            loc[d] = dof_handler.get_global_dof(spring->node_i->id, d);
            loc[d + 6] = dof_handler.get_global_dof(spring->node_j->id, d);
        }

        // Scatter to global
        for (int i = 0; i < 12; ++i) {
            if (loc[i] < 0) continue;
            for (int j = 0; j < 12; ++j) {
                if (loc[j] < 0) continue;
                double val = K_elem(i, j) * stiffness_factor;
                if (std::abs(val) > 1e-20) {
                    triplets.emplace_back(loc[i], loc[j], val);
                }
            }
        }
    }

    Eigen::SparseMatrix<double> K(total_dofs, total_dofs);
    K.setFromTriplets(triplets.begin(), triplets.end());
    return K;
}

Eigen::VectorXd NonlinearSolver::assemble_gap_forces(
    const std::vector<SpringElement*>& springs,
    const DOFHandler& dof_handler,
    int total_dofs) const
{
    Eigen::VectorXd F_gap = Eigen::VectorXd::Zero(total_dofs);

    for (const auto* spring : springs) {
        if (!spring->has_gap()) continue;

        // Get element gap forces
        Eigen::Matrix<double, 12, 1> f_elem = spring->compute_gap_forces();

        // Build location array
        std::array<int, 12> loc;
        for (int d = 0; d < 6; ++d) {
            loc[d] = dof_handler.get_global_dof(spring->node_i->id, d);
            loc[d + 6] = dof_handler.get_global_dof(spring->node_j->id, d);
        }

        // Scatter to global
        for (int i = 0; i < 12; ++i) {
            if (loc[i] >= 0) {
                F_gap(loc[i]) += f_elem(i);
            }
        }
    }

    return F_gap;
}

std::vector<bool> NonlinearSolver::encode_spring_states(
    const std::vector<SpringElement*>& springs) const
{
    std::vector<bool> state;
    state.reserve(springs.size() * 6);
    for (const auto* spring : springs) {
        for (int d = 0; d < 6; ++d) {
            state.push_back(spring->is_active[d]);
        }
    }
    return state;
}

bool NonlinearSolver::detect_oscillation(
    const std::vector<std::vector<bool>>& history) const
{
    if (history.size() < 3) return false;

    // Check if current state matches any previous state
    const auto& current = history.back();

    for (size_t i = 0; i < history.size() - 1; ++i) {
        if (history[i] == current) {
            // We've seen this state before - oscillation!
            return true;
        }
    }

    // Also check for 2-cycle: A -> B -> A -> B
    if (history.size() >= 4) {
        size_t n = history.size();
        if (history[n-1] == history[n-3] && history[n-2] == history[n-4]) {
            return true;
        }
    }

    return false;
}

} // namespace grillex
