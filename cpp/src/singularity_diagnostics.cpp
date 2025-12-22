#include "grillex/singularity_diagnostics.hpp"
#include "grillex/dof_handler.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace grillex {

std::string SingularityDiagnostics::to_string() const {
    std::ostringstream oss;

    if (!is_singular) {
        oss << "System is well-constrained (no rigid body modes detected).";
        return oss.str();
    }

    oss << summary_message << "\n\n";
    oss << detailed_message;

    if (!suggested_fixes.empty()) {
        oss << "\n\nSuggested fixes:\n";
        for (size_t i = 0; i < suggested_fixes.size(); ++i) {
            oss << "  " << (i + 1) << ". " << suggested_fixes[i] << "\n";
        }
    }

    return oss.str();
}

std::string SingularityDiagnostics::to_json() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    oss << "{\n";
    oss << "  \"is_singular\": " << (is_singular ? "true" : "false") << ",\n";
    oss << "  \"n_rigid_body_modes\": " << n_rigid_body_modes << ",\n";

    // Nodes needing constraints
    oss << "  \"nodes_needing_constraints\": [";
    for (size_t i = 0; i < nodes_needing_constraints.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << nodes_needing_constraints[i];
    }
    oss << "],\n";

    // DOFs to constrain
    oss << "  \"dofs_to_constrain\": [";
    for (size_t i = 0; i < dofs_to_constrain.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << "{\"node_id\": " << dofs_to_constrain[i].first
            << ", \"local_dof\": " << dofs_to_constrain[i].second << "}";
    }
    oss << "],\n";

    // Rigid body modes
    oss << "  \"rigid_body_modes\": [\n";
    for (size_t i = 0; i < rigid_body_modes.size(); ++i) {
        const auto& mode = rigid_body_modes[i];
        oss << "    {\n";
        oss << "      \"mode_number\": " << mode.mode_number << ",\n";
        oss << "      \"eigenvalue\": " << mode.eigenvalue << ",\n";
        oss << "      \"mode_type\": \"" << rigid_body_mode_type_to_string(mode.mode_type) << "\",\n";
        oss << "      \"description\": \"" << mode.description << "\",\n";
        oss << "      \"involved_nodes\": [";
        for (size_t j = 0; j < mode.involved_nodes.size(); ++j) {
            if (j > 0) oss << ", ";
            oss << mode.involved_nodes[j];
        }
        oss << "]\n";
        oss << "    }";
        if (i < rigid_body_modes.size() - 1) oss << ",";
        oss << "\n";
    }
    oss << "  ]\n";
    oss << "}";

    return oss.str();
}

SingularityDiagnostics SingularityAnalyzer::analyze(
    const Eigen::SparseMatrix<double>& K,
    const DOFHandler& dof_handler,
    const Settings& settings) const
{
    // Create identity mass matrix for standard eigenvalue problem
    Eigen::SparseMatrix<double> M(K.rows(), K.cols());
    M.setIdentity();
    return analyze(K, M, dof_handler, settings);
}

SingularityDiagnostics SingularityAnalyzer::analyze(
    const Eigen::SparseMatrix<double>& K,
    const Eigen::SparseMatrix<double>& M,
    const DOFHandler& dof_handler,
    const Settings& settings) const
{
    SingularityDiagnostics result;

    int n = static_cast<int>(K.rows());
    if (n == 0) {
        result.is_singular = true;
        result.summary_message = "Empty system (no DOFs).";
        result.detailed_message = "The stiffness matrix has zero size.";
        return result;
    }

    // Convert to dense for eigenvalue computation
    // (For large systems, could use iterative methods, but for diagnostics
    // we typically work with smaller reduced systems)
    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
    Eigen::MatrixXd M_dense = Eigen::MatrixXd(M);

    // Ensure symmetry
    K_dense = (K_dense + K_dense.transpose()) / 2.0;
    M_dense = (M_dense + M_dense.transpose()) / 2.0;

    // Solve generalized eigenvalue problem
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver;

    // Check if M is usable (positive semi-definite with some positive values)
    double M_trace = M_dense.trace();
    bool use_generalized = settings.use_mass_matrix && M_trace > 1e-14;

    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenvectors;

    if (use_generalized) {
        try {
            solver.compute(K_dense, M_dense);
            if (solver.info() == Eigen::Success) {
                eigenvalues = solver.eigenvalues();
                eigenvectors = solver.eigenvectors();
            } else {
                // Fall back to standard eigenvalue problem
                use_generalized = false;
            }
        } catch (...) {
            use_generalized = false;
        }
    }

    if (!use_generalized) {
        // Use standard eigenvalue problem K*φ = λ*φ
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> std_solver(K_dense);
        if (std_solver.info() != Eigen::Success) {
            result.is_singular = true;
            result.summary_message = "Eigenvalue computation failed.";
            result.detailed_message = "Unable to compute eigenvalues of stiffness matrix.";
            return result;
        }
        eigenvalues = std_solver.eigenvalues();
        eigenvectors = std_solver.eigenvectors();
    }

    // Count and analyze rigid body modes
    int n_modes_to_check = std::min(settings.n_modes_to_check, n);

    for (int i = 0; i < n_modes_to_check; ++i) {
        double lambda = eigenvalues(i);

        // Check if this is a rigid body mode
        if (std::abs(lambda) < settings.eigenvalue_threshold) {
            result.n_rigid_body_modes++;

            RigidBodyModeInfo mode_info;
            mode_info.mode_number = i;
            mode_info.eigenvalue = lambda;

            // Get the eigenvector for this mode
            Eigen::VectorXd phi = eigenvectors.col(i);

            // Identify mode type and participating DOFs
            mode_info.mode_type = identify_mode_type(phi, dof_handler);

            auto participating_dofs = extract_dof_participation(
                phi, dof_handler, settings.participation_threshold);

            // Collect involved nodes and DOFs
            std::set<int> node_set;
            for (const auto& dof_part : participating_dofs) {
                mode_info.involved_global_dofs.push_back(dof_part.global_dof);
                if (dof_part.node_id >= 0) {
                    node_set.insert(dof_part.node_id);
                }
            }
            mode_info.involved_nodes.assign(node_set.begin(), node_set.end());

            // Generate description and fix suggestion
            mode_info.description = "Unconstrained " +
                                    rigid_body_mode_type_to_string(mode_info.mode_type);
            mode_info.suggested_fix = generate_fix_suggestion(
                mode_info.mode_type, participating_dofs);

            result.rigid_body_modes.push_back(mode_info);

            // Add to unconstrained DOFs (avoid duplicates)
            for (const auto& dof_part : participating_dofs) {
                bool already_added = false;
                for (const auto& existing : result.unconstrained_dofs) {
                    if (existing.global_dof == dof_part.global_dof) {
                        already_added = true;
                        break;
                    }
                }
                if (!already_added &&
                    result.unconstrained_dofs.size() < static_cast<size_t>(settings.max_dofs_to_report)) {
                    result.unconstrained_dofs.push_back(dof_part);
                }
            }
        }
    }

    result.is_singular = (result.n_rigid_body_modes > 0);

    // Build nodes needing constraints and DOFs to constrain
    std::set<int> all_nodes;
    for (const auto& mode : result.rigid_body_modes) {
        for (int node_id : mode.involved_nodes) {
            all_nodes.insert(node_id);
        }
        if (!mode.suggested_fix.empty()) {
            result.suggested_fixes.push_back(mode.suggested_fix);
        }
    }
    result.nodes_needing_constraints.assign(all_nodes.begin(), all_nodes.end());

    // Determine specific DOFs to constrain
    for (const auto& dof_part : result.unconstrained_dofs) {
        if (dof_part.node_id >= 0 && dof_part.local_dof >= 0) {
            result.dofs_to_constrain.emplace_back(dof_part.node_id, dof_part.local_dof);
        }
    }

    // Build diagnostic messages
    build_diagnostic_messages(result);

    return result;
}

bool SingularityAnalyzer::is_singular(const Eigen::SparseMatrix<double>& K) const {
    return count_rigid_body_modes(K) > 0;
}

int SingularityAnalyzer::count_rigid_body_modes(
    const Eigen::SparseMatrix<double>& K,
    double threshold) const
{
    int n = static_cast<int>(K.rows());
    if (n == 0) return 0;

    // For small systems, compute all eigenvalues
    // For large systems, just compute a few lowest ones
    int n_to_compute = std::min(10, n);

    Eigen::MatrixXd K_dense = Eigen::MatrixXd(K);
    K_dense = (K_dense + K_dense.transpose()) / 2.0;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(K_dense);
    if (solver.info() != Eigen::Success) {
        return -1; // Error
    }

    Eigen::VectorXd eigenvalues = solver.eigenvalues();

    int count = 0;
    for (int i = 0; i < n_to_compute && i < n; ++i) {
        if (std::abs(eigenvalues(i)) < threshold) {
            count++;
        }
    }

    return count;
}

RigidBodyModeType SingularityAnalyzer::identify_mode_type(
    const Eigen::VectorXd& eigenvector,
    const DOFHandler& dof_handler) const
{
    // Count participation by DOF type
    double sum_ux = 0.0, sum_uy = 0.0, sum_uz = 0.0;
    double sum_rx = 0.0, sum_ry = 0.0, sum_rz = 0.0;
    double sum_warp = 0.0;

    int n = static_cast<int>(eigenvector.size());
    double total_participation = 0.0;

    for (int global_dof = 0; global_dof < n; ++global_dof) {
        double participation = eigenvector(global_dof) * eigenvector(global_dof);
        total_participation += participation;

        // Determine local DOF type
        // Standard assumption: 6 DOFs per node, DOFs ordered by node
        int local_dof = global_dof % 7; // Account for possible warping DOF

        // Check if this might be a warping DOF
        bool is_warping_dof = dof_handler.is_warping_dof(global_dof);

        if (is_warping_dof || local_dof == 6) {
            sum_warp += participation;
        } else {
            // Map to standard 6-DOF
            local_dof = global_dof % 6;
            switch (local_dof) {
                case 0: sum_ux += participation; break;
                case 1: sum_uy += participation; break;
                case 2: sum_uz += participation; break;
                case 3: sum_rx += participation; break;
                case 4: sum_ry += participation; break;
                case 5: sum_rz += participation; break;
            }
        }
    }

    if (total_participation < 1e-14) {
        return RigidBodyModeType::Mixed;
    }

    // Normalize participations
    sum_ux /= total_participation;
    sum_uy /= total_participation;
    sum_uz /= total_participation;
    sum_rx /= total_participation;
    sum_ry /= total_participation;
    sum_rz /= total_participation;
    sum_warp /= total_participation;

    // Find dominant DOF type
    double threshold = 0.5; // 50% threshold for dominant type

    if (sum_ux > threshold) return RigidBodyModeType::TranslationX;
    if (sum_uy > threshold) return RigidBodyModeType::TranslationY;
    if (sum_uz > threshold) return RigidBodyModeType::TranslationZ;
    if (sum_rx > threshold) return RigidBodyModeType::RotationX;
    if (sum_ry > threshold) return RigidBodyModeType::RotationY;
    if (sum_rz > threshold) return RigidBodyModeType::RotationZ;
    if (sum_warp > threshold) return RigidBodyModeType::Warping;

    return RigidBodyModeType::Mixed;
}

std::vector<DOFParticipation> SingularityAnalyzer::extract_dof_participation(
    const Eigen::VectorXd& eigenvector,
    const DOFHandler& dof_handler,
    double threshold) const
{
    std::vector<DOFParticipation> result;

    int n = static_cast<int>(eigenvector.size());

    // Compute total participation for normalization
    double total = eigenvector.squaredNorm();
    if (total < 1e-14) {
        return result;
    }

    // Create list of all participations
    std::vector<std::pair<int, double>> all_participations;
    for (int global_dof = 0; global_dof < n; ++global_dof) {
        double participation = eigenvector(global_dof) * eigenvector(global_dof) / total;
        if (participation >= threshold) {
            all_participations.emplace_back(global_dof, participation);
        }
    }

    // Sort by participation (descending)
    std::sort(all_participations.begin(), all_participations.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    // Extract DOF information
    for (const auto& [global_dof, participation] : all_participations) {
        DOFParticipation dof_part;
        dof_part.global_dof = global_dof;
        dof_part.participation = participation;

        // Try to map to node and local DOF
        // This depends on how DOFHandler structures the global DOFs
        int node_id = dof_handler.get_node_from_global_dof(global_dof);
        int local_dof = dof_handler.get_local_dof_from_global_dof(global_dof);

        dof_part.node_id = node_id;
        dof_part.local_dof = local_dof;

        // Check if this is a warping DOF
        if (dof_handler.is_warping_dof(global_dof)) {
            dof_part.local_dof = 6; // Warping
            dof_part.element_id = dof_handler.get_element_from_warping_dof(global_dof);
        }

        result.push_back(dof_part);
    }

    return result;
}

std::string SingularityAnalyzer::generate_fix_suggestion(
    RigidBodyModeType mode_type,
    const std::vector<DOFParticipation>& participating_dofs) const
{
    std::ostringstream oss;

    // Get the node with highest participation
    int primary_node = -1;
    if (!participating_dofs.empty()) {
        primary_node = participating_dofs[0].node_id;
    }

    switch (mode_type) {
        case RigidBodyModeType::TranslationX:
            oss << "Add X-direction support (fix UX) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "any node";
            }
            break;

        case RigidBodyModeType::TranslationY:
            oss << "Add Y-direction support (fix UY) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "any node";
            }
            break;

        case RigidBodyModeType::TranslationZ:
            oss << "Add Z-direction support (fix UZ) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "any node";
            }
            break;

        case RigidBodyModeType::RotationX:
            oss << "Add X-rotation restraint (fix RX) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "any node";
            }
            oss << ", or add supports at multiple nodes to prevent rotation about X axis";
            break;

        case RigidBodyModeType::RotationY:
            oss << "Add Y-rotation restraint (fix RY) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "any node";
            }
            oss << ", or add supports at multiple nodes to prevent rotation about Y axis";
            break;

        case RigidBodyModeType::RotationZ:
            oss << "Add Z-rotation restraint (fix RZ) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "any node";
            }
            oss << ", or add supports at multiple nodes to prevent rotation about Z axis";
            break;

        case RigidBodyModeType::Warping:
            oss << "Add warping restraint (fix WARP) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "a beam end with warping DOF enabled";
            }
            break;

        case RigidBodyModeType::Mixed:
            oss << "Add supports to constrain the structure. Consider adding a fixed support (all DOFs) at ";
            if (primary_node >= 0) {
                oss << "node " << primary_node;
            } else {
                oss << "one corner node";
            }
            break;
    }

    return oss.str();
}

void SingularityAnalyzer::build_diagnostic_messages(SingularityDiagnostics& result) const {
    std::ostringstream summary;
    std::ostringstream detail;

    if (!result.is_singular) {
        result.summary_message = "System is well-constrained.";
        result.detailed_message = "No rigid body modes detected. The stiffness matrix is non-singular.";
        return;
    }

    // Summary message
    summary << "SINGULAR SYSTEM: " << result.n_rigid_body_modes << " rigid body mode(s) detected.";

    if (!result.nodes_needing_constraints.empty()) {
        summary << " Nodes needing constraints: ";
        for (size_t i = 0; i < result.nodes_needing_constraints.size() && i < 5; ++i) {
            if (i > 0) summary << ", ";
            summary << result.nodes_needing_constraints[i];
        }
        if (result.nodes_needing_constraints.size() > 5) {
            summary << " (and " << (result.nodes_needing_constraints.size() - 5) << " more)";
        }
    }

    result.summary_message = summary.str();

    // Detailed message
    detail << "The stiffness matrix is singular, meaning the structure has unconstrained\n";
    detail << "degrees of freedom that allow rigid body motion.\n\n";

    detail << "Detected rigid body modes:\n";
    for (const auto& mode : result.rigid_body_modes) {
        detail << "  Mode " << (mode.mode_number + 1) << ": " << mode.description;
        detail << " (eigenvalue = " << std::scientific << std::setprecision(2)
               << mode.eigenvalue << ")\n";

        if (!mode.involved_nodes.empty()) {
            detail << "    Involved nodes: ";
            for (size_t i = 0; i < mode.involved_nodes.size() && i < 10; ++i) {
                if (i > 0) detail << ", ";
                detail << mode.involved_nodes[i];
            }
            if (mode.involved_nodes.size() > 10) {
                detail << " ...";
            }
            detail << "\n";
        }
    }

    detail << "\nUnconstrained DOFs with highest participation:\n";
    for (size_t i = 0; i < result.unconstrained_dofs.size() && i < 10; ++i) {
        const auto& dof = result.unconstrained_dofs[i];
        detail << "  Global DOF " << dof.global_dof;
        if (dof.node_id >= 0) {
            detail << " (Node " << dof.node_id << ", " << local_dof_name(dof.local_dof) << ")";
        }
        detail << ": " << std::fixed << std::setprecision(1)
               << (dof.participation * 100.0) << "% participation\n";
    }

    result.detailed_message = detail.str();
}

std::string SingularityAnalyzer::local_dof_name(int local_dof) {
    switch (local_dof) {
        case 0: return "UX";
        case 1: return "UY";
        case 2: return "UZ";
        case 3: return "RX";
        case 4: return "RY";
        case 5: return "RZ";
        case 6: return "WARP";
        default: return "DOF" + std::to_string(local_dof);
    }
}

} // namespace grillex
