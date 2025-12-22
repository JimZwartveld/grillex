/**
 * @file singularity_diagnostics.hpp
 * @brief Singularity detection and diagnostics using eigenvalue analysis.
 *
 * This module provides advanced singularity diagnostics by analyzing the
 * eigenvalue structure of the stiffness matrix. It can identify:
 * - Rigid body modes (free-floating structures)
 * - Partially constrained systems (missing supports)
 * - Specific unconstrained DOFs and nodes
 *
 * The implementation uses eigenvalue analysis to detect near-zero eigenvalues
 * and analyzes the corresponding eigenvectors to identify which DOFs are
 * participating in the rigid body motion.
 */

#ifndef GRILLEX_SINGULARITY_DIAGNOSTICS_HPP
#define GRILLEX_SINGULARITY_DIAGNOSTICS_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <string>
#include <map>

namespace grillex {

// Forward declarations
class DOFHandler;

/**
 * @brief Type of rigid body mode detected
 */
enum class RigidBodyModeType {
    TranslationX,      ///< Translation in X direction
    TranslationY,      ///< Translation in Y direction
    TranslationZ,      ///< Translation in Z direction
    RotationX,         ///< Rotation about X axis
    RotationY,         ///< Rotation about Y axis
    RotationZ,         ///< Rotation about Z axis
    Warping,           ///< Warping mode
    Mixed              ///< Combined mode (multiple DOF types)
};

/**
 * @brief Convert RigidBodyModeType to string
 */
inline std::string rigid_body_mode_type_to_string(RigidBodyModeType type) {
    switch (type) {
        case RigidBodyModeType::TranslationX: return "X translation";
        case RigidBodyModeType::TranslationY: return "Y translation";
        case RigidBodyModeType::TranslationZ: return "Z translation";
        case RigidBodyModeType::RotationX: return "X rotation";
        case RigidBodyModeType::RotationY: return "Y rotation";
        case RigidBodyModeType::RotationZ: return "Z rotation";
        case RigidBodyModeType::Warping: return "warping";
        case RigidBodyModeType::Mixed: return "mixed mode";
        default: return "unknown";
    }
}

/**
 * @brief Information about a detected rigid body mode
 */
struct RigidBodyModeInfo {
    /// Mode number (0-indexed)
    int mode_number = 0;

    /// Eigenvalue (should be near zero for rigid body modes)
    double eigenvalue = 0.0;

    /// Type of rigid body mode detected
    RigidBodyModeType mode_type = RigidBodyModeType::Mixed;

    /// Node IDs with significant participation in this mode
    std::vector<int> involved_nodes;

    /// Global DOF indices with significant participation
    std::vector<int> involved_global_dofs;

    /// Description of the mode
    std::string description;

    /// Suggested fix for this mode
    std::string suggested_fix;
};

/**
 * @brief DOF participation information
 */
struct DOFParticipation {
    /// Node ID
    int node_id = -1;

    /// Local DOF index (0=UX, 1=UY, 2=UZ, 3=RX, 4=RY, 5=RZ, 6=WARP)
    int local_dof = -1;

    /// Global DOF index
    int global_dof = -1;

    /// Participation magnitude (0 to 1, normalized)
    double participation = 0.0;

    /// Element ID for warping DOFs (-1 for node-level DOFs)
    int element_id = -1;
};

/**
 * @brief Result of singularity diagnostics
 */
struct SingularityDiagnostics {
    /// Whether the system is singular (has rigid body modes)
    bool is_singular = false;

    /// Number of rigid body modes detected
    int n_rigid_body_modes = 0;

    /// Detailed information about each rigid body mode
    std::vector<RigidBodyModeInfo> rigid_body_modes;

    /// DOFs with highest participation in rigid body modes
    std::vector<DOFParticipation> unconstrained_dofs;

    /// Summary message for display
    std::string summary_message;

    /// Detailed diagnostic message
    std::string detailed_message;

    /// Suggested fixes (one per rigid body mode)
    std::vector<std::string> suggested_fixes;

    /// Nodes that need additional constraints
    std::vector<int> nodes_needing_constraints;

    /// DOFs that need to be constrained (as pairs of {node_id, local_dof})
    std::vector<std::pair<int, int>> dofs_to_constrain;

    /**
     * @brief Get formatted string representation
     */
    std::string to_string() const;

    /**
     * @brief Get machine-readable summary (JSON-like format)
     */
    std::string to_json() const;
};

/**
 * @brief Settings for singularity analysis
 */
struct SingularityAnalyzerSettings {
    /// Eigenvalue threshold for rigid body mode detection
    /// Eigenvalues with |λ| < threshold are considered rigid body modes
    double eigenvalue_threshold = 1e-8;

    /// Number of lowest eigenvalues to check
    int n_modes_to_check = 10;

    /// Minimum participation to include a DOF in the report
    /// DOFs with participation < threshold are excluded
    double participation_threshold = 0.01;

    /// Maximum number of unconstrained DOFs to report
    int max_dofs_to_report = 20;

    /// Whether to use mass matrix (if available) for better mode identification
    bool use_mass_matrix = true;
};

/**
 * @brief Singularity analyzer using eigenvalue decomposition
 *
 * This class analyzes structural systems for singularity by computing
 * eigenvalues and identifying rigid body modes. Unlike simple diagonal
 * checks, this approach can identify:
 *
 * 1. Which specific DOFs are unconstrained
 * 2. The type of rigid body motion (translation, rotation, warping)
 * 3. Suggested fixes based on mode analysis
 *
 * Usage:
 *   SingularityAnalyzer analyzer;
 *   SingularityDiagnostics result = analyzer.analyze(K, M, dof_handler);
 *
 *   if (result.is_singular) {
 *       std::cout << result.detailed_message << std::endl;
 *       for (const auto& fix : result.suggested_fixes) {
 *           std::cout << "Suggestion: " << fix << std::endl;
 *       }
 *   }
 */
class SingularityAnalyzer {
public:
    /// Alias for backwards compatibility
    using Settings = SingularityAnalyzerSettings;

    /**
     * @brief Default constructor
     */
    SingularityAnalyzer() = default;

    /**
     * @brief Analyze stiffness matrix for singularity
     * @param K Stiffness matrix (sparse, symmetric)
     * @param dof_handler DOF handler for node/DOF mapping
     * @param settings Analysis settings
     * @return SingularityDiagnostics result
     *
     * This method computes eigenvalues of K and identifies rigid body modes.
     * Without a mass matrix, mode shapes represent displacement patterns.
     */
    SingularityDiagnostics analyze(
        const Eigen::SparseMatrix<double>& K,
        const DOFHandler& dof_handler,
        const Settings& settings = Settings{}) const;

    /**
     * @brief Analyze K and M matrices for singularity (preferred method)
     * @param K Stiffness matrix (sparse, symmetric)
     * @param M Mass matrix (sparse, symmetric positive semi-definite)
     * @param dof_handler DOF handler for node/DOF mapping
     * @param settings Analysis settings
     * @return SingularityDiagnostics result
     *
     * This method solves the generalized eigenvalue problem K*φ = λ*M*φ
     * which provides physically meaningful mode shapes for identification.
     */
    SingularityDiagnostics analyze(
        const Eigen::SparseMatrix<double>& K,
        const Eigen::SparseMatrix<double>& M,
        const DOFHandler& dof_handler,
        const Settings& settings = Settings{}) const;

    /**
     * @brief Quick check for singularity (without detailed diagnostics)
     * @param K Stiffness matrix
     * @return true if system appears singular
     *
     * Performs a fast eigenvalue check without detailed mode analysis.
     * Use analyze() for full diagnostics.
     */
    bool is_singular(const Eigen::SparseMatrix<double>& K) const;

    /**
     * @brief Get the number of rigid body modes in the matrix
     * @param K Stiffness matrix
     * @param threshold Eigenvalue threshold for detection
     * @return Number of eigenvalues below threshold
     */
    int count_rigid_body_modes(
        const Eigen::SparseMatrix<double>& K,
        double threshold = 1e-8) const;

private:
    /**
     * @brief Identify mode type from eigenvector pattern
     */
    RigidBodyModeType identify_mode_type(
        const Eigen::VectorXd& eigenvector,
        const DOFHandler& dof_handler) const;

    /**
     * @brief Extract DOF participation from eigenvector
     */
    std::vector<DOFParticipation> extract_dof_participation(
        const Eigen::VectorXd& eigenvector,
        const DOFHandler& dof_handler,
        double threshold) const;

    /**
     * @brief Generate fix suggestion based on mode type
     */
    std::string generate_fix_suggestion(
        RigidBodyModeType mode_type,
        const std::vector<DOFParticipation>& participating_dofs) const;

    /**
     * @brief Build summary and detailed messages
     */
    void build_diagnostic_messages(SingularityDiagnostics& result) const;

    /**
     * @brief Get local DOF name (UX, UY, etc.)
     */
    static std::string local_dof_name(int local_dof);
};

} // namespace grillex

#endif // GRILLEX_SINGULARITY_DIAGNOSTICS_HPP
