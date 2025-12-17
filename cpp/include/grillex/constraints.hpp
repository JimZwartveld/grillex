#ifndef GRILLEX_CONSTRAINTS_HPP
#define GRILLEX_CONSTRAINTS_HPP

#include <vector>
#include <set>
#include <map>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "grillex/dof_handler.hpp"

namespace grillex {

/**
 * @brief Represents a simple equality constraint between two DOFs
 *
 * Constrains the slave DOF to equal the master DOF:
 * u_slave = u_master
 *
 * This eliminates the slave DOF from the system by replacing it with
 * a reference to the master DOF in the transformation matrix.
 */
struct EqualityConstraint {
    int slave_node_id;      ///< Node ID of the slave DOF
    int slave_local_dof;    ///< Local DOF index at slave node (0-6)
    int master_node_id;     ///< Node ID of the master DOF
    int master_local_dof;   ///< Local DOF index at master node (0-6)

    /**
     * @brief Construct an equality constraint
     * @param slave_node Node ID of slave
     * @param slave_dof Local DOF of slave (0-6)
     * @param master_node Node ID of master
     * @param master_dof Local DOF of master (0-6)
     */
    EqualityConstraint(int slave_node, int slave_dof, int master_node, int master_dof)
        : slave_node_id(slave_node), slave_local_dof(slave_dof),
          master_node_id(master_node), master_local_dof(master_dof) {}
};

/**
 * @brief Represents a rigid link constraint between two nodes
 *
 * A rigid link enforces kinematic compatibility between a slave node and
 * a master node separated by an offset vector r. The slave node's motion
 * is completely determined by the master node's motion:
 *
 * u_slave = u_master + θ_master × r
 * θ_slave = θ_master
 *
 * Where:
 * - u_slave, u_master are translation DOFs (UX, UY, UZ)
 * - θ_slave, θ_master are rotation DOFs (RX, RY, RZ)
 * - r is the offset vector from master to slave in global coordinates
 *
 * The cross product θ × r is implemented as:
 * [u_sx]   [u_mx]   [ 0   -rz   ry] [θ_mx]
 * [u_sy] = [u_my] + [rz    0  -rx] [θ_my]
 * [u_sz]   [u_mz]   [-ry  rx    0] [θ_mz]
 */
struct RigidLink {
    int slave_node_id;      ///< Node ID of the slave (constrained) node
    int master_node_id;     ///< Node ID of the master (independent) node
    Eigen::Vector3d offset; ///< Offset vector from master to slave [m] in global coords

    /**
     * @brief Construct a rigid link
     * @param slave_node Node ID of slave
     * @param master_node Node ID of master
     * @param r Offset vector from master to slave in global coordinates [m]
     */
    RigidLink(int slave_node, int master_node, const Eigen::Vector3d& r)
        : slave_node_id(slave_node), master_node_id(master_node), offset(r) {}

    /**
     * @brief Compute the 3x3 skew-symmetric matrix for offset
     *
     * Returns the matrix R such that θ × r = R * θ:
     * R = [ 0   -rz   ry]
     *     [rz    0  -rx]
     *     [-ry  rx    0]
     *
     * @return 3x3 skew-symmetric matrix
     */
    Eigen::Matrix3d skew_matrix() const {
        Eigen::Matrix3d R;
        R << 0.0, -offset.z(), offset.y(),
             offset.z(), 0.0, -offset.x(),
             -offset.y(), offset.x(), 0.0;
        return R;
    }

    /**
     * @brief Compute the full 6x6 transformation block for rigid link kinematics
     *
     * Returns the transformation matrix T_RL that relates slave DOFs to master DOFs:
     *
     * [u_S]   [I   R] [u_M]
     * [θ_S] = [0   I] [θ_M]
     *
     * Where:
     * - I is the 3x3 identity matrix
     * - R is the 3x3 skew-symmetric matrix from skew_matrix()
     * - 0 is the 3x3 zero matrix
     *
     * The resulting 6x6 matrix has the structure:
     * T_RL = [I_3  R  ]
     *        [0_3  I_3]
     *
     * This transforms master DOFs [u_M; θ_M] to slave DOFs [u_S; θ_S].
     *
     * @return 6x6 transformation matrix
     */
    Eigen::Matrix<double, 6, 6> transformation_block_6x6() const {
        Eigen::Matrix<double, 6, 6> T;
        T.setZero();

        // Top-left: Identity for translation-to-translation
        T.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        // Top-right: Skew matrix for rotation-to-translation coupling
        T.block<3, 3>(0, 3) = skew_matrix();

        // Bottom-left: Zero (no translation-to-rotation coupling)
        // Already zero from setZero()

        // Bottom-right: Identity for rotation-to-rotation
        T.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

        return T;
    }
};

/**
 * @brief Handles multi-point constraints (MPC) for structural analysis
 *
 * The ConstraintHandler manages kinematic constraints between DOFs, including:
 * - Simple equality constraints: u_slave = u_master
 * - Rigid link constraints: rigid body motion from master to slave
 *
 * **Transformation Matrix Approach:**
 *
 * The handler builds a transformation matrix T that relates the full DOF vector
 * (with slave DOFs) to a reduced DOF vector (only master/independent DOFs):
 *
 *     u_full = T * u_reduced
 *
 * The global stiffness matrix and force vector are then transformed:
 *
 *     K_reduced = T^T * K_full * T
 *     F_reduced = T^T * F_full
 *
 * After solving for u_reduced, the full displacements are recovered:
 *
 *     u_full = T * u_reduced
 *
 * **DOF Classification:**
 * - Independent DOFs: DOFs that are not constrained (master DOFs)
 * - Dependent DOFs: DOFs that are constrained to other DOFs (slave DOFs)
 *
 * The reduced system only contains independent DOFs.
 *
 * Example usage:
 * @code
 * ConstraintHandler constraints;
 *
 * // Tie two nodes together (all DOFs equal)
 * constraints.add_rigid_link(slave_node, master_node, Eigen::Vector3d::Zero());
 *
 * // Add a rigid offset connection
 * constraints.add_rigid_link(slave_node, master_node, Eigen::Vector3d(0.5, 0.0, 0.0));
 *
 * // Build transformation and reduce system
 * auto result = constraints.reduce_system(K, F, dof_handler);
 * auto& K_red = result.K_reduced;
 * auto& F_red = result.F_reduced;
 *
 * // Solve reduced system
 * Eigen::VectorXd u_red = solver.solve(K_red, F_red);
 *
 * // Recover full displacements
 * Eigen::VectorXd u_full = constraints.expand_displacements(u_red);
 * @endcode
 */
class ConstraintHandler {
public:
    /**
     * @brief Result of system reduction
     */
    struct ReducedSystem {
        Eigen::SparseMatrix<double> K_reduced;  ///< Reduced stiffness matrix
        Eigen::VectorXd F_reduced;               ///< Reduced force vector
        Eigen::SparseMatrix<double> T;           ///< Transformation matrix
        int n_full;                               ///< Number of full DOFs
        int n_reduced;                            ///< Number of reduced (independent) DOFs
    };

    /**
     * @brief Construct an empty constraint handler
     */
    ConstraintHandler() = default;

    /**
     * @brief Add a simple equality constraint
     *
     * Constrains u_slave = u_master for a single DOF pair.
     *
     * @param slave_node Node ID of the slave DOF
     * @param slave_dof Local DOF index at slave (0-6)
     * @param master_node Node ID of the master DOF
     * @param master_dof Local DOF index at master (0-6)
     */
    void add_equality_constraint(int slave_node, int slave_dof,
                                 int master_node, int master_dof);

    /**
     * @brief Add a rigid link constraint between two nodes
     *
     * Creates a kinematic constraint where the slave node's motion is
     * completely determined by the master node's motion:
     *
     * u_slave = u_master + θ_master × offset
     * θ_slave = θ_master
     *
     * This constrains all 6 standard DOFs of the slave node.
     *
     * @param slave_node Node ID of the slave (constrained) node
     * @param master_node Node ID of the master (independent) node
     * @param offset Offset vector from master to slave in global coords [m]
     */
    void add_rigid_link(int slave_node, int master_node,
                        const Eigen::Vector3d& offset);

    /**
     * @brief Build the transformation matrix T
     *
     * Constructs the sparse transformation matrix that maps reduced
     * (independent) DOFs to full DOFs:
     *
     *     u_full = T * u_reduced
     *
     * The matrix has dimensions (n_full × n_reduced) where:
     * - n_full = total number of DOFs in original system
     * - n_reduced = number of independent DOFs (n_full - num_constraints)
     *
     * @param dof_handler DOF handler with DOF numbering
     * @return Sparse transformation matrix T
     */
    Eigen::SparseMatrix<double> build_transformation_matrix(
        const DOFHandler& dof_handler) const;

    /**
     * @brief Reduce the global system using constraints
     *
     * Transforms the global stiffness matrix and force vector to eliminate
     * slave (dependent) DOFs:
     *
     *     K_reduced = T^T * K * T
     *     F_reduced = T^T * F
     *
     * @param K Global stiffness matrix (n_full × n_full)
     * @param F Global force vector (n_full)
     * @param dof_handler DOF handler with DOF numbering
     * @return ReducedSystem containing K_reduced, F_reduced, and T
     */
    ReducedSystem reduce_system(
        const Eigen::SparseMatrix<double>& K,
        const Eigen::VectorXd& F,
        const DOFHandler& dof_handler) const;

    /**
     * @brief Expand reduced displacements to full DOF vector
     *
     * Recovers the full displacement vector from the reduced solution:
     *
     *     u_full = T * u_reduced
     *
     * @param u_reduced Reduced displacement vector from solver
     * @param T Transformation matrix from reduce_system
     * @return Full displacement vector including slave DOFs
     */
    Eigen::VectorXd expand_displacements(
        const Eigen::VectorXd& u_reduced,
        const Eigen::SparseMatrix<double>& T) const;

    /**
     * @brief Get all equality constraints
     * @return Vector of equality constraints
     */
    const std::vector<EqualityConstraint>& get_equality_constraints() const {
        return equalities_;
    }

    /**
     * @brief Get all rigid link constraints
     * @return Vector of rigid link constraints
     */
    const std::vector<RigidLink>& get_rigid_links() const {
        return rigid_links_;
    }

    /**
     * @brief Get number of slave DOFs (DOFs that will be eliminated)
     * @param dof_handler DOF handler for node DOF lookup
     * @return Number of DOFs that will be constrained out
     */
    int num_slave_dofs(const DOFHandler& dof_handler) const;

    /**
     * @brief Check if any constraints are defined
     * @return True if at least one constraint exists
     */
    bool has_constraints() const {
        return !equalities_.empty() || !rigid_links_.empty();
    }

    /**
     * @brief Clear all constraints
     */
    void clear() {
        equalities_.clear();
        rigid_links_.clear();
    }

private:
    std::vector<EqualityConstraint> equalities_;  ///< Equality constraints
    std::vector<RigidLink> rigid_links_;           ///< Rigid link constraints

    /**
     * @brief Build map of slave global DOFs to their constraint info
     *
     * For each slave DOF, stores which master DOFs it depends on
     * and with what coefficients.
     *
     * @param dof_handler DOF handler for node DOF lookup
     * @return Map from slave global DOF to list of (master global DOF, coefficient)
     */
    std::map<int, std::vector<std::pair<int, double>>> build_constraint_map(
        const DOFHandler& dof_handler) const;

    /**
     * @brief Get set of slave global DOFs
     *
     * @param dof_handler DOF handler for node DOF lookup
     * @return Set of global DOF indices that are constrained (slaves)
     */
    std::set<int> get_slave_dofs(const DOFHandler& dof_handler) const;
};

}  // namespace grillex

#endif  // GRILLEX_CONSTRAINTS_HPP
