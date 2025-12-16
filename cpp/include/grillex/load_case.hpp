#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace grillex {

// Forward declarations
class Model;
class DOFHandler;

/**
 * @brief Distributed load representation for Phase 7 internal actions
 *
 * Represents a linearly varying distributed load along an element in local coordinates.
 * Used by Phase 7 to compute accurate internal actions using differential equations.
 *
 * Load varies linearly: q(x) = q_start + (q_end - q_start) * x / L
 */
struct DistributedLoad {
    double q_start = 0.0;  ///< Load intensity at element start [kN/m]
    double q_end = 0.0;    ///< Load intensity at element end [kN/m]

    /**
     * @brief Check if load is uniform (constant along length)
     */
    bool is_uniform() const { return std::abs(q_start - q_end) < 1e-10; }

    /**
     * @brief Check if load is zero
     */
    bool is_zero() const { return std::abs(q_start) < 1e-10 && std::abs(q_end) < 1e-10; }

    /**
     * @brief Get load intensity at position x along element
     * @param x Position along element [m]
     * @param L Element length [m]
     * @return Load intensity [kN/m]
     */
    double at(double x, double L) const {
        if (L < 1e-10) return q_start;
        return q_start + (q_end - q_start) * x / L;
    }
};

/**
 * @brief Type classification for load cases
 */
enum class LoadCaseType {
    Permanent,      ///< Dead loads, self-weight, fixed equipment
    Variable,       ///< Live loads, imposed loads, traffic
    Environmental,  ///< Wind, snow, temperature
    Accidental      ///< Impact, explosion, seismic (ultimate limit)
};

/**
 * @brief Nodal load structure
 *
 * Stores a concentrated force/moment at a node
 */
struct NodalLoad {
    int node_id;                           ///< Node where load is applied
    int local_dof;                         ///< Local DOF index (0-6: UX, UY, UZ, RX, RY, RZ, WARP)
    double value;                          ///< Load magnitude [kN] or [kN·m]

    NodalLoad(int node, int dof, double val)
        : node_id(node), local_dof(dof), value(val) {}
};

/**
 * @brief Line load structure (trapezoidal distribution)
 *
 * Represents a distributed load along a beam element with linearly
 * varying intensity from start to end.
 */
struct LineLoad {
    int element_id;                        ///< BeamElement where load is applied
    Eigen::Vector3d w_start;               ///< Load intensity at start [kN/m] (global coords)
    Eigen::Vector3d w_end;                 ///< Load intensity at end [kN/m] (global coords)

    LineLoad(int elem, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
        : element_id(elem), w_start(start), w_end(end) {}
};

/**
 * @brief Load case containing all loads for a specific scenario
 *
 * A LoadCase groups all loads (nodal, distributed, acceleration) that
 * represent a single loading scenario (e.g., "Dead Load", "Live Load", etc.).
 *
 * Usage:
 *   auto lc = model.create_load_case("Dead Load", LoadCaseType::Permanent);
 *   lc->add_nodal_load(node_id, DOFIndex::UY, -10.0);
 *   lc->add_line_load(beam_id, w_start, w_end);
 *   model.analyze();  // Analyzes all load cases
 */
class LoadCase {
public:
    /**
     * @brief Construct a load case
     * @param id Unique identifier
     * @param name Descriptive name (e.g., "Dead Load")
     * @param type Load case type classification
     */
    LoadCase(int id, const std::string& name, LoadCaseType type = LoadCaseType::Permanent);

    // Getters
    int id() const { return id_; }
    const std::string& name() const { return name_; }
    LoadCaseType type() const { return type_; }

    // Load management

    /**
     * @brief Add a nodal load to this load case
     * @param node_id Node ID
     * @param local_dof Local DOF index (0-6)
     * @param value Load value [kN] or [kN·m]
     *
     * Loads accumulate if called multiple times for same node/DOF.
     */
    void add_nodal_load(int node_id, int local_dof, double value);

    /**
     * @brief Add a distributed line load to a beam element
     * @param element_id Beam element ID
     * @param w_start Load intensity vector at start [kN/m] (global coords)
     * @param w_end Load intensity vector at end [kN/m] (global coords)
     *
     * For uniform load: w_start == w_end
     * For linearly varying: interpolate between start and end
     */
    void add_line_load(int element_id,
                      const Eigen::Vector3d& w_start,
                      const Eigen::Vector3d& w_end);

    /**
     * @brief Set acceleration field for this load case
     * @param accel 6-component acceleration [ax, ay, az, αx, αy, αz]
     *              Linear accelerations in [m/s²], angular in [rad/s²]
     * @param ref_point Reference point for acceleration field [m]
     *
     * Used for gravity (accel = [0, 0, -9.81, 0, 0, 0]) or
     * rotating reference frames (centrifugal effects).
     */
    void set_acceleration_field(const Eigen::Vector<double, 6>& accel,
                               const Eigen::Vector3d& ref_point = Eigen::Vector3d::Zero());

    /**
     * @brief Clear all loads in this case
     */
    void clear();

    /**
     * @brief Check if load case has any loads
     */
    bool is_empty() const;

    /**
     * @brief Assemble global load vector for this load case
     * @param model Model containing the structure
     * @param dof_handler DOF handler for global DOF mapping
     * @return Global load vector F (size = total_dofs)
     *
     * Combines:
     * - Direct nodal loads
     * - Equivalent nodal forces from line loads
     * - Inertial forces from acceleration field
     */
    Eigen::VectorXd assemble_load_vector(const Model& model,
                                        const DOFHandler& dof_handler) const;

    // Accessors for internal data (needed by BeamElement for Phase 7)
    const std::vector<NodalLoad>& get_nodal_loads() const { return nodal_loads_; }
    const std::vector<LineLoad>& get_line_loads() const { return line_loads_; }
    const Eigen::Vector<double, 6>& get_acceleration() const { return acceleration_; }
    const Eigen::Vector3d& get_acceleration_ref_point() const { return acceleration_ref_point_; }

private:
    int id_;
    std::string name_;
    LoadCaseType type_;

    std::vector<NodalLoad> nodal_loads_;
    std::vector<LineLoad> line_loads_;

    // Acceleration field (optional)
    Eigen::Vector<double, 6> acceleration_;        // [ax, ay, az, αx, αy, αz]
    Eigen::Vector3d acceleration_ref_point_;       // Reference point for field
};

/**
 * @brief Results for a single load case analysis
 *
 * Stores the computed displacements and reactions for a specific load case.
 */
struct LoadCaseResult {
    LoadCase* load_case;                   ///< Associated load case (non-owning pointer)
    Eigen::VectorXd displacements;         ///< Global displacement vector
    Eigen::VectorXd reactions;             ///< Reaction forces at constraints
    bool success;                          ///< Analysis succeeded
    std::string error_message;             ///< Error message if failed

    LoadCaseResult() : load_case(nullptr), success(false) {}
    LoadCaseResult(LoadCase* lc)
        : load_case(lc), success(false) {}
};

} // namespace grillex
