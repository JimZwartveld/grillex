#pragma once

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <map>
#include <stdexcept>
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

/**
 * @brief Term in a load combination (load case + factor)
 */
struct LoadCombinationTerm {
    LoadCase* load_case;                   ///< Load case (non-owning pointer)
    double factor;                         ///< Load factor to apply
    bool explicit_factor;                  ///< True if factor was explicitly set

    LoadCombinationTerm(LoadCase* lc, double f, bool is_explicit = false)
        : load_case(lc), factor(f), explicit_factor(is_explicit) {}
};

/**
 * @brief Load combination for code-based analysis (ULS, SLS, etc.)
 *
 * A LoadCombination combines multiple load cases with factors to produce
 * combined results. Two approaches are supported:
 *
 * 1. **Type-based factors**: Set default factors for each load case type
 *    (Permanent, Variable, Environmental, Accidental) in the constructor.
 *    When adding load cases, they automatically get the factor for their type.
 *
 * 2. **Explicit factors**: Override the type-based factor by specifying
 *    an explicit factor when adding a load case.
 *
 * Usage Example 1 (type-based, Eurocode ULS):
 * @code
 *   auto combo = LoadCombination(1, "ULS-STR", 1.35, 1.5, 1.5, 1.0);
 *   combo.add_load_case(dead_load);   // Uses 1.35 (Permanent type)
 *   combo.add_load_case(live_load);   // Uses 1.5 (Variable type)
 *   combo.add_load_case(wind);        // Uses 1.5 (Environmental type)
 * @endcode
 *
 * Usage Example 2 (explicit factors):
 * @code
 *   auto combo = LoadCombination(2, "Custom");
 *   combo.add_load_case(dead_load, 1.0);    // Explicit factor 1.0
 *   combo.add_load_case(live_load, 0.7);    // Explicit factor 0.7 (reduction)
 * @endcode
 *
 * Usage Example 3 (mixed):
 * @code
 *   auto combo = LoadCombination(3, "Wind-ULS", 1.0, 0.5, 1.5, 1.0);
 *   combo.add_load_case(dead_load);       // Uses 1.0 (Permanent type)
 *   combo.add_load_case(live_load);       // Uses 0.5 (Variable type)
 *   combo.add_load_case(wind);            // Uses 1.5 (Environmental type)
 *   combo.add_load_case(special, 0.9);    // Explicit factor 0.9
 * @endcode
 */
class LoadCombination {
public:
    /**
     * @brief Construct a load combination with type-based factors
     * @param id Unique identifier
     * @param name Descriptive name (e.g., "ULS-STR", "SLS-Characteristic")
     * @param permanent_factor Factor for Permanent load cases (default: 1.0)
     * @param variable_factor Factor for Variable load cases (default: 1.0)
     * @param environmental_factor Factor for Environmental load cases (default: 1.0)
     * @param accidental_factor Factor for Accidental load cases (default: 1.0)
     */
    LoadCombination(int id, const std::string& name,
                   double permanent_factor = 1.0,
                   double variable_factor = 1.0,
                   double environmental_factor = 1.0,
                   double accidental_factor = 1.0);

    // Getters
    int id() const { return id_; }
    const std::string& name() const { return name_; }

    /**
     * @brief Get the factor for a load case type
     * @param type Load case type
     * @return Factor for that type
     */
    double get_type_factor(LoadCaseType type) const;

    /**
     * @brief Set the factor for a load case type
     * @param type Load case type
     * @param factor Factor to use for that type
     */
    void set_type_factor(LoadCaseType type, double factor);

    /**
     * @brief Add a load case using the type-based factor
     * @param load_case Load case to add
     *
     * The factor is determined by the load case's type.
     */
    void add_load_case(LoadCase* load_case);

    /**
     * @brief Add a load case with an explicit factor
     * @param load_case Load case to add
     * @param factor Explicit factor to use (overrides type-based)
     */
    void add_load_case(LoadCase* load_case, double factor);

    /**
     * @brief Remove a load case from the combination
     * @param load_case Load case to remove
     * @return true if found and removed, false if not found
     */
    bool remove_load_case(LoadCase* load_case);

    /**
     * @brief Clear all load cases from the combination
     */
    void clear();

    /**
     * @brief Get all terms (load cases + factors) in the combination
     */
    const std::vector<LoadCombinationTerm>& get_terms() const { return terms_; }

    /**
     * @brief Get the number of load cases in the combination
     */
    size_t size() const { return terms_.size(); }

    /**
     * @brief Check if the combination is empty
     */
    bool empty() const { return terms_.empty(); }

    /**
     * @brief Get combined displacements from individual load case results
     * @param results Map of load case ID to LoadCaseResult
     * @return Combined displacement vector (factored sum)
     *
     * Combined result = Σ (factor_i * displacement_i)
     *
     * @throws std::runtime_error if a required load case result is missing
     */
    Eigen::VectorXd get_combined_displacements(
        const std::map<int, LoadCaseResult>& results) const;

    /**
     * @brief Get combined reactions from individual load case results
     * @param results Map of load case ID to LoadCaseResult
     * @return Combined reaction vector (factored sum)
     *
     * Combined result = Σ (factor_i * reaction_i)
     *
     * @throws std::runtime_error if a required load case result is missing
     */
    Eigen::VectorXd get_combined_reactions(
        const std::map<int, LoadCaseResult>& results) const;

private:
    int id_;
    std::string name_;

    // Type-based factors (used when no explicit factor is given)
    double permanent_factor_;
    double variable_factor_;
    double environmental_factor_;
    double accidental_factor_;

    // Terms: (load_case, factor, explicit_flag)
    std::vector<LoadCombinationTerm> terms_;
};

} // namespace grillex
