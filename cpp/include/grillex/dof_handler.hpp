#pragma once

#include "grillex/node.hpp"
#include "grillex/node_registry.hpp"
#include "grillex/beam_element.hpp"
#include <map>
#include <set>
#include <vector>
#include <utility>

namespace grillex {

/**
 * @brief Information about a warping DOF for a specific element at a node
 *
 * Warping DOFs are element-specific because warping displacement is a
 * cross-section phenomenon in the local element direction. Non-collinear
 * elements at a shared node have incompatible warping modes.
 */
struct WarpingDOFInfo {
    int element_id;     ///< Element ID
    int node_id;        ///< Node ID where this warping DOF is located
    bool is_node_i;     ///< True if this is at node_i of the element, false for node_j
    int global_dof;     ///< Assigned global DOF number

    bool operator<(const WarpingDOFInfo& other) const {
        if (element_id != other.element_id) return element_id < other.element_id;
        return node_id < other.node_id;
    }
};

/**
 * @brief Represents a group of warping DOFs that should be coupled (share the same global DOF)
 *
 * For collinear elements meeting at a node, their warping DOFs should be coupled
 * because warping deformation is geometrically compatible along a continuous beam.
 */
struct WarpingCoupling {
    std::vector<WarpingDOFInfo> coupled_dofs;  ///< DOFs that share the same global DOF
    int master_dof;                             ///< The global DOF number used by all coupled DOFs
};

/**
 * @brief Handles global DOF numbering for structural analysis
 *
 * The DOFHandler assigns unique global DOF numbers to all active degrees of freedom
 * in the model. It supports both standard 6-DOF nodes (UX, UY, UZ, RX, RY, RZ) and
 * 7-DOF nodes with warping (UX, UY, UZ, RX, RY, RZ, WARP).
 *
 * **Warping DOF Handling:**
 * Standard DOFs (0-5) are nodal - all elements connected to a node share these DOFs.
 * Warping DOFs (6) are element-specific by default because warping is a cross-section
 * phenomenon in the local element direction. Non-collinear elements have incompatible
 * warping modes.
 *
 * When number_dofs_with_elements() is called, the handler automatically:
 * 1. Assigns unique warping DOFs to each element end
 * 2. Detects collinear element groups at shared nodes
 * 3. Couples warping DOFs for collinear elements (they share the same global DOF)
 *
 * Users can also manually control warping coupling via set_warping_continuous() and
 * release_warping_coupling().
 */
class DOFHandler {
public:
    /**
     * @brief Construct an empty DOFHandler
     */
    DOFHandler();

    /**
     * @brief Assign global DOF numbers to all nodes (legacy mode)
     *
     * This method assigns DOF numbers based on nodes only (no element-specific
     * warping handling). Use number_dofs_with_elements() for proper warping support.
     *
     * @param registry The node registry containing all nodes
     */
    void number_dofs(NodeRegistry& registry);

    /**
     * @brief Assign global DOF numbers with element-specific warping handling
     *
     * This is the preferred method for models with warping DOFs. It:
     * 1. Numbers standard DOFs (0-5) per node
     * 2. Numbers warping DOFs (6) per element-end
     * 3. Automatically couples warping DOFs for collinear elements
     *
     * @param registry The node registry containing all nodes
     * @param elements Vector of beam elements in the model
     * @param collinearity_tolerance_deg Angle tolerance for collinearity detection (default 5°)
     */
    void number_dofs_with_elements(
        NodeRegistry& registry,
        const std::vector<BeamElement*>& elements,
        double collinearity_tolerance_deg = 5.0);

    /**
     * @brief Get total number of DOFs in the system
     *
     * @return int Total number of active DOFs
     */
    int total_dofs() const;

    /**
     * @brief Get global DOF number for a specific node and local DOF (standard DOFs only)
     *
     * For warping DOF (local_dof=6), use get_warping_dof() instead as warping
     * DOFs are element-specific.
     *
     * @param node_id Node ID
     * @param local_dof Local DOF index (0-5 for standard DOFs)
     * @return int Global DOF number, or -1 if DOF is not active
     */
    int get_global_dof(int node_id, int local_dof) const;

    /**
     * @brief Get warping DOF for a specific element at a specific node
     *
     * Warping DOFs are element-specific. This returns the global DOF number
     * for the warping DOF of the given element at the given node.
     *
     * @param element_id Element ID
     * @param node_id Node ID (must be node_i or node_j of the element)
     * @return int Global DOF number for warping, or -1 if not active/found
     */
    int get_warping_dof(int element_id, int node_id) const;

    /**
     * @brief Get location array for an element
     *
     * The location array maps element local DOFs to global DOF numbers.
     * For 14-DOF elements, uses element-specific warping DOFs.
     *
     * @param elem The beam element (can be 12-DOF or 14-DOF)
     * @return std::vector<int> Location array mapping local to global DOFs
     */
    std::vector<int> get_location_array(const BeamElementBase& elem) const;

    /**
     * @brief Check if any node has warping DOF active
     *
     * @return bool True if at least one warping DOF exists
     */
    bool has_warping_dofs() const;

    /**
     * @brief Clear all DOF numbering
     *
     * Resets the DOF handler to its initial state.
     */
    void clear();

    /**
     * @brief Manually specify that warping should be continuous between elements
     *
     * Use this to override automatic collinearity detection and force warping
     * DOFs to be coupled at a specific node.
     *
     * @param node_id Node ID where elements connect
     * @param element_ids IDs of elements that should share warping DOF at this node
     */
    void set_warping_continuous(int node_id, const std::vector<int>& element_ids);

    /**
     * @brief Manually release warping coupling between two elements
     *
     * Use this to override automatic collinearity detection and make warping
     * DOFs independent at a specific node.
     *
     * @param node_id Node ID where elements connect
     * @param element1_id First element ID
     * @param element2_id Second element ID
     */
    void release_warping_coupling(int node_id, int element1_id, int element2_id);

    /**
     * @brief Get all warping coupling information
     *
     * @return const std::vector<WarpingCoupling>& Vector of warping coupling groups
     */
    const std::vector<WarpingCoupling>& get_warping_couplings() const;

    /**
     * @brief Get the collinearity tolerance used for warping DOF coupling
     *
     * @return double Angle tolerance in degrees
     */
    double get_collinearity_tolerance() const { return collinearity_tolerance_; }

private:
    int total_dofs_;                                    ///< Total number of DOFs
    bool has_warping_;                                  ///< True if any warping DOFs exist
    double collinearity_tolerance_;                     ///< Angle tolerance for collinearity (degrees)
    std::map<std::pair<int,int>, int> dof_map_;        ///< (node_id, local_dof) -> global_dof

    /// Warping DOF map: (element_id, node_id) -> global_dof
    std::map<std::pair<int,int>, int> warping_dof_map_;

    /// Groups of collinear elements at each node (for warping DOF coupling)
    std::vector<WarpingCoupling> warping_couplings_;

    /// User-specified forced couplings: node_id -> set of element IDs to couple
    std::map<int, std::set<int>> forced_couplings_;

    /// User-specified released couplings: node_id -> pairs of (elem1, elem2) to not couple
    std::map<int, std::set<std::pair<int,int>>> released_couplings_;

    /**
     * @brief Identify groups of collinear elements at a node
     *
     * @param node_id Node ID to analyze
     * @param elements All elements connecting to this node
     * @param tolerance_deg Angle tolerance for collinearity
     * @return std::vector<std::vector<BeamElement*>> Groups of collinear elements
     */
    std::vector<std::vector<BeamElement*>> identify_collinear_groups(
        int node_id,
        const std::vector<BeamElement*>& node_elements,
        double tolerance_deg) const;
};

} // namespace grillex
