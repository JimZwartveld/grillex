/**
 * @file warnings.hpp
 * @brief Warning system for questionable model configurations.
 *
 * Warnings indicate potential issues that don't prevent analysis
 * but may indicate modeling errors or produce unreliable results.
 */

#ifndef GRILLEX_WARNINGS_HPP
#define GRILLEX_WARNINGS_HPP

#include <string>
#include <vector>
#include <map>

namespace grillex {

/**
 * @brief Warning codes for questionable model configurations.
 *
 * These codes identify potential issues that don't block analysis
 * but may indicate modeling problems.
 */
enum class WarningCode {
    // === Geometry Warnings (100-199) ===

    /// Beam has extreme aspect ratio (length/depth > 100 or < 2)
    EXTREME_ASPECT_RATIO = 100,

    /// Very short element (may cause numerical issues)
    SMALL_ELEMENT = 101,

    /// Very long element (may need subdivision for accuracy)
    LARGE_ELEMENT = 102,

    /// Non-collinear beams share warping DOF (may be intentional)
    NON_COLLINEAR_WARPING = 103,

    // === Stiffness Warnings (200-299) ===

    /// Large stiffness contrast between adjacent elements (ratio > 1e6)
    STIFFNESS_CONTRAST = 200,

    /// Near-singularity detected (condition number > 1e12)
    NEAR_SINGULARITY = 201,

    /// Very stiff spring (may cause numerical issues)
    VERY_STIFF_SPRING = 202,

    /// Very soft spring (may not provide effective restraint)
    VERY_SOFT_SPRING = 203,

    // === Property Warnings (300-399) ===

    /// Near-zero property value (e.g., very small area)
    NEAR_ZERO_PROPERTY = 300,

    /// Material property may be in wrong units
    POSSIBLE_UNIT_ERROR = 301,

    /// Section properties inconsistent (e.g., Iy > A^2)
    INCONSISTENT_SECTION = 302,

    // === Load Warnings (400-499) ===

    /// Very large load magnitude (may indicate unit error)
    LARGE_LOAD = 400,

    /// Load applied at unsupported node
    LOAD_AT_FREE_NODE = 401,

    /// Acceleration load may need point mass elements
    ACCELERATION_WITHOUT_MASS = 402,

    // === Analysis Warnings (500-599) ===

    /// Large displacements detected (geometric nonlinearity may be needed)
    LARGE_DISPLACEMENT = 500,

    /// High stress detected (material nonlinearity may be needed)
    HIGH_STRESS = 501,

    /// Solver used iterative refinement
    SOLVER_REFINEMENT = 502
};

/**
 * @brief Warning severity levels.
 */
enum class WarningSeverity {
    /// Minor issue, likely acceptable
    Low = 0,

    /// Potentially problematic, review recommended
    Medium = 1,

    /// Likely indicates a modeling error
    High = 2
};

/**
 * @brief Convert warning code to string representation.
 */
inline std::string warning_code_to_string(WarningCode code) {
    switch (code) {
        case WarningCode::EXTREME_ASPECT_RATIO: return "EXTREME_ASPECT_RATIO";
        case WarningCode::SMALL_ELEMENT: return "SMALL_ELEMENT";
        case WarningCode::LARGE_ELEMENT: return "LARGE_ELEMENT";
        case WarningCode::NON_COLLINEAR_WARPING: return "NON_COLLINEAR_WARPING";
        case WarningCode::STIFFNESS_CONTRAST: return "STIFFNESS_CONTRAST";
        case WarningCode::NEAR_SINGULARITY: return "NEAR_SINGULARITY";
        case WarningCode::VERY_STIFF_SPRING: return "VERY_STIFF_SPRING";
        case WarningCode::VERY_SOFT_SPRING: return "VERY_SOFT_SPRING";
        case WarningCode::NEAR_ZERO_PROPERTY: return "NEAR_ZERO_PROPERTY";
        case WarningCode::POSSIBLE_UNIT_ERROR: return "POSSIBLE_UNIT_ERROR";
        case WarningCode::INCONSISTENT_SECTION: return "INCONSISTENT_SECTION";
        case WarningCode::LARGE_LOAD: return "LARGE_LOAD";
        case WarningCode::LOAD_AT_FREE_NODE: return "LOAD_AT_FREE_NODE";
        case WarningCode::ACCELERATION_WITHOUT_MASS: return "ACCELERATION_WITHOUT_MASS";
        case WarningCode::LARGE_DISPLACEMENT: return "LARGE_DISPLACEMENT";
        case WarningCode::HIGH_STRESS: return "HIGH_STRESS";
        case WarningCode::SOLVER_REFINEMENT: return "SOLVER_REFINEMENT";
        default: return "UNKNOWN_WARNING";
    }
}

/**
 * @brief Convert severity to string representation.
 */
inline std::string severity_to_string(WarningSeverity severity) {
    switch (severity) {
        case WarningSeverity::Low: return "LOW";
        case WarningSeverity::Medium: return "MEDIUM";
        case WarningSeverity::High: return "HIGH";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Structured warning information for Grillex.
 */
struct GrillexWarning {
    /// Machine-readable warning code
    WarningCode code;

    /// Warning severity level
    WarningSeverity severity;

    /// Human-readable warning message
    std::string message;

    /// Element IDs involved in the warning
    std::vector<int> involved_elements;

    /// Node IDs involved in the warning
    std::vector<int> involved_nodes;

    /// Additional key-value details for diagnostics
    std::map<std::string, std::string> details;

    /// Suggested fix for the warning
    std::string suggestion;

    /**
     * @brief Construct warning with code, severity, and message.
     */
    GrillexWarning(WarningCode code, WarningSeverity severity, const std::string& message)
        : code(code), severity(severity), message(message) {}

    /**
     * @brief Get string representation of the warning code.
     */
    std::string code_string() const { return warning_code_to_string(code); }

    /**
     * @brief Get string representation of the severity.
     */
    std::string severity_string() const { return severity_to_string(severity); }

    /**
     * @brief Get formatted warning string for display.
     */
    std::string to_string() const {
        std::string result = "[" + severity_string() + "] [" + code_string() + "] " + message;

        if (!involved_elements.empty()) {
            result += "\n  Elements: ";
            for (size_t i = 0; i < involved_elements.size(); ++i) {
                if (i > 0) result += ", ";
                result += std::to_string(involved_elements[i]);
            }
        }

        if (!involved_nodes.empty()) {
            result += "\n  Nodes: ";
            for (size_t i = 0; i < involved_nodes.size(); ++i) {
                if (i > 0) result += ", ";
                result += std::to_string(involved_nodes[i]);
            }
        }

        for (const auto& kv : details) {
            result += "\n  " + kv.first + ": " + kv.second;
        }

        if (!suggestion.empty()) {
            result += "\n  Suggestion: " + suggestion;
        }

        return result;
    }

    // === Factory methods for common warnings ===

    /**
     * @brief Create warning for extreme aspect ratio.
     */
    static GrillexWarning extreme_aspect_ratio(int element_id, double ratio) {
        GrillexWarning warn(WarningCode::EXTREME_ASPECT_RATIO, WarningSeverity::Medium,
            "Beam has extreme aspect ratio");
        warn.involved_elements.push_back(element_id);
        warn.details["aspect_ratio"] = std::to_string(ratio);
        warn.suggestion = ratio > 100
            ? "Very slender beam - consider Euler-Bernoulli formulation"
            : "Very deep beam - consider Timoshenko formulation with shear deformation";
        return warn;
    }

    /**
     * @brief Create warning for small element.
     */
    static GrillexWarning small_element(int element_id, double length) {
        GrillexWarning warn(WarningCode::SMALL_ELEMENT, WarningSeverity::Medium,
            "Very short element may cause numerical issues");
        warn.involved_elements.push_back(element_id);
        warn.details["length"] = std::to_string(length) + " m";
        warn.suggestion = "Consider merging with adjacent element or using rigid link";
        return warn;
    }

    /**
     * @brief Create warning for stiffness contrast.
     */
    static GrillexWarning stiffness_contrast(int elem1, int elem2, double ratio) {
        GrillexWarning warn(WarningCode::STIFFNESS_CONTRAST, WarningSeverity::High,
            "Large stiffness contrast between adjacent elements");
        warn.involved_elements.push_back(elem1);
        warn.involved_elements.push_back(elem2);
        warn.details["stiffness_ratio"] = std::to_string(ratio);
        warn.suggestion = "May cause numerical issues. Check if this is intentional (e.g., rigid link) "
                         "or consider gradual stiffness transition";
        return warn;
    }

    /**
     * @brief Create warning for near singularity.
     */
    static GrillexWarning near_singularity(double condition_number) {
        GrillexWarning warn(WarningCode::NEAR_SINGULARITY, WarningSeverity::High,
            "Stiffness matrix is poorly conditioned");
        warn.details["condition_number"] = std::to_string(condition_number);
        warn.suggestion = "Results may be inaccurate. Check for nearly-singular constraints, "
                         "extreme stiffness ratios, or insufficient boundary conditions";
        return warn;
    }

    /**
     * @brief Create warning for large displacement.
     */
    static GrillexWarning large_displacement(int node_id, double displacement, double ratio) {
        GrillexWarning warn(WarningCode::LARGE_DISPLACEMENT, WarningSeverity::Medium,
            "Large displacement detected - linear analysis may be invalid");
        warn.involved_nodes.push_back(node_id);
        warn.details["max_displacement"] = std::to_string(displacement) + " m";
        warn.details["displacement_to_length_ratio"] = std::to_string(ratio);
        warn.suggestion = "Consider geometric nonlinear analysis if displacements exceed 1% of span";
        return warn;
    }

    /**
     * @brief Create warning for near-zero property.
     */
    static GrillexWarning near_zero_property(int element_id, const std::string& property_name,
                                              double value) {
        GrillexWarning warn(WarningCode::NEAR_ZERO_PROPERTY, WarningSeverity::High,
            "Near-zero property value may indicate modeling error");
        warn.involved_elements.push_back(element_id);
        warn.details["property"] = property_name;
        warn.details["value"] = std::to_string(value);
        warn.suggestion = "Check section/material property values and units";
        return warn;
    }
};

/**
 * @brief Collection of warnings from model validation.
 */
class WarningList {
public:
    /// List of warnings
    std::vector<GrillexWarning> warnings;

    /**
     * @brief Add a warning to the list.
     */
    void add(const GrillexWarning& warning) {
        warnings.push_back(warning);
    }

    /**
     * @brief Add a warning to the list (move semantics).
     */
    void add(GrillexWarning&& warning) {
        warnings.push_back(std::move(warning));
    }

    /**
     * @brief Check if any warnings exist.
     */
    bool has_warnings() const { return !warnings.empty(); }

    /**
     * @brief Get count of warnings.
     */
    size_t count() const { return warnings.size(); }

    /**
     * @brief Get count of warnings by severity.
     */
    size_t count_by_severity(WarningSeverity severity) const {
        size_t count = 0;
        for (const auto& w : warnings) {
            if (w.severity == severity) ++count;
        }
        return count;
    }

    /**
     * @brief Get all warnings with given severity or higher.
     */
    std::vector<GrillexWarning> get_by_min_severity(WarningSeverity min_severity) const {
        std::vector<GrillexWarning> result;
        for (const auto& w : warnings) {
            if (static_cast<int>(w.severity) >= static_cast<int>(min_severity)) {
                result.push_back(w);
            }
        }
        return result;
    }

    /**
     * @brief Clear all warnings.
     */
    void clear() { warnings.clear(); }

    /**
     * @brief Get formatted summary string.
     */
    std::string summary() const {
        if (warnings.empty()) return "No warnings";

        std::string result = std::to_string(warnings.size()) + " warning(s): ";
        result += std::to_string(count_by_severity(WarningSeverity::High)) + " high, ";
        result += std::to_string(count_by_severity(WarningSeverity::Medium)) + " medium, ";
        result += std::to_string(count_by_severity(WarningSeverity::Low)) + " low";
        return result;
    }
};

}  // namespace grillex

#endif  // GRILLEX_WARNINGS_HPP
