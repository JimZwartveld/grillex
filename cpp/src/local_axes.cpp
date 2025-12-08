#include "grillex/local_axes.hpp"
#include <cmath>
#include <stdexcept>

namespace grillex {

LocalAxes::LocalAxes(const Eigen::Vector3d& end_a,
                     const Eigen::Vector3d& end_b,
                     double roll_angle) {
    // Step 1: Compute x_axis = normalize(end_b - end_a)
    Eigen::Vector3d beam_vector = end_b - end_a;
    double length = beam_vector.norm();

    if (length < 1e-10) {
        throw std::invalid_argument("Beam length is too small (near-zero)");
    }

    x_axis = beam_vector / length;

    // Step 2: Determine reference direction based on beam orientation
    Eigen::Vector3d global_x(1.0, 0.0, 0.0);
    Eigen::Vector3d global_z(0.0, 0.0, 1.0);
    Eigen::Vector3d reference;

    // Check if beam is nearly vertical (alignment with global Z)
    double dot_z = std::abs(x_axis.dot(global_z));

    if (dot_z > 0.99) {
        // Nearly vertical beam - use global X as reference
        reference = global_x;
    } else {
        // Use global Z as reference
        reference = global_z;
    }

    // Step 3: Compute z_axis perpendicular to x_axis in the reference plane
    // z_axis = normalize(reference - (reference · x_axis) * x_axis)
    // This is the Gram-Schmidt orthogonalization
    Eigen::Vector3d z_temp = reference - reference.dot(x_axis) * x_axis;
    double z_norm = z_temp.norm();

    if (z_norm < 1e-10) {
        throw std::runtime_error("Failed to compute local z-axis (degenerate case)");
    }

    z_axis = z_temp / z_norm;

    // Step 4: Compute y_axis = z_axis × x_axis (right-handed system)
    y_axis = z_axis.cross(x_axis);

    // Step 5: Apply roll rotation if specified
    if (std::abs(roll_angle) > 1e-10) {
        apply_roll(roll_angle);
    }

    // Step 6: Build rotation matrix
    build_rotation_matrix();
}

Eigen::Vector3d LocalAxes::to_local(const Eigen::Vector3d& global) const {
    // Transform from global to local: v_local = R^T * v_global
    return rotation_matrix.transpose() * global;
}

Eigen::Vector3d LocalAxes::to_global(const Eigen::Vector3d& local) const {
    // Transform from local to global: v_global = R * v_local
    return rotation_matrix * local;
}

void LocalAxes::build_rotation_matrix() {
    // Rotation matrix with local axes as columns
    // R = [x_axis | y_axis | z_axis]
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
}

void LocalAxes::apply_roll(double roll_angle) {
    // Rotate y and z axes about x-axis by roll_angle
    // Rotation matrix about x-axis:
    // R_x(θ) = [1    0         0     ]
    //          [0  cos(θ)  -sin(θ) ]
    //          [0  sin(θ)   cos(θ) ]

    double c = std::cos(roll_angle);
    double s = std::sin(roll_angle);

    // Store original y and z
    Eigen::Vector3d y_old = y_axis;
    Eigen::Vector3d z_old = z_axis;

    // Apply rotation: only y and z change (x remains unchanged)
    y_axis = c * y_old - s * z_old;
    z_axis = s * y_old + c * z_old;
}

} // namespace grillex
