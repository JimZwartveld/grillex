#pragma once

#include <Eigen/Dense>

namespace grillex {

/**
 * @brief Local coordinate system for beam elements
 *
 * Computes and stores the local coordinate system for a beam element
 * based on two end points and an optional roll angle.
 *
 * The local axes are defined as:
 * - x_axis: Along the beam from end_a to end_b
 * - y_axis: Perpendicular to x in the reference plane
 * - z_axis: Completes right-handed system (y × x)
 *
 * For nearly vertical beams (|x · global_z| > 0.99), global_x is used
 * as the reference direction. Otherwise, global_z is used.
 *
 * The roll angle allows rotation of y and z axes about the beam axis (x).
 */
class LocalAxes {
public:
    Eigen::Matrix3d rotation_matrix;  ///< Rotation matrix (global to local)
    Eigen::Vector3d x_axis;           ///< Local x-axis (along beam)
    Eigen::Vector3d y_axis;           ///< Local y-axis
    Eigen::Vector3d z_axis;           ///< Local z-axis
    double roll = 0.0;                ///< Roll angle about x-axis [radians]

    /**
     * @brief Construct local axes from two points
     *
     * @param end_a First endpoint position [m]
     * @param end_b Second endpoint position [m]
     * @param roll_angle Roll angle about x-axis [radians], default 0.0
     */
    LocalAxes(const Eigen::Vector3d& end_a,
              const Eigen::Vector3d& end_b,
              double roll_angle = 0.0);

    /**
     * @brief Transform a vector from global to local coordinates
     *
     * @param global Vector in global coordinates
     * @return Eigen::Vector3d Vector in local coordinates
     */
    Eigen::Vector3d to_local(const Eigen::Vector3d& global) const;

    /**
     * @brief Transform a vector from local to global coordinates
     *
     * @param local Vector in local coordinates
     * @return Eigen::Vector3d Vector in global coordinates
     */
    Eigen::Vector3d to_global(const Eigen::Vector3d& local) const;

private:
    /**
     * @brief Build rotation matrix from axis vectors
     *
     * The rotation matrix has local axes as columns:
     * R = [x_axis | y_axis | z_axis]
     *
     * This matrix transforms from global to local: v_local = R^T * v_global
     */
    void build_rotation_matrix();

    /**
     * @brief Apply roll angle rotation about x-axis
     *
     * @param roll_angle Roll angle [radians]
     */
    void apply_roll(double roll_angle);
};

} // namespace grillex
