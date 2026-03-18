#ifndef HELIOS_ESTIMATOR__STATE_HPP_
#define HELIOS_ESTIMATOR__STATE_HPP_

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace helios{

    // ====================
    // ERROR-STATE DIMENSION:-
    // del_x ∈ R_15:
    //   [0:3]  del_θ   — rotation error in so(3)  (Lie algebra)
    //   [3:6]  del_p   — position error in R³
    //   [6:9]  del_v   — velocity error in R³
    //   [9:12] del_ba  — accelerometer bias error
    //  [12:15] del_bg  — gyroscope bias error
    // ====================

    static constexpr int kStateDim = 15;

    using Matrix15d = Eigen::Matrix<double, kStateDim, kStateDim>;
    using Vector15d = Eigen::Matrix<double, kStateDim, 1>;

    // ====================
    // NOMINAL STATE ON SE(3)
    // ====================
    struct State{
        // Pose - rotation R in SO(3) and position in R_3
        // Sophus stores this as unit quat and translation vec
        Sophus::SE3d T;

        // Velocity in world frame (ENU)
        Eigen::Vector3d v{Eigen::Vector3d::Zero()};

        // IMU Biases - random walk
        Eigen::Vector3d ba{Eigen::Vector3d::Zero()}; // accel. bias [m/s^2]
        Eigen::Vector3d bg{Eigen::Vector3d::Zero()}; // gyro. bias [rad/s]

        // Default constructor - identity pose, zero velocity and biases
        State() : T{Sophus::SE3d::rotX(0.0)} {}

        // Convenience Constructor
        State(const Sophus::SE3d& pose,
              const Eigen::Vector3d& velo,
              const Eigen::Vector3d& accel_bias,
              const Eigen::Vector3d& gyro_bias)
        : T{pose}, v{velo}, ba{accel_bias}, bg{gyro_bias} {}
    };

    // ====================
    // ERROR-STATE COVARIANCE
    // P in R_15x15 - on Tangent Space at current state
    // Block structure:
    // P[0:3, 0:3]     = rotation covariance
    // P[3:6, 3:6]     = position covariance
    // P[6:9, 6:9]     = velocity covariance
    // P[9:12, 9:12]   = accel bias covariance
    // P[12:15, 12:15] = gyro bias covariance
    // ====================
    using Covariance = Matrix15d;

    // ====================
    // IMU MEASUREMENT
    // ====================
    struct ImuMeasurement{
        Eigen::Vector3d accel; // raw accelerometer [m/s^2]
        Eigen::Vector3d gyro; // raw gyroscope [rad/s]
        double dt; // time since last IMU sample [s]
    };

    // ====================
    // ODOM MEASUREMENT
    // ====================
    struct OdomMeasurement{
        Sophus::SE3d pose;
        Eigen::Matrix<double, 6, 6> cov; 
        enum class Source {LIDAR, VISUAL} source;
    };

    // ====================
    // ESEKF NOISE PARAMS
    // ====================
    struct NoiseParams{
        // IMU continuous-time noise spectral densities (From datasheet->converted to SI)
        double acc_noise{3.9939e-3};  // accel. noise [m/s^2/Hz^(1/2)]
        double gyro_noise{1.5642e-3}; // gyro. noise [rad/s/Hz^(1/2)]

        // Bias Random Walk (Stationary readings -> Allan Variance Algorithm)
        double acc_bias_noise{6.4886e-5};   // [m/s³/√Hz]
        double gyro_bias_noise{9.9703e-6};  // [rad/s²/√Hz]

        // Initial Cov. Diagonal Values
        double init_rot_cov{1e-4};    // [rad²]
        double init_pos_cov{1e-4};    // [m²]
        double init_vel_cov{1e-4};    // [m²/s²]
        double init_ba_cov{1e-6};     // [m²/s⁴]
        double init_bg_cov{1e-8};     // [rad²/s²]
    };

} // namespace helios

#endif