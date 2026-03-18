#include <cmath>
#include <stdexcept>

#include "helios_estimator/esekf.hpp"

namespace helios {

    // ====================
    // CONSTRUCTOR
    // ====================
    ESEKF::ESEKF(const NoiseParams& params) : params_{params}{
        // Build continuous-time process noise covariance Q_c (12x12)
        // Order: [accel noise, gyro noise, accel bias noise, gyro bias noise]
        Q_.setZero();
        Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * params_.acc_noise * params_.acc_noise;
        Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * params_.gyro_noise * params_.gyro_noise;
        Q_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * params_.acc_bias_noise * params_.acc_bias_noise;
        Q_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * params_.gyro_bias_noise * params_.gyro_bias_noise;
    }

    // ====================
    // Initialize
    // ====================
    void ESEKF::initialize(const OdomMeasurement& meas){
        state_.T = meas.pose;
        state_.v = Eigen::Vector3d::Zero();
        state_.ba = Eigen::Vector3d::Zero();
        state_.bg = Eigen::Vector3d::Zero();

        // Initialize covariance with small uncertainty
        P_.setZero();
        P_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * params_.init_rot_cov;
        P_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * params_.init_pos_cov;
        P_.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * params_.init_vel_cov;
        P_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * params_.init_ba_cov;
        P_.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * params_.init_bg_cov;

        initialized_ = true;
    }

    void ESEKF::setState(const State& state, const Covariance& cov){
        state_ = state;
        P_ = cov;
    }

    void ESEKF::reset(){
        initialized_ = false;
        state_ = State{};
        P_.setZero();
    }

    // ====================
    // PREDICT STEP
    // ====================
    void ESEKF::predict(const ImuMeasurement& imu){
        if(!initialized_) return;

        // Remove bias estimates from raw measurements
        const Eigen::Vector3d a = imu.accel - state_.ba;
        const Eigen::Vector3d w = imu.gyro - state_.bg;
        const double dt = imu.dt;

        // ====================
        // Nominal State Propagation
        // ====================

        // Rotate acceleration to world frame
        const Eigen::Vector3d a_world = state_.T.so3() * a;

        // Integrate rotation: R = R * Exp(w * dt)
        // Sophus::SO3d::exp() implements Rodrigues formula
        state_.T.so3() = state_.T.so3() * Sophus::SO3d::exp(w * dt);

        // Integrate position: p = p + vdt + 1/2 * (a_world + g) * dt^2
        const Eigen::Vector3d acc_net = a_world + kGravity_;
        state_.T.translation() += state_.v * dt + 0.5 * acc_net * dt * dt;

        // Integrate velocity: v = v + (a_world + g) * dt
        state_.v += acc_net * dt;

        // Biases are random walk - so no change in nominal state.
        // noise will be included in cov propagation

        // ====================
        // Error State Cov. Propagation
        // ====================
        
        // Discrete time: P = F * P * F.transpose() + G * Q * G.transpose()
        const Matrix15d F = buildF(a, w, dt);
        const auto G = buildG(dt);

        P_ = F * P_ * F.transpose() + G * Q_ * G.transpose();

        // Symmetrize to prevent numerical drift
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // ====================
    // F MATRIX
    // ====================
    // Error-state Jacobian - linearization of error dynamics around nominal state
    Matrix15d ESEKF::buildF(
        const Eigen::Vector3d& a_corrected,
        const Eigen::Vector3d& w_corrected,
        double dt
    ) const {
        Matrix15d F = Matrix15d::Identity();

        // R_true = R * del_R = R * Exp(del_theta)
        //        = R * (I + hat(del_theta)) 
        // a_world,true = R_true * a + g
        // ====================

        // Rotation of world-frame accel - used in position and velocity rows
        const Eigen::Matrix3d R = state_.T.so3().matrix();
        const Eigen::Matrix3d a_x = Sophus::SO3d::hat(a_corrected); // skew-symm

        // d(del_theta_next / del_theta_current) - rotation error rotates with angular velocity
        F.block<3,3>(0,0) = Sophus::SO3d::exp(-w_corrected * dt).matrix();

        // d(del_theta) / d(del_bg): rotation error depends on gyro bias (integration)
        F.block<3,3>(0,12) = -Eigen::Matrix3d::Identity() * dt;

        // d(del_p) / d(del_theta): position error depends on rotation error via cross product
        F.block<3,3>(3,0) = -R * a_x * dt * dt * 0.5;

        // d(del_p) / d(del_v): position error integrated velocity error
        F.block<3,3>(3,6) = Eigen::Matrix3d::Identity() * dt;

        // d(del_p) / d(del_ba): position error depends on accel bias via double integration
        F.block<3,3>(3,9) = -R * dt * dt * 0.5;

        // d(del_v) / d(del_theta): velocity error depends on rotation error
        F.block<3,3>(6,0) = -R * a_x * dt;

        // d(del_v) / d(del_ba): velocity error depends on accel bias (integration)
        F.block<3,3>(6,9) = -R * dt;

        return F;
    }

    // ====================
    // G MATRIX
    // ====================
    // Maps IMU Noise [na, nw, nba, nbg] (12-dim) to error-state-noise (15-dim)
    Eigen::Matrix<double, kStateDim, 12> ESEKF::buildG(double dt) const{
        Eigen::Matrix<double, kStateDim, 12> G = Eigen::Matrix<double, kStateDim, 12>::Zero();

        const Eigen::Matrix3d R = state_.T.so3().matrix();

        // Rotation affected by gyro noise
        G.block<3,3>(0,3) = -Eigen::Matrix3d::Identity() * dt;

        // Position affected by accel noise
        G.block<3,3>(3,0) = - R * dt * dt * 0.5;

        // Velocity affected by accel noise
        G.block<3,3>(6,0) = - R * dt;

        // Biases driven by respective noise
        G.block<3,3>(9,6) = Eigen::Matrix3d::Identity() * dt;
        G.block<3,3>(12,9) = Eigen::Matrix3d::Identity() * dt;

        return G;
    }
    
    // ====================
    // UPDATE STEP
    // ====================
    std::optional<State> ESEKF::update(const OdomMeasurement& meas){
        if(!initialized_) return std::nullopt;

        // ====================
        // Compute innovation in Lie Algebra 
        // ====================
        // Innovation: difference between measured and predicted pose in se(3)
        // del_z = Log(T_predicted.inverse() * T-meas)
        // Result is 6-vector [del_theta, del_p] in Lie Algebra
        Eigen::Matrix<double, 6, 1> innovation;

        // Rotation innovation: SO(3) log in local frame (right perturbation)
        innovation.head<3>() = (state_.T.so3().inverse() * meas.pose.so3()).log();

        // Position innovation: plain subtraction in global frame
        innovation.tail<3>() = meas.pose.translation() - state_.T.translation();

        // Mahalanobis Distance Check
        const auto H = buildH();
        const auto R = buildR(meas);

        const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
        const double mahal = innovation.transpose() * S.ldlt().solve(innovation);

        // Threshold: Chi-squared with 6DOF at 99.9% = 22.46
        constexpr double kMahalThreshold = 22.46;
        if(mahal > kMahalThreshold) return std::nullopt;

        // ====================
        // Kalman Gain
        // ====================
        const Eigen::Matrix<double, kStateDim, 6> K = S.ldlt().solve(H * P_).transpose();

        // ====================
        // Error-State Correction
        // ====================
        const Vector15d dx = K * innovation;

        // ====================
        // Apply Correction to nominal state (boxplus on SE(3))
        // ====================
        applyCorrection(dx);

        // ====================
        // Update Covariance (Joeseph Form)
        // ====================
        const Matrix15d IKH = Matrix15d::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() +  K * R * K.transpose();

        P_ = 0.5 * (P_ + P_.transpose());

        return state_;
    }

    // ====================
    // H MATRIX
    // ====================
    // 6x15 Measurement Jacobian
    // Maps 15-dim error to 6-dim pose residual in se(3)
    Eigen::Matrix<double, 6, kStateDim> ESEKF::buildH() const {
        Eigen::Matrix<double, 6, kStateDim> H = Eigen::Matrix<double, 6, kStateDim>::Zero();

        // Rotation residual (rows 0:3) depend on rotation error (cols 0:3)
        H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

        // Position residual (rows 3:6) depend on position error (cols 3:6)
        H.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

        // Velocity and biases unobservable by pose measurement.

        return H;
    }

    // ====================
    // R MATRIX
    // ====================
    // 6x6 measurement noise covariance
    // LiDAR and visual odom have different noise characs
    Eigen::Matrix<double, 6, 6> ESEKF::buildR(const OdomMeasurement& meas) const {
        if(meas.cov.norm() > 1e-9) return meas.cov;

        // Fallback defaults by source
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();

        if(meas.source == OdomMeasurement::Source::LIDAR){
            // LiDAR: low position noise, moderate rotation noise
            R.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 1e-4; // rotation [rad^2]
            R.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 1e-3; // position [m^2]
        }
        else{
            // Visual: moderate noise depending on scene texture
            R.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 1e-3; // rotation [rad^2]
            R.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 1e-2; // position [m^2]
        }

        return R;
    }

    // ====================
    // APPLY CORRECTION
    // ====================
    // Boxplus on SE(3)
    void ESEKF::applyCorrection(const Vector15d& dx){
        // Rotation Correction: R = R . Exp(del_theta)
        state_.T.so3() = state_.T.so3() * Sophus::SO3d::exp(dx.head<3>());

        // Position: plain addition in global frame
        state_.T.translation() += dx.segment<3>(3);

        // Velocity Correction
        state_.v += dx.segment<3>(6);

        // Bias Corrections
        state_.ba += dx.segment<3>(9);
        state_.bg += dx.segment<3>(12);
    }

    // ====================
    // ACCESSORS
    // ====================
    State ESEKF::getState() const {
        return state_;
    }

    Covariance ESEKF::getCov() const {
        return P_;
    }

    bool ESEKF::isInitialized() const {
        return initialized_;
    }

} // namespace helios