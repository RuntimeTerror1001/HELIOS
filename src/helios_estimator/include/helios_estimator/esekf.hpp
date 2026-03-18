#ifndef HELIOS_ESTIMATOR__ESEKF_HPP_
#define HELIOS_ESTIMATOR__ESEKF_HPP_

#include <optional>
#include <string>

#include "helios_estimator/state.hpp"

namespace helios{

    class ESEKF{
        public:
            explicit ESEKF(const NoiseParams& params);

            // ====================
            // PRIMARY  INTERFACE
            // ====================

            // Predict: propagate state forward using one IMU measurement
            // Called at IMU Rate (~200Hz)
            void predict(const ImuMeasurement& imu);

            // Update: correct state with an odom measurement
            // Returns updated state, or nullopt if measurement rejected
            std::optional<State> update(const OdomMeasurement& meas);

            // ====================
            // ACCESSORS
            // ====================
            State getState() const;
            Covariance getCov() const;
            bool isInitialized() const;

            // ====================
            // EXTERNAL
            // ====================

            // Initialize state from first odom measurement
            void initialize(const OdomMeasurement& meas);

            // Set state externally
            void setState(const State& state, const Covariance& cov);

            // Reset to uninitialized
            void reset();
            
        private:
            // ====================
            // PREDICT HELPERS
            // ====================

            // Build 15x15 error-state transition matrix F
            // F maps del_x_k -> del_x_k+1 linearly
            Matrix15d buildF(const Eigen::Vector3d& accel_corrected,
                             const Eigen::Vector3d& gyro_corrected,
                             double dt) const;
            
            // Build 15x12 noise input matrix G
            // Maps IMU noise vector to error-state noise
            Eigen::Matrix<double, kStateDim, 12> buildG(double dt) const;

            // ====================
            // UPDATE HELPERS
            // ====================

            // Build 6x15 measurement jacobian H
            // Maps error state del_x to pose residual del_z in SE(3)
            Eigen::Matrix<double, 6, kStateDim> buildH() const;

            // Build 6x6 measurement covariance R
            Eigen::Matrix<double, 6, 6> buildR(const OdomMeasurement& meas) const;

            // Apply error-state correction del_x to nominal state via boxplus on SE(3)
            void applyCorrection(const Vector15d& dx);
            
            // ====================
            // STATE
            // ====================
            State state_;
            Covariance P_{Covariance::Zero()};
            bool initialized_{false};

            // ====================
            // PROCESS NOISE COVARIANCE (12x12, built once in constructor)
            // ====================
            Eigen::Matrix<double, 12, 12> Q_;

            // ====================
            // GRAVITY VECTOR IN WORLD FRAME(ENU)
            // ====================
            const Eigen::Vector3d kGravity_{0.0, 0.0, -9.80511};

            // ====================
            // NOISE PARAMS
            // ====================
            NoiseParams params_;
    };

} // namespace helios

#endif