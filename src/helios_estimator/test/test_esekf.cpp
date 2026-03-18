#include <gtest/gtest.h>

#include "helios_estimator/esekf.hpp"

namespace helios{
    
    class ESEKFTest : public ::testing::Test{
        protected:
            void SetUp() override{
                NoiseParams params;
                // Low noise for deterministic tests
                params.acc_noise = 1e-4;
                params.gyro_noise = 1e-5;
                params.acc_bias_noise = 1e-6;
                params.gyro_bias_noise = 1e-7;

                esekf_ = std::make_unique<ESEKF>(params);

                // Initialize at origin
                OdomMeasurement init_meas;
                init_meas.pose = Sophus::SE3d();
                init_meas.cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
                init_meas.source = OdomMeasurement::Source::LIDAR;
                esekf_->initialize(init_meas);
            }

            std::unique_ptr<ESEKF> esekf_;
    };

    // ====================
    // PREDICT TESTS
    // ====================
    TEST_F(ESEKFTest, IsInitializedAfterInit){
        EXPECT_TRUE(esekf_->isInitialized());
    }

    TEST_F(ESEKFTest, PredictZeroAngularVelocity_RotationUnchanged){
        // With zero angular velocity, rotation should not change
        ImuMeasurement imu;
        imu.accel = Eigen::Vector3d(0.0, 0.0, 9.80511); // gravity compensation
        imu.gyro = Eigen::Vector3d::Zero();
        imu.dt = 0.005;

        const Sophus::SO3d R_before = esekf_->getState().T.so3();
        esekf_->predict(imu);
        const Sophus::SO3d R_after = esekf_->getState().T.so3();

        const double rot_diff = (R_before.matrix() - R_after.matrix()).norm();
        EXPECT_LT(rot_diff, 1e-9);
    }

    TEST_F(ESEKFTest, PredictWithAngularVelocity_RotationChanges){
        ImuMeasurement imu;
        imu.accel = Eigen::Vector3d(0.0, 0.0, 9.80511);
        imu.gyro  = Eigen::Vector3d(0.0, 0.0, 0.1);  // 0.1 rad/s yaw rate
        imu.dt    = 0.1;

        const Sophus::SO3d R_before = esekf_->getState().T.so3();
        esekf_->predict(imu);
        const Sophus::SO3d R_after = esekf_->getState().T.so3();

        const double rot_diff = (R_before.matrix() - R_after.matrix()).norm();
        EXPECT_GT(rot_diff, 1e-6);  // rotation must have changed
    }

    TEST_F(ESEKFTest, PredictCovarianceGrows)
    {
        // Covariance should increase with each predict step (no updates)
        const double trace_before = esekf_->getCov().trace();

        ImuMeasurement imu;
        imu.accel = Eigen::Vector3d(0.0, 0.0, 9.80511);
        imu.gyro  = Eigen::Vector3d::Zero();
        imu.dt    = 0.005;

        for (int i = 0; i < 100; ++i) { esekf_->predict(imu); }

        const double trace_after = esekf_->getCov().trace();
        EXPECT_GT(trace_after, trace_before);
    }

    // ====================
    // UPDATE TESTS
    // ====================
    TEST_F(ESEKFTest, UpdateWithLiDAR_SmallCorrection_TightCovariance){
        // Measurement offset by 1m in X
        OdomMeasurement meas;
        meas.pose = Sophus::SE3d(
            Sophus::SO3d{},
            Eigen::Vector3d(0.02, 0.0, 0.0)
        );
        meas.cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
        meas.source = OdomMeasurement::Source::LIDAR;

        // Multiple updates should drive state toward meas
        for(int i = 0; i < 20; ++i){
            esekf_->update(meas);
        }

        const double px = esekf_->getState().T.translation().x();
        EXPECT_NEAR(px, 0.02, 0.005);
    }

    TEST_F(ESEKFTest, UpdateWithLiDAR_LargeCorrection_UncertainInit){
        // Inflate position covariance so Kalman gain is meaningful
        // In real operation you'd do this when initial position is uncertain
        OdomMeasurement init;
        init.pose       = Sophus::SE3d{};
        init.cov        = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
        init.source     = OdomMeasurement::Source::LIDAR;

        NoiseParams params;
        params.acc_noise       = 1e-4;
        params.gyro_noise      = 1e-5;
        params.init_pos_cov    = 1.0;   // high initial position uncertainty
        params.init_rot_cov    = 1e-4;
        params.init_vel_cov    = 1e-4;
        params.init_ba_cov     = 1e-6;
        params.init_bg_cov     = 1e-7;

        auto esekf_uncertain = std::make_unique<ESEKF>(params);
        esekf_uncertain->initialize(init);

        OdomMeasurement meas;
        meas.pose   = Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 0.0, 0.0));
        meas.cov    = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
        meas.source = OdomMeasurement::Source::LIDAR;

        for (int i = 0; i < 20; ++i) {
            esekf_uncertain->update(meas);
        }

        const double px = esekf_uncertain->getState().T.translation().x();
        EXPECT_NEAR(px, 1.0, 0.05);
    }

    TEST_F(ESEKFTest, UpdateCovarianceDecreases)
    {
        // After an update, covariance should decrease
        // First inflate covariance with predictions
        ImuMeasurement imu;
        imu.accel = Eigen::Vector3d(0.0, 0.0, 9.80511);
        imu.gyro  = Eigen::Vector3d::Zero();
        imu.dt    = 0.005;
        for (int i = 0; i < 50; ++i) { esekf_->predict(imu); }

        const double trace_before = esekf_->getCov().trace();

        OdomMeasurement meas;
        meas.pose       = Sophus::SE3d{};
        meas.cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
        meas.source     = OdomMeasurement::Source::LIDAR;
        esekf_->update(meas);

        const double trace_after = esekf_->getCov().trace();
        EXPECT_LT(trace_after, trace_before);
    }

    TEST_F(ESEKFTest, UpdateOutlierRejected_StateUnchanged)
    {
        // Measurement far from prediction should be rejected (Mahalanobis gate)
        const Sophus::SE3d T_before = esekf_->getState().T;

        OdomMeasurement outlier;
        outlier.pose = Sophus::SE3d(
            Sophus::SO3d{},
            Eigen::Vector3d(1000.0, 0.0, 0.0));   // 1 km away — definitely outlier
        outlier.cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-8;
        outlier.source     = OdomMeasurement::Source::LIDAR;

        const auto result = esekf_->update(outlier);
        EXPECT_FALSE(result.has_value());  // should be rejected

        const double pos_diff =
            (T_before.translation() - esekf_->getState().T.translation()).norm();
        EXPECT_LT(pos_diff, 1e-9);  // state unchanged
    }

    TEST_F(ESEKFTest, ResetClearsState)
    {
        esekf_->reset();
        EXPECT_FALSE(esekf_->isInitialized());
    }

} // namespace helios

int main(int argc, char** argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}