#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "helios_estimator/esekf.hpp"
#include "helios_estimator/state.hpp"

namespace helios{

    using namespace std::chrono_literals;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class EstimatorNode : public rclcpp_lifecycle::LifecycleNode{
        public:
            explicit EstimatorNode(const rclcpp::NodeOptions& options)
            : rclcpp_lifecycle::LifecycleNode("estimator_node", options){
                imu_cb_group_ = create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive
                );
                odom_cb_group_ = create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive
                );

                RCLCPP_INFO(get_logger(), "Estimator Node constructed.");
            }

            // ====================
            // LIFECYCLE
            // ====================
            CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
                RCLCPP_INFO(get_logger(), "Configuring...");

                // Parameters
                declare_parameter<double>("acc_noise", 3.9939e-3);
                declare_parameter<double>("gyro_noise", 1.5642e-3);
                declare_parameter<double>("acc_bias_noise", 6.4886e-5);
                declare_parameter<double>("gyro_bias_noise", 9.9703e-6);
                declare_parameter<double>("init_pos_cov", 1e-4);
                declare_parameter<double>("init_rot_cov", 1e-4);
                declare_parameter<double>("init_vel_cov", 1e-4);
                declare_parameter<double>("init_ba_cov", 1e-6);
                declare_parameter<double>("init_bg_cov", 1e-8);
                declare_parameter<std::string>("odom_frame", "odom");
                declare_parameter<std::string>("base_frame", "base_link");

                NoiseParams params;
                params.acc_noise = get_parameter("acc_noise").as_double();
                params.gyro_noise = get_parameter("gyro_noise").as_double();
                params.acc_bias_noise = get_parameter("acc_bias_noise").as_double();
                params.gyro_bias_noise = get_parameter("gyro_bias_noise").as_double();
                params.init_pos_cov = get_parameter("init_pos_cov").as_double();
                params.init_rot_cov = get_parameter("init_rot_cov").as_double();
                params.init_vel_cov = get_parameter("init_vel_cov").as_double();
                params.init_ba_cov = get_parameter("init_ba_cov").as_double();
                params.init_bg_cov = get_parameter("init_bg_cov").as_double();

                odom_frame_ = get_parameter("odom_frame").as_string();
                base_frame_ = get_parameter("base_frame").as_string();

                esekf_ = std::make_unique<ESEKF>(params);

                // QoS
                rclcpp::QoS sensor_qos(10);
                sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
                sensor_qos.durability(rclcpp::DurabilityPolicy::Volatile);

                rclcpp::QoS reliable_qos(10);
                reliable_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

                // Publishers
                odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
                    "helios/odometry", reliable_qos
                );

                // TF Broadcaster
                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

                RCLCPP_INFO(get_logger(), "Configuration complete.");
                return CallbackReturn::SUCCESS;
            }

            CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
                RCLCPP_INFO(get_logger(), "Activating...");

                odom_pub_->on_activate();

                // QoS
                rclcpp::QoS sensor_qos(10);
                sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
                sensor_qos.durability(rclcpp::DurabilityPolicy::Volatile);

                rclcpp::QoS reliable_qos(10);
                reliable_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

                // IMU Subscriber - 200Hz, highest priority
                rclcpp::SubscriptionOptions imu_opts;
                imu_opts.callback_group = imu_cb_group_;
                imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                    "/helios/imu/data", sensor_qos,
                    [this](const sensor_msgs::msg::Imu::SharedPtr msg){
                        imuCallback(msg);
                    }, imu_opts
                );

                // LiDAR Odometry Subscriber - from LIO-SAM
                rclcpp::SubscriptionOptions odom_opts;
                odom_opts.callback_group = odom_cb_group_;
                lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    "helios/lidar_odom", reliable_qos,
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                        lidarOdomCallback(msg);
                    }, odom_opts
                );

                // Visual Odometry Subscriber - from OpenVINS
                visual_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                    "helios/visual_odom", reliable_qos,
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg){
                        visualOdomCallback(msg);
                    }, odom_opts
                );

                first_imu_received_ = false;

                RCLCPP_INFO(get_logger(), "Activated. Waiting for first odometry...");
                return CallbackReturn::SUCCESS;
            }

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
                RCLCPP_INFO(get_logger(), "Deactivating...");

                imu_sub_.reset();
                lidar_odom_sub_.reset();
                visual_odom_sub_.reset();
                odom_pub_->on_deactivate();

                return CallbackReturn::SUCCESS;
            }

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
                RCLCPP_INFO(get_logger(), "Cleaning up...");
                esekf_->reset();
                odom_pub_.reset();
                tf_broadcaster_.reset();
                return CallbackReturn::SUCCESS;
            }

            CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
                RCLCPP_INFO(get_logger(), "Shutting down.");
                return CallbackReturn::SUCCESS;
            }

        private:
            
            // ====================
            // IMU CALLBACK - PREDICT STEP
            // ====================
            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
                std::lock_guard<std::mutex> lock(node_mtx_);

                if(!esekf_->isInitialized()) return;

                const rclcpp::Time current_time(msg->header.stamp);
                
                // Bootstrap: initialise last_imu_time_ from the first actual message
                // This guarantees last_imu_time_ and current_time are on the same clock
                if (!first_imu_received_) {
                    last_imu_time_     = current_time;
                    first_imu_received_ = true;
                    return;   // skip predict on first message — dt would be zero anyway
                }

                const double dt = (current_time - last_imu_time_).seconds();
                last_imu_time_  = current_time;

                // Sanity check dt - reject if too large or negative
                if(dt <= 0.0 || dt > 0.1){
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "IMU dt out of range: %.4f s - skipping predict.", dt);
                    return;
                }

                ImuMeasurement imu;
                imu.accel = Eigen::Vector3d(
                    msg->linear_acceleration.x,
                    msg->linear_acceleration.y,
                    msg->linear_acceleration.z
                );
                imu.gyro = Eigen::Vector3d(
                    msg->angular_velocity.x,
                    msg->angular_velocity.y,
                    msg->angular_velocity.z
                );
                imu.dt = dt;

                esekf_->predict(imu);

                // Buffer IMU measurement and resulting state for retropropagation
                imu_buffer_.push_back({current_time, imu});
                state_buffer_.push_back({current_time, esekf_->getState(), esekf_->getCov()});

                // Trim old entried beyond buffer duration
                while(!imu_buffer_.empty() &&
                      (current_time - imu_buffer_.front().stamp).seconds() > kBufferDuration){
                        imu_buffer_.pop_front();
                }

                while(!state_buffer_.empty() && 
                      (current_time - state_buffer_.front().stamp).seconds() > kBufferDuration){
                        state_buffer_.pop_front();
                }

                // Hard Cap - Size
                if(imu_buffer_.size() > kMaxBufferSize) imu_buffer_.pop_front();
                if(state_buffer_.size() > kMaxBufferSize) state_buffer_.pop_front();

                publishState(msg->header.stamp);
            }

            void retropropagateAndUpdate(
                const OdomMeasurement& meas,
                const rclcpp::Time& meas_stamp
            ){
                if(state_buffer_.empty()){
                    // No buffer yet - fallback to direct update
                    esekf_->update(meas);
                    return;
                }

                if (meas_stamp < state_buffer_.front().stamp) {
                    // Measurement too old.
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Odometry measurement beyond buffer history. Direct Update.");
                    esekf_->update(meas);
                    return;
                }

                // 1. Find closest state in buffer at or before meas_stamp
                auto it = state_buffer_.begin();
                for(auto jt = state_buffer_.begin(); jt != state_buffer_.end(); ++jt){
                    if(jt->stamp <= meas_stamp) it = jt;
                    else break;
                }

                // 2. Roll back ESEKF to state at meas_stamp
                esekf_->setState(it->state, it->cov);

                // 3. Apply update at the historical state
                const auto result = esekf_->update(meas);
                if(!result.has_value()){
                    // Rejected - restore current state (last in buffer)
                    const auto& latest = state_buffer_.back();
                    esekf_->setState(latest.state, latest.cov);
                    return;
                }

                // Save new, corrected historical anchor
                it->state = esekf_->getState();
                it->cov = esekf_->getCov();

                // 4. Repropagate forward using buffered IMU from meas_stamp -> now and overwrite the state buffer
                // Find IMU measurements after meas_stamp
                auto next_it = std::next(it);

                // it->stamp is the anchor timestamp — intentionally unchanged.
                // We overwrite state/cov but keep the timestamp as the repropagation start marker.
                for(const auto& stamped_imu : imu_buffer_){
                    if(stamped_imu.stamp <= it->stamp) continue;  // skip old entries

                    esekf_->predict(stamped_imu.meas);

                    if(next_it != state_buffer_.end()){
                        next_it->state = esekf_->getState();
                        next_it->cov   = esekf_->getCov();
                        ++next_it;
                    }
                }
            }

            // ====================
            // LIDAR ODOMETRY CALLBACK - UPDATE STEP
            // ====================
            void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
                std::lock_guard<std::mutex> lock(node_mtx_);

                OdomMeasurement meas = odomMsgToMeasurement(msg, OdomMeasurement::Source::LIDAR);

                if(!esekf_->isInitialized()) {
                    RCLCPP_INFO(get_logger(), "Initializing ESEKF from 1st LiDAR odometry.");
                    esekf_->initialize(meas);
                    return;
                }

                const rclcpp::Time meas_stamp(msg->header.stamp);
                retropropagateAndUpdate(meas, meas_stamp);
            }

            // ====================
            // VISUAL ODOMETRY CALLBACK - UPDATE STEP
            // ====================
            void visualOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
                std::lock_guard<std::mutex> lock(node_mtx_);

                if(!esekf_->isInitialized()) return;

                OdomMeasurement meas = odomMsgToMeasurement(msg, OdomMeasurement::Source::VISUAL);
                
                const rclcpp::Time meas_stamp(msg->header.stamp);
                retropropagateAndUpdate(meas, meas_stamp);
            }

            // ====================
            // PUBLISH FUSED ODOM + TF
            // ====================
            void publishState(const rclcpp::Time& stamp){
                const State state = esekf_->getState();
                const Covariance P = esekf_->getCov();

                // ====================
                // Odometry Message
                // ====================
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = stamp;
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = base_frame_;

                // Pose
                const Eigen::Quaterniond q(state.T.so3().matrix());
                odom_msg.pose.pose.orientation.w = q.w();
                odom_msg.pose.pose.orientation.x = q.x();
                odom_msg.pose.pose.orientation.y = q.y();
                odom_msg.pose.pose.orientation.z = q.z();
                odom_msg.pose.pose.position.x = state.T.translation().x();
                odom_msg.pose.pose.position.y = state.T.translation().y();
                odom_msg.pose.pose.position.z = state.T.translation().z();

                // Pose Cov - upper-left 6x6 block of P (rotation + position)
                for(int i = 0; i < 6; ++i){
                    for(int j = 0; j < 6; ++j){
                        odom_msg.pose.covariance[i * 6 + j] = P(i,j);
                    }
                }

                // Velocity
                odom_msg.twist.twist.linear.x = state.v.x();
                odom_msg.twist.twist.linear.y = state.v.y();
                odom_msg.twist.twist.linear.z = state.v.z();

                // Velocity Cov - rows/cols 6:9 of P
                for(int i = 0; i < 3; ++i){
                    for(int j = 0; j < 3; ++j){
                        odom_msg.twist.covariance[i * 6 + j] = P(i + 6, j + 6);
                    }
                }

                odom_pub_->publish(odom_msg);

                // ====================
                // TF : odom -> base_link
                // ====================
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = stamp;
                tf_msg.header.frame_id = odom_frame_;
                tf_msg.child_frame_id = base_frame_;

                tf_msg.transform.translation.x = state.T.translation().x();
                tf_msg.transform.translation.y = state.T.translation().y();
                tf_msg.transform.translation.z = state.T.translation().z();
                tf_msg.transform.rotation.w = q.w();
                tf_msg.transform.rotation.x = q.x();
                tf_msg.transform.rotation.y = q.y();
                tf_msg.transform.rotation.z = q.z();

                tf_broadcaster_->sendTransform(tf_msg);
            }

            // ====================
            // CONVERT nav_msgs/Odometry to OdomMeasurement
            // ====================
            OdomMeasurement odomMsgToMeasurement(
                const nav_msgs::msg::Odometry::SharedPtr& msg,
                OdomMeasurement::Source source
            ) const {
                OdomMeasurement meas;
                meas.source = source;

                const auto& p = msg->pose.pose.position;
                const auto& o = msg->pose.pose.orientation;

                const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
                meas.pose = Sophus::SE3d(
                    Sophus::SO3d(q.normalized()),
                    Eigen::Vector3d(p.x, p.y, p.z)
                );

                // Extract 6x6 pose cov from message
                for(int i = 0; i < 6; ++i){
                    for(int j = 0; j < 6; ++j){
                        meas.cov(i,j) = msg->pose.covariance[i * 6 + j];
                    }
                }

                return meas;
            }

            // ====================
            // MEMBERS
            // ====================
            std::unique_ptr<ESEKF> esekf_;

            // Publishers
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            
            // Subscribers
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_odom_sub_;

            // TF
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            // Callback groups
            rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
            rclcpp::CallbackGroup::SharedPtr odom_cb_group_;

            // State
            rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
            bool first_imu_received_{false};

            // Frame IDs
            std::string odom_frame_{"odom"};
            std::string base_frame_{"base_link"};

            // IMU + State buffers for retropropagation
            struct StampedImu{
                rclcpp::Time stamp;
                ImuMeasurement meas;
            };

            struct StampedState{
                rclcpp::Time stamp;
                State state;
                Covariance cov;
            };

            std::deque<StampedImu> imu_buffer_; // rolling IMU window
            std::deque<StampedState> state_buffer_; // state at each IMU step
            mutable std::mutex node_mtx_; // protect both buffers

            static constexpr double kBufferDuration = 0.5; // secs to keep
            static constexpr size_t kMaxBufferSize = 200; // ~1s at 200Hz
    };

} // namespace helios

RCLCPP_COMPONENTS_REGISTER_NODE(helios::EstimatorNode)