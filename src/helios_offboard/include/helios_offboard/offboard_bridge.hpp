#ifndef HELIOS_OFFBOARD__OFFBOARD_BRIDGE_HPP_
#define HELIOS_OFFBOARD__OFFBOARD_BRIDGE_HPP_

#include <chrono>
#include <memory>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace helios{

    class OffboardBridge : public rclcpp_lifecycle::LifecycleNode{
        public:
            explicit OffboardBridge(const rclcpp::NodeOptions& options);
            ~OffboardBridge() override = default;

            // Lifecycle transitions
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_configure(const rclcpp_lifecycle::State& state) override;

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State& state) override;

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State& state) override;

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_cleanup(const rclcpp_lifecycle::State& state) override;

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
            on_shutdown(const rclcpp_lifecycle::State& state) override;

        private:
            // ====================
            // Callbacks
            // ====================
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
            void vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

            // ====================
            // Control Loop
            // ====================
            void heartbeatTimerCallback();
            void startupSequenceCallback();

            // ====================
            // PX4 Command Helpers
            // ====================
            void publishOffboardControlMode();
            void publishTrajectorySetpoint(
                float vx, float vy, float vz, float yaw_rate);
            void publishVehicleCommand(uint16_t command,
                float param1 = 0.0f, float param2 = 0.0f);
            void arm();
            void disarm();
            void engageOffboardMode();

            // ====================
            // Altitude Hold
            // ====================
            float computeAltitudeHold() const;

            // ====================
            // Publishers (Lifecycle-Managed)
            // ====================
            rclcpp_lifecycle::LifecyclePublisher<
                px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
            rclcpp_lifecycle::LifecyclePublisher<
                px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
            rclcpp_lifecycle::LifecyclePublisher<
                px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
            
            // ====================
            // Subsribers
            // ====================
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
            rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

            // ====================
            // Timers
            // ====================
            rclcpp::TimerBase::SharedPtr heartbeat_timer_; // 10 Hz - offboard keepalive
            rclcpp::TimerBase::SharedPtr startup_timer_; // startup sequencing

            // ====================
            // Callback Groups
            // ====================
            rclcpp::CallbackGroup::SharedPtr status_cb_group_;
            rclcpp::CallbackGroup::SharedPtr cmd_vel_cb_group_;

            // ====================
            // State (Mutex-protected where written from multiple callbacks)
            // ====================
            mutable std::mutex state_mutex_;
            geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
            px4_msgs::msg::VehicleStatus latest_status_;
            px4_msgs::msg::VehicleOdometry latest_odom_;
            rclcpp::Time last_cmd_vel_time_;

            // ====================
            // Startup Sequence State
            // ====================
            enum class StartupState{
                WAITING_FOR_STATUS,
                SENDING_OFFBOARD,
                ARMING,
                TAKING_OFF,
                CRUISING
            };
            std::atomic<StartupState> startup_state_{StartupState::WAITING_FOR_STATUS};
            int startup_counter_{0};

            // ====================
            // Parameters
            // ====================
            double cruise_altitude_{3.0};   // metres, positive up (NED: negative z)
            double max_velocity_xy_{2.0};   // m/s
            double max_velocity_z_{1.0};    // m/s
            double altitude_kp_{0.5};       // altitude hold P gain
            double cmd_vel_timeout_{2.0};   // seconds before hover on silence

            // ====================
            // Consts
            // ====================
            static constexpr uint8_t kSystemID_ = 1;
            static constexpr uint8_t kComponentID_ = 1; 
    };

} // namespace helios

#endif