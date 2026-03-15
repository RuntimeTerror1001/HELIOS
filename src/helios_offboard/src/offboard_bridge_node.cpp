#include "helios_offboard/offboard_bridge.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp_components/register_node_macro.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace helios {

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using namespace std::chrono_literals;

  // ====================
  // CONSTRUCTOR
  // ====================
  OffboardBridge::OffboardBridge(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("offboard_bridge", options){
    // Callback groups - status and cmd_vel can be processed concurrently
    status_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_vel_cb_group_ =  create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(get_logger(), "OffboardBridge Constructed.");
  }

  // ====================
  // LIFECYCLE
  // ====================

  // on_configure
  CallbackReturn OffboardBridge::on_configure(const rclcpp_lifecycle::State&){
    RCLCPP_INFO(get_logger(), "Configuring...");

    // Declare Parameters with descriptors
    declare_parameter<double>("cruise_altitude", 3.0);
    declare_parameter<double>("max_velocity_xy", 2.0);
    declare_parameter<double>("max_velocity_z", 1.0);
    declare_parameter<double>("altitude_kp", 0.5);
    declare_parameter<double>("cmd_vel_timeout", 2.0);

    cruise_altitude_  = get_parameter("cruise_altitude").as_double();
    max_velocity_xy_  = get_parameter("max_velocity_xy").as_double();
    max_velocity_z_   = get_parameter("max_velocity_z").as_double();
    altitude_kp_      = get_parameter("altitude_kp").as_double();
    cmd_vel_timeout_  = get_parameter("cmd_vel_timeout").as_double();

    RCLCPP_INFO(get_logger(),
      "Params: cruise_alt=%.1f m, max_vxy=%.1f m/s, alt_kp=%.2f",
      cruise_altitude_, max_velocity_xy_, altitude_kp_);

    // QoS for PX4 Topics - BestEffort as required by uXRCE-DDS
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Publishers
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos);

    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos);

    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos);

    RCLCPP_INFO(get_logger(), "Configuration Complete.");
    return CallbackReturn::SUCCESS;
  }

  // on_activate
  CallbackReturn OffboardBridge::on_activate(const rclcpp_lifecycle::State&){
    RCLCPP_INFO(get_logger(), "Activating...");

    offboard_mode_pub_->on_activate();
    trajectory_pub_->on_activate();
    vehicle_cmd_pub_->on_activate();

    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Subscribers with callback groups
    rclcpp::SubscriptionOptions status_opts;
    status_opts.callback_group = status_cb_group_;
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg){
        vehicleStatusCallback(msg);
      }, status_opts
    );

    rclcpp::SubscriptionOptions odom_opts;
    odom_opts.callback_group = status_cb_group_;
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
        vehicleOdomCallback(msg);
      }, odom_opts
    );

    rclcpp::SubscriptionOptions cmd_vel_opts;
    cmd_vel_opts.callback_group = cmd_vel_cb_group_;
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/helios/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        cmdVelCallback(msg);
      }, cmd_vel_opts
    );

    // Heartbeat Timer - 10 Hz, keeps PX4 in offboard mode
    heartbeat_timer_ = create_wall_timer(100ms,
      [this](){ heartbeatTimerCallback(); });

    // Startup Sequencer - runs at 5 Hz
    startup_state_ = StartupState::WAITING_FOR_STATUS;
    startup_counter_ = 0;
    last_cmd_vel_time_ = now();

    startup_timer_ = create_wall_timer(200ms,
      [this](){ startupSequenceCallback(); });

    RCLCPP_INFO(get_logger(), "Activated. Starting Startup Sequence...");
    return CallbackReturn::SUCCESS;
  }

  // on_deactivate
  CallbackReturn OffboardBridge::on_deactivate(const rclcpp_lifecycle::State&){
    RCLCPP_INFO(get_logger(), "Deactivating...");

    heartbeat_timer_.reset();
    startup_timer_.reset();
    cmd_vel_sub_.reset();
    status_sub_.reset();
    odom_sub_.reset();

    offboard_mode_pub_->on_deactivate();
    trajectory_pub_->on_deactivate();
    vehicle_cmd_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
  }

  // on_cleanup
  CallbackReturn OffboardBridge::on_cleanup(const rclcpp_lifecycle::State&){
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    offboard_mode_pub_.reset();
    trajectory_pub_.reset();
    vehicle_cmd_pub_.reset();

    return CallbackReturn::SUCCESS;
  }

  // on_shutdown
  CallbackReturn OffboardBridge::on_shutdown(const rclcpp_lifecycle::State&){
    RCLCPP_INFO(get_logger(), "Shutting down.");
    return CallbackReturn::SUCCESS;
  }

  // ====================
  // CALLBACKS
  // ====================
  void OffboardBridge::vehicleStatusCallback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg){
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_status_ = *msg;
    }

  void OffboardBridge::vehicleOdomCallback(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_odom_ = *msg;
    }

  void OffboardBridge::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg){
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_cmd_vel_ = msg;
      last_cmd_vel_time_ = now();
    }
    
  // ====================
  // TIMER CALLBACKS
  // ====================

  // Heartbeat Timer (10 Hz)
  void OffboardBridge::heartbeatTimerCallback(){
    // Must publish OffboardControlMode at >2 Hz or PX4 exits offboard
    publishOffboardControlMode();

    // Only send setpoints if drone is in CRUISING state
    if(startup_state_ != StartupState::CRUISING) return;

    // Check cmd_vel timeout - hover in place if silent
    const bool cmd_vel_fresh = (now() - last_cmd_vel_time_).seconds() < cmd_vel_timeout_;

    float vx = 0.0f, vy = 0.0f, yaw_rate = 0.0f;

    if(cmd_vel_fresh && latest_cmd_vel_){
      std::lock_guard<std::mutex> lock(state_mutex_);

      // Clamp to safety limits
      vx = std::clamp(
        static_cast<float>(latest_cmd_vel_->linear.x),
        -static_cast<float>(max_velocity_xy_),
        static_cast<float>(max_velocity_xy_)
      );

      vy = std::clamp(
        static_cast<float>(latest_cmd_vel_->linear.y),
        -static_cast<float>(max_velocity_xy_),
        static_cast<float>(max_velocity_xy_)
      );

      yaw_rate = static_cast<float>(latest_cmd_vel_->angular.z);
    }

    else if(!cmd_vel_fresh && startup_state_ == StartupState::CRUISING){
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "cmd_vel timeout - hovering in place.");
    }

    const float vz = computeAltitudeHold();
    publishTrajectorySetpoint(vx, vy, vz, yaw_rate);
  }

  // Startup Sequence (5 Hz)
  void OffboardBridge::startupSequenceCallback(){
    px4_msgs::msg::VehicleStatus status;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status = latest_status_;
    }

    switch (startup_state_){
      case StartupState::WAITING_FOR_STATUS:
        if(status.pre_flight_checks_pass){
          RCLCPP_INFO(get_logger(),
            "Preflight checks pass. Sending offboard mode...");
          
          startup_state_ = StartupState::SENDING_OFFBOARD;
          startup_counter_ = 0;
        }
        else{
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Waiting for preflight checks...");
        }
        break;

      case StartupState::SENDING_OFFBOARD:
        // Must stream setpoints before PX4 accepts offboard mode
        publishTrajectorySetpoint(0.0f, 0.0f, 0.0f, 0.0f);
        startup_counter_++;
        if(startup_counter_ >= 10) { // ~2 seconds of streaming
          engageOffboardMode();
          startup_state_= StartupState::ARMING;
          startup_counter_ = 0;
          RCLCPP_INFO(get_logger(), "Offboard Mode requested. Arming...");
        }
        break;

      case StartupState::ARMING:
        arm();
        startup_counter_++;
        if(status.arming_state == 2) { // 2 = Armed
          RCLCPP_INFO(get_logger(), "Armed! Taking off to %.1fm...", cruise_altitude_);
          startup_state_ = StartupState::TAKING_OFF;
          startup_counter_ = 0;
        }
        else if(startup_counter_ > 25){
          RCLCPP_ERROR(get_logger(),
            "Failed to arm after 5 seconds. Check preflight");
          startup_state_ = StartupState::WAITING_FOR_STATUS;
          startup_counter_ = 0;
        }
        break;

      case StartupState::TAKING_OFF: {
        // Command climb velocity until cruise altitude
        // PX4 NED Frame: z(down) -> cruise_altitude = -cruise_altitude_
        float current_z = 0.0f;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          current_z = latest_odom_.position[2];
        }
        const float target_z = -static_cast<float>(cruise_altitude_);
        const float error_z = target_z - current_z;
        
        if(std::abs(error_z) < 0.3f){
          RCLCPP_INFO(get_logger(),
            "Reached cruise altitude %.1fm. Helios is ready.", cruise_altitude_);
          startup_state_ = StartupState::CRUISING;
          startup_timer_->cancel();
        }
        else{
          // Climb at up to max_velocity_z_
          const float vz = std::clamp(
            error_z * static_cast<float>(altitude_kp_),
            -static_cast<float>(max_velocity_z_),
            static_cast<float>(max_velocity_z_)
          );
          publishTrajectorySetpoint(0.0f, 0.0f, vz, 0.0f);
        }
        break;
      }

      case StartupState::CRUISING:
        // Handled by heartbeat timer
        startup_timer_->cancel();
        break;
    }
  }

  // ====================
  // ALTITUDE HOLD
  // ====================
  float OffboardBridge::computeAltitudeHold() const {
    float curr_z = 0.0f;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      curr_z = latest_odom_.position[2];
    }

    const float target_z = -static_cast<float>(cruise_altitude_);
    const float error_z = target_z - curr_z;

    return std::clamp(
      error_z * static_cast<float>(altitude_kp_),
      -static_cast<float>(max_velocity_z_),
      static_cast<float>(max_velocity_z_)
    );
  }

  // ====================
  // PX4 PUBLISH HELPERS
  // ====================
  void OffboardBridge::publishOffboardControlMode(){
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = true; // Velocity Control Mode
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboard_mode_pub_->publish(msg);
  }

  void OffboardBridge::publishTrajectorySetpoint(float vx, float vy, float vz, float yaw_rate){
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = {NAN, NAN, NAN}; // not used in velocity mode
    msg.velocity = {vx, vy, vz};
    msg.yawspeed = yaw_rate;
    msg.yaw = NAN;
    trajectory_pub_->publish(msg);
  }

  void OffboardBridge::publishVehicleCommand(uint16_t command, float param1, float param2){
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = kSystemID_;
    msg.target_component = kComponentID_;
    msg.source_system = kSystemID_;
    msg.source_component = kComponentID_;
    msg.from_external = true;
    vehicle_cmd_pub_->publish(msg);
  }

  void OffboardBridge::arm(){
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0f
    );
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Arm command sent.");
  }

  void OffboardBridge::disarm(){
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      0.0f
    );
    RCLCPP_INFO(get_logger(), "Disarm command send.");
  }

  void OffboardBridge::engageOffboardMode(){
    // MAX_CMD_DO_SET_MODE: param1 = 1 (custom), param2 = 6 (offboard)
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0f, 6.0f
    );
    RCLCPP_INFO(get_logger(), "Offboard mode command sent.");
  }
} // namespace helios

RCLCPP_COMPONENTS_REGISTER_NODE(helios::OffboardBridge)