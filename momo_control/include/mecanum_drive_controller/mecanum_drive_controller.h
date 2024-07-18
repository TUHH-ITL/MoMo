#pragma once

#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include "mecanum_drive_controller/odometry.h"
#include "mecanum_drive_controller/speed_limiter.h"
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>

namespace mecanum_drive_controller
{

  class MecanumDriveController : public controller_interface::Controller<
                                     hardware_interface::VelocityJointInterface>
  {
  public:
    MecanumDriveController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::VelocityJointInterface *hw,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new
     * velocity commands \param time   Current time \param period Time since the
     * last called to update
     */
    void update(const ros::Time &time, const ros::Duration &period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time &time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time & /*time*/);

  private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;

    std::vector<hardware_interface::JointHandle> left_wheel_joints_;
    std::vector<hardware_interface::JointHandle> right_wheel_joints_;

    ros::Time time_previous_;

    std::vector<double> vel_left_previous_;
    std::vector<double> vel_right_previous_;

    std::vector<double> vel_left_desired_previous_;
    std::vector<double> vel_right_desired_previous_;

    struct VelocityCommand
    {
      double lin_x;
      double lin_y;
      double ang;
      ros::Time stamp;

      VelocityCommand() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<VelocityCommand> command_;
    VelocityCommand command_struct_;
    ros::Subscriber sub_command_;

    /// Publish executed commands
    std::shared_ptr<
        realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>>
        cmd_vel_pub_;

    /// Odometry related:
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>
        odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>>
        tf_odom_pub_;
    Odometry odometry_;

    /// Controller state publisher
    std::shared_ptr<realtime_tools::RealtimePublisher<
        control_msgs::JointTrajectoryControllerState>>
        controller_state_pub_;

    double wheel_separation_;
    double wheel_distance_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    double wheel_separation_multiplier_;
    double wheel_distance_multiplier_;
    double left_front_wheel_radius_multiplier_;
    double right_front_wheel_radius_multiplier_;
    double left_rear_wheel_radius_multiplier_;
    double right_rear_wheel_radius_multiplier_;

    double cmd_vel_timeout_;
    bool allow_multiple_cmd_vel_publishers_;

    std::string base_frame_id_;
    std::string odom_frame_id_;

    bool enable_odom_tf_;

    /// Number of wheel joints:
    size_t wheel_joints_size_;

    /// Speed limiters:
    VelocityCommand last1_cmd_;
    VelocityCommand last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

    bool publish_velocity_cmd_;
    bool publish_wheel_joint_controller_state_;

  private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist &command);

    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the whel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelNames(ros::NodeHandle &controller_nh,
                       const std::string &wheel_param,
                       std::vector<std::string> &wheel_names);

    /**
     * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and
     * separation \param root_nh Root node handle \param left_wheel_name Name of
     * the left wheel joint \param right_wheel_name Name of the right wheel joint
     */
    bool setOdomParamsFromUrdf(ros::NodeHandle &root_nh,
                               const std::string &left_front_wheel_name,
                               const std::string &left_rear_wheel_name,
                               const std::string &right_front_wheel_name,
                               const std::string &right_rear_wheel_name,
                               bool lookup_wheel_separation,
                               bool lookup_wheel_distance,
                               bool lookup_wheel_radius);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle &root_nh,
                          ros::NodeHandle &controller_nh);

    void publishWheelData(const ros::Time &time, const ros::Duration &period,
                          const std::vector<double> vel_left_desired,
                          const std::vector<double> right_left_desired);
  };

  PLUGINLIB_EXPORT_CLASS(mecanum_drive_controller::MecanumDriveController,
                         controller_interface::ControllerBase);
} // namespace mecanum_drive_controller
