#include <cmath>
#include "mecanum_drive_controller/mecanum_drive_controller.h"
#include <tf/transform_datatypes.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

static double euclideanOfVectors(const urdf::Vector3 &vec1,
                                 const urdf::Vector3 &vec2)
{
  return std::sqrt(std::pow(vec1.x - vec2.x, 2) + std::pow(vec1.y - vec2.y, 2) +
                   std::pow(vec1.z - vec2.z, 2));
}

/*
 * \brief Check that a link exists and has a geometry collision.
 * \param link The link
 * \return true if the link has a collision element with geometry
 */
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr &link)
{
  if (!link)
  {
    ROS_ERROR("Link pointer is null.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name
                             << " does not have collision description. Add "
                                "collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link "
                     << link->name
                     << " does not have collision geometry description. Add "
                        "collision geometry description for link to urdf.");
    return false;
  }
  return true;
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr &link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_DEBUG_STREAM("Link " << link->name
                             << " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr &link)
{
  if (!hasCollisionGeometry(link))
  {
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::SPHERE)
  {
    ROS_DEBUG_STREAM("Link " << link->name << " does not have sphere geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr &wheel_link,
                           double &wheel_radius)
{
  if (isCylinder(wheel_link))
  {
    wheel_radius =
        (static_cast<urdf::Cylinder *>(wheel_link->collision->geometry.get()))
            ->radius;
    return true;
  }
  else if (isSphere(wheel_link))
  {
    wheel_radius =
        (static_cast<urdf::Sphere *>(wheel_link->collision->geometry.get()))
            ->radius;
    return true;
  }

  ROS_ERROR_STREAM("Wheel link " << wheel_link->name
                                 << " is NOT modeled as a cylinder or sphere!");
  return false;
}

namespace mecanum_drive_controller
{

  MecanumDriveController::MecanumDriveController()
      : open_loop_(false), command_struct_(), wheel_separation_(0.0),
        wheel_distance_(0.0), wheel_radius_(0.0),
        wheel_separation_multiplier_(1.0), wheel_distance_multiplier_(1.0),
        left_front_wheel_radius_multiplier_(1.0),
        right_front_wheel_radius_multiplier_(1.0),
        left_rear_wheel_radius_multiplier_(1.0),
        right_rear_wheel_radius_multiplier_(1.0), cmd_vel_timeout_(0.5),
        allow_multiple_cmd_vel_publishers_(true), base_frame_id_("base_link"),
        odom_frame_id_("odom"), enable_odom_tf_(true), wheel_joints_size_(0),
        publish_velocity_cmd_(false), publish_wheel_joint_controller_state_(false) {}

  bool MecanumDriveController::init(
      hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh,
      ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Get joint names from the parameter server
    std::vector<std::string> left_wheel_names, right_wheel_names;
    if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
        !getWheelNames(controller_nh, "right_wheel", right_wheel_names))
    {
      return false;
    }

    if (left_wheel_names.size() != right_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_, "#left wheels ("
                                        << left_wheel_names.size() << ") != "
                                        << "#right wheels ("
                                        << right_wheel_names.size() << ").");
      return false;
    }
    else
    {
      wheel_joints_size_ = left_wheel_names.size();

      left_wheel_joints_.resize(wheel_joints_size_);
      right_wheel_joints_.resize(wheel_joints_size_);
    }

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                                     << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    controller_nh.param("wheel_separation_multiplier",
                        wheel_separation_multiplier_,
                        wheel_separation_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by "
                                     << wheel_separation_multiplier_ << ".");
    controller_nh.param("wheel_separation_multiplier", wheel_distance_multiplier_,
                        wheel_distance_multiplier_);
    ROS_INFO_STREAM_NAMED(name_, "Wheel separation will be multiplied by "
                                     << wheel_distance_multiplier_ << ".");

    if (controller_nh.hasParam("wheel_radius_multiplier"))
    {
      double wheel_radius_multiplier;
      controller_nh.getParam("wheel_radius_multiplier", wheel_radius_multiplier);

      left_front_wheel_radius_multiplier_ = wheel_radius_multiplier;
      right_front_wheel_radius_multiplier_ = wheel_radius_multiplier;
      left_rear_wheel_radius_multiplier_ = wheel_radius_multiplier;
      right_rear_wheel_radius_multiplier_ = wheel_radius_multiplier;
    }
    else
    {
      controller_nh.param("left_front_wheel_radius_multiplier",
                          left_front_wheel_radius_multiplier_,
                          left_front_wheel_radius_multiplier_);
      controller_nh.param("right_front_wheel_radius_multiplier",
                          right_front_wheel_radius_multiplier_,
                          right_front_wheel_radius_multiplier_);
      controller_nh.param("left_rear_wheel_radius_multiplier",
                          left_rear_wheel_radius_multiplier_,
                          left_rear_wheel_radius_multiplier_);
      controller_nh.param("right_rear_wheel_radius_multiplier",
                          right_rear_wheel_radius_multiplier_,
                          right_rear_wheel_radius_multiplier_);
    }

    ROS_INFO_STREAM_NAMED(name_, "Left front wheel radius will be multiplied by "
                                     << left_front_wheel_radius_multiplier_
                                     << ".");
    ROS_INFO_STREAM_NAMED(name_, "Right front wheel radius will be multiplied by "
                                     << right_front_wheel_radius_multiplier_
                                     << ".");
    ROS_INFO_STREAM_NAMED(name_, "Left rear wheel radius will be multiplied by "
                                     << left_rear_wheel_radius_multiplier_
                                     << ".");
    ROS_INFO_STREAM_NAMED(name_, "Right rear wheel radius will be multiplied by "
                                     << right_rear_wheel_radius_multiplier_
                                     << ".");

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size",
                        velocity_rolling_window_size,
                        velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
                                     << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(
        name_, "Velocity commands will be considered old if they are older than "
                   << cmd_vel_timeout_ << "s.");

    controller_nh.param("allow_multiple_cmd_vel_publishers",
                        allow_multiple_cmd_vel_publishers_,
                        allow_multiple_cmd_vel_publishers_);
    ROS_INFO_STREAM_NAMED(
        name_,
        "Allow mutiple cmd_vel publishers is "
            << (allow_multiple_cmd_vel_publishers_ ? "enabled" : "disabled"));

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_,
                          "Publishing to tf is "
                              << (enable_odom_tf_ ? "enabled" : "disabled"));

    controller_nh.param("linear/x/has_velocity_limits",
                        limiter_lin_.has_velocity_limits,
                        limiter_lin_.has_velocity_limits);
    controller_nh.param("linear/x/has_acceleration_limits",
                        limiter_lin_.has_acceleration_limits,
                        limiter_lin_.has_acceleration_limits);
    controller_nh.param("linear/x/has_jerk_limits", limiter_lin_.has_jerk_limits,
                        limiter_lin_.has_jerk_limits);
    controller_nh.param("linear/x/max_velocity", limiter_lin_.max_velocity,
                        limiter_lin_.max_velocity);
    controller_nh.param("linear/x/min_velocity", limiter_lin_.min_velocity,
                        -limiter_lin_.max_velocity);
    controller_nh.param("linear/x/max_acceleration",
                        limiter_lin_.max_acceleration,
                        limiter_lin_.max_acceleration);
    controller_nh.param("linear/x/min_acceleration",
                        limiter_lin_.min_acceleration,
                        -limiter_lin_.max_acceleration);
    controller_nh.param("linear/x/max_jerk", limiter_lin_.max_jerk,
                        limiter_lin_.max_jerk);
    controller_nh.param("linear/x/min_jerk", limiter_lin_.min_jerk,
                        -limiter_lin_.max_jerk);

    controller_nh.param("angular/z/has_velocity_limits",
                        limiter_ang_.has_velocity_limits,
                        limiter_ang_.has_velocity_limits);
    controller_nh.param("angular/z/has_acceleration_limits",
                        limiter_ang_.has_acceleration_limits,
                        limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/has_jerk_limits", limiter_ang_.has_jerk_limits,
                        limiter_ang_.has_jerk_limits);
    controller_nh.param("angular/z/max_velocity", limiter_ang_.max_velocity,
                        limiter_ang_.max_velocity);
    controller_nh.param("angular/z/min_velocity", limiter_ang_.min_velocity,
                        -limiter_ang_.max_velocity);
    controller_nh.param("angular/z/max_acceleration",
                        limiter_ang_.max_acceleration,
                        limiter_ang_.max_acceleration);
    controller_nh.param("angular/z/min_acceleration",
                        limiter_ang_.min_acceleration,
                        -limiter_ang_.max_acceleration);
    controller_nh.param("angular/z/max_jerk", limiter_ang_.max_jerk,
                        limiter_ang_.max_jerk);
    controller_nh.param("angular/z/min_jerk", limiter_ang_.min_jerk,
                        -limiter_ang_.max_jerk);

    controller_nh.param("publish_velocity_cmd", publish_velocity_cmd_, publish_velocity_cmd_);

    controller_nh.param("publish_wheel_joint_controller_state",
                        publish_wheel_joint_controller_state_,
                        publish_wheel_joint_controller_state_);

    // If either parameter is not available, we need to look up the value in the
    // URDF
    bool lookup_wheel_separation =
        !controller_nh.getParam("wheel_separation", wheel_separation_);
    bool lookup_wheel_distance =
        !controller_nh.getParam("wheel_separation", wheel_distance_);
    bool lookup_wheel_radius =
        !controller_nh.getParam("wheel_radius", wheel_radius_);

    if (!setOdomParamsFromUrdf(root_nh, left_wheel_names[0], left_wheel_names[1],
                               right_wheel_names[0], right_wheel_names[1],
                               lookup_wheel_separation, lookup_wheel_distance,
                               lookup_wheel_radius))
    {
      return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    const double ws = wheel_separation_multiplier_ * wheel_separation_;
    const double wd = wheel_distance_multiplier_ * wheel_distance_;
    const double lfwr = left_front_wheel_radius_multiplier_ * wheel_radius_;
    const double rfwr = right_front_wheel_radius_multiplier_ * wheel_radius_;
    const double lrwr = left_rear_wheel_radius_multiplier_ * wheel_radius_;
    const double rrwr = right_rear_wheel_radius_multiplier_ * wheel_radius_;
    odometry_.setWheelParams(ws, wd, lfwr, lrwr, rfwr, rrwr);
    ROS_INFO_STREAM_NAMED(name_, "Odometry params : wheel separation "
                                     << ws << ", wheel distance " << wd
                                     << ", left front wheel radius " << lfwr
                                     << ", left rear wheel radius " << lrwr
                                     << ", right front wheel radius " << rfwr
                                     << ", right rear wheel radius " << rfwr);

    setOdomPubFields(root_nh, controller_nh);

    if (publish_velocity_cmd_)
    {
      cmd_vel_pub_.reset(
          new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(
              controller_nh, "cmd_vel_out", 100));
    }

    // Wheel joint controller state:
    if (publish_wheel_joint_controller_state_)
    {
      controller_state_pub_.reset(new realtime_tools::RealtimePublisher<
                                  control_msgs::JointTrajectoryControllerState>(
          controller_nh, "wheel_joint_controller_state", 100));

      const size_t num_wheels = wheel_joints_size_ * 2;

      controller_state_pub_->msg_.joint_names.resize(num_wheels);

      controller_state_pub_->msg_.desired.positions.resize(num_wheels);
      controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
      controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.desired.effort.resize(num_wheels);

      controller_state_pub_->msg_.actual.positions.resize(num_wheels);
      controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
      controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.actual.effort.resize(num_wheels);

      controller_state_pub_->msg_.error.positions.resize(num_wheels);
      controller_state_pub_->msg_.error.velocities.resize(num_wheels);
      controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
      controller_state_pub_->msg_.error.effort.resize(num_wheels);

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        controller_state_pub_->msg_.joint_names[i] = left_wheel_names[i];
        controller_state_pub_->msg_.joint_names[i + wheel_joints_size_] =
            right_wheel_names[i];
      }

      vel_left_previous_.resize(wheel_joints_size_, 0.0);
      vel_right_previous_.resize(wheel_joints_size_, 0.0);
      vel_left_desired_previous_.resize(wheel_joints_size_, 0.0);
      vel_right_desired_previous_.resize(wheel_joints_size_, 0.0);
    }

    // Get the joint object to use in the realtime loop
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      ROS_INFO_STREAM_NAMED(name_, "Adding left wheel with joint name: "
                                       << left_wheel_names[i]
                                       << " and right wheel with joint name: "
                                       << right_wheel_names[i]);
      left_wheel_joints_[i] =
          hw->getHandle(left_wheel_names[i]);
      right_wheel_joints_[i] =
          hw->getHandle(right_wheel_names[i]);
    }

    sub_command_ = controller_nh.subscribe(
        "cmd_vel", 1, &MecanumDriveController::cmdVelCallback, this);

    return true;
  }

  void MecanumDriveController::update(const ros::Time &time,
                                      const ros::Duration &period)
  {
    const double ws = wheel_separation_multiplier_ * wheel_separation_;
    const double wd = wheel_distance_multiplier_ * wheel_distance_;
    const double lfwr = left_front_wheel_radius_multiplier_ * wheel_radius_;
    const double rfwr = right_front_wheel_radius_multiplier_ * wheel_radius_;
    const double lrwr = left_rear_wheel_radius_multiplier_ * wheel_radius_;
    const double rrwr = right_rear_wheel_radius_multiplier_ * wheel_radius_;

    odometry_.setWheelParams(ws, wd, lfwr, lrwr, rfwr, rrwr);

    if (wheel_joints_size_ != 2)
    {
      ROS_ERROR_STREAM_NAMED(
          name_, "mecanum drive only for four wheel type , error wheel joint");
      return;
    }

    if (open_loop_)
    {
      odometry_.updateOpenLoop(last0_cmd_.lin_x, last0_cmd_.lin_y, last0_cmd_.ang,
                               time);
    }
    else
    {

      const double lfp = left_wheel_joints_[0].getPosition();
      const double lrp = left_wheel_joints_[1].getPosition();
      const double rfp = right_wheel_joints_[0].getPosition();
      const double rrp = right_wheel_joints_[1].getPosition();
      if (std::isnan(lfp) || std::isnan(lrp) || std::isnan(rfp) ||
          std::isnan(rrp))
        return;

      // Estimate linear and angular velocity using joint information
      odometry_.update(lfp, lrp, rfp, rrp, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
          tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped &odom_frame =
            tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }

    // MOVE ROBOT
    // Retreive current velocity command and time step:
    VelocityCommand curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin_x = 0.0;
      curr_cmd.lin_y = 0.0;
      curr_cmd.ang = 0.0;
    }

    const double cmd_dt(period.toSec());

    limiter_lin_.limit(curr_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x,
                       cmd_dt);
    limiter_lin_.limit(curr_cmd.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y,
                       cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    if (publish_velocity_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
    {
      cmd_vel_pub_->msg_.header.stamp = time;
      cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin_x;
      cmd_vel_pub_->msg_.twist.linear.y = curr_cmd.lin_y;
      cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
      cmd_vel_pub_->unlockAndPublish();
    }

    if (wheel_joints_size_ != 2)
    {
      ROS_ERROR_STREAM_NAMED(
          name_, "mecanum drive only for four wheel type , error wheel joint");
      return;
    }

    double k = wheel_distance_ / 2. + wheel_separation_ / 2.;
    Eigen::MatrixXd A(4, 3);
    A << 1, -1, -k, 1, 1, -k, 1, 1, k, 1, -1, k;
    Eigen::VectorXd X(3);
    X << curr_cmd.lin_x, curr_cmd.lin_y, curr_cmd.ang;

    Eigen::VectorXd b = A * X;
    std::vector<double> left_wheel_vel(2);
    left_wheel_vel[0] = b(0) / lfwr;
    left_wheel_vel[1] = b(1) / lrwr;
    std::vector<double> right_wheel_vel(2);
    right_wheel_vel[0] = b(2) / rfwr;
    right_wheel_vel[1] = b(3) / rrwr;

    // Set wheels velocities:
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(left_wheel_vel[i]);
      right_wheel_joints_[i].setCommand(right_wheel_vel[i]);
    }

    publishWheelData(time, period, left_wheel_vel, right_wheel_vel);
    time_previous_ = time;
  }

  void MecanumDriveController::starting(const ros::Time &time)
  {
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;
    time_previous_ = time;

    odometry_.init(time);
  }

  void MecanumDriveController::stopping(const ros::Time & /*time*/) { brake(); }

  void MecanumDriveController::brake()
  {
    const double vel = 0.0;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      left_wheel_joints_[i].setCommand(vel);
      right_wheel_joints_[i].setCommand(vel);
    }
  }

  void MecanumDriveController::cmdVelCallback(
      const geometry_msgs::Twist &command)
  {
    if (isRunning())
    {
      // check that we don't have multiple publishers on the command topic
      if (!allow_multiple_cmd_vel_publishers_ &&
          sub_command_.getNumPublishers() > 1)
      {
        ROS_ERROR_STREAM_THROTTLE_NAMED(
            1.0, name_,
            "Detected "
                << sub_command_.getNumPublishers()
                << " publishers. Only 1 publisher is allowed. Going to brake.");
        brake();
        return;
      }

      if (!std::isfinite(command.angular.z) || !std::isfinite(command.linear.x) ||
          !std::isfinite(command.linear.y))
      {
        ROS_WARN_THROTTLE(1.0, "Received NaN in velocity command. Ignoring.");
        return;
      }

      command_struct_.ang = command.angular.z;
      command_struct_.lin_x = command.linear.x;
      command_struct_.lin_y = command.linear.y;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT(command_struct_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                                 << "Ang: " << command_struct_.ang << ", "
                                 << "Lin x: " << command_struct_.lin_x << ", "
                                 << "Lin y: " << command_struct_.lin_y << ", "
                                 << "Stamp: " << command_struct_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_,
                      "Can't accept new commands. Controller is not running.");
    }
  }

  bool MecanumDriveController::getWheelNames(
      ros::NodeHandle &controller_nh, const std::string &wheel_param,
      std::vector<std::string> &wheel_names)
  {
    XmlRpc::XmlRpcValue wheel_list;
    if (!controller_nh.getParam(wheel_param, wheel_list))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve wheel param '"
                                        << wheel_param << "'.");
      return false;
    }

    if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (wheel_list.size() == 0)
      {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param
                                                      << "' is an empty list");
        return false;
      }

      for (int i = 0; i < wheel_list.size(); ++i)
      {
        if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #"
                                                        << i
                                                        << " isn't a string.");
          return false;
        }
      }

      wheel_names.resize(wheel_list.size());
      for (int i = 0; i < wheel_list.size(); ++i)
      {
        wheel_names[i] = static_cast<std::string>(wheel_list[i]);
      }
    }
    else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      wheel_names.push_back(wheel_list);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(
          name_, "Wheel param '"
                     << wheel_param
                     << "' is neither a list of strings nor a string.");
      return false;
    }

    return true;
  }

  bool MecanumDriveController::setOdomParamsFromUrdf(
      ros::NodeHandle &root_nh, const std::string &left_front_wheel_name,
      const std::string &left_rear_wheel_name,
      const std::string &right_front_wheel_name,
      const std::string &right_rear_wheel_name, bool lookup_wheel_separation,
      bool lookup_wheel_distance, bool lookup_wheel_radius)
  {
    if (!(lookup_wheel_separation || lookup_wheel_radius ||
          lookup_wheel_distance))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have
      // to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str = "";
    if (!res || !root_nh.getParam(model_param_name, robot_model_str))
    {
      ROS_ERROR_NAMED(
          name_, "Robot description couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    urdf::JointConstSharedPtr left_front_wheel_joint(
        model->getJoint(left_front_wheel_name));
    urdf::JointConstSharedPtr left_rear_wheel_joint(
        model->getJoint(left_rear_wheel_name));
    urdf::JointConstSharedPtr right_front_wheel_joint(
        model->getJoint(right_front_wheel_name));
    urdf::JointConstSharedPtr right_rear_wheel_joint(
        model->getJoint(right_rear_wheel_name));

    if (lookup_wheel_separation)
    {
      // Get wheel separation
      if (!left_front_wheel_joint)
      {
        ROS_ERROR_STREAM_NAMED(
            name_, left_front_wheel_name
                       << " couldn't be retrieved from model description");
        return false;
      }

      if (!right_front_wheel_joint)
      {
        ROS_ERROR_STREAM_NAMED(
            name_, right_front_wheel_name
                       << " couldn't be retrieved from model description");
        return false;
      }

      ROS_INFO_STREAM(
          "left front wheel to origin: "
          << left_front_wheel_joint->parent_to_joint_origin_transform.position.x
          << ","
          << left_front_wheel_joint->parent_to_joint_origin_transform.position.y
          << ", "
          << left_front_wheel_joint->parent_to_joint_origin_transform.position.z);
      ROS_INFO_STREAM(
          "right front wheel to origin: "
          << right_front_wheel_joint->parent_to_joint_origin_transform.position.x
          << ","
          << right_front_wheel_joint->parent_to_joint_origin_transform.position.y
          << ", "
          << right_front_wheel_joint->parent_to_joint_origin_transform.position
                 .z);

      wheel_separation_ = euclideanOfVectors(
          left_front_wheel_joint->parent_to_joint_origin_transform.position,
          right_front_wheel_joint->parent_to_joint_origin_transform.position);
    }

    if (lookup_wheel_distance)
    {
      // Get wheel separation
      if (!left_front_wheel_joint)
      {
        ROS_ERROR_STREAM_NAMED(
            name_, left_front_wheel_name
                       << " couldn't be retrieved from model description");
        return false;
      }

      if (!left_rear_wheel_joint)
      {
        ROS_ERROR_STREAM_NAMED(
            name_, left_rear_wheel_name
                       << " couldn't be retrieved from model description");
        return false;
      }

      ROS_INFO_STREAM(
          "left front wheel to origin: "
          << left_front_wheel_joint->parent_to_joint_origin_transform.position.x
          << ","
          << left_front_wheel_joint->parent_to_joint_origin_transform.position.y
          << ", "
          << left_front_wheel_joint->parent_to_joint_origin_transform.position.z);
      ROS_INFO_STREAM(
          "left rear wheel to origin: "
          << left_rear_wheel_joint->parent_to_joint_origin_transform.position.x
          << ","
          << left_rear_wheel_joint->parent_to_joint_origin_transform.position.y
          << ", "
          << left_rear_wheel_joint->parent_to_joint_origin_transform.position.z);

      wheel_distance_ = euclideanOfVectors(
          left_front_wheel_joint->parent_to_joint_origin_transform.position,
          left_rear_wheel_joint->parent_to_joint_origin_transform.position);
    }

    if (lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_front_wheel_joint->child_link_name),
                          wheel_radius_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve "
                                          << left_front_wheel_name
                                          << " wheel radius");
        return false;
      }
    }

    return true;
  }

  void MecanumDriveController::setOdomPubFields(ros::NodeHandle &root_nh,
                                                ros::NodeHandle &controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(
        controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(pose_cov_list[2]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(pose_cov_list[3]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(pose_cov_list[4]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(pose_cov_list[5])};
    odom_pub_->msg_.twist.twist.linear.y = 0;
    odom_pub_->msg_.twist.twist.linear.z = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
        static_cast<double>(twist_cov_list[5])};
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(
        root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }

  void MecanumDriveController::publishWheelData(
      const ros::Time &time, const ros::Duration &period,
      const std::vector<double> vel_left_desired,
      const std::vector<double> vel_right_desired)
  {
    if (publish_wheel_joint_controller_state_ &&
        controller_state_pub_->trylock())
    {
      const double cmd_dt(period.toSec());
      // Compute desired wheels velocities, that is before applying limits:

      controller_state_pub_->msg_.header.stamp = time;

      for (size_t i = 0; i < wheel_joints_size_; ++i)
      {
        const double control_duration = (time - time_previous_).toSec();

        const double left_wheel_acc =
            (left_wheel_joints_[i].getVelocity() - vel_left_previous_[i]) /
            control_duration;
        const double right_wheel_acc =
            (right_wheel_joints_[i].getVelocity() - vel_right_previous_[i]) /
            control_duration;

        controller_state_pub_->msg_.actual.positions[i] =
            left_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.velocities[i] =
            left_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.accelerations[i] = left_wheel_acc;
        controller_state_pub_->msg_.actual.effort[i] =
            left_wheel_joints_[i].getEffort();

        controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_] =
            right_wheel_joints_[i].getPosition();
        controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_] =
            right_wheel_joints_[i].getVelocity();
        controller_state_pub_->msg_.actual.accelerations[i + wheel_joints_size_] =
            right_wheel_acc;
        controller_state_pub_->msg_.actual.effort[i + wheel_joints_size_] =
            right_wheel_joints_[i].getEffort();

        controller_state_pub_->msg_.desired.positions[i] +=
            vel_left_desired[i] * cmd_dt;
        controller_state_pub_->msg_.desired.velocities[i] = vel_left_desired[i];
        controller_state_pub_->msg_.desired.accelerations[i] =
            (vel_left_desired[i] - vel_left_desired_previous_[i]) * cmd_dt;
        controller_state_pub_->msg_.desired.effort[i] =
            std::numeric_limits<double>::quiet_NaN();

        controller_state_pub_->msg_.desired.positions[i + wheel_joints_size_] +=
            vel_right_desired[i] * cmd_dt;
        controller_state_pub_->msg_.desired.velocities[i + wheel_joints_size_] =
            vel_right_desired[i];
        controller_state_pub_->msg_.desired
            .accelerations[i + wheel_joints_size_] =
            (vel_right_desired[i] - vel_right_desired_previous_[i]) * cmd_dt;
        controller_state_pub_->msg_.desired.effort[i + wheel_joints_size_] =
            std::numeric_limits<double>::quiet_NaN();

        // Error
        controller_state_pub_->msg_.error.positions[i] =
            controller_state_pub_->msg_.desired.positions[i] -
            controller_state_pub_->msg_.actual.positions[i];
        controller_state_pub_->msg_.error.velocities[i] =
            controller_state_pub_->msg_.desired.velocities[i] -
            controller_state_pub_->msg_.actual.velocities[i];
        controller_state_pub_->msg_.error.accelerations[i] =
            controller_state_pub_->msg_.desired.accelerations[i] -
            controller_state_pub_->msg_.actual.accelerations[i];
        controller_state_pub_->msg_.error.effort[i] =
            controller_state_pub_->msg_.desired.effort[i] -
            controller_state_pub_->msg_.actual.effort[i];

        controller_state_pub_->msg_.error.positions[i + wheel_joints_size_] =
            controller_state_pub_->msg_.desired
                .positions[i + wheel_joints_size_] -
            controller_state_pub_->msg_.actual.positions[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.velocities[i + wheel_joints_size_] =
            controller_state_pub_->msg_.desired
                .velocities[i + wheel_joints_size_] -
            controller_state_pub_->msg_.actual.velocities[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.accelerations[i + wheel_joints_size_] =
            controller_state_pub_->msg_.desired
                .accelerations[i + wheel_joints_size_] -
            controller_state_pub_->msg_.actual
                .accelerations[i + wheel_joints_size_];
        controller_state_pub_->msg_.error.effort[i + wheel_joints_size_] =
            controller_state_pub_->msg_.desired.effort[i + wheel_joints_size_] -
            controller_state_pub_->msg_.actual.effort[i + wheel_joints_size_];

        // Save previous velocities to compute acceleration
        vel_left_previous_[i] = left_wheel_joints_[i].getVelocity();
        vel_right_previous_[i] = right_wheel_joints_[i].getVelocity();
        vel_left_desired_previous_[i] = vel_left_desired[i];
        vel_right_desired_previous_[i] = vel_right_desired[i];
      }

      controller_state_pub_->unlockAndPublish();
    }
  }

} // namespace mecanum_drive_controller
