#include "mecanum_drive_controller/odometry.h"
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>

namespace mecanum_drive_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
      : timestamp_(0.0), x_(0.0), y_(0.0), heading_(0.0), linear_vel_x(0.0),
        linear_vel_y(0.0), angular_vel(0.0), wheel_separation_(0.0),
        wheel_distance_(0.0), left_front_wheel_radius_(0.0),
        left_rear_wheel_radius_(0.0), right_front_wheel_radius_(0.0),
        right_rear_wheel_radius_(0.0), left_front_wheel_old_pos_(0.0),
        left_rear_wheel_old_pos_(0.0), right_front_wheel_old_pos_(0.0),
        right_rear_wheel_old_pos_(0.0),
        velocity_rolling_window_size_(velocity_rolling_window_size),
        linear_accumulator_x_(RollingWindow::window_size = velocity_rolling_window_size),
        linear_accumulator_y_(RollingWindow::window_size = velocity_rolling_window_size),
        angular_velaccumulator_(RollingWindow::window_size = velocity_rolling_window_size),
        integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2, _3))
  {
  }

  void Odometry::init(const ros::Time &time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  // mecanum odom update
  bool Odometry::update(double left_front_pos, double left_rear_pos,
                        double right_front_pos, double right_rear_pos,
                        const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double left_front_wheel_cur_pos =
        left_front_pos * left_front_wheel_radius_;
    const double left_rear_wheel_cur_pos =
        left_rear_pos * left_rear_wheel_radius_;
    const double right_front_wheel_cur_pos =
        right_front_pos * right_front_wheel_radius_;
    const double right_rear_wheel_cur_pos =
        right_rear_pos * right_rear_wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_front_wheel_est_vel =
        left_front_wheel_cur_pos - left_front_wheel_old_pos_;
    const double left_rear_wheel_est_vel =
        left_rear_wheel_cur_pos - left_rear_wheel_old_pos_;
    const double right_front_wheel_est_vel =
        right_front_wheel_cur_pos - right_front_wheel_old_pos_;
    const double right_rear_wheel_est_vel =
        right_rear_wheel_cur_pos - right_rear_wheel_old_pos_;

    /// Update old position with current:
    left_front_wheel_old_pos_ = left_front_wheel_cur_pos;
    left_rear_wheel_old_pos_ = left_rear_wheel_cur_pos;
    right_front_wheel_old_pos_ = right_front_wheel_cur_pos;
    right_rear_wheel_old_pos_ = right_rear_wheel_cur_pos;

    // mecanum kinenamic
    double k = wheel_distance_ / 2. + wheel_separation_ / 2.;
    Eigen::MatrixXd A(4, 3);
    A << 1, -1, -k, 1, 1, -k, 1, 1, k, 1, -1, k;
    Eigen::VectorXd b(4);
    b << left_front_wheel_est_vel, left_rear_wheel_est_vel,
        right_front_wheel_est_vel, right_rear_wheel_est_vel;

    Eigen::VectorXd X = A.colPivHouseholderQr().solve(b);
    // Eigen::VectorXd X = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    /// Integrate odometry:
    integrate_fun_(X(0), X(1), X(2));

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_accumulator_x_(X(0) / dt);
    linear_accumulator_y_(X(1) / dt);
    angular_velaccumulator_(X(2) / dt);

    linear_vel_x = bacc::rolling_mean(linear_accumulator_x_);
    linear_vel_y = bacc::rolling_mean(linear_accumulator_y_);
    angular_vel = bacc::rolling_mean(angular_velaccumulator_);

    return true;
  }

  void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular,
                                const ros::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_vel_x = linear_x;
    linear_vel_y = linear_y;
    angular_vel = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear_x * dt, linear_y * dt, angular * dt);
  }

  void Odometry::setWheelParams(double wheel_separation, double wheel_distance,
                                double left_front_wheel_radius,
                                double left_rear_wheel_radius,
                                double right_front_wheel_radius,
                                double right_rear_wheel_radius)
  {
    wheel_separation_ = wheel_separation;
    wheel_distance_ = wheel_distance;
    left_front_wheel_radius_ = left_front_wheel_radius;
    left_rear_wheel_radius_ = left_rear_wheel_radius;
    right_front_wheel_radius_ = right_front_wheel_radius;
    right_rear_wheel_radius_ = right_rear_wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(
      size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear_x, double linear_y,
                                double angular)
  {
    heading_ += angular;
    x_ += linear_x;
    y_ += linear_y;
  }

  void Odometry::resetAccumulators()
  {
    linear_accumulator_x_ = RollingMeanAccumulator(RollingWindow::window_size =
                                                       velocity_rolling_window_size_);
    linear_accumulator_y_ = RollingMeanAccumulator(RollingWindow::window_size =
                                                       velocity_rolling_window_size_);
    angular_velaccumulator_ = RollingMeanAccumulator(RollingWindow::window_size =
                                                         velocity_rolling_window_size_);
  }

} // namespace mecanum_drive_controller
