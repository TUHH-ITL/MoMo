#pragma once

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>
#include <eigen3/Eigen/Dense>

namespace mecanum_drive_controller
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:
    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double, double)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the
     * velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param left_pos  Left  wheel position [rad]
     * \param right_pos Right wheel position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    bool update(double left_front_pos, double left_rear_pos,
                double right_front_pos, double right_rear_pos,
                const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
    void updateOpenLoop(double linear_x, double linear_y, double angular,
                        const ros::Time &time);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const { return heading_; }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const { return x_; }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const { return y_; }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return sqrt(pow(linear_vel_x, 2) + pow(linear_vel_y, 2));
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinearX() const { return linear_vel_x; }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinearY() const { return linear_vel_y; }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const { return angular_vel; }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param wheel_separation   Separation between left and right wheels [m]
     * \param left_wheel_radius  Left wheel radius [m]
     * \param right_wheel_radius Right wheel radius [m]
     */
    void setWheelParams(double wheel_separation, double wheel_distance,
                        double left_front_wheel_radius,
                        double left_rear_wheel_radius,
                        double right_front_wheel_radius,
                        double right_rear_wheel_radius);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:
    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean>>
        RollingMeanAccumulator;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt)
     * computed by encoders \param angular Angular velocity [rad] (angular
     * displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear_x, double linear_y, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;       //   [m]
    double y_;       //   [m]
    double heading_; // [rad]

    /// Current velocity:
    double linear_vel_x; //   [m/s]
    double linear_vel_y;
    double angular_vel; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double wheel_separation_;
    double wheel_distance_;
    double left_front_wheel_radius_;
    double right_front_wheel_radius_;
    double left_rear_wheel_radius_;
    double right_rear_wheel_radius_;

    /// Previou wheel position/state [rad]:
    double left_front_wheel_old_pos_;
    double left_rear_wheel_old_pos_;
    double right_front_wheel_old_pos_;
    double right_rear_wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAccumulator linear_accumulator_x_;
    RollingMeanAccumulator linear_accumulator_y_;
    RollingMeanAccumulator angular_velaccumulator_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };
} // namespace mecanum_drive_controller