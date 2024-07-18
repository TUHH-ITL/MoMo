# API Reference

The API reference section documents the Application Programming Interfaces (APIs) exposed by MoMo's software modules. It includes detailed descriptions of functions, parameters, and usage examples for each API. This section serves as a reference guide for developers integrating with or extending MoMo's software stack.

## momo_control

In this section, the design and implementation of the momo_control ROS package for controlling a 4-wheel mecanum mobile robot are detailed. The software components described below enable robust motion control, odometry calculation, and velocity limiting to ensure safe and effective operation of the robotic platform within ROS Noetic framework.

### mecanum_drive_controller

The Mecanum Drive Controller is responsible for managing the robot's motion by computing appropriate joint commands based on received velocity inputs. Utilizing mecanum drive kinematics, the controller translates desired linear and angular velocities into individual wheel velocities. This ensures the robot moves in the intended direction and can perform omnidirectional maneuvers effectively. The controller also integrates odometry calculations to track the robot's position and orientation using wheel encoders and publishes this information for localization and navigation purposes.
The controller subscribes to velocity commands through the cmd_vel topic and publishes joint commands to control the robot's motion. It is configured with parameters such as wheel names, publishing rates, pose and twist covariance matrices, and command velocity timeout to optimize performance and responsiveness. Additionally, it supports dynamic reconfiguration through ROS parameters, enabling adjustments to wheel properties and odometry settings based on the robot's configuration.

### odometry

The Odometry module computes and publishes accurate odometry information essential for localization and navigation of the robot. By continuously updating wheel positions and velocities, derived from wheel encoders, the odometry module estimates the robot's pose (position and orientation) relative to its starting point. This information is critical for autonomous navigation tasks, as it provides real-time feedback on the robot's position within its environment.
The odometry module supports open-loop and closed-loop control strategies, enabling seamless integration with the motion controller for precise maneuvering. It publishes odometry data through the odom topic in the form of nav_msgs/Odometry messages, including position (x, y), orientation (yaw angle), and linear and angular velocities. The module's configuration parameters, such as wheel radius and separation distance, are set from the robot's URDF (Unified Robot Description Format), ensuring consistency between physical specifications and computational models.

### speed_limiter

The  enhances operational safety by enforcing velocity, acceleration, and jerk limits on the robot's motion. This prevents the robot from exceeding predefined speed thresholds, thus minimizing the risk of collisions and ensuring smooth andcontrolled movements. The speed limiter module adjusts linear and angular velocities based on specified limits, maintaining safe operating conditions during various maneuvers.
Implemented as a configurable ROS node, the speed limiter subscribes to velocity commands and applies constraints to ensure compliance with safety parameters. It operates in conjunction with the mecanum drive controller, dynamically adjusting speed limits based on the robot's operational mode and environmental conditions. By publishing limited velocity commands through the publish_cmd topic, it enables downstream modules to execute controlled motions while adhering to safety protocols.

## Parameters and Topics

The software components are configured using ROS parameters, allowing flexibility and adaptability to different robotic platforms and operational scenarios. Key parameters include wheel properties (radius, separation), covariance matrices for odometry data, and velocity constraints for the speed limiter. These parameters are crucial for optimizing performance and ensuring compatibility with the robot's mechanical design and operational requirements.

### Parameters

- `left_wheel`: Specifies the joint names for the left wheels.
- `right_wheel`: Specifies the joint names for the right wheels.
- `publish_rate`: The rate (in Hz) at which the controller state is published.
- `pose_covariance_diagonal`: Diagonal elements of the covariance matrix for the pose, indicating confidence in the estimated pose.
- `twist_covariance_diagonal`: Diagonal elements of the covariance matrix for the twist, indicating confidence in the estimated twist.
- `cmd_vel_timeout`: The timeout (in seconds) for velocity commands.
- `publish_velocity_cmd`: Determines whether the velocity command should be published.
- `publish_wheel_joint_controller_state`: Determines whether the wheel joint controller state should be published.
- `base_frame_id`: The frame ID for the base of the robot.
- `left_wheel_radius_multiplier`: Multiplier for the radius of the left wheels.
- `right_wheel_radius_multiplier`: Multiplier for the radius of the right wheels.
- `enable_odom_tf`: Determines whether the odometry should be published to TF.
- `wheel_distance`: Distance between the front and rear wheels.
- `wheel_radius`: Radius of the wheels.
- `wheel_separation`: Separation distance between the left and right wheels.
- `required_drive_mode`: Specifies the required drive mode for the controller.
- `linear.x.has_velocity_limits`: Enables velocity limits for linear x direction.
- `linear.x.has_acceleration_limits`: Enables acceleration limits for linear x direction.
- `linear.x.max_velocity`: Maximum allowable velocity for linear x direction.
- `linear.x.min_velocity`: Minimum allowable velocity for linear x direction.
- `linear.x.max_acceleration`: Maximum allowable acceleration for linear x direction.
- `linear.x.min_acceleration`: Minimum allowable acceleration for linear x direction.
- `linear.y.has_velocity_limits`: Enables velocity limits for linear y direction.
- `linear.y.has_acceleration_limits`: Enables acceleration limits for linear y direction.
- `linear.y.max_velocity`: Maximum allowable velocity for linear y direction.
- `linear.y.min_velocity`: Minimum allowable velocity for linear y direction.
- `linear.y.max_acceleration`: Maximum allowable acceleration for linear y direction.
- `linear.y.min_acceleration`: Minimum allowable acceleration for linear y direction.
- `angular.z.has_velocity_limits`: Enables velocity limits for angular z direction.
- `angular.z.has_acceleration_limits`: Enables acceleration limits for angular z direction.
- `angular.z.max_velocity`: Maximum allowable angular velocity for angular z direction.
- `angular.z.min_velocity`: Minimum allowable angular velocity for angular z direction.
- `angular.z.max_acceleration`: Maximum allowable angular acceleration for angular z direction.
- `angular.z.min_acceleration`: Minimum allowable angular acceleration for angular z direction.

### Subscribed Topics

- `cmd_vel` (geometry_msgs/Twist): Velocity commands to control robot motion.

### Published Topics

- `odom` (nav_msgs/Odometry): Odometry information including position, orientation, and velocities.
- `tf` (tf/tfMessage): Transformation from odom to robot base_footprint.
- `publish_cmd` (geometry_msgs/TwistStamped): Limited velocity commands after applying speed limiters.