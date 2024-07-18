# Top Modules

Here, you can find exemplary top modules that have been designed. These can help you as a reference when building your own custom top module. At this point there is only one module that has been built but more will be added and documented here in the future.

## Perception Module

The autonomous navigation of mobile robots depends largely on reliable localization systems. Various environmental influences such as direct sunlight, reflection, map ambiguity, the movement of the robot, or dynamic obstacles can disrupt localization systems and jeopardize process reliability. A monitoring software will be developed and evaluated using data-driven models to detect and monitor these faults. The aim is to control the localization quality during operation, identify errors, and make adjustments to ensure the safety and efficiency of the robots. This includes collecting relevant environmental data, creating models to estimate the uncertainty of the robot pose, and using synthetic and real-world sensor data for modeling. The models are validated experimentally to ensure transferability to real application scenarios. The perception and localization module that will be used for data collection and validation is shown in the following figure.

<div style="text-align:center;">
<img src="../assets/perception_module.png" alt="perception-module-img" style="width:500px;">
</div>

The module is equipped with reflective markers (1) that a motion-capturing system uses to track the movement of the robot. Four light sensors (3) and four webcams (4) are mounted on the robot. Additional sensors are a 360-degree LiDAR (2), a tracking camera (6), and a depth camera (8). This research project utilizes some hardware that is very use-case-specific. The modular connector (7) allows mounting the localization module in seconds, enabling flexible cooperation with researchers requiring a different top module. The transmission of the motors allows accurate positioning and flexible kinematics, making it possible to investigate localization performance for various types of movements.

The **CAD parts and software** for the perception module can be found in the GitHub repository under the *humble-perception-module* branch.
