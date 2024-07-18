# Troubleshooting

This section contains troubleshooting guidelines for common issues and errors encountered while working with MoMo's software stack. It provides solutions, workarounds, and debugging techniques for diagnosing and resolving problems effectively.

## Common Issues and Solutions

- **Issue**: MoMo does not respond to the movement commands.
   - **Solution**: Make sure the Control PC is turned on and check the connection using `ping <IP-address>` from the Main PC. Restart the bash scripts and make sure you run them in sequence with a delay of 5 seconds between each script.

   ---

- **Issue**: Can I adjust MoMo's top speed? 
   - **Solution**: Absolutely, you can modify both the linear and angular velocities, as well as the accelerations, within the momo_control package located in the catkin_ws on the Control PC. Update the settings in the yaml file and rebuild the package to apply the changes.


## Frequently Asked Questions (FAQs)

This section provides answers to frequently asked questions about MoMo's software stack. It covers common queries, issues, and concerns raised by users during development, deployment, and operation of MoMo.

### General Questions

- **Question**: What version of Ubuntu is recommended?
   - **Answer**: For the Main PC, Ubuntu 22.04 (Jammy Jellyfish) is recommended due to its compatibility with ROS 2 Humble. For the Control PC, Ubuntu 20.04 (Focal Fossa) is necessary to ensure the ROS Bridge functions correctly.

   ---

- **Question**: Which version of ROS should I use?
   - **Answer**: For the Main PC, ROS 2 Humble is advised because of its extended support period. On the Control PC, you should run ROS Noetic along with ROS 2 Humble to ensure compatibility with the ROS Bridge.

   ---

- **Question**: Why is it necessary to use two computers? Can't I use just one for all tasks?
   - **Answer**: The powerful Maxon actuators currently in use lack ROS 2 drivers, which necessitates the use of two computers. As soon as these drivers become available, it will be possible to consolidate tasks onto a single computer. Until then, we manage high-level and low-level controls on separate computers via the ROS Bridge.


### Technical Questions

- **Question**: Why can't I view the lidar data even though I've configured the lidar properly?
   - **Answer**: Ensure there are no IP address conflicts among connected interfaces by checking the output of the `ifconfig` command. This will help verify that all network settings are correctly configured.

---

- **Question**: What types of sensors are compatible with MoMo?
   - **Answer**: MoMo is designed to be fully open-source and configurable, supporting a wide range of sensors that can be mounted as top modules. Compatible sensors include 2D lidar, 3D lidar, depth cameras, stereo cameras, and many others.

### Deployment Questions

- **Question**: What is the battery life of MoMo?
   - **Answer**: The battery duration varies based on usage and workload. Typically, under heavy usage, MoMo's battery can last throughout a full working day.

   ---

- **Question**: Is MoMo suitable for outdoor use?
   - **Answer**: MoMo is primarily designed for indoor settings and isn't waterproof. Therefore, it is not recommended for outdoor operation. If you choose to use it outdoors, please do so at your own risk.

## Contact Support

If you encounter an issue that is not covered in this documentation or if you need further assistance, [open an issue](https://github.com/TUHH-ITL/MoMo/issues) in the Github repository. Please feel free to contact [markus.knitt@tuhh.de](mailto:markus.knitt@tuhh.de).
