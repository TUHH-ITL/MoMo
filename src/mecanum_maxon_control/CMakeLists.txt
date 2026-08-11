cmake_minimum_required(VERSION 3.8)
project(mecanum_maxon_control)

find_package(ament_cmake REQUIRED)
find_package(canopen REQUIRED)
find_package(lely_core_libraries REQUIRED)

cogen_dcf(maxon_mecanum_bus)

install(PROGRAMS
  scripts/mecanum_epos4_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  config/controller.yaml
  config/teleop_joy.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
