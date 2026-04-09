cmake_minimum_required(VERSION 3.8)
project(drone_tracker)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  drone_tracker/detection_node.py
  drone_tracker/tracker_node.py
  drone_tracker/pid_controller_node.py
  drone_tracker/flight_bridge_node.py
  drone_tracker/target_spawner_node.py
  drone_tracker/visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, worlds
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
