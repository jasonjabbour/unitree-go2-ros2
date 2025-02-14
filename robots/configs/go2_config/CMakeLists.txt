cmake_minimum_required(VERSION 3.5)
project(go2_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds maps_manual
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist2twiststamped.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_clock.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()