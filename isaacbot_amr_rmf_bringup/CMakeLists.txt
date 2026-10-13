cmake_minimum_required(VERSION 3.10)
project(isaacbot_amr_rmf_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
