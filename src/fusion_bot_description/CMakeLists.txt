cmake_minimum_required(VERSION 3.8)
project(fusion_bot_description)

find_package(ament_cmake REQUIRED)

# Install model, URDF, launch, config directories
install(DIRECTORY
  models urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
