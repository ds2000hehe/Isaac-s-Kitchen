cmake_minimum_required(VERSION 3.5)
project(image_llm_interface)

find_package(ament_cmake REQUIRED)

# Install python scripts
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}/scripts
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
