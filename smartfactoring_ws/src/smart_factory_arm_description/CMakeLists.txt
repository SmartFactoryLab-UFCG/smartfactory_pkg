cmake_minimum_required(VERSION 3.8)
project(smart_factory_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(urdf REQUIRED)

# # Install Python execs
# ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY meshes launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
