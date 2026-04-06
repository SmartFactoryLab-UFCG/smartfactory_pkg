cmake_minimum_required(VERSION 3.22)
project(smart_factory_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(lab_objects_instance src/lab_objects_instance.cpp)
ament_target_dependencies(
  lab_objects_instance
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "shape_msgs"
)

add_executable(move_box_test src/move_box_test.cpp)
ament_target_dependencies(
  move_box_test
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
)

target_link_libraries(lab_objects_instance
  yaml-cpp
)

add_executable(mtc_ur10_test src/mtc_ur10_test.cpp)
target_include_directories(mtc_ur10_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mtc_ur10_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_ur10_test
  "moveit_task_constructor_core"
  "rclcpp"
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config launch srdf DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(TARGETS lab_objects_instance RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS move_box_test RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS mtc_ur10_test RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()