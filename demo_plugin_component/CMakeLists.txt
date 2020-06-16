cmake_minimum_required(VERSION 3.5)
project(demo_plugin_component)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

set(plugins_dependencies
  pluginlib
)

set(node_dependencies
  ${plugins_dependencies}
  rclcpp
  rclcpp_components
  rcutils
  std_msgs
)

add_library(${PROJECT_NAME}_plugins SHARED src/message_writer_derived_export.cpp)
ament_target_dependencies(${PROJECT_NAME}_plugins ${plugins_dependencies})

add_library(${PROJECT_NAME} SHARED
  src/talker_node.cpp
  src/crtp_talker_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${node_dependencies})
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ros2_playground::TalkerNode"
  EXECUTABLE talker_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ros2_playground::TalkerCrtpDerivedA"
  EXECUTABLE crtp_talker_a_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ros2_playground::TalkerCrtpDerivedB"
  EXECUTABLE crtp_talker_b_node
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES plugins_description.xml
  DESTINATION share
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugins)
pluginlib_export_plugin_description_file(demo_plugin_component plugins_description.xml)

ament_package()
