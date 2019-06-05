cmake_minimum_required(VERSION 3.5)

project(demo_nodes_secure_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# For demo 'listener_best_effort'
find_package(demo_nodes_cpp REQUIRED)

# Quiet as an example that it could
# be done by default
find_package(sros2_cmake QUIET)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  set_property(TARGET ${target} PROPERTY CXX_STANDARD 14)
  set_property(TARGET ${target} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${target} PROPERTY CXX_EXTENSIONS OFF)
  ament_target_dependencies(${target}
    "rclcpp"
    "std_msgs"
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Tutorials of Publish/Subscribe with Topics
custom_executable(topics talker)
custom_executable(topics listener)

# Optional SROS2
if(sros2_cmake_FOUND)

  function(custom_secure_node target)
    # Keep those at function scope
    set(SECURITY ON)
    set(POLICY_FILE "${PROJECT_SOURCE_DIR}/src/policies/sample_policy.xml")

    # TODO: This leading slash is annoying
    ros2_secure_node(NODES "/${target}")
  endfunction()

  custom_secure_node(talker)
  custom_secure_node(listener)
  custom_secure_node(listener_best_effort)

else()
  message(WARNING "sros2_cmake NOT found!")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)
  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
  set(tutorial_tests "talker:listener")

  macro(tests)
    set(tutorial_tests_to_test ${tutorial_tests})

    foreach(tutorial_test ${tutorial_tests_to_test})
      string(REPLACE ":" ";" tutorial_executables "${tutorial_test}")
      set(DEMO_NODES_CPP_EXPECTED_OUTPUT "")
      foreach(executable ${tutorial_executables})
        list(APPEND DEMO_NODES_CPP_EXPECTED_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/test/${executable}")
      endforeach()

      set(DEMO_NODES_CPP_EXECUTABLE "")
      foreach(executable ${tutorial_executables})
        list(APPEND DEMO_NODES_CPP_EXECUTABLE "$<TARGET_FILE:${executable}>")
      endforeach()

      string(REPLACE ";" "_" exe_list_underscore "${tutorial_executables}")
      configure_file(
        test/test_executables_tutorial.py.in
        test_${exe_list_underscore}${target_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}.py.configured"
      )

      add_launch_test(
        "${CMAKE_CURRENT_BINARY_DIR}/test_${exe_list_underscore}${target_suffix}_$<CONFIG>.py"
        TARGET test_tutorial_${exe_list_underscore}${target_suffix}
        TIMEOUT 60
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation}
      )
      foreach(executable ${tutorial_executables})
        set_property(
          TEST test_tutorial_${exe_list_underscore}${target_suffix}
          APPEND PROPERTY DEPENDS ${executable}${target_suffix})
      endforeach()
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(tests)
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

#colcon build --symlink-install --packages-select demo_nodes_secure_cpp --event-handler console_cohesion+ --cmake-force-configure
