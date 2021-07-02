cmake_minimum_required(VERSION 3.0.0)
project(fci_experiments VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

## Dependencies
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
# find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  src/examples/examples_common.cpp
)

target_link_libraries(examples_common PUBLIC  ${Franka_LIBRARIES} )



set(EXAMPLES
  cartesian_impedance_control
 
)

foreach(example ${EXAMPLES})
  add_executable(${example} src/examples/${example}.cpp)
  target_link_libraries(${example} ${Franka_LIBRARIES} examples_common )
endforeach()


# target_link_libraries(joint_impedance_control Threads::Threads)
# target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


# add_executable(hello_world src/main.cpp)
# target_link_libraries(hello_world
#   ${Franka_LIBRARIES}
# )


# add_executable(echo_robot_state src/echo_robot_state.cpp)
# target_link_libraries(echo_robot_state
#   ${Franka_LIBRARIES}
# )

# add_executable(print_joint_poses src/examples/print_joint_poses.cpp)
# target_link_libraries(print_joint_poses
#   ${Franka_LIBRARIES}
# )

# add_executable(cartesian_impedance_control src/examples/cartesian_impedance_control.cpp)
# target_link_libraries(cartesian_impedance_control
#   ${Franka_LIBRARIES}
# )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
