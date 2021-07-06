cmake_minimum_required(VERSION 3.0.0)
project(fci_experiments VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

## Dependencies
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(Poco REQUIRED COMPONENTS Net)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# custom library
add_library(examples_common STATIC
  src/examples/examples_common.cpp
)

target_link_libraries(examples_common PUBLIC  ${Franka_LIBRARIES} )


# build
set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
)

foreach(example ${EXAMPLES})
  add_executable(${example} src/examples/${example}.cpp)
  target_link_libraries(${example} ${Franka_LIBRARIES} examples_common )
endforeach()


target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)


add_executable(udp_server src/communication/udp_server.cpp)
target_link_libraries(udp_server Poco::Net)

add_executable(udp_client src/communication/udp_client.cpp)
target_link_libraries(udp_client Poco::Net)

add_executable(udp_send_state src/communication/udp_send_state.cpp)
target_link_libraries(udp_send_state Poco::Net ${Franka_LIBRARIES} )


include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


# install
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
