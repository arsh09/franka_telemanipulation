cmake_minimum_required(VERSION 3.0.0)
project(fci_experiments VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

## Dependencies
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(Boost 1.58.0 REQUIRED system thread)

include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)



# custom library
add_library(examples_common STATIC
  src/examples/examples_common.cpp
)
target_link_libraries(examples_common PUBLIC  ${Franka_LIBRARIES} )

# custom library (network client) 
add_library(network_client STATIC
  src/teleop/network_client.cpp
)
target_link_libraries(network_client PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES})

# custom library (network server) 
add_library(network_server STATIC
  src/teleop/network_server.cpp
)
target_link_libraries(network_client PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES})

# custom library (teleop follower) 
add_library(follower STATIC
  src/teleop/follower.cpp
)
target_link_libraries(follower PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES} examples_common)

# custom library (teleop leader) 
add_library(leader STATIC
  src/teleop/leader.cpp
  
)
target_link_libraries(leader PUBLIC  ${Franka_LIBRARIES} ${Boost_LIBRARIES} examples_common)



# build franka example
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
  print_joint_poses
)

foreach(example ${EXAMPLES})
  add_executable(${example} src/examples/${example}.cpp)
  target_link_libraries( ${example} ${Franka_LIBRARIES} examples_common )
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
 
# build network examples
set(EXTRA_NETWORK_EXAMPLES
  franka_master_client
  franka_slave_server
)

foreach(network_ex ${EXTRA_NETWORK_EXAMPLES})
  add_executable(${network_ex} src/extra/${network_ex}.cpp)
  target_link_libraries(${network_ex} ${Franka_LIBRARIES} ${Boost_LIBRARIES} examples_common )
endforeach()


# example for telemanipulation library
add_executable(follower_franka src/follower_franka.cpp)
target_link_libraries(follower_franka 
  follower
  network_server
)

add_executable(leader_franka src/leader_franka.cpp)
target_link_libraries(leader_franka 
  leader
  network_client
)

# install
include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
