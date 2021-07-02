cmake_minimum_required(VERSION 3.0.0)
project(fci_experiments VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

## Dependencies
# find_package(Poco REQUIRED COMPONENTS Net Foundation)
# find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)


add_executable(hello_world src/main.cpp)
target_link_libraries(hello_world
  ${Franka_LIBRARIES}
)


add_executable(echo_robot_state src/echo_robot_state.cpp)
target_link_libraries(echo_robot_state
  ${Franka_LIBRARIES}
)

add_executable(print_joint_poses src/examples/print_joint_poses.cpp)
target_link_libraries(print_joint_poses
  ${Franka_LIBRARIES}
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
