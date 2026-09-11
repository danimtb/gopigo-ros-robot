cmake_minimum_required(VERSION 3.22)
project(gopigo3_ros CXX)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gopigo3_cpp REQUIRED)

add_executable(gopigo3_ros_node
               src/main.cpp
               src/gopigo3_ros_node.cpp
               src/gopigo3_driver.cpp
               src/convoy_controller.cpp)

set_target_properties(gopigo3_ros_node PROPERTIES
                      CXX_STANDARD 17
                      CXX_STANDARD_REQUIRED ON)

target_link_libraries(gopigo3_ros_node PRIVATE rclcpp::rclcpp
                                               ${geometry_msgs_TARGETS}
                                               ${std_msgs_TARGETS}
                                               gopigo3)

include(GNUInstallDirs)
install(TARGETS gopigo3_ros_node RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
