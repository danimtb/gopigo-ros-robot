cmake_minimum_required(VERSION 3.22)
project(gopigo3_ros CXX)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gopigo3_cpp REQUIRED)

add_executable(gopigo3_ros_node
               src/main.cpp
               src/gopigo3_ros_node.cpp
               src/gopigo3_driver.cpp)

target_link_libraries(gopigo3_ros_node PRIVATE rclcpp::rclcpp
                                               ${geometry_msgs_TARGETS}
                                               gopigo3)

include(GNUInstallDirs)
install(TARGETS gopigo3_ros_node RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
