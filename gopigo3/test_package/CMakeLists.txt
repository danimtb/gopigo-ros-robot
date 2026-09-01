cmake_minimum_required(VERSION 3.16)
project(test_package_gopigo3 LANGUAGES CXX)

find_package(gopigo3_cpp REQUIRED CONFIG)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE gopigo3)
