cmake_minimum_required(VERSION 3.16)

project(ekf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PahoMqttCpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ekf
      src/main.cpp)

target_include_directories(ekf
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ekf
     PahoMqttCpp::paho-mqttpp3
     Eigen3::Eigen )