cmake_minimum_required(VERSION 3.16)

project(visualizer_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

find_package(PahoMqttCpp REQUIRED)

add_executable(visualizer_ui
         src/main.cpp)

target_link_libraries(visualizer_ui
PRIVATE
      Qt6::Widgets
      PahoMqttCpp::paho-mqttpp3
)