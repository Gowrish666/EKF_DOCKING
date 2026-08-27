cmake_minimum_required(VERSION 3.16)

project(simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PahoMqttCpp REQUIRED)

add_executable(simulator
     src/main.cpp)

target_include_directories(simulator
    PRIVATE
       ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(simulator
     PRIVATE
         paho-mqttpp3
         paho-mqtt3as
         )