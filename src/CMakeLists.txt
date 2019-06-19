cmake_minimum_required(VERSION 3.10)
project(src)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(src
        FusionEKF.cpp
        FusionEKF.h
        json.hpp
        kalman_filter.cpp
        kalman_filter.h
        main.cpp
        measurement_package.h
        tools.cpp
        tools.h)
