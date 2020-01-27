cmake_minimum_required(VERSION 3.9)
project(GA)

set(CMAKE_CXX_STANDARD 17)

add_executable(GA main.cpp Robot.cpp Robot.h Population.cpp Population.h)