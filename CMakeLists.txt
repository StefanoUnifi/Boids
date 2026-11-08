cmake_minimum_required(VERSION 3.25)
project(Boids)

set(CMAKE_CXX_STANDARD 23)

set(SOURCE_FILES
        src/main.cpp
        src/Boid.cpp
        src/Boid.h
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})