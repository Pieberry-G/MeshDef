cmake_minimum_required(VERSION 3.13.0)

file(GLOB_RECURSE SRCS "*.cpp")
file(GLOB_RECURSE HEADERS "*.h")

add_library(ImGuizmo STATIC ImGuizmo.cpp ImGuizmo.h)
target_include_directories(ImGuizmo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
