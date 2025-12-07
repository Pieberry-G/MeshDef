cmake_minimum_required(VERSION 3.13.0)

file(GLOB_RECURSE C_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c" )
file(GLOB_RECURSE CPP_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )
file(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

add_library(MeshFix STATIC ${C_SRCS} ${CPP_SRCS} ${HEADERS})

target_include_directories(MeshFix PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/Kernel)
target_include_directories(MeshFix PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/TMesh)
