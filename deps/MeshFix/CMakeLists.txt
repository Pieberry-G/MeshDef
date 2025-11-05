cmake_minimum_required(VERSION 3.13.0)

file(GLOB_RECURSE C_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/MeshFix-V2.1/src/*.c" )
file(GLOB_RECURSE CPP_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/MeshFix-V2.1/src/*.cpp" )
file(GLOB_RECURSE HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/MeshFix-V2.1/include/*.h")

add_library(MeshFix STATIC ${C_SRCS} ${CPP_SRCS} ${HEADERS})

target_include_directories(MeshFix PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MeshFix-V2.1/include/Kernel>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MeshFix-V2.1/include/TMesh>
    $<INSTALL_INTERFACE:include>
)