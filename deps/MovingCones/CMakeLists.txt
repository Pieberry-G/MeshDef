cmake_minimum_required(VERSION 3.13.0)

file(GLOB_RECURSE SRCS "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(MovingCones STATIC "${SRCS}" "${HEADERS}")
target_include_directories(MovingCones PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_compile_definitions(MovingCones PUBLIC _USE_MATH_DEFINES)
target_compile_definitions(MovingCones PUBLIC "$<$<CONFIG:DEBUG>:_DEBUG>")
target_compile_definitions(MovingCones PUBLIC "$<$<CONFIG:RELEASE>:NDEBUG>")

add_subdirectory(deps)
target_link_libraries(MovingCones MovingCones_deps)
target_include_directories(MovingCones PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps>
    $<INSTALL_INTERFACE:include>
)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/deps/suitesparse/bin/" DESTINATION "${CMAKE_BINARY_DIR}")
