cmake_minimum_required(VERSION 3.13.0)

### Project
set(PROJECT_TARGET MeshDef)
project(${PROJECT_TARGET})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

file(GLOB_RECURSE SRCS "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_executable(${PROJECT_TARGET} "${SRCS}" "${HEADERS}")
target_include_directories(${PROJECT_TARGET} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_TARGET})
target_precompile_headers(${PROJECT_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/pch.h)
target_compile_options(${PROJECT_TARGET} PRIVATE /bigobj)

target_compile_definitions(${PROJECT_TARGET} PRIVATE MD_PLATFORM_WINDOWS)
target_compile_definitions(${PROJECT_TARGET} PRIVATE "$<$<CONFIG:DEBUG>:MD_DEBUG>")
target_compile_definitions(${PROJECT_TARGET} PRIVATE "$<$<CONFIG:RELEASE>:MD_NDEBUG>")

### Python
set(Python3_ROOT_DIR "C:/Software/anaconda3/envs/wonder3d_plus")
add_definitions(-DPython3_ROOT_DIR="${Python3_ROOT_DIR}")
set(Python3_EXECUTABLE "${Python3_ROOT_DIR}/python")
set(Python3_FIND_REGISTRY NEVER)
set(Python3_FIND_STRATEGY LOCATION)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Python3_EXECUTABLE: ${Python3_EXECUTABLE}")
message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_TARGET}  ${Python3_LIBRARIES})
target_include_directories(${PROJECT_TARGET}  PUBLIC ${Python3_INCLUDE_DIRS})

file(COPY "${Python3_ROOT_DIR}/python310.dll" DESTINATION "${CMAKE_BINARY_DIR}/bin/release")
file(COPY "${Python3_ROOT_DIR}/python310.dll" DESTINATION "${CMAKE_BINARY_DIR}/bin/debug")
target_compile_definitions(${PROJECT_TARGET} PRIVATE PYTHON_HOME="${Python3_ROOT_DIR}")


add_subdirectory(deps)
target_link_libraries(${PROJECT_TARGET} project_deps)