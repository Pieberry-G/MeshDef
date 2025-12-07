cmake_minimum_required(VERSION 3.13.0)
 
add_library(project_deps INTERFACE)

### Common Dependencies
add_subdirectory(polyscope)
add_subdirectory(spdlog)
target_link_libraries(project_deps INTERFACE polyscope spdlog)
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/polyscope/deps/glad/include")
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/polyscope/deps/stb")

add_subdirectory(MovingCones)
target_link_libraries(project_deps INTERFACE MovingCones)

### Python
add_subdirectory(pybind11)
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/pybind11/include")

### Linear System
add_subdirectory(osi_clp)
add_subdirectory(MeshFix)
target_link_libraries(project_deps INTERFACE lib_OsiClpSolver lib_clp lib_Osi lib_CoinUtils MeshFix)
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/osi_clp/CoinUtils/src")
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/osi_clp/Clp/src")
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/osi_clp/Clp/src/OsiClp")
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/osi_clp/Osi/src")
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/osi_clp/Osi/src/Osi")

add_library(eigen INTERFACE)
target_include_directories(project_deps INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
target_link_libraries(project_deps INTERFACE eigen)

set_target_properties(polyscope PROPERTIES FOLDER "Dependencies")
set_target_properties(spdlog PROPERTIES FOLDER "Dependencies")
set_target_properties(stb PROPERTIES FOLDER "Dependencies")
set_target_properties(glad PROPERTIES FOLDER "Dependencies")
set_target_properties(imgui PROPERTIES FOLDER "Dependencies")
set_target_properties(MeshFix PROPERTIES FOLDER "Dependencies")
