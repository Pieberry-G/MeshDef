cmake_minimum_required(VERSION 3.13.0)
 
add_subdirectory(polyscope)
add_subdirectory(spdlog)

set_target_properties(polyscope PROPERTIES FOLDER "Dependencies")
set_target_properties(spdlog PROPERTIES FOLDER "Dependencies")
set_target_properties(stb PROPERTIES FOLDER "Dependencies")
