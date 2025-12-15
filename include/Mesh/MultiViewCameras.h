#pragma once

#include <glm/glm.hpp>
 
namespace MeshDef {
    
    struct MultiViewCameras
    {
        std::vector<glm::mat4> viewMatrices;
        glm::mat4 projMatrix;
    };
    
    glm::mat4 Orthographic(float right, float left = 0.0f, float top = 0.0f, float bottom = 0.0f, float zNear = 1.0f, float zFar = 50.0f);
    glm::mat4 Perspective(float fovy = 2 * glm::atan(32.0f/70.0f), float aspect = 1.0f, float zNear = 1.0f, float zFar = 50.0f);
    std::vector<glm::mat4> Views(float distance = 10.0f, float right = 0.0f, std::vector<int> angles = {});
    MultiViewCameras MakeMultiViewCameras(const std::string& camType = "ortho", const std::vector<int>& angles = {});

} // namespace MeshDef