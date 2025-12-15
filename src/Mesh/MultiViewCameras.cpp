#include "Mesh/MultiViewCameras.h"

#include <glm/gtc/matrix_transform.hpp>

namespace MeshDef {
    
    glm::mat4 Orthographic(float right, float left, float top, float bottom, float zNear, float zFar)
    {
        if (left == 0.0f) { left = -right; }
        if (top == 0.0f) { top = right; }
        if (bottom == 0.0f) { bottom = -top; }
        
        return glm::ortho(left, right, bottom, top, zNear, zFar);
    }

    glm::mat4 Perspective(float fovy, float aspect, float zNear, float zFar)
    {
        return glm::perspective(fovy, aspect, zNear, zFar);
    }

    std::vector<glm::mat4> Views(float distance, std::vector<int> angles)
    {
        if (angles.empty())
        {
            angles = { 0, -45, -90, 180, 90, 45 };
        }

        std::vector<glm::mat4> views(angles.size(), glm::mat4(1.0f));

        for (int i = 0; i < views.size(); i++)
        {
            glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), glm::radians((float)angles[i]), glm::vec3(0.0f, 1.0f, 0.0f));
            glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -distance));
            views[i] = translation * rotation;
        }
        
        return views;
    }
    
    MultiViewCameras MakeMultiViewCameras(const std::string& camType, const std::vector<int>& angles)
    {
        std::vector<glm::mat4> viewMatrices;
        glm::mat4 projMatrix;
        
        if (camType == "ortho")
        {
            float distance = 10.0f;
            float right = 1.0f;
            viewMatrices = Views(distance, angles);
            projMatrix = Orthographic(right);
        }
        else
        {
            float distance = 1 / glm::tan(glm::radians(49.13f) / 2.0);
            float right = 1.0f;
            viewMatrices = Views(distance, angles);
            projMatrix = Perspective();
        }

        return MultiViewCameras{viewMatrices, projMatrix};
    }

} // namespace MeshDef