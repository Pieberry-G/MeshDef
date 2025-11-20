#pragma once

#include "Mesh/Model.h"
#include "Mesh/EditMesh.h"
#include "Mesh/OBJLoader.h"

#include "Eigen/Dense"
 
namespace MeshDef {

    glm::mat4 Orthographic(float right, float left = 0.0f, float top = 0.0f, float bottom = 0.0f, float zNear = 1.0f, float zFar = 50.0f);
    glm::mat4 Perspective(float fovy = 2 * glm::atan(32.0f/70.0f), float aspect = 1.0f, float zNear = 1.0f, float zFar = 50.0f);
    std::vector<glm::mat4> Views(float distance = 10.0f, float right = 0.0f, std::vector<int> angles = {});
    void MakeCameras(std::vector<glm::mat4>& viewMatrices, glm::mat4& projMatrix, const std::string& camType = "ortho", const std::vector<int>& angles = {});
    
    bool VertInsideSelectBox (const Eigen::Vector3d &botLeftOrigin,
                                        const Eigen::Vector3d &pointOnBotLeftRay,
                                        const Eigen::Vector3d &botRightOrigin,
                                        const Eigen::Vector3d &pointOnBotRightRay,
                                        const Eigen::Vector3d &topRightOrigin,
                                        const Eigen::Vector3d &pointOnTopRightRay,
                                        const Eigen::Vector3d &topLeftOrigin,
                                        const Eigen::Vector3d &pointOnTopLeftRay,
                                        Eigen::Vector3d vertex);

    // load an Edit Mesh object from an OBJ file.
    EditMesh* LoadEditMeshFromOBJFile(const std::string& filepath);

    std::unique_ptr<Model> LoadModelFromFile(const std::string& filepath);

} // namespace MeshDef