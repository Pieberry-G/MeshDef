#pragma once

#include "Mesh/Model.h"
#include "Mesh/EditMesh.h"
#include "Mesh/OBJLoader.h"

#include "Eigen/Dense"
 
namespace MeshDef {

    std::vector<size_t> CalculateCandidateHandles(const std::vector<std::array<double, 3>>& vertices, const std::vector<std::array<int, 3>>& faces);

    std::vector<size_t> SelectHandlesByVLM(const std::string& imagesDir);
    
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