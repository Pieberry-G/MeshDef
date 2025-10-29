#pragma once

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

namespace MeshDef {

class DrawMesh
{
public:
    void UpdateMeshData(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<size_t>>& faces);



private:
    std::string m_Name;

    std::vector<glm::vec3> m_Vertices;
    std::vector<std::vector<size_t>> m_Faces;

    glm::mat4 m_Transform = glm::mat4(1.0f);  // object's model transformations

    polyscope::SurfaceMesh* m_PsMesh = nullptr;
};

} // namespace MeshDef