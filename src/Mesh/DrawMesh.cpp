#include "Mesh/DrawMesh.h"

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

namespace MeshDef {

void DrawMesh::UpdateMeshData(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<size_t>>& faces)
{
    m_Vertices = vertices;
    m_Faces = faces;
}



} // namespace MeshDef