#include "Mesh/SelfIntersections.h"

#include <detectIntersections.h>

namespace MeshDef {

    int DetectSelfIntersections(const std::vector<Eigen::Vector3d>& trisData)
    {
        T_MESH::Basic_TMesh* mesh = new T_MESH::Basic_TMesh();

        for (size_t i = 0; i < trisData.size(); i+=3)
        {
            const Eigen::Vector3d v0_data = trisData[i];
            const Eigen::Vector3d v1_data = trisData[i + 1];
            const Eigen::Vector3d v2_data = trisData[i + 2];
            T_MESH::Vertex* v0 = mesh->newVertex(v0_data(0), v0_data(1), v0_data(2));
            T_MESH::Vertex* v1 = mesh->newVertex(v1_data(0), v1_data(1), v1_data(2));
            T_MESH::Vertex* v2 = mesh->newVertex(v2_data(0), v2_data(1), v2_data(2));

            T_MESH::Edge* e0 = mesh->CreateEdge(v0, v1);
            T_MESH::Edge* e1 = mesh->CreateEdge(v1, v2);
            T_MESH::Edge* e2 = mesh->CreateEdge(v2, v0);
            T_MESH::Triangle* t0 = mesh->CreateTriangle(e0, e1, e2);
        }
        
        int intersectingCount = mesh->selectIntersectingTriangles(50, true);
        
        delete(mesh);
        return intersectingCount;
    }
    
} //namespace MeshDef