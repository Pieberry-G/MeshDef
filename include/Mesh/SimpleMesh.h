#pragma once

#include <Eigen/Core>

namespace MeshDef {

    class SimpleMesh
    {
    public:
        SimpleMesh() = default;
        SimpleMesh(const Eigen::MatrixXd& _vertices, const Eigen::MatrixXi& _faces);
        
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;

        Eigen::MatrixXd getVertices() const { return vertices; }
        void setVertices(const Eigen::MatrixXd& newVertices) { vertices = newVertices; }

        Eigen::MatrixXi getFaces() const { return faces; }
    };

} // namespace MeshDef