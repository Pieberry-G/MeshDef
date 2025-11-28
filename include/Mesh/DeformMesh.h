#pragma once

#include <Eigen/Core>

namespace MeshDef {

    class DeformMesh
    {
    public:
        Eigen::MatrixXd getVertices() const { return vertices; }
        Eigen::MatrixXi getFaces() const { return faces; }
        Eigen::VectorXi getFixedVertIndices() const { return fixedVertIndices; }
        Eigen::VectorXi getMovingVertIndices() const { return movingVertIndices; }
        Eigen::MatrixXd getTargetPositions() const { return targetPositions; }
        
        void setVertices(const Eigen::MatrixXd& _vertices) { vertices = _vertices; }
        void setFaces(const Eigen::MatrixXi& _faces) { faces = _faces; }
        void setFixedVertIndices(const Eigen::VectorXi& _fixedVertIndices) { fixedVertIndices = _fixedVertIndices; }
        void setMovingVertIndices(const Eigen::VectorXi& _movingVertIndices) { movingVertIndices = _movingVertIndices; }
        void setTargetPositions(const Eigen::MatrixXd& _targetPositions) { targetPositions = _targetPositions; }

    private:
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;

        Eigen::VectorXi fixedVertIndices;
        Eigen::VectorXi movingVertIndices;
        Eigen::MatrixXd targetPositions;
    };

} // namespace MeshDef