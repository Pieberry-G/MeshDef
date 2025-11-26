#pragma once

#include <Eigen/Core>

namespace MeshDef {

    int DetectSelfIntersections(const std::vector<Eigen::Vector3d>& trisData);
    
} // namespace MeshDef