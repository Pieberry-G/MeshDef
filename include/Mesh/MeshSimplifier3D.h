#pragma once

#include "Mesh/EditMesh.h"
#include "Mesh/EdgeHeap.h"
#include "Mesh/Quadric.h"

namespace MeshDef {

class MeshSimplifier3D
{
public: // constructor
    MeshSimplifier3D(EditMesh& pMesh);
    
    void InitQSlim();
    void SimplifyQSlim(double threshold, int decreaseTris);
    void InitProgressiveHull(bool outerHull);
    void SimplifyProgressiveHull(double threshold, int decreaseTris, bool outerHull);
    void CompactMesh();
    // void VertexRemoval(double threshold, int decreaseTris);
    // void WriteMeshIntoObjFile(const char* filePath);

protected:
    void ComputeTriQ(size_t triI, Quadric& Q);
    bool GetMinErr(size_t edgeI, double& minErr);
    
    bool GetMinVolume(size_t edgeI, double& minVolume, bool outerHull);
    void GetAdjacentTrisData(size_t edgeI, std::vector<Eigen::Vector3d>& adjacentTrisData, bool includeDirectFaces);
    bool SolveLinearProgrammingForVertex(const std::vector<Eigen::Vector3d>& adjacentTrisData, Eigen::Vector3d& optimalPos, double& volumeIncrease, bool outerHull);

    std::size_t CollapseEdge(std::size_t he, std::vector<size_t>& affectedHE, std::vector<size_t>& deletedHE, bool optimizePos = true);
    bool Collapsable(size_t edgeI);

private:
    EditMesh& mesh;

    // QSlim
    int editCount_QSlim;
    EdgeHeap edgesToCollapse;
    std::vector<Quadric> vertQuadrics;
    std::vector<int> twinInside; //-1: self inside, edgeInd: twin inside, -2: neigther is inside
    std::vector<Eigen::Vector3d> edgeBestPoses;
};

} // namespace MeshDef