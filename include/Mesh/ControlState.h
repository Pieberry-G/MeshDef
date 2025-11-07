#pragma once

namespace MeshDef
{
    enum class OperationType
    {
        None = 0,
        LoadNewMesh,
        SaveMesh,
        SimpQSlim,
        Deform,
        DeformOneSolve,
        SetDeform,
        ResetDeform,
        InputMovVec,
        LoadControlMesh
    };
    
} // namespace MeshDef