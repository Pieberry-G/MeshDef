#pragma once

namespace MeshDef {

enum class OperationState
{
	None = 0,
	LoadNewMesh,
	SaveMesh,
	SimplifyQSlim,
	SimplifyOuterHull,
	SimplifyInnerHull,
	Deform,
	DeformOneSolve,
	SetDeform,
	ResetDeform,
	InputMovVec,
	LoadControlMesh
};

namespace State {
	
	extern OperationState opState;

} // namespace State
} // namespace MeshDef