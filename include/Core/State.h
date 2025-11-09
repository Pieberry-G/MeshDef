#pragma once

#include <glm/glm.hpp>

namespace MeshDef {

enum class EditOperation
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

class ControlState
{
public:
	ControlState()
		: op(EditOperation::None), selectActive(false), selectDirty(false) {}
	
public:
	EditOperation op;
	bool selectActive;
	bool selectDirty;  // requires processing
	glm::vec2 selectStart;
	glm::vec2 selectEnd;
};

namespace State {
	
	extern ControlState cState;

} // namespace State
} // namespace MeshDef