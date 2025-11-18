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
	
	SetVertConstraint,
	FinishDeformation,

	
	Deform,
	DeformOneSolve,
	SetDeform,
	ResetDeform,
	InputMovVec,
	LoadControlMesh
};

inline std::string editOperationToString(EditOperation op)
{
	static const std::unordered_map<EditOperation, std::string> colorMap = {
		{ EditOperation::SimplifyQSlim,		"SimplifyQSlim"		},
		{ EditOperation::SimplifyOuterHull,	"SimplifyOuterHull"	},
		{ EditOperation::SimplifyInnerHull,	"SimplifyInnerHull"	},
	};
	auto it = colorMap.find(op);
	return it != colorMap.end() ? it->second : "Unknown";
}

class ControlState
{
public:
	ControlState()
		: op(EditOperation::None), selectActive(false) {}
	
public:
	EditOperation op;
	bool selectActive;
	glm::vec2 selectStart;
	glm::vec2 selectEnd;
};

namespace State {
	
	extern ControlState cState;

} // namespace State
} // namespace MeshDef