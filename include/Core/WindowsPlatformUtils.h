#pragma once

#include "Mesh/SimpleMesh.h"

namespace MeshDef {

	void InitializePython(const SimpleMesh& mesh);

	class FileDialogs
	{
	public:
		// These return empty string if cancelled
		static std::string OpenFile(const char* filter);
		static std::string SaveFile(const char* filter);
	};

}