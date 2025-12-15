#pragma once

#include "Mesh/DeformMesh.h"

namespace MeshDef {

	class FileDialogs
	{
	public:
		// These return empty string if cancelled
		static std::string OpenFile(const char* filter);
		static std::string SaveFile(const char* filter);
	};

}