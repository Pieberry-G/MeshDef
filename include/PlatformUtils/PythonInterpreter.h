#pragma once

#include "PlatformUtils/PythonInterpreter.h"

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

namespace MeshDef {
	
	class PythonInterpreter
	{
	public:
		PythonInterpreter();
		
		pybind11::object ExecutePythonFun(const std::string& pythonfile, const std::string& func);

		static PythonInterpreter* Get() { return s_Instance; }
		
	private:
		std::unique_ptr<pybind11::scoped_interpreter> interpreter;
		bool bInitialized = false;

	private:
		static PythonInterpreter* s_Instance;
	};

}