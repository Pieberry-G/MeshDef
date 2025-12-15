#include "PlatformUtils/PythonInterpreter.h"

#include <Windows.h>

#include <codecvt>
#include <fileutils.h>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

namespace MeshDef {

	PythonInterpreter* PythonInterpreter::s_Instance = nullptr;

	PythonInterpreter::PythonInterpreter()
	{
		MD_CORE_ASSERT(!s_Instance, "Python Interpreter already exists!");
		s_Instance = this;

		std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
		std::wstring pythonHome = converter.from_bytes(PYTHON_HOME);
			
		PyConfig config;
		PyConfig_InitPythonConfig(&config);
		PyConfig_SetString(&config, &config.home, pythonHome.c_str());
			
		std::wstring condaEnvDllDir = pythonHome + L"/Library/bin";
		SetDllDirectoryW(condaEnvDllDir.c_str());
			
		interpreter = std::make_unique<pybind11::scoped_interpreter>(&config);
		
		pybind11::module::import("sys").attr("path").attr("append")("../python");
		
		bInitialized = true;
	}

	pybind11::object PythonInterpreter::ExecutePythonFun(const std::string& pythonfile, const std::string& func)
	{
		pybind11::module::import("sys").attr("path").attr("append")("../python");
		
		pybind11::object result = pybind11::module_::import(pythonfile.c_str()).attr(func.c_str())();

		return result;
	}

} // namespace MeshDef