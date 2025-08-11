#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace MeshDef {

	class Log
	{
	public:
		static void Init();
		static std::shared_ptr<spdlog::logger>& GetCoreLogger();
		static std::shared_ptr<spdlog::logger>& GetClientLogger();
	private:
		static std::shared_ptr<spdlog::logger> s_CoreLogger;
		static std::shared_ptr<spdlog::logger> s_ClientLogger;
	};

} // namespace MeshDef

//Core log macros
#define MD_CORE_TRACE(...)	::MeshDef::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define MD_CORE_INFO(...)	::MeshDef::Log::GetCoreLogger()->info(__VA_ARGS__)
#define MD_CORE_WARN(...)	::MeshDef::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define MD_CORE_ERROR(...)	::MeshDef::Log::GetCoreLogger()->error(__VA_ARGS__)
#define MD_CORE_FATAL(...)	::MeshDef::Log::GetCoreLogger()->critical(__VA_ARGS__)

//Client log macros
#define MD_TRACE(...)		::MeshDef::Log::GetClientLogger()->trace(__VA_ARGS__)
#define MD_INFO(...)		::MeshDef::Log::GetClientLogger()->info(__VA_ARGS__)
#define MD_WARN(...)		::MeshDef::Log::GetClientLogger()->warn(__VA_ARGS__)
#define MD_ERROR(...)		::MeshDef::Log::GetClientLogger()->error(__VA_ARGS__)
#define MD_FATAL(...)		::MeshDef::Log::GetClientLogger()->critical(__VA_ARGS__)