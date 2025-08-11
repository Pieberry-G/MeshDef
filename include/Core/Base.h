#pragma once

#include "Core/Log.h"

#ifdef MD_DEBUG
	#define MD_DEBUGBREAK() __debugbreak()
	#define MD_ENABLE_ASSERTS
#else
	#define MD_DEBUGBREAK()
#endif

#define MD_EXPAND_MACRO(x) x
#define MD_STRINGIFY_MACRO(x) #x

#ifdef MD_ENABLE_ASSERTS
	// Alteratively we could use the same "default" message for both "WITH_MSG" and "NO_MSG" and
	// provide support for custom formatting by concatenating the formatting string instead of having the format inside the default message
	#define MD_INTERNAL_ASSERT_IMPL(type, check, msg, ...) { if(!(check)) { MD##type##ERROR(msg, __VA_ARGS__); MD_DEBUGBREAK(); } }
	#define MD_INTERNAL_ASSERT_WITH_MSG(type, check, ...) MD_INTERNAL_ASSERT_IMPL(type, check, "Assertion failed: {0}", __VA_ARGS__)
	#define MD_INTERNAL_ASSERT_NO_MSG(type, check) MD_INTERNAL_ASSERT_IMPL(type, check, "Assertion '{0}' failed at {1}:{2}", MD_STRINGIFY_MACRO(check), std::filesystem::path(__FILE__).filename().string(), __LINE__)

	#define MD_INTERNAL_ASSERT_GET_MACRO_NAME(arg1, arg2, macro, ...) macro
	#define MD_INTERNAL_ASSERT_GET_MACRO(...) MD_EXPAND_MACRO( MD_INTERNAL_ASSERT_GET_MACRO_NAME(__VA_ARGS__, MD_INTERNAL_ASSERT_WITH_MSG, MD_INTERNAL_ASSERT_NO_MSG) )

	// Currently accepts at least the condition and one additional parameter (the message) being optional
	#define MD_ASSERT(...) MD_EXPAND_MACRO( MD_INTERNAL_ASSERT_GET_MACRO(__VA_ARGS__)(_, __VA_ARGS__) )
	#define MD_CORE_ASSERT(...) MD_EXPAND_MACRO( MD_INTERNAL_ASSERT_GET_MACRO(__VA_ARGS__)(_CORE_, __VA_ARGS__) )
#else
	#define MD_ASSERT(...)
	#define MD_CORE_ASSERT(...)
#endif

#define MD_BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }
