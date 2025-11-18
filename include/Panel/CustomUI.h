#pragma once

#include "EventSystem/ApplicationEvent.h"

namespace MeshDef {

	class MeshProcessUI
	{
	public:
		MeshProcessUI();

		void DrawUI();

		// Event callback
		using EventCallbackFn = std::function<void(MeshDef::Event&)>;
		const EventCallbackFn& GetEventCallbackFn() const { return m_EventCallback; }
		void SetEventCallback(const EventCallbackFn& callback) { m_EventCallback = callback; }

		const float GetSimplificationThreshold() const { return m_SimplificationThreshold; }

	private:
		float m_SimplificationThreshold = 0.8f;

		EventCallbackFn m_EventCallback;
	};

} // namespace MeshDef