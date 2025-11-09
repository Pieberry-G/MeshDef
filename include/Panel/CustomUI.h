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

		const float GetQSlimThreshold() const { return m_QSlimThreshold; }

	private:
		float m_QSlimThreshold = -0.1f;

		EventCallbackFn m_EventCallback;
	};

} // namespace MeshDef