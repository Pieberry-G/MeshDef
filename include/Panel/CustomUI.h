#pragma once

#include "EventSystem/ApplicationEvent.h"

namespace MeshDef {

	class MeshProcessUI
	{
	public:
		void Init();

		void DrawUI();
		std::function<void()> GetDrawUIFunction() { return std::bind(&MeshProcessUI::DrawUI, this); }

		// Event callback
		using EventCallbackFn = std::function<void(MeshDef::Event&)>;
		const EventCallbackFn& GetEventCallbackFn() const { return m_EventCallback; }
		void SetEventCallback(const EventCallbackFn& callback) { m_EventCallback = callback; }

		float GetQSlimThreshold() { return m_QSlimThreshold; }

	private:
		float m_QSlimThreshold = 0.0f;

		EventCallbackFn m_EventCallback;
	};

} // namespace MeshDef