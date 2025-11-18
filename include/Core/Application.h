#pragma once

#include "Panel/MainMenu.h"
#include "Core/Scene.h"

#include "EventSystem/ApplicationEvent.h"
#include "EventSystem/KeyEvent.h"
#include "EventSystem/MouseEvent.h"

namespace MeshDef {

	class Application
	{
	public:
		Application();

		void Run();
		
		void OnEvent(Event& e);
		bool OnWindowClose(WindowCloseEvent& e);
		bool OnKeyReleased(KeyReleasedEvent& e);
		bool OnMouseButtonPressed(MouseButtonPressedEvent& e);
		bool OnMouseButtonReleased(MouseButtonReleasedEvent& e);
		bool OnMouseMoved(MouseMovedEvent& e);
		bool OnMeshEdited(MeshEditedEvent& e);

		std::unique_ptr<Scene>& GetScene() { return m_Scene; }
		const MeshProcessUI& GetMeshProcessUI() const { return m_MeshProcessUI; }

		static Application* Get() { return s_Instance; }

	protected:
		void InitEventSystem();
		
	private:
		MainMenu m_MainMenu;
		MeshProcessUI m_MeshProcessUI;
		
		std::unique_ptr<Scene> m_Scene;

	private:
		static Application* s_Instance;
	};

} // namespace MeshDef