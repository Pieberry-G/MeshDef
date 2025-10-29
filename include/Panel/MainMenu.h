#pragma once

#include "EventSystem/Input.h"
#include "EventSystem/KeyEvent.h"

namespace MeshDef {

	class MainMenu
	{
	public:
		MainMenu();

		void DrawMainMenu();
		std::function<void()> GetDrawUIFunction() { return std::bind(&MainMenu::DrawMainMenu, this); }

		bool OnKeyReleased(KeyReleasedEvent& e);

		//void NewProject();
		//void ExportMesh();
		//void ExportMeshPart();
	};

} // namespace MeshDef