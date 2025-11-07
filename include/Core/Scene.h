#pragma once

#include "Panel/CustomUI.h"

#include "EventSystem/ApplicationEvent.h"
#include "EventSystem/KeyEvent.h"
#include "EventSystem/MouseEvent.h"

#include "Mesh/Model.h"

namespace MeshDef {

	class Scene
	{
	public:
		Scene();
		void Clean();

		bool OnKeyReleased(KeyReleasedEvent& e);
		bool OnAppUpdate(AppUpdateEvent& e);
		bool OnAppRender(AppRenderEvent& e);

		void LoadModelFromFile(const std::string& filepath);
		
		MeshProcessUI& GetMeshProcessUI() { return m_MeshProcessUI; }

	protected:
		void OnSimplifyQSlim();
		void OnSimplifyOuterHull();
		void OnSimplifyInnerHull();

	private:
		std::unique_ptr<Model> m_Model;

		MeshProcessUI m_MeshProcessUI;
	};

} // namespace MeshDef