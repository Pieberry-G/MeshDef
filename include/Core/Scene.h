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

		void Tick();

		bool OnKeyPressed(KeyPressedEvent& e);
		bool OnKeyReleased(KeyReleasedEvent& e);
		bool OnMouseButtonPressed(MouseButtonPressedEvent& e);
		bool OnMouseButtonReleased(MouseButtonReleasedEvent& e);
		bool OnMouseMoved(MouseMovedEvent& e);
		bool OnMeshEdited(MeshEditedEvent& e);

		void LoadModelFromFile(const std::string& filepath);
		
	protected:
		void MeshSimplification(EditOperation op);
		void MeshDeformation(EditOperation op);

		void SelectVerts(int vertFlag);

	private:
		std::unique_ptr<Model> m_Model;
	};

} // namespace MeshDef