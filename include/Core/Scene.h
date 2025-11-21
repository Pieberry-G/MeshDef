#pragma once

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

		void LoadModel(const std::string& filePath);
		void MeshSimplification(EditOperation op);
		void MeshDeformation(EditOperation op);

		void RenderMultiViewImages(glm::vec2 imageSize = {1024, 1024});
		
	protected:
		void SelectVerts(int vertFlag);

	private:
		std::unique_ptr<Model> m_Model;
	};

} // namespace MeshDef