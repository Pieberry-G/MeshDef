#include "Core/Scene.h"

#include "Core/Application.h"
#include "Core/State.h"
#include "EventSystem/Input.h"
#include "Mesh/MeshUtils.h"

namespace MeshDef {

	Scene::Scene()
	{
		polyscope::state::tickSceneCallback = std::bind(&Scene::Tick, this);

		LoadModelFromFile("../assets/meshes/cow1.obj");
	}

	void Scene::Clean()
	{
		polyscope::removeAllStructures();
		m_Model = nullptr;
	}

	void Scene::Tick()
	{
		if (State::cState.selectDirty)
		{
			glm::vec3 s_bl, s_tr;
			if (State::cState.selectStart.x > State::cState.selectEnd.x)
			{
				s_bl.x = State::cState.selectEnd.x;
				s_tr.x = State::cState.selectStart.x;
			}
			else
			{
				s_bl.x = State::cState.selectStart.x;
				s_tr.x = State::cState.selectEnd.x;
			}
			if (State::cState.selectStart.y > State::cState.selectEnd.y)
			{
				s_bl.y = State::cState.selectEnd.y;
				s_tr.y = State::cState.selectStart.y;
			}
			else
			{
				s_bl.y = State::cState.selectStart.y;
				s_tr.y = State::cState.selectEnd.y;
			}
			
			//determine which vertices are in the selection box
			GLint select_viewport[4];
			glGetIntegerv(GL_VIEWPORT,select_viewport);
			glm::vec3 bl     = glm::unProject(glm::vec3(s_bl.x,s_bl.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 bl_ray = glm::unProject(glm::vec3(s_bl.x,s_bl.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 br     = glm::unProject(glm::vec3(s_tr.x,s_bl.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 br_ray = glm::unProject(glm::vec3(s_tr.x,s_bl.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 tr     = glm::unProject(glm::vec3(s_tr.x,s_tr.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 tr_ray = glm::unProject(glm::vec3(s_tr.x,s_tr.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 tl     = glm::unProject(glm::vec3(s_bl.x,s_tr.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));
			glm::vec3 tl_ray = glm::unProject(glm::vec3(s_bl.x,s_tr.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,select_viewport[2],select_viewport[3]));

			int vert_size = m_Model->info_sizev();
			for (int i = 0; i < vert_size; i++)
			{
				Eigen::Vector3d vert = m_Model->info_vertex(i);
				if (VertInsideSelectBox(Eigen::Vector3d(bl.x,bl.y,bl.z),
										Eigen::Vector3d(bl_ray.x,bl_ray.y,bl_ray.z),
										Eigen::Vector3d(br.x,br.y,br.z),
										Eigen::Vector3d(br_ray.x,br_ray.y,br_ray.z),
										Eigen::Vector3d(tr.x,tr.y,tr.z),
										Eigen::Vector3d(tr_ray.x,tr_ray.y,tr_ray.z),
										Eigen::Vector3d(tl.x,tl.y,tl.z),
										Eigen::Vector3d(tl_ray.x,tl_ray.y,tl_ray.z),
										vert))
				{
					if (Input::IsKeyPressed(Key::LeftShift)) { m_Model->deselect_vert(i); }
					else { m_Model->select_vert(i); }
				}
			}

			m_Model->DrawSelectedVertice();

			State::cState.selectDirty = false;
		}
		
		if (State::cState.selectActive)
		{
			polyscope::drawSelectionBox(State::cState.selectStart, State::cState.selectEnd, glm::vec3(1.0f, 0.0f, 0.0f));
		}
	}
	
	bool Scene::OnKeyPressed(KeyPressedEvent& e)
	{
		return false;
	}

	bool Scene::OnKeyReleased(KeyReleasedEvent& e)
	{
		KeyCode key = e.GetKeyCode();
		switch (key)
		{
		case Key::A:
			State::cState.selectActive = false;
			return true;
		}
		return false;
	}

	bool Scene::OnMouseButtonPressed(MouseButtonPressedEvent& e)
	{
		MouseCode button = e.GetMouseButton();
		switch (button)
		{
		case Mouse::ButtonLeft:
			if (Input::IsKeyPressed(Key::A))
			{
				State::cState.selectActive = true;
				std::pair<float, float> mousePos = Input::GetMousePosition();
				State::cState.selectStart = glm::vec2(mousePos.first, mousePos.second);
				State::cState.selectEnd = glm::vec2(mousePos.first, mousePos.second);
			}
			return true;
		}
		return false;
	}

	bool Scene::OnMouseButtonReleased(MouseButtonReleasedEvent& e)
	{
		MouseCode button = e.GetMouseButton();
		switch (button)
		{
		case Mouse::ButtonLeft:
			if (Input::IsKeyPressed(Key::A))
			{
				State::cState.selectActive = false;
				State::cState.selectDirty = true;
				std::pair<float, float> mousePos = Input::GetMousePosition();
				State::cState.selectEnd = glm::vec2(mousePos.first, mousePos.second);
			}
			return true;
		}
		return false;
	}

	bool Scene::OnMouseMoved(MouseMovedEvent& e)
	{
		std::pair<float, float> mousePos = { e.GetX(), e.GetY() };

		if (Input::IsKeyPressed(Key::A) && State::cState.selectActive)
		{
			State::cState.selectEnd = glm::vec2(mousePos.first, mousePos.second);
			return true;
		}
		return false;
	}

	bool Scene::OnAppUpdate(AppUpdateEvent& e)
	{
		static const std::unordered_map<std::string, std::function<void()>> functionMap = {
			{ "SimplifyQSlim",		MD_BIND_EVENT_FN(Scene::OnSimplifyQSlim)	 },
			{ "SimplifyOuterHull",	MD_BIND_EVENT_FN(Scene::OnSimplifyOuterHull) },
			{ "SimplifyInnerHull",	MD_BIND_EVENT_FN(Scene::OnSimplifyInnerHull) },
		};

		std::string command = e.GetCommand();
		auto it = functionMap.find(command);
		if (it != functionMap.end())
		{
			it->second();
			return true;
		}
		else
		{
			MD_CORE_ERROR("Could not find the relevant function!");
			return false;
		}
	}

	bool Scene::OnAppRender(AppRenderEvent& e)
	{
		//static const std::unordered_map<std::string, std::function<void()>> functionMap = {
		//	{ "ImGuizmoUsed",			 GC_BIND_EVENT_FN(Scene::OnImGuizmoUsed)		  },
		//	{ "InteractiveSphereSelect", GC_BIND_EVENT_FN(Scene::InteractiveSphereSelect) },
		//	{ "InteractiveFillRegion",   GC_BIND_EVENT_FN(Scene::InteractiveFillRegion)   },
		//	{ "EraseSelectedFaces",		 GC_BIND_EVENT_FN(Scene::EraseSelectedFaces)	  },
		//	{ "AddSelectedFaces",		 GC_BIND_EVENT_FN(Scene::AddSelectedFaces)		  },
		//};

		//std::string command = e.GetCommand();
		//auto it = functionMap.find(command);
		//if (it != functionMap.end()) {
		//	it->second();
		//	return true;
		//} else {
		//	GC_CORE_ERROR("Could not find the relevant function!");
		//	return false;
		//}
		return false;
	}

	 void Scene::LoadModelFromFile(const std::string& filepath)
	 {
	 	m_Model = loadModelFromFile(filepath);
	 	m_Model->DrawMeshToPolyscope();
	 }

	void Scene::OnSimplifyQSlim()
	{
		const MeshProcessUI& meshProcessUI = Application::Get()->GetMeshProcessUI();
		float QSlimThreshold = meshProcessUI.GetQSlimThreshold();
		if (QSlimThreshold > 0)
		{
			m_Model->GetEditMesh()->SimplifyQSlim(QSlimThreshold, -1);
		}
		else
		{
			m_Model->GetEditMesh()->SimplifyQSlim(0.0f, 1);
		}
		m_Model->DrawMeshToPolyscope();
	}

	void Scene::OnSimplifyOuterHull()
	{
		const MeshProcessUI& meshProcessUI = Application::Get()->GetMeshProcessUI();
		float QSlimThreshold = meshProcessUI.GetQSlimThreshold();
		if (QSlimThreshold > 0)
		{
			m_Model->GetEditMesh()->SimplifyOuterHull(QSlimThreshold, -1);
		}
		else
		{
			m_Model->GetEditMesh()->SimplifyOuterHull(0.0f, 1);
		}
		m_Model->DrawMeshToPolyscope();
	}

	void Scene::OnSimplifyInnerHull()
	{
		const MeshProcessUI& meshProcessUI = Application::Get()->GetMeshProcessUI();
		float QSlimThreshold = meshProcessUI.GetQSlimThreshold();
		if (QSlimThreshold > 0)
		{
			m_Model->GetEditMesh()->SimplifyInnerHull(QSlimThreshold, -1);
		}
		else
		{
			m_Model->GetEditMesh()->SimplifyInnerHull(0.0f, 1);
		}
		m_Model->DrawMeshToPolyscope();
	}

} // namespace MeshDef