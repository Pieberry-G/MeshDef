#include "Core/Scene.h"

#include "Mesh/MeshUtils.h"

namespace MeshDef {

	Scene::Scene()
	{
		m_MeshProcessUI.Init();
		polyscope::state::userCallbacks.push_back(m_MeshProcessUI.GetDrawUIFunction());

		m_Model = loadModelFromFile("../assets/meshes/cow1.obj");
		m_Model->DrawMeshToPolyscope();
	}

	void Scene::Clean()
	{
		
	}

	bool Scene::OnKeyReleased(KeyReleasedEvent& e)
	{
		//static const std::unordered_map<KeyCode, std::function<void()>> functionMap = {
		//	{ Key::Q, GC_BIND_EVENT_FN(Scene::AutoSelectRegion)		     },
		//	{ Key::W, GC_BIND_EVENT_FN(Scene::RepairSelectedRegion)      },
		//	{ Key::E, GC_BIND_EVENT_FN(Scene::DigHoleOnSelectedRegion)	 },
		//	{ Key::R, GC_BIND_EVENT_FN(Scene::PlaceGemsOnSelectedRegion) },
		//	{ Key::T, GC_BIND_EVENT_FN(Scene::BooleanOpDifference)		 },

		//	{ Key::Y, GC_BIND_EVENT_FN(Scene::CleanSurface)				 },
		//	{ Key::U, GC_BIND_EVENT_FN(Scene::AutoRecognizeGems)		 },
		//	{ Key::I, GC_BIND_EVENT_FN(Scene::PlaceGemsAtTargets)		 },
		//	{ Key::O, GC_BIND_EVENT_FN(Scene::BooleanOpDifference)		 },
		//};

		//KeyCode key = e.GetKeyCode();
		//auto it = functionMap.find(key);
		//if (it != functionMap.end()) {
		//	it->second();
		//	return true;
		//}
		//else {
		//	return false;
		//}
		return false;
	}

	bool Scene::OnAppUpdate(AppUpdateEvent& e)
	{
		static const std::unordered_map<std::string, std::function<void()>> functionMap = {
			{ "ExecuteQSlim",			 MD_BIND_EVENT_FN(Scene::OnExecuteQSlim)		  },
		};

		std::string command = e.GetCommand();
		auto it = functionMap.find(command);
		if (it != functionMap.end()) {
			it->second();
			return true;
		} else {
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

	void Scene::OnExecuteQSlim()
	{
		float QSlimThreshold = m_MeshProcessUI.GetQSlimThreshold();
		if (QSlimThreshold > 0)
		{
			m_Model->GetEditMesh()->collapseEdge_QSlim(QSlimThreshold, -1);
		}
		else
		{
			m_Model->GetEditMesh()->collapseEdge_QSlim(0.0f, 1);
		}
		m_Model->DrawMeshToPolyscope();
	}

} // namespace MeshDef