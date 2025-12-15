#include "Core/Scene.h"

#include "Core/Application.h"
#include "Core/State.h"
#include "EventSystem/Input.h"
#include "Mesh/MeshUtils.h"
#include "Mesh/MultiViewCameras.h"

#include <pybind11/pybind11.h>

namespace MeshDef {

	void Scene::CalMovingCones()
	{
		std::vector<size_t> candidateHandles = CalculateCandidateHandles(m_Model->GetEditMesh()->getVertices(), m_Model->GetEditMesh()->getFaces());
		m_Model->ShowCandidateHandles(candidateHandles);
		
		MultiViewCameras cameras = MakeMultiViewCameras();
		std::string imagesDir = "../output/" + m_Model->GetName() + "/CandidateHandles/";
		m_Model->RenderMultiViewImages({1024, 1024}, cameras, imagesDir);

		//std::vector<size_t> selectedHandlesIndex = SelectHandlesByVLM(imagesDir);
		std::vector<size_t> selectedHandlesIndex = { 7, 13 };

		m_Model->RemoveAllQuantities();
		m_Model->ShowSelectedHandles(candidateHandles, selectedHandlesIndex);
	}

	Scene::Scene()
	{
		polyscope::state::tickSceneCallback = std::bind(&Scene::Tick, this);

		LoadModel("../assets/meshes/cow1.obj");
		//LoadModel("../assets/data/kitten.obj");

		// DeformTarget mesh(m_Model->GetEditMesh()->get_vertices(), m_Model->GetEditMesh()->get_faces());
		// InitializePython(mesh);

		CalMovingCones();
		// SelectHandles();
	}

	void Scene::Clean()
	{
		polyscope::removeAllStructures();
		m_Model = nullptr;
	}

	void Scene::Tick()
	{
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
			// case Key::R:
			// 	RenderMultiViewImages();
			// 	return true;
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
					return true;
				}
			case Mouse::ButtonRight:
				if (Input::IsKeyPressed(Key::A))
				{
					State::cState.selectActive = true;
					std::pair<float, float> mousePos = Input::GetMousePosition();
					State::cState.selectStart = glm::vec2(mousePos.first, mousePos.second);
					State::cState.selectEnd = glm::vec2(mousePos.first, mousePos.second);
					return true;
				}
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
					std::pair<float, float> mousePos = Input::GetMousePosition();
					State::cState.selectEnd = glm::vec2(mousePos.first, mousePos.second);
					SelectVerts(1);
					return true;
				}
			case Mouse::ButtonRight:
				if (Input::IsKeyPressed(Key::A))
				{
					State::cState.selectActive = false;
					std::pair<float, float> mousePos = Input::GetMousePosition();
					State::cState.selectEnd = glm::vec2(mousePos.first, mousePos.second);
					SelectVerts(2);
					return true;
			}
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

	bool Scene::OnMeshEdited(MeshEditedEvent& e)
	{
		EditOperation op = e.GetEditOperation();
		State::cState.op = op;
		switch (op)
		{
			case EditOperation::SimplifyQSlim:		{ MeshSimplification(op); return true; }
			case EditOperation::SimplifyOuterHull:	{ MeshSimplification(op); return true; }
			case EditOperation::SimplifyInnerHull:	{ MeshSimplification(op); return true; }
			case EditOperation::SetVertConstraint:	{ MeshDeformation(op); return true; }
			case EditOperation::FinishDeformation:	{ MeshDeformation(op); return true; }
			default:
				MD_CORE_ERROR("Could not find the relevant function!");
				return false;
		}
	}

	 void Scene::LoadModel(const std::string& filepath)
	 {
	 	m_Model = LoadModelFromFile(filepath);
	 	m_Model->DrawMeshToPolyscope();
	 }

	void Scene::MeshSimplification(EditOperation op)
	{
		const MeshProcessUI& meshProcessUI = Application::Get()->GetMeshProcessUI();
		float threshold = meshProcessUI.GetSimplificationThreshold();
		
		switch (op)
		{
			case EditOperation::SimplifyQSlim:
			{
				if (threshold > 0) { m_Model->GetEditMesh()->SimplifyQSlim(threshold, -1); }
				else { m_Model->GetEditMesh()->SimplifyQSlim(0.0f, 1); }
				break;
			}
			case EditOperation::SimplifyOuterHull:
			{
				if (threshold > 0) { m_Model->GetEditMesh()->SimplifyOuterHull(threshold, -1); }
				else { m_Model->GetEditMesh()->SimplifyOuterHull(0.0f, 1); }
				break;
			}
			case EditOperation::SimplifyInnerHull:
			{
				if (threshold > 0) { m_Model->GetEditMesh()->SimplifyInnerHull(threshold, -1); }
				else { m_Model->GetEditMesh()->SimplifyInnerHull(0.0f, 1); }
				break;
			}
		}
		
		m_Model->DrawMeshToPolyscope();
	}

	void Scene::MeshDeformation(EditOperation op)
	{
		switch (op)
		{
			case EditOperation::SetVertConstraint:
			{
				m_Model->GetEditMesh()->SetVertConstraint();
				break;
			}
			case EditOperation::FinishDeformation:
			{
				m_Model->GetEditMesh()->dragToDeform({ 0.5, 0, 0});
					
				// const EditMeshPtr editMesh = m_Model->GetEditMesh();
				//
				// int nFixedVerts = 0, nMovingVerts = 0;
				// for (int i = 0; i < editMesh->get_vert_size(); i++)
				// {
				// 	int vertFlag = editMesh->getVertFlag(i);
				// 	switch (vertFlag)
				// 	{
				// 		case 1: ++nFixedVerts; break;
				// 		case 2: ++nMovingVerts; break;
				// 	}
				// }
				//
				// Eigen::VectorXi fixedVertIndices(nFixedVerts);
				// Eigen::VectorXi movingVertIndices(nMovingVerts);
				// Eigen::MatrixXd targetPositions(nMovingVerts, 3);
				//
				// int fI = 0, mI = 0;
				// for (int i = 0; i < editMesh->get_vert_size(); i++)
				// {
				// 	int vertFlag = editMesh->getVertFlag(i);
				// 	switch (vertFlag)
				// 	{
				// 		case 1: fixedVertIndices(fI++) = i; break;
				// 		case 2: targetPositions.row(mI) = editMesh->get_vertex(i); movingVertIndices(mI++) = i; break;
				// 	}
				// }
				//
				// DeformMesh mesh;
				// mesh.setVertices(editMesh->get_vertices());
				// mesh.setFaces(editMesh->get_faces());
				// mesh.setFixedVertIndices(fixedVertIndices);
				// mesh.setMovingVertIndices(movingVertIndices);
				// mesh.setTargetPositions(targetPositions);
				//
				// InitializePython(mesh);

				m_Model->DrawMeshToPolyscope();
				break;
			}
		}
	}

	void Scene::SelectVerts(int vertFlag)
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
		glm::vec3 bl     = glm::unProject(glm::vec3(s_bl.x,s_bl.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 bl_ray = glm::unProject(glm::vec3(s_bl.x,s_bl.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 br     = glm::unProject(glm::vec3(s_tr.x,s_bl.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 br_ray = glm::unProject(glm::vec3(s_tr.x,s_bl.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 tr     = glm::unProject(glm::vec3(s_tr.x,s_tr.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 tr_ray = glm::unProject(glm::vec3(s_tr.x,s_tr.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 tl     = glm::unProject(glm::vec3(s_bl.x,s_tr.y,0), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));
		glm::vec3 tl_ray = glm::unProject(glm::vec3(s_bl.x,s_tr.y,1), polyscope::view::viewMat, polyscope::view::getCameraPerspectiveMatrix(), glm::vec4(0.0,0.0,polyscope::view::windowWidth,polyscope::view::windowHeight));

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
				else { m_Model->select_vert(i, vertFlag); }
			}
		}

		m_Model->DrawSelectedVertice();
	}

} // namespace MeshDef