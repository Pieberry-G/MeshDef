#include "Panel/CustomUI.h"

#include "Core/Application.h"
#include "Core/Scene.h"

#include <imgui.h>

namespace MeshDef {

    MeshProcessUI::MeshProcessUI()
    {
        polyscope::state::userCallbacks.push_back(std::bind(&MeshProcessUI::DrawUI, this));
    }

    void MeshProcessUI::DrawUI()
    {
        std::unique_ptr<Scene>& scene = Application::Get()->GetScene();
        
        ImGui::PushID("Mesh Process UI");
        ImGui::Begin("Mesh Process UI", nullptr);

        ImGui::InputFloat("Threshold", &m_SimplificationThreshold, 0.1f, 0.0f, "%.2f");

        if (ImGui::Button("Simplify QSlim"))
        {
            scene->MeshSimplification(EditOperation::SimplifyQSlim);
        }

        if (ImGui::Button("Simplify OuterHull"))
        {
            scene->MeshSimplification(EditOperation::SimplifyOuterHull);
        }

        if (ImGui::Button("Simplify InnerHull"))
        {
            scene->MeshSimplification(EditOperation::SimplifyInnerHull);
        }

        ImGui::Separator();

        if (ImGui::Button("Set Vert Constraint"))
        {
            scene->MeshDeformation(EditOperation::SetVertConstraint);
        }

        if (ImGui::Button("Finish Deformation"))
        {
            scene->MeshDeformation(EditOperation::FinishDeformation);
        }
    }

} // namespace MeshDef