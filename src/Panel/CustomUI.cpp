#include "Panel/CustomUI.h"

#include "Core/Application.h"

#include <imgui.h>

namespace MeshDef {

    MeshProcessUI::MeshProcessUI()
    {
        polyscope::state::userCallbacks.push_back(std::bind(&MeshProcessUI::DrawUI, this));
    }

    void MeshProcessUI::DrawUI()
    {
        ImGui::PushID("Mesh Process UI");
        ImGui::Begin("Mesh Process UI", nullptr);

        ImGui::InputFloat("Threshold", &m_SimplificationThreshold, 0.1f, 0.0f, "%.2f");

        if (ImGui::Button("Simplify QSlim"))
        {
            MeshEditedEvent event(EditOperation::SimplifyQSlim);
            m_EventCallback(event);
        }

        if (ImGui::Button("Simplify OuterHull"))
        {
            MeshEditedEvent event(EditOperation::SimplifyOuterHull);
            m_EventCallback(event);
        }

        if (ImGui::Button("Simplify InnerHull"))
        {
            MeshEditedEvent event(EditOperation::SimplifyInnerHull);
            m_EventCallback(event);
        }

        ImGui::Separator();

        if (ImGui::Button("Set Vert Constraint"))
        {
            MeshEditedEvent event(EditOperation::SetVertConstraint);
            m_EventCallback(event);
        }

        if (ImGui::Button("Finish Deformation"))
        {
            MeshEditedEvent event(EditOperation::FinishDeformation);
            m_EventCallback(event);
        }
    }

} // namespace MeshDef