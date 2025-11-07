#include "Panel/CustomUI.h"

#include "Core/Application.h"

#include <imgui.h>
#include <filesystem>

namespace MeshDef {

    void MeshProcessUI::Init()
    {

    }

    void MeshProcessUI::DrawUI()
    {
        ImGui::PushID("Mesh Process UI");
        ImGui::Begin("Mesh Process UI", nullptr);

        ImGui::InputFloat("Threshold", &m_QSlimThreshold, 0.1f, 0.0f, "%.2f");

        if (ImGui::Button("Simplify QSlim"))
        {
            AppUpdateEvent event("SimplifyQSlim");
            m_EventCallback(event);
        }

        if (ImGui::Button("Simplify OuterHull"))
        {
            AppUpdateEvent event("SimplifyOuterHull");
            m_EventCallback(event);
        }

        if (ImGui::Button("Simplify InnerHull"))
        {
            AppUpdateEvent event("SimplifyInnerHull");
            m_EventCallback(event);
        }
    }

} // namespace MeshDef