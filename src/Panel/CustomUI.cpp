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

        if (ImGui::InputFloat("Threshold", &m_QSlimThreshold, 0.1f, 0.0f, "%.2f"))
        {
            if (m_QSlimThreshold < 0.0f)
            {
                m_QSlimThreshold = 0.0f;
            }
        }

        if (ImGui::Button("Execute QSlim"))
        {
            AppUpdateEvent event("ExecuteQSlim");
            m_EventCallback(event);
        }
    }

} // namespace MeshDef