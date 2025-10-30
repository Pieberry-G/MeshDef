#include "Core/Application.h"
#include "Core/EntryPoint.h"

//#include "TinyRenderer/RenderTool.h"
//
//#include "Mesh/SurfaceCleanerTool.h"
//#include "Mesh/RegionSelectionTool.h"
//#include "Mesh/GeometryTool.h"
//#include "Mesh/PlacementTool.h"
//#include "Mesh/BooleanTool.h"

#include <polyscope/polyscope.h>

namespace MeshDef {

    Application* Application::s_Instance = nullptr;

    Application::Application()
    {
        MD_CORE_WARN("MeshDef Application Launch!");

        MD_CORE_ASSERT(!s_Instance, "Application already exists!");
        s_Instance = this;

        polyscope::init();
        //polyscope::render::engine->setEventCallback(MD_BIND_EVENT_FN(Application::OnEvent));
        //polyscope::registerGroup("Gems");
        //polyscope::registerGroup("GemSettings");

        //TinyRenderer::RenderTool::Init();

        //m_ResourceManager = ResourceManager::Get();
        //m_ResourceManager->PreloadGemSettings();
        //m_ResourceManager->PreloadMandrel();
        //m_ResourceManager->PreloadCylinder();

        //polyscope::state::edgeLengthScale = 0.3;

        m_Scene = std::make_unique<Scene>();
        m_Scene->GetMeshProcessUI().SetEventCallback(MD_BIND_EVENT_FN(Application::OnEvent));
    }

    void Application::Run()
    {
        // Give control to the polyscope gui
        polyscope::show();
    }

    void Application::OnEvent(Event& e)
    {
        EventDispatcher dispatcher(e);
        dispatcher.Dispatch<WindowCloseEvent>(MD_BIND_EVENT_FN(OnWindowClose));
        dispatcher.Dispatch<KeyReleasedEvent>(MD_BIND_EVENT_FN(Application::OnKeyReleased));
        dispatcher.Dispatch<AppUpdateEvent>(MD_BIND_EVENT_FN(Application::OnAppUpdate));
        dispatcher.Dispatch<AppRenderEvent>(MD_BIND_EVENT_FN(Application::OnAppRender));
    }

    bool Application::OnWindowClose(WindowCloseEvent& e)
    {
        polyscope::popContext();
        return true;
    }

    bool Application::OnKeyReleased(KeyReleasedEvent& e)
    {
        // if (e.IsRepeat()) return false;

        if (m_MainMenu.OnKeyReleased(e)) return true;
        if (m_Scene->OnKeyReleased(e)) return true;
        return false;
    }

    bool Application::OnAppUpdate(AppUpdateEvent& e)
    {
        if (m_Scene->OnAppUpdate(e)) return true;
        return false;
    }

    bool Application::OnAppRender(AppRenderEvent& e)
    {
        if (m_Scene->OnAppRender(e)) return true;
        return false;
    }

} // namespace MeshDef