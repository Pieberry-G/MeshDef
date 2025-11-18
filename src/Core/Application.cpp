#include "Core/Application.h"
#include "Core/EntryPoint.h"

#include <polyscope/polyscope.h>
#include <GLFW/glfw3.h>

namespace MeshDef {

    Application* Application::s_Instance = nullptr;

    Application::Application()
    {
        MD_CORE_WARN("MeshDef Application Launch!");

        MD_CORE_ASSERT(!s_Instance, "Application already exists!");
        s_Instance = this;
        
        polyscope::init();
        InitEventSystem();

        m_Scene = std::make_unique<Scene>();
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
        dispatcher.Dispatch<MouseButtonPressedEvent>(MD_BIND_EVENT_FN(Application::OnMouseButtonPressed));
        dispatcher.Dispatch<MouseButtonReleasedEvent>(MD_BIND_EVENT_FN(Application::OnMouseButtonReleased));
        dispatcher.Dispatch<MouseMovedEvent>(MD_BIND_EVENT_FN(Application::OnMouseMoved));
        dispatcher.Dispatch<MeshEditedEvent>(MD_BIND_EVENT_FN(Application::OnMeshEdited));
    }

    bool Application::OnWindowClose(WindowCloseEvent& e)
    {
        polyscope::popContext();
        return true;
    }

    bool Application::OnKeyReleased(KeyReleasedEvent& e)
    {
        // if (e.IsRepeat()) return false;

        if (m_MainMenu.OnKeyReleased(e)) { return true; }
        if (m_Scene->OnKeyReleased(e)) { return true; }
        return false;
    }

    bool Application::OnMouseButtonPressed(MouseButtonPressedEvent& e)
    {
        // if (e.IsRepeat()) return false;

        if (m_Scene->OnMouseButtonPressed(e)) { return true; }
        return false;
    }

    bool Application::OnMouseButtonReleased(MouseButtonReleasedEvent& e)
    {
        // if (e.IsRepeat()) return false;

        if (m_Scene->OnMouseButtonReleased(e)) { return true; }
        return false;
    }

    bool Application::OnMouseMoved(MouseMovedEvent& e)
    {
        // if (e.IsRepeat()) return false;

        if (m_Scene->OnMouseMoved(e)) { return true; }
        return false;
    }

    bool Application::OnMeshEdited(MeshEditedEvent& e)
    {
        if (m_Scene->OnMeshEdited(e)) { return true; }
        return false;
    }

    void Application::InitEventSystem()
    {
        m_MeshProcessUI.SetEventCallback(MD_BIND_EVENT_FN(Application::OnEvent));
        
        auto window = (GLFWwindow*)polyscope::render::engine->getNativeWindow();

        static GLFWwindowsizefun originalWindowSizeCallback = glfwSetWindowSizeCallback(window, nullptr);
        static GLFWwindowclosefun originalWindowCloseCallback = glfwSetWindowCloseCallback(window, nullptr);
        static GLFWkeyfun originalKeyCallback = glfwSetKeyCallback(window, nullptr);
        static GLFWcharfun originalCharCallback = glfwSetCharCallback(window, nullptr);
        static GLFWmousebuttonfun originalMouseButtonCallback = glfwSetMouseButtonCallback(window, nullptr);
        static GLFWscrollfun originalScrollCallback = glfwSetScrollCallback(window, nullptr);
        static GLFWcursorposfun originalCursorPosCallback = glfwSetCursorPosCallback(window, nullptr);
        
        using EventCallbackFn = std::function<void(Event&)>;
        static EventCallbackFn eventCallback = MD_BIND_EVENT_FN(Application::OnEvent);
        
        glfwSetWindowSizeCallback(window, [](GLFWwindow* window, int width, int height) {
            WindowResizeEvent event(width, height);
            eventCallback(event);

            if (originalWindowSizeCallback)
            {
                originalWindowSizeCallback(window, width, height);
            }
        });

        glfwSetWindowCloseCallback(window, [](GLFWwindow* window) {
            WindowCloseEvent event;
            eventCallback(event);

            if (originalWindowCloseCallback)
            {
                originalWindowCloseCallback(window);
            }
        });

        glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
            switch (action)
            {
                case GLFW_PRESS:
                {
                    KeyPressedEvent event(key, false);
                    eventCallback(event);
                    break;
                }
                case GLFW_RELEASE:
                {
                    KeyReleasedEvent event(key);
                    eventCallback(event);
                    break;
                }
                case GLFW_REPEAT:
                {
                    KeyPressedEvent event(key, true);
                    eventCallback(event);
                    break;
                }
            }

            if (originalKeyCallback)
            {
                originalKeyCallback(window, key, scancode, action, mods);
            }
        });

        glfwSetCharCallback(window, [](GLFWwindow* window, unsigned int keycode) {
            KeyTypedEvent event(keycode);
            eventCallback(event);

            if (originalCharCallback)
            {
                originalCharCallback(window, keycode);
            }
        });

        glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods) {
            switch (action)
            {
                case GLFW_PRESS:
                {
                    MouseButtonPressedEvent event(button);
                    eventCallback(event);
                    break;
                }
                case GLFW_RELEASE:
                {
                    MouseButtonReleasedEvent event(button);
                    eventCallback(event);
                    break;
                }
            }

            if (originalMouseButtonCallback)
            {
                originalMouseButtonCallback(window, button, action, mods);
            }
        });

        glfwSetScrollCallback(window, [](GLFWwindow* window, double xOffset, double yOffset) {
            MouseScrolledEvent event((float)xOffset, (float)yOffset);
            eventCallback(event);

            if (originalScrollCallback)
            {
                originalScrollCallback(window, xOffset, yOffset);
            }
        });

        glfwSetCursorPosCallback(window, [](GLFWwindow* window, double xPos, double yPos) {
            MouseMovedEvent event((float)xPos, (float)(polyscope::view::windowHeight - yPos));
            eventCallback(event);

            if (originalCursorPosCallback)
            {
                originalCursorPosCallback(window, xPos, yPos);
            }
        });
    }

} // namespace MeshDef