#pragma once

int main()
{
	MeshDef::Log::Init();
	MD_CORE_WARN("Log system initialized!");

	MeshDef::Application* app = new MeshDef::Application();
	app->Run();
	delete app;
}