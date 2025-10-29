#pragma once

namespace MeshDef {

	class GemSettingSelectionUI
	{
	public:
		void Init();

		void DrawUI();
		std::function<void()> GetDrawUIFunction() { return std::bind(&GemSettingSelectionUI::DrawUI, this); }
	};

	class GemPatternUI
	{
	public:
		void DrawUI();
		std::function<void()> GetDrawUIFunction() { return std::bind(&GemPatternUI::DrawUI, this); }
	};

} // namespace MeshDef