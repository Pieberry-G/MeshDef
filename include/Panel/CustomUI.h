#pragma once

#include "EventSystem/ApplicationEvent.h"

namespace MeshDef {

	class MeshProcessUI
	{
	public:
		MeshProcessUI();

		void DrawUI();

		const float GetSimplificationThreshold() const { return m_SimplificationThreshold; }

	private:
		float m_SimplificationThreshold = 0.8f;
	};

} // namespace MeshDef