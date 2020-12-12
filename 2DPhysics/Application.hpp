#pragma once

#include "Physics.hpp"

class Application {

public:

	Application();

	void Initialize();
	void Run();

private:

	void Update();
	void Render(ZorkLib::SimpleRenderer& sr);
	void OnKeyEvent(UINT32 key, bool down);
	void OnMouseEvent(UINT32 key, INT32 x, INT32 y, bool down);

	void ClearObjects();

	LARGE_INTEGER m_timerFreq, m_timerPrev, m_timerNow;

	ZorkLib::Window* m_pWindow;
	Physics m_Physics;
	std::vector<Object*> m_Objects;
	bool m_KeepRunning;
	

};