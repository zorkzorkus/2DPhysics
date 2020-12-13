#include "Application.hpp"

Application::Application() : m_pWindow(nullptr), m_Physics(m_Objects) {}

void Application::Initialize() {

	m_pWindow = new ZorkLib::Window(L"2DPhysics");
	m_pWindow->SetSimpleRenderFunction(std::bind(&Application::Render, this, std::placeholders::_1));
	m_pWindow->SetKeyEventFunction(std::bind(&Application::OnKeyEvent, this, std::placeholders::_1, std::placeholders::_2));
	m_pWindow->SetMouseEventFunction(std::bind(&Application::OnMouseEvent, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	m_KeepRunning = true;
	QueryPerformanceCounter(&m_timerPrev);
	QueryPerformanceFrequency(&m_timerFreq);
}

void Application::Run() {

	while (m_KeepRunning) {
		m_pWindow->HandleMessages();
		m_pWindow->Render();
		Update();
	}

}

void Application::Update() {

	QueryPerformanceCounter(&m_timerNow);
	float elapsedTime = static_cast<float>(m_timerNow.QuadPart - m_timerPrev.QuadPart) / static_cast<float>(m_timerFreq.QuadPart);
	if (elapsedTime < 0.001) return;
	m_timerPrev = m_timerNow;

	m_Physics.Update(elapsedTime * sSpeedFactor);

}

void Application::Render(ZorkLib::SimpleRenderer& sr) {

	sr.Fill(ZorkLib::ColorEnum::White);

	for (auto& obj : m_Objects) {

		Vector2f pos = obj->Position();
		pos(1) = 1440.f - pos(1);

		if (Circle* circle = dynamic_cast<Circle*>(obj)) {
			auto c = ZorkLib::Ellipse(pos(0), pos(1), circle->Radius());
			sr.FillEllipse(c, obj->m_Color, circle->Angle());
		} else if (Box* box = dynamic_cast<Box*>(obj)) {
			auto r = ZorkLib::Rectangle::RectCentered(pos(0), pos(1), box->HalfSize()(0) * 2.f, box->HalfSize()(1) * 2.f);
			sr.FillRectangle(r, obj->m_Color, box->Angle());
		}


	}

}

void Application::OnKeyEvent(UINT32 key, bool down) {

	Object* newObject;
	ZorkLib::Point cursorPos = m_pWindow->GetCursorPos();
	float x = cursorPos.x;
	float y = 1440.f - cursorPos.y;

	if (down) { // DOWN

	} else { // UP

		switch (key) {
			case VK_ESCAPE:
				m_KeepRunning = false;
				break;
			case VK_SPACE:
				ClearObjects();
				break;
			case 0x30:
				newObject = new Box(Vector2f(15.f, 15.f), 15.f, Vector2f(x, y));
				newObject->Acceleration() = Vector2f(0.f, cGravity);
				newObject->Angle() = 45.f;
				m_Objects.push_back(newObject);
				break;
			case 0x31:
				newObject = new Box(Vector2f(300, 30), cInfiniteMass, Vector2f(x, y));
				newObject->Angle() = 180;
				m_Objects.push_back(newObject);
				break;
			case 0x32:
				newObject = new Box(Vector2f(30, 300), cInfiniteMass, Vector2f(x, y));
				m_Objects.push_back(newObject);
				break;
			case 0x33:
				newObject = new Box(Vector2f(160.f, 20.f), 999.f, Vector2f(x, y));
				newObject->AngularVelocity() = -5.f;
				newObject->Acceleration() = Vector2f(0.f, 0.f);
				m_Objects.push_back(newObject);
				break;
			case 0x34:
				newObject = new Box(Vector2f(100, 100), 999.f, Vector2f(1280, 720));
				newObject->Acceleration() = Vector2f(0.f, 0.f);
				m_Objects.push_back(newObject);
				// box1 goes to H = 820, we want tiny intersection: sqrt(2)*50 - 5
				newObject = new Box(Vector2f(50, 50), 999.f, Vector2f(1280, 720 + 100 + sqrt(2) * 50.f - 5));
				newObject->Angle() = 45.f;
				newObject->Acceleration() = Vector2f(0.f, 0.f);
				m_Objects.push_back(newObject);
				break;
		}

	}

}

void Application::OnMouseEvent(UINT32 key, INT32 x, INT32 y, bool down) {

	Object* newObject;

	if (down) { // DOWN

		switch (key) {
			case VK_LBUTTON:

				newObject = new Box(Vector2f(15.f, 15.f), 15.f, Vector2f(x, 1440.f - y));
				//newObject = new Circle(15.f, 15.f, Vector2f(x, 1440.f - y));
				newObject->Acceleration() = Vector2f(0.f, cGravity);
				m_Objects.push_back(newObject);

				break;

			case VK_RBUTTON:

				newObject = new Box(Vector2f(100.f, 30.f), cInfiniteMass, Vector2f(x, 1440.f - y));
				newObject->Acceleration() = Vector2f(0.f, cGravity);
				m_Objects.push_back(newObject);

				break;

			case VK_MBUTTON:

				newObject = new Box(Vector2f(160.f, 20.f), 999.f, Vector2f(x, 1440.f - y));
				newObject->AngularVelocity() = 5.f;
				newObject->Acceleration() = Vector2f(0.f, cGravity);
				m_Objects.push_back(newObject);

				break;
		}

	} else { // UP



	}

}

void Application::ClearObjects() {
	for (auto& obj : m_Objects) {
		delete obj;
	}
	m_Objects.clear();
}
