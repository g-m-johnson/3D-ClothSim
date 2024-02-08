#include "Mouse.h"
#include "Collision.h"
#include "ObjectManager.h"
#include "Cloth.h"

void Mouse::Update()
{
	Matrix4x4f view = Graphics::GetViewMatrix();
	Matrix4x4f project = Graphics::GetProjectionMatrix();
	Vector3f camPos = Demo::GetEyePosition();

	Vector3f rayDir = GetRayDirection(view, project, camPos);

	m_ray = Geometry::Ray(m_mouseWorld, m_mouseWorld + 100.f * rayDir);

	Cloth* c = ObjectManager::Instance().GetClothPtr();
	Vector3f i;
	if (Collision::IntersectRayQuad(m_ray, c->GetQuad(), i))
	{	
		Graphics::BeginPrimitiveBatch();
		Graphics::DrawLine(m_ray.start, Vector3f(0,0,0), Colour::Red);
		Graphics::EndPrimitiveBatch();

		if (Input::GetMouseState().m_leftButton && m_isMouseReleased)
		{
			ObjectManager::Instance().GetClothPtr()->ApplyExternalForce(i);
			m_isMouseReleased = false;
		}
	}

	if (!Input::GetMouseState().m_leftButton)
	{
		m_isMouseReleased = true;
	}
}

Play3d::Vector3f Mouse::GetRayDirection(const Matrix4x4f& viewMatrix, const Matrix4x4f& projectMatrix, const Vector3f& cameraPosWorldSpace)
{
	// Get mouse pos and screen size
	Vector2f m = Vector2f(Input::GetMouseState().m_x, Input::GetMouseState().m_y);
	Vector2f s = Vector2f(Graphics::GetDisplaySurfaceSize().m_width, Graphics::GetDisplaySurfaceSize().m_height);

	// 3d Normalised Device Coordinates
	float nds_x = (2.f * m.x) / s.x - 1.f;
	float nds_y = 1.f - (2.f * m.y) / s.y;
	Vector4f nds = Vector4f(nds_x, nds_y, 0.f, 1.f);

	Matrix4x4f invViewMat = AffineInverse(viewMatrix);
	Matrix4x4f invProjectMat = InversePerspectiveProjectRH(projectMatrix);

	Vector4f mouseCamPos = Transform(invProjectMat, nds);
	Vector4f mouseWorld = Transform(invViewMat, mouseCamPos);
	mouseWorld = mouseWorld / mouseWorld.w;
	m_mouseWorld = mouseWorld.xyz();
	Vector3f rayDir = mouseWorld.xyz() - cameraPosWorldSpace;

	return normalize(rayDir);
}

void Mouse::Raycast()
{
	Matrix4x4f view = Graphics::GetViewMatrix();
	Matrix4x4f project = Graphics::GetProjectionMatrix();
	Vector3f camPos = Demo::GetEyePosition();

	Vector3f rayDir = GetRayDirection(view, project, camPos);
}

void Mouse::DebugDrawMouseRay()
{
	Matrix4x4f view = Graphics::GetViewMatrix();
	Matrix4x4f project = Graphics::GetProjectionMatrix();
	Vector3f camPos = Demo::GetEyePosition();

	Vector3f rayDir = GetRayDirection(view, project, camPos);

	Vector3f rayEnd = m_mouseWorld + 100.f * rayDir;
	Graphics::DrawLine(m_mouseWorld, rayEnd, Colour::White);

}
