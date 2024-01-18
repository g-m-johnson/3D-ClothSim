#pragma once
#include "../Play3d/Play3d.h"
using namespace Play3d;

class ClothPoint;
class ClothStick;

class Cloth
{
public:
	Cloth(int width, int height, int spacing);
	~Cloth();

	void Initialise();
	void Update();
	void Render();
	void Destroy();

	Graphics::MeshId& GetClothMesh(){ return m_clothMesh; }

	const Vector3f GetGravity() const {return m_gravity;}
	const float GetDrag() const {return m_drag;}
	const float GetElasticity() const {return m_elasticity;}

	void UpdateBuffer();

private:
	void CreateClothMesh();
	void CreatePointsAndSticks();

	std::vector<Vector3f> m_positions;

	Vector3f m_gravity{ 0.0f, -0.981f, 0.0f };
	
	std::vector<ClothPoint*> m_vPoints;
	std::vector<ClothStick*> m_vSticks;

	Graphics::MeshId m_clothMesh;

	float m_drag{ 0.00001f };
	float m_elasticity{ 0.01f };

	float m_width;
	float m_height;
	float m_spacing;

};

