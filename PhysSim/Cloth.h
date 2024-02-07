#pragma once
#include "Geometry.h"

class ClothPoint;
class ClothStick;

class Cloth
{
public:
	Cloth(float width, float height);
	~Cloth();

	void Initialise();
	void Update();
	void Render();
	void Destroy();

	void CalculateForces();

	Graphics::MeshId& GetClothMesh(){ return m_clothMesh; }

	Geometry::Plane& GetPlane() { return m_plane; }
	Geometry::Quad& GetQuad() { return m_quad; }

	std::vector<ClothPoint*> GetCorners() { return m_cornerPoints; }

	const Vector3f GetGravity() const {return m_gravity;}
	const float GetDrag() const {return m_dragCoeff;}
	const Vector3f GetWindForce() const {return m_windForce;}

	void UpdatePositionBuffer();
	void RecalculateNormals();
	void UpdateNormalBuffer();

private:
	void CreateClothMesh();
	void CreateMaterials();
	void CreatePointsAndSticks();

	std::vector<Vector3f> m_positions;
	std::vector<u32> m_indices;
	std::vector<Vector3f> m_normals;

	std::vector<ClothPoint*> m_vPoints;
	std::vector<ClothStick*> m_vSticks;

	std::vector<ClothPoint*> m_cornerPoints;

	Geometry::Plane m_plane;
	Geometry::Quad m_quad;

	Vector3f m_gravity{ 0.0f, -9.81f, 0.0f };
	Vector3f m_windForce{ 0, 0, 0 };

	Graphics::MaterialId m_wireframeMat;
	Graphics::MaterialId m_solidMat;

	Graphics::MeshId m_clothMesh;

	float m_mass{  1460.f };
	float m_dragCoeff{ 0.1f };

	float m_width;
	float m_height;

	int m_noPoints;
};

