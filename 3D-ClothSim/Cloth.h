#pragma once
#include "Geometry.h"
#include "3D-ClothSimMain.h"

class ClothParticle;
class ClothConstraint;

class Cloth
{
public:
	Cloth(u32 x, u32 y, float spacing);
	~Cloth();

	void Initialise();
	void Update();
	void Render();
	void Destroy();

	void ApplyExternalForceToRadius(Vector3f pos, float radius);

	Geometry::Quad& GetQuad() { return m_quad; }

	const Vector3f GetGravity() const {return m_gravity;}
	const float GetDrag() const {return m_dragCoeff;}
	const Vector3f GetWindForce() const {return m_windForce;}

private:

	void CreateClothMesh();
	void CreateMaterials();
	void CreateParticles();
	void CreateConstraints();

	void UpdatePositionBuffer();
	void RecalculateNormals();
	void UpdateNormalBuffer();

	void CalculateForces();
	void CalculateWindForce();


	Geometry::Quad m_quad;

	std::vector<Vector3f> m_positions;
	std::vector<Vector3f> m_normals;
	std::vector<u32> m_indices;

	std::vector<ClothParticle*> m_vParticles;
	std::vector<ClothConstraint*> m_vConstraints;

	std::vector<ClothParticle*> m_cornerPoints;

	Vector3f m_gravity{ 0.0f, Y_GRAVITY, 0.0f };
	Vector3f m_windForce{ 0, 0, 0 };

	Graphics::MeshId m_clothMesh;
	Graphics::MaterialId m_wireframeMat;
	Graphics::MaterialId m_solidMat;

	float m_dragCoeff{ AIR_DRAG_COEFFICIENT };

	float m_spacing;
	u32 m_noPoints;
	u32 m_x;
	u32 m_y;
};

