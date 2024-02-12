#include "Cloth.h"
#include "ClothParticle.h"
#include "ClothConstraint.h"

Cloth::Cloth(u32 x, u32 y, float spacing)
	: m_x(x)
	, m_y(y)
	, m_spacing(spacing)
	, m_noPoints(x * y)
{
	m_positions.resize(m_noPoints);
	m_normals.resize(m_noPoints);
}

Cloth::~Cloth()
{
	Destroy();
}

void Cloth::Initialise()
{
	CreateMaterials();
	CreateClothMesh();
	CreatePointsAndSticks();

	m_plane = Geometry::Plane(Vector3f(0, 0, 1), 0);
	m_quad = Geometry::Quad( m_cornerPoints[0]->GetPosition(),
							 m_cornerPoints[1]->GetPosition(), 
							 m_cornerPoints[2]->GetPosition(), 
							 m_cornerPoints[3]->GetPosition() );

}


void Cloth::UpdatePositionBuffer()
{
	Graphics::Mesh* m = Resources::GetPtr<Graphics::Mesh>(m_clothMesh);
	ID3D11DeviceContext* pDC = Graphics::GetDeviceContext();
	D3D11_MAPPED_SUBRESOURCE data;

	ID3D11Buffer* pBuffer = m->GetStreamBufferPtr(0);
	HRESULT hrPos = pDC->Map(pBuffer, 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
	if (SUCCEEDED(hrPos))
	{
		memcpy(data.pData, m_positions.data(), sizeof(Vector3f) * m_positions.size());
		pDC->Unmap(pBuffer, 0);
	}
}

void Cloth::RecalculateNormals()
{
	std::fill(m_normals.begin(), m_normals.end(), Vector3f(0,0,0));

	for (int i = 0; i < m_indices.size(); i += 3)
	{
		Vector3f AB = m_positions[m_indices[i + 1]] - m_positions[m_indices[i]];
		Vector3f AC = m_positions[m_indices[i + 2]] - m_positions[m_indices[i]];
		Vector3f fN = cross(AB, AC);
		fN = normalize(fN);

		m_normals[m_indices[i]] += fN;
		m_normals[m_indices[i + 1]] += fN;
		m_normals[m_indices[i + 2]] += fN;
	}

	for (int i = 0; i < m_normals.size(); i++)
	{
		m_normals[i] = normalize(m_normals[i]);
	}
}

void Cloth::UpdateNormalBuffer()
{
	RecalculateNormals();
	
	Graphics::Mesh* m = Resources::GetPtr<Graphics::Mesh>(m_clothMesh);
	ID3D11DeviceContext* pDC = Graphics::GetDeviceContext();
	D3D11_MAPPED_SUBRESOURCE data;

	ID3D11Buffer* pBuffer = m->GetStreamBufferPtr(2);
	HRESULT hrPos = pDC->Map(pBuffer, 0, D3D11_MAP::D3D11_MAP_WRITE_DISCARD, 0, &data);
	if (SUCCEEDED(hrPos))
	{
		memcpy(data.pData, m_normals.data(), sizeof(Vector3f) * m_normals.size());
		pDC->Unmap(pBuffer, 0);
	}
}

void Cloth::CreateClothMesh()
{
	std::vector<u32> colours(m_noPoints, 0xFFFFffff);


	/// CALCULATE POSITIONS ----------------------------------------------------

	std::vector<Vector3f> positions(m_noPoints);
	for (u32 i = 0; i < m_y; i++)
	{
		for (u32 j = 0; j < m_x; j++)
		{
			u32 index = (i * m_x) + j;
			positions[index] = Vector3f(j * m_spacing, i * m_spacing, 0);
		}
	}
	m_positions = positions;

	/// CALCULATE INDICES ------------------------------------------------------

	std::vector<u32> indices;
	for (u32 i = 0; i < m_y - 1; i++)
	{
		for (u32 j = 0; j < m_x - 1; j++)
		{
			u32 squ = (i * m_x) + j;

			indices.push_back(squ);
			indices.push_back(squ + 1);
			indices.push_back(squ + m_x + 1);
			indices.push_back(squ);
			indices.push_back(squ + m_x + 1);
			indices.push_back(squ + m_x);
		}
	}
	m_indices = indices;

	/// CALCULATE VERTEX NORMALS -----------------------------------------------

	std::vector<Vector3f> normals(m_noPoints, Vector3f(0, 0, 0));
	for (int i = 0; i < indices.size(); i += 3)
	{
		Vector3f AB = positions[indices[i + 1]] - positions[indices[i]];
		Vector3f AC = positions[indices[i + 2]] - positions[indices[i]];
		Vector3f fN = cross(AB, AC);
		fN = normalize(fN);

		normals[indices[i]] += fN;
		normals[indices[i + 1]] += fN;
		normals[indices[i + 2]] += fN;
	}

	for (int i = 0; i < normals.size(); i++)
	{
		normals[i] = normalize(normals[i]);
	}

	m_normals = normals;

	/// CREATE MESH ------------------------------------------------------------

	Graphics::MeshDesc desc;
	Graphics::StreamInfo streamInfos[5];
	streamInfos[0].m_type = Graphics::StreamType::POSITION;
	streamInfos[0].m_pData = positions.data();
	streamInfos[0].m_dataSize = positions.size() * sizeof(Vector3f);
	streamInfos[0].m_flags = Graphics::StreamFlags::DYNAMIC_STREAM;

	streamInfos[1].m_type = Graphics::StreamType::COLOUR;
	streamInfos[1].m_pData = colours.data();
	streamInfos[1].m_dataSize = colours.size() * sizeof(u32);

	streamInfos[2].m_type = Graphics::StreamType::NORMAL;
	streamInfos[2].m_pData = normals.data();
	streamInfos[2].m_dataSize = normals.size() * sizeof(Vector3f);
	streamInfos[2].m_flags = Graphics::StreamFlags::DYNAMIC_STREAM;

	streamInfos[3].m_type = Graphics::StreamType::INDEX;
	streamInfos[3].m_pData = indices.data();
	streamInfos[3].m_dataSize = indices.size() * sizeof(u32);

	desc.m_pStreams = streamInfos;
	desc.m_streamCount = 4;
	desc.m_vertexCount = (u32)positions.size();
	desc.m_indexCount = (u32)indices.size();
	
	m_clothMesh = Resources::CreateAsset<Graphics::Mesh>(desc);
}

void Cloth::CreateMaterials()
{
	// Define wire frame and solid materials

	Graphics::SimpleMaterialDesc desc_wf;
	desc_wf.m_state.m_cullMode = Graphics::CullMode::NONE;
	desc_wf.m_state.m_fillMode = Graphics::FillMode::WIREFRAME;
	Colour::Seagreen.as_float_rgba_srgb(&desc_wf.m_constants.diffuseColour.x);
	desc_wf.m_bEnableLighting = false;

	m_wireframeMat = Resources::CreateAsset<Graphics::Material>(desc_wf);
	

	Graphics::SimpleMaterialDesc desc_s;
	desc_s.m_state.m_cullMode = Graphics::CullMode::NONE;
	desc_s.m_state.m_fillMode = Graphics::FillMode::SOLID;
	desc_s.m_bEnableLighting = true;
	desc_s.m_lightCount = 1;

	Colour::Deeppink.as_float_rgba_srgb(&desc_s.m_constants.diffuseColour.x);
	m_solidMat = Resources::CreateAsset<Graphics::Material>(desc_s);
	
}

void Cloth::CreatePointsAndSticks()
{
	for (u32 i = 0; i < m_y; i++)
	{
		for (u32 j = 0; j < m_x; j++)
		{
			u32 index = (i * m_x) + j;

			ClothParticle* point = new ClothParticle(this, m_positions[index]);
			point->SetMass(m_mass / m_noPoints);
	


			if (i == m_y - 1 && (j == 0 || j == m_x - 1))
			{
				point->SetIsPinned(true);
			}
			else
			{
				point->SetVelocity(m_gravity);
			}
	
			m_cornerPoints.resize(4);
	
			if (j == 0 && i == 0)
			{
				// A
				m_cornerPoints.at(0) = point;
			}
			else if (j == 0 && i == m_y - 1)
			{
				// B
				m_cornerPoints.at(1) = point;
			}
			else if (j == m_x - 1 && i == m_y - 1)
			{
				// C
				m_cornerPoints.at(2) = point;
			}
			else if (j == m_x - 1 && i == 0)
			{
				// D
				m_cornerPoints.at(3) = point;
			}
	
	
			m_vPoints.push_back(point);
		}
	}

	for (int y = 0; y < m_y; y++)
	{
		for (int x = 0; x < m_x; x++)
		{
			if (x < m_x - 1)
			{
				// Horizontal
				ClothConstraint* pC = new ClothConstraint(m_vPoints[(y * m_x) + x], m_vPoints[(y * m_x) + (x + 1)]);
				m_vSticks.push_back(pC);
			}

			if (y < m_y - 1)
			{
				// Vertical 
				ClothConstraint* pC = new ClothConstraint(m_vPoints[(y * m_x) + x], m_vPoints[(y + 1) * m_x + x]);
				m_vSticks.push_back(pC);
			}

			if (x < m_x - 1 && y < m_y - 1)
			{
				ClothConstraint* pC = new ClothConstraint(m_vPoints[(y * m_x) + x], m_vPoints[(y + 1) * m_x + (x + 1)]);
				m_vSticks.push_back(pC);
			}

			if (x > 0 && y < m_y - 1)
			{
				ClothConstraint* pC = new ClothConstraint(m_vPoints[(y * m_x) + x], m_vPoints[(y + 1) * m_x + (x - 1)]);
				m_vSticks.push_back(pC);
			}
		}

	}
}

void Cloth::CalculateWindForce()
{
// 	Vector3f wind = Vector3f((float)((rand() % 200) - 100) / 100.f, 0, ((float)((rand() % 200) - 100) / 100.f));
// 	if (wind == Vector3f(0, 0, 0))
// 	{
// 		wind = Vector3f(1, 0, 0);
// 	}
// 	wind = normalize(wind);
// 	m_windForce = wind * ((float)(rand() % 100) * (float)sin(System::GetElapsedTime()));

	Vector3f windx(1, 0, 0);
	Vector3f windz(0, 0, 1);
	m_windForce = (windz * 100.f * cosf((float)System::GetElapsedTime()));
}

void Cloth::Update()
{
	//CalculateWindForce();
	CalculateForces();

	for (int i = 0; i < m_vPoints.size(); i++)
	{
		m_vPoints[i]->VerletIntegration();
		m_positions[i] = m_vPoints[i]->GetPosition();
	}

	UpdatePositionBuffer();
	UpdateNormalBuffer();

	m_quad = Geometry::Quad(m_cornerPoints[0]->GetPosition(),
		m_cornerPoints[1]->GetPosition(),
		m_cornerPoints[2]->GetPosition(),
		m_cornerPoints[3]->GetPosition());
}

void Cloth::Render()
{
	//Graphics::SetMaterial(wireframeMaterial);
	Graphics::SetMaterial(m_solidMat);
	Graphics::DrawMesh(m_clothMesh, MatrixTranslate<f32>(0.f, 0.f, 0.f));
}

void Cloth::Destroy()
{
	for (ClothParticle* point : m_vPoints)
	{
		delete point;
	}
	for (ClothConstraint* stick : m_vSticks)
	{
		delete stick;
	}
}

void Cloth::ApplyExternalForce(Vector3f pos)
{
	// Find closest particle to pos
	ClothParticle* closestParticle = m_vPoints[0];
	float shortestlenSqr = 100.f;
	for (ClothParticle* p : m_vPoints)
	{
		Vector3f AB = p->GetPosition() - pos;
		float ls = lengthSqr(AB);
		if (ls < shortestlenSqr)
		{
			shortestlenSqr = ls;
			closestParticle = p;
		}
	}

	closestParticle->ApplyExternalForce(Vector3f(0,0,10000));
}

void Cloth::ApplyExternalForceToRadius(Vector3f pos, float radius)
{
	std::vector<ClothParticle*> inRadius;

	for (ClothParticle* p : m_vPoints)
	{
		Vector3f AB = p->GetPosition() - pos;
		if (lengthSqr(AB) < radius * radius)
		{
			p->ApplyExternalForce(Vector3f(0,0,1000));
		}
	}

}

void Cloth::CalculateForces()
{
	for (ClothParticle* p : m_vPoints)
	{
		p->ZeroForceVector();

		if (!p->GetIsPinned())
		{
			p->CalculateForces();
		}
	}

	for (ClothConstraint* s : m_vSticks)
	{
		s->CalculateSpringForces();
	}
}
