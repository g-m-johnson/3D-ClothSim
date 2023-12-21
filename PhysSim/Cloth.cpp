#include "Cloth.h"
#include "ClothPoint.h"
#include "ClothStick.h"

Cloth::Cloth(int width, int height, int spacing, Vector2f start)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			ClothPoint* point = new ClothPoint(this, Vector2f(start.x + (x * spacing), start.y + (y * spacing)));
			
			if (x != 0)
			{
				ClothPoint* leftPoint = m_vPoints[m_vPoints.size() - 1];
				ClothStick* stick = new ClothStick(point, leftPoint);
				leftPoint->AddStick(stick, 0);
				point->AddStick(stick, 0);
				m_vSticks.push_back(stick);
			}
			if (y != 0)
			{
				ClothPoint* upPoint = m_vPoints[x + (y - 1) * (width)];
				ClothStick* stick = new ClothStick(point, upPoint);
				upPoint->AddStick(stick, 1);
				point->AddStick(stick, 1);
				m_vSticks.push_back(stick);
			}

			if (y == 0)
			{
				point->SetIsPinned(true);
			}

			m_vPoints.push_back(point);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::Update(float dT)
{
	for (ClothPoint* p : m_vPoints)
	{
		p->Update(dT);
	}
	for (ClothStick* s : m_vSticks)
	{
		s->Update();
	}
}

void Cloth::Render()
{
	for (ClothStick* s : m_vSticks)
	{
		s->Render();
	}
}

void Cloth::Destroy()
{
	for (ClothPoint* point : m_vPoints)
	{
		delete point;
	}
	for (ClothStick* stick : m_vSticks)
	{
		delete stick;
	}
}
