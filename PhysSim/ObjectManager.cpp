#include "ObjectManager.h"
#include "Cloth.h"

void ObjectManager::CreateScene()
{
	m_pCloth = new Cloth(30, 20, 1);
	m_pCloth->Initialise();
}

void ObjectManager::UpdateScene()
{
	m_pCloth->Update();
}

void ObjectManager::RenderScene()
{
	m_pCloth->Render();
}

void ObjectManager::DestroyScene()
{
	delete m_pCloth;
}
