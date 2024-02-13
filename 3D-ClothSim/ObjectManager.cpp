#include "ObjectManager.h"
#include "Cloth.h"

void ObjectManager::CreateScene()
{
	m_pCloth = new Cloth(X_PARTICLE_NO, Y_PARTICLE_NO, PARTICLE_SPACING);
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
