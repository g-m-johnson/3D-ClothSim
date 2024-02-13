#pragma once

class Cloth;

class ObjectManager
{
public:

	static ObjectManager& Instance()
	{
		static ObjectManager* Instance = new ObjectManager();
		return *Instance;
	}

	void CreateScene();

	void UpdateScene();
	void RenderScene();

	void DestroyScene();

	Cloth* GetClothPtr() { return m_pCloth; }

private:
	
	Cloth* m_pCloth;
};

