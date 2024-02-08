#define PLAY_IMPLEMENTATION
#include "../Play3d/Play3d.h"
using namespace Play3d;

#include "PhysSimMain.h"
#include "Mouse.h"
#include "ObjectManager.h"

int PlayMain()
{
	SystemDesc systemDesc;
	systemDesc.title = "ClothSim";
	systemDesc.width = 2160;
	systemDesc.height = 1080;
	System::Initialise(systemDesc);

	srand((int)time(0));

	Demo::SetDebugCameraPosition(Vector3f(20, 10, -50), 0, 0);
	Demo::SetDebugCameraFOV(kfPi / 4.f, 0.1f, 75.f);

	ObjectManager::Instance().CreateScene();



	Graphics::SetLightColour(0, ColourValue(0xFFFFFF));
	Graphics::SetLightDirection(0, Vector3f(1, 1, 1));


	//---------------------------MAIN UPDATE LOOP-------------------------------

	bool bKeepGoing = true;
	while (bKeepGoing)
	{
		if (System::BeginFrame() != RESULT_OK || Input::IsKeyPressed(VK_ESCAPE))
		{
			bKeepGoing = false;
		}

		Demo::UpdateDebugCamera();
		Demo::SetDebugCameraMatrices();
		Demo::DrawDebugGrid();

		Mouse::Instance().Update();

		ObjectManager::Instance().UpdateScene();
		ObjectManager::Instance().RenderScene();

		Graphics::BeginPrimitiveBatch();
		Mouse::Instance().DebugDrawMouseRay();
		Graphics::EndPrimitiveBatch();

		System::EndFrame();
	}

	ObjectManager::Instance().DestroyScene();

	System::Shutdown();

	return 0;
}