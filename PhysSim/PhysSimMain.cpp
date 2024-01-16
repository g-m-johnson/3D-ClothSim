#define PLAY_IMPLEMENTATION
//#define PLAY_USING_GAMEOBJECT_MANAGER
//#include "Play.h"
#include "../Play3d/Play3d.h"
using namespace Play3d;

#include "PhysSimMain.h"
#include "Cloth.h"
#include "Mouse.h"

Cloth* g_pCloth = new Cloth(39, 20, 20);

int PlayMain()
{
	SystemDesc systemDesc;
	systemDesc.title = "ClothSim";
	systemDesc.width = 1920;
	systemDesc.height = 1080;
	System::Initialise(systemDesc);

	Demo::SetDebugCameraPosition(Vector3f(20, 10, -30), 0, 0);
	Demo::SetDebugCameraFOV(kfPi / 4.f, 0.1f, 75.f);

	g_pCloth->Initialise();


	// Define wire frame and solid materials
	Graphics::MaterialId wireframeMaterial;
	{
		Graphics::SimpleMaterialDesc desc;
		desc.m_state.m_cullMode = Graphics::CullMode::NONE;
		desc.m_state.m_fillMode = Graphics::FillMode::WIREFRAME;
		Colour::Seagreen.as_float_rgba_srgb(&desc.m_constants.diffuseColour.x);
		desc.m_bEnableLighting = false;

		wireframeMaterial = Resources::CreateAsset<Graphics::Material>(desc);
	}
	Graphics::MaterialId solidMaterial;
	{
		Graphics::SimpleMaterialDesc desc;
		desc.m_state.m_cullMode = Graphics::CullMode::NONE;
		desc.m_state.m_fillMode = Graphics::FillMode::SOLID;
		desc.m_bEnableLighting = true;
		desc.m_lightCount = 1;

		Colour::Seagreen.as_float_rgba_srgb(&desc.m_constants.diffuseColour.x);
		solidMaterial = Resources::CreateAsset<Graphics::Material>(desc);
	}

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

		// Set Material And Draw the mesh
		//Graphics::SetMaterial(wireframeMaterial);
		Graphics::SetMaterial(solidMaterial);
		Graphics::DrawMesh(g_pCloth->GetClothMesh(), MatrixTranslate<f32>(0.f, 0.f, 0.f));

		System::EndFrame();
	}

	delete g_pCloth;

	System::Shutdown();

	return 0;
}