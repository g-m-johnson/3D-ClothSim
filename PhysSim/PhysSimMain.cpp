#define PLAY_IMPLEMENTATION
#define PLAY_USING_GAMEOBJECT_MANAGER
#include "Play.h"
#include "PhysSimMain.h"
#include "Cloth.h"
#include "Mouse.h"

Cloth* g_pCloth = new Cloth(29, 15, 25, Vector2f(100, 100));

// The entry point for a PlayBuffer program
void MainGameEntry(PLAY_IGNORE_COMMAND_LINE)
{
	Play::CreateManager(DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_SCALE);
	Play::CentreAllSpriteOrigins();
	Mouse::Instance().Initialise();
}

// Called by PlayBuffer every frame (60 times a second!)
bool MainGameUpdate(float elapsedTime)
{
	Play::ClearDrawingBuffer(Play::Colour(72, 63, 94));

	Mouse::Instance().Update();

	g_pCloth->Update(elapsedTime);
	g_pCloth->Render();

	Play::PresentDrawingBuffer();

	return Play::KeyDown(VK_ESCAPE);
}

// Gets called once when the player quits the game 
int MainGameExit(void)
{
	g_pCloth->Destroy();
	delete g_pCloth;
	delete &Mouse::Instance();

	Play::DestroyManager();
	return PLAY_OK;
}