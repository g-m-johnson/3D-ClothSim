#define PLAY_IMPLEMENTATION
#define PLAY_USING_GAMEOBJECT_MANAGER
#include "Play.h"
#include "Cloth.h"

constexpr int DISPLAY_WIDTH = 1000;
constexpr int DISPLAY_HEIGHT = 700;
constexpr int DISPLAY_SCALE = 1;

Cloth* g_pCloth = new Cloth(16, 10, 50, Vector2f(100, 100));
float g_lastElapsedTime = 0.F;

// The entry point for a PlayBuffer program
void MainGameEntry(PLAY_IGNORE_COMMAND_LINE)
{
	Play::CreateManager(DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_SCALE);
	Play::CentreAllSpriteOrigins();
}

// Called by PlayBuffer every frame (60 times a second!)
bool MainGameUpdate(float elapsedTime)
{
	Play::ClearDrawingBuffer(Play::Colour(72, 63, 94));

	g_pCloth->Update(elapsedTime);
	g_pCloth->Render();

	Play::PresentDrawingBuffer();

	g_lastElapsedTime = elapsedTime;
	return Play::KeyDown(VK_ESCAPE);
}

// Gets called once when the player quits the game 
int MainGameExit(void)
{
	Play::DestroyManager();
	return PLAY_OK;
}