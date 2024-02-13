#pragma once

constexpr int DISPLAY_WIDTH	= 2160;
constexpr int DISPLAY_HEIGHT = 1080;

constexpr int CURSOR_RADIUS = 3;

constexpr int X_PARTICLE_NO = 15;
constexpr int Y_PARTICLE_NO = 10;
constexpr float PARTICLE_SPACING = 1.f;

constexpr float AIR_DRAG_COEFFICIENT = 0.5f;
constexpr float Y_GRAVITY = -9.81f;

constexpr float PARTICLE_MASS = 0.1f;
constexpr float MASS_PER_FACE = 0.1f;

constexpr float SPRING_CONSTANT = 100.f;
constexpr float DAMPING_CONSTANT = 3.f;