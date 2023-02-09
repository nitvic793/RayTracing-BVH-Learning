#pragma once

#include <TLAS.h>

namespace Tmpl8
{

class MyApp : public TheApp
{
public:
	// game flow methods
	void Init();
	void Tick( float deltaTime );
	void Shutdown() { /* implement if you want to do something on exit */ }
	// input handling
	void MouseUp( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseDown( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float y ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int key ) { /* implement if you want to handle keys */ }
	void KeyDown( int key ) { /* implement if you want to handle keys */ }

	void Animate();
	float3 Trace(bvh::Ray& ray, int rayDepth);

	// data members
	int2 mousePos;

	const char* GetAppName() const override { return "BVH-Construction"; }

	static constexpr uint BVH_COUNT = 16;

	bvh::BVHInstance bvhInstances[BVH_COUNT];
	bvh::TLAS tlas;
	bvh::Mesh* mesh = nullptr;

	float3 p0, p1, p2;
	float3* accumulator;

	float* skyPixels = nullptr;
	int skyWidth, skyHeight, skyBpp;
};

} // namespace Tmpl8