#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

// application data
BVH gBvh;

void MyApp::Init()
{
    // Ensure "Current Directory" (relative path) is always the .exe's folder
    {
        char currentDir[1024] = {};
        GetModuleFileName(0, currentDir, 1024);
        char* lastSlash = strrchr(currentDir, '\\');
        if (lastSlash)
        {
            *lastSlash = 0;
            SetCurrentDirectory(currentDir);
        }
    }

    gBvh = BVH("Assets/armadillo.tri", 30000);
}

void MyApp::Tick( float deltaTime )
{	
    // clear the screen to black
    screen->Clear(0);

    Timer t;

    float3 p0(-1, 1, 2), p1(1, 1, 2), p2(-1, -1, 2);
	Ray ray;

	constexpr uint WIDTH = SCRWIDTH;
	constexpr uint HEIGHT = SCRHEIGHT;
    constexpr uint TILE_SCALE = 80;
    constexpr uint TILE_COUNT = 6400;
    constexpr uint TILE_SIZE = 8;

    #pragma omp parallel for schedule(dynamic)
    for (int tile = 0; tile < TILE_COUNT; ++tile)
    {
        int x = tile % TILE_SCALE;
        int y = tile / TILE_SCALE;

        Ray ray;
        for (int v = 0; v < TILE_SIZE; ++v)
        {
            for (int u = 0; u < TILE_SIZE; ++u)
            {
                float3 pixelPos = ray.Orig + p0 +
                    (p1 - p0) * ((x * TILE_SIZE + u) / (float)WIDTH) +
                    (p2 - p0) * ((y * TILE_SIZE + v) / (float)HEIGHT);

                ray.Dir = normalize(pixelPos - ray.Orig);
                ray.T = FLOAT_DIST_MAX;
                ray.rD = float3(1 / ray.Dir.x, 1 / ray.Dir.y, 1 / ray.Dir.z);
                    
                ray.Orig = float3(-1, 0.5f, -4.5f);
                gBvh.Intersect(ray);

                ray.Orig = float3(1, 0.5f, -4.5f);
                gBvh.Intersect(ray);

                uint c = ray.T < FLOAT_DIST_MAX ? (255 - (int)((ray.T - 3) * 80)) : 0;
                screen->Plot(x * TILE_SIZE + u, y * TILE_SIZE + v, c * 0x10101);
            }
        }
    }

	float elapsed = t.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);
}