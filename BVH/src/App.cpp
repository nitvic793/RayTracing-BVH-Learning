#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>
#include <TLAS.h>

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

// application data
constexpr uint BVH_COUNT = 16;
BVH gBvh[BVH_COUNT];
TLAS tlas;

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

    for (uint i = 0; i < BVH_COUNT; ++i)
        gBvh[i] = BVH("Assets/armadillo.tri", 30000);

    tlas = TLAS(gBvh, BVH_COUNT);
    tlas.Build();
}

void MyApp::Tick( float deltaTime )
{	
    // clear the screen to black
    screen->Clear(0);

    Timer t;

    static float a[16] = { 0 }, h[16] = { 5, 4, 3, 2, 1, 5, 4, 3 }, s[16] = { 0 };
    for (int i = 0, x = 0; x < 4; x++)
    {
        for (int y = 0; y < 4; y++, i++)
        {
            mat4 R, T = mat4::Translate((x - 1.5f) * 2.5f, 0, (y - 1.5f) * 2.5f);
            if ((x + y) & 1)
                R = mat4::RotateX(a[i]) * mat4::RotateZ(a[i]);
            else
                R = mat4::Translate(0, h[i / 2], 0);

            if ((a[i] += (((i * 13) & 7) + 2) * 0.005f) > 2 * PI)
                a[i] -= 2 * PI;

            if ((s[i] -= 0.01f, h[i] += s[i]) < 0)
                s[i] = 0.2f;

            gBvh[i].SetTransform(T * R * mat4::Scale(0.75f));
        }
    }

    tlas.Build();

    static float angle = 0.f;
    angle += 0.01f;

    constexpr auto TWO_PI = 2.f * PI;
    if (angle > TWO_PI)
        angle -= TWO_PI;

    gBvh[0].SetTransform(mat4::Translate(float3(-1.3f, 0, 0)));
    gBvh[1].SetTransform(mat4::Translate(float3(1.3f, 0, 0)) * mat4::RotateY(angle));

    float3 p0 = TransformPosition(float3(-1, 1, 2), mat4::RotateX(0.5f));
    float3 p1 = TransformPosition(float3(1, 1, 2), mat4::RotateX(0.5f));
    float3 p2 = TransformPosition(float3(-1, -1, 2), mat4::RotateX(0.5f));
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
        ray.Orig = float3(0, 4.5f, -8.5f);
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
                    
                tlas.Intersect(ray);

                uint c = ray.T < FLOAT_DIST_MAX ? (int)(255 / (1 + max(0.f, ray.T - 4))) : 0;
                screen->Plot(x * 8 + u, y * 8 + v, c * 0x10101);
            }
        }
    }

	float elapsed = t.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);
}