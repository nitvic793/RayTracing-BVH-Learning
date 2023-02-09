#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>
#include <TLAS.h>

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }


constexpr uint BVH_COUNT = 256;

struct Transforms
{
    float3* position = nullptr;
    float3* direction = nullptr;
    float3* orientation = nullptr;
    uint    count = 0;

    Transforms() = default;
    Transforms(uint N) :
        count(N)
    {
        position = new float3[N];
        direction = new float3[N];
        orientation = new float3[N];
    }

    ~Transforms()
    {
        if(position)
            delete[] position;

        if (direction)
            delete[] direction;

        if (orientation)
            delete[] orientation;

        position = direction = orientation = nullptr;
    }
};

// application data
BVH* gBvh;
BVHInstance bvhInstances[BVH_COUNT];
TLAS tlas;
Transforms* t;


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

    gBvh = new BVH("Assets/armadillo.tri", 30000);
    for (uint i = 0; i < BVH_COUNT; ++i)
        bvhInstances[i] = BVHInstance(gBvh);

    tlas = TLAS(bvhInstances, BVH_COUNT);
    tlas.Build();

    t = new Transforms(BVH_COUNT);

    for (uint i = 0; i < BVH_COUNT; ++i)
    {
        t->position[i] = float3(RandomFloat(), RandomFloat(), RandomFloat()) - 0.5f;
        t->position[i] *= 4;
        t->direction[i] = normalize(t->position[i]) * 0.05f;
        t->orientation[i] = float3(RandomFloat(), RandomFloat(), RandomFloat()) * 2.5f;
    }
}

void MyApp::Tick( float deltaTime )
{	
    // clear the screen to black
    screen->Clear(0);

    Timer timer;
    for (int i = 0; i < BVH_COUNT; i++)
    {
        mat4 R = mat4::RotateX(t->orientation[i].x) *
            mat4::RotateY(t->orientation[i].y) *
            mat4::RotateZ(t->orientation[i].z) * mat4::Scale(0.2f);

        bvhInstances[i].SetTransform(mat4::Translate(t->position[i]) * R);
        t->position[i] += t->direction[i], t->orientation[i] += t->direction[i];
        if (t->position[i].x < -3 || t->position[i].x > 3) t->direction[i].x *= -1;
        if (t->position[i].y < -3 || t->position[i].y > 3) t->direction[i].y *= -1;
        if (t->position[i].z < -3 || t->position[i].z > 3) t->direction[i].z *= -1;
    }

    tlas.Build();

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

	float elapsed = timer.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);
}