#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>
#include <TLAS.h>

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

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

//constexpr uint BVH_COUNT = 16;
constexpr uint PIXEL_COUNT = SCRWIDTH * SCRHEIGHT;

// application data
//BVH* gBvh;
//BVHInstance bvhInstances[BVH_COUNT];
//TLAS tlas;
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

    Mesh* mesh = new Mesh("Assets/teapot.obj", "Assets/bricks.png");
    for (uint i = 0; i < BVH_COUNT; ++i)
        bvhInstances[i] = BVHInstance(mesh->bvh, i);

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

    accumulator = new float3[PIXEL_COUNT];
}

void MyApp::Tick( float deltaTime )
{	
    // clear the screen to black
    screen->Clear(0);

    Timer timer;
    Animate();

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
                ray.Hit.T = FLOAT_DIST_MAX;
                ray.rD = float3(1 / ray.Dir.x, 1 / ray.Dir.y, 1 / ray.Dir.z);
                  
                uint pixelAddress = x * TILE_SIZE + u + (y * TILE_SIZE + v) * SCRWIDTH;
                accumulator[pixelAddress] = Trace(ray);
                //tlas.Intersect(ray);

                //uint c = ray.Hit.T < FLOAT_DIST_MAX ? (int)(255 / (1 + max(0.f, ray.Hit.T - 4))) : 0;
                //screen->Plot(x * 8 + u, y * 8 + v, c * 0x10101);
            }
        }
    }

    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        int r = min(255, (int)(255 * accumulator[i].x));
        int g = min(255, (int)(255 * accumulator[i].y));
        int b = min(255, (int)(255 * accumulator[i].z));
        screen->pixels[i] = (r << 16) + (g << 8) + b;
        //screen->Plot(r, g, b);
    }

	float elapsed = timer.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);
}

void MyApp::Animate()
{
    static float a[16] = { 0 }, h[16] = { 5, 4, 3, 2, 1, 5, 4, 3 }, s[16] = { 0 };
    for (int i = 0, x = 0; x < 4; x++) for (int y = 0; y < 4; y++, i++)
    {
        mat4 R, T = mat4::Translate((x - 1.5f) * 2.5f, 0, (y - 1.5f) * 2.5f);
        if ((x + y) & 1) R = mat4::RotateY(a[i]);
        else R = mat4::Translate(0, h[i / 2], 0);
        if ((a[i] += (((i * 13) & 7) + 2) * 0.005f) > 2 * PI) a[i] -= 2 * PI;
        if ((s[i] -= 0.01f, h[i] += s[i]) < 0) s[i] = 0.2f;
        bvhInstances[i].SetTransform(T * R * mat4::Scale(1.5f));
    }

    tlas.Build();
}

float3 MyApp::Trace(bvh::Ray& ray)
{
    tlas.Intersect(ray);
    Intersection i = ray.Hit;
    if (i.T == FLOAT_DIST_MAX) 
        return float3(0);
    return float3(i.U, i.V, 1 - (i.U + i.V));
}
