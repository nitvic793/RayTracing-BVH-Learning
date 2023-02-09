#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>
#include <TLAS.h>

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

constexpr uint PIXEL_COUNT = SCRWIDTH * SCRHEIGHT;

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

    mesh = new Mesh("Assets/teapot.obj", "Assets/bricks.png");
    for (uint i = 0; i < BVH_COUNT; ++i)
        bvhInstances[i] = BVHInstance(mesh->bvh, i);

    tlas = TLAS(bvhInstances, BVH_COUNT);
    tlas.Build();

    accumulator = new float3[PIXEL_COUNT];

    constexpr float aspectRatio = (float)SCRWIDTH / SCRHEIGHT;

    p0 = TransformPosition(float3(-aspectRatio, 1, 2), mat4::RotateX(0.5f));
    p1 = TransformPosition(float3(aspectRatio, 1, 2), mat4::RotateX(0.5f));
    p2 = TransformPosition(float3(-aspectRatio, -1, 2), mat4::RotateX(0.5f));

    skyPixels = stbi_loadf("Assets/sky_19.hdr", &skyWidth, &skyHeight, &skyBpp, 0);
    for (int i = 0; i < skyWidth * skyHeight * 3; i++) 
        skyPixels[i] = sqrtf(skyPixels[i]);
}

void MyApp::Tick( float deltaTime )
{	
    // clear the screen to black
    screen->Clear(0);

    Timer timer;
    Animate();

	Ray ray;

	constexpr uint WIDTH = SCRWIDTH;
	constexpr uint HEIGHT = SCRHEIGHT;
    constexpr uint TILE_SCALE = SCRWIDTH / 8;
    constexpr uint TILE_COUNT = (SCRWIDTH * SCRHEIGHT / 64);
    constexpr uint TILE_SIZE = 8;

    // render the scene: multithreaded tiles
    static float angle = 0; angle += 0.01f;
    mat4 M1 = mat4::RotateY(angle);
    mat4 M2 = M1 * mat4::RotateX(-0.65f);

    // setup screen plane in world space
    p0 = TransformPosition(float3(-1, 1, 1.5f), M2);
    p1 = TransformPosition(float3(1, 1, 1.5f), M2);
    p2 = TransformPosition(float3(-1, -1, 1.5f), M2);

    float3 camPos = TransformPosition(float3(0, -2, -8.5f), M1);

    #pragma omp parallel for schedule(dynamic)
    for (int tile = 0; tile < TILE_COUNT; ++tile)
    {
        int x = tile % TILE_SCALE;
        int y = tile / TILE_SCALE;

        Ray ray;
        ray.Orig = camPos;
        for (int v = 0; v < TILE_SIZE; ++v)
        {
            for (int u = 0; u < TILE_SIZE; ++u)
            {
                uint pixelAddress = x * TILE_SIZE + u + (y * TILE_SIZE + v) * SCRWIDTH;
                accumulator[pixelAddress] = 0;
                constexpr uint SAMPLE_COUNT = 4;
                constexpr float contribution = 1.f / (float)SAMPLE_COUNT;

                for (int s = 0; s < SAMPLE_COUNT; ++s)
                {
                    float3 pixelPos = ray.Orig + p0 +
                        (p1 - p0) * ((x * TILE_SIZE + u + RandomFloat()) / (float)WIDTH) +
                        (p2 - p0) * ((y * TILE_SIZE + v + RandomFloat()) / (float)HEIGHT);

                    ray.Dir = normalize(pixelPos - ray.Orig);
                    ray.Hit.T = FLOAT_DIST_MAX;
                    ray.rD = float3(1 / ray.Dir.x, 1 / ray.Dir.y, 1 / ray.Dir.z);

                    uint pixelAddress = x * TILE_SIZE + u + (y * TILE_SIZE + v) * SCRWIDTH;
                    accumulator[pixelAddress] += contribution * Trace(ray, 0);
                }
            }
        }
    }

    for (int i = 0; i < PIXEL_COUNT; i++)
    {
        int r = min(255, (int)(255 * accumulator[i].x));
        int g = min(255, (int)(255 * accumulator[i].y));
        int b = min(255, (int)(255 * accumulator[i].z));
        screen->pixels[i] = (r << 16) + (g << 8) + b;
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

constexpr float3 RGB8toRGB32F(uint c)
{
    float s = 1 / 256.0f;
    int r = (c >> 16) & 255;  // extract the red byte
    int g = (c >> 8) & 255;   // extract the green byte
    int b = c & 255;          // extract the blue byte
    return float3(r * s, g * s, b * s);
}

float3 MyApp::Trace(bvh::Ray& ray, int rayDepth)
{
    tlas.Intersect(ray);
    Intersection i = ray.Hit;

    if (i.T == FLOAT_DIST_MAX)
    {
        const uint u = (uint)(skyWidth * atan2f(ray.Dir.z, ray.Dir.x) * INV2PI - 0.5f);
        const uint v = (uint)(skyHeight * acosf(ray.Dir.y) * INVPI - 0.5f);
        const uint skyPixelIdx = (u + v * skyWidth) % (skyWidth * skyHeight);;
        const float3 skyColor = float3(skyPixels[skyPixelIdx * 3], skyPixels[skyPixelIdx * 3 + 1], skyPixels[skyPixelIdx * 3 + 2]);
        return 0.65f * skyColor;
    }

    uint triIdx = i.instPrim & 0xfffff;
    uint instIdx = i.instPrim >> 20; 

    TriEx& triEx = mesh->triEx[triIdx];
    Surface* tex = mesh->texture;

    float2 uv = i.U * triEx.uv1 + i.V * triEx.uv2 + (1 - (i.U + i.V)) * triEx.uv0;

    int iu = (int)(uv.x * tex->width) % tex->width;
    int iv = (int)(uv.y * tex->height) % tex->height;

    uint texel = tex->pixels[iu + iv * tex->width];

    float3 albedo = RGB8toRGB32F(texel);

    float3 N = i.U * triEx.N1 + i.V * triEx.N2 + (1 - (i.U + i.V)) * triEx.N0;
    N = normalize(TransformVector(N, bvhInstances[instIdx].GetTransform()));

    float3 worldPos = ray.Orig - i.T * ray.Dir;

    bool mirror = (instIdx * 17) & 1;

    if (mirror)
    {
        Ray secondary;
        secondary.Dir = ray.Dir - 2 * N * dot(N, ray.Dir);
        secondary.Orig = worldPos + secondary.Dir * 0.01f;
        secondary.Hit.T = FLOAT_DIST_MAX;
        if (rayDepth >= 10) 
            return float3(0);
        return Trace(secondary, rayDepth + 1);
    }
    else
    {
        float3 lightPos(3, 8, -2);
        float3 lightColor(250, 250, 220);
        float3 ambient(0.2f, 0.2f, 0.4f);

        float3 L = lightPos - worldPos;
        float dist = length(L);
        L *= 1.f / dist; // Normalize direction to light
        float NdotL = max(0.f, dot(N, L));
        float attenuation = 1.f / (dist * dist);

        float3 color = albedo * (ambient + NdotL * lightColor * 3.f * attenuation);

        return color;
    }

    return float3(0);
}
