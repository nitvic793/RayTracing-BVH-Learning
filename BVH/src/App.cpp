#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>

#pragma warning (disable: 4996)

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

#define USE_SSE 1

constexpr uint N = 20944;
constexpr uint BINS = 8;

void Subdivide(uint nodeIdx);
void UpdateNodeBounds(uint nodeIdx);

struct AABB
{
    float3 bmin = 1e30f;
    float3 bmax = -1e30f;

    void Grow(float3 p) 
    { 
        bmin = fminf(bmin, p); 
        bmax = fmaxf(bmax, p); 
    }

    void Grow(const AABB& bb) 
    { 
        bmin = fminf(bmin, bb.bmin);
        bmax = fmaxf(bmax, bb.bmax);
    }

    float Area()
    {
        float3 e = bmax - bmin; // box extent
        return e.x * e.y + e.y * e.z + e.z * e.x;
    }
};

struct Bin
{
    AABB Bounds;
    int TriCount = 0;
};


// application data
Tri tri[N];
Tri original[N];
uint triIdx[N];
BVHNode* bvhNode = 0;
uint rootNodeIdx = 0, nodesUsed = 2;

constexpr float IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax)
{
    float tx1 = (bmin.x - ray.Orig.x) * ray.rD.x; 
    float tx2 = (bmax.x - ray.Orig.x) * ray.rD.x;
    float tmin = min(tx1, tx2);
    float tmax = max(tx1, tx2);
    float ty1 = (bmin.y - ray.Orig.y) * ray.rD.y;
    float ty2 = (bmax.y - ray.Orig.y) * ray.rD.y;

    tmin = max(tmin, min(ty1, ty2)); 
    tmax = min(tmax, max(ty1, ty2));

    float tz1 = (bmin.z - ray.Orig.z) * ray.rD.z; 
    float tz2 = (bmax.z - ray.Orig.z) * ray.rD.z;

    tmin = max(tmin, min(tz1, tz2)); 
    tmax = min(tmax, max(tz1, tz2));

    if (tmax >= tmin && tmin < ray.T && tmax > 0) 
        return tmin; 
    else 
        return FLOAT_DIST_MAX;
}

#if USE_SSE

float IntersectAABB_SSE(const Ray& ray, const __m128& bmin4, const __m128& bmax4)
{
    static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
    __m128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, mask4), ray.O4), ray.rD4);
    __m128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, mask4), ray.O4), ray.rD4);
    __m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
    float tmax = min(vmax4.m128_f32[0], min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
    float tmin = max(vmin4.m128_f32[0], max(vmin4.m128_f32[1], vmin4.m128_f32[2]));

    if (tmax >= tmin && tmin < ray.T && tmax > 0) 
        return tmin; 
    else 
        return FLOAT_DIST_MAX;
}

#endif

void IntersectBVH(Ray& ray)
{
    BVHNode* node = &bvhNode[rootNodeIdx], * stack[64];
    uint stackPtr = 0;
    while (1)
    {
        if (node->IsLeaf())
        {
            for (uint i = 0; i < node->TriCount; i++)
                IntersectTri(ray, tri[triIdx[node->LeftFirst + i]]);
            if (stackPtr == 0) 
                break; 
            else 
                node = stack[--stackPtr];
            continue;
        }

        BVHNode* child1 = &bvhNode[node->LeftFirst];
        BVHNode* child2 = &bvhNode[node->LeftFirst + 1];
#if USE_SSE
        float dist1 = IntersectAABB_SSE(ray, child1->AABBMin4, child1->AABBMax4);
        float dist2 = IntersectAABB_SSE(ray, child2->AABBMin4, child2->AABBMax4);
#else
        float dist1 = IntersectAABB(ray, child1->AABBMin, child1->AABBMax);
        float dist2 = IntersectAABB(ray, child2->AABBMin, child2->AABBMax);
#endif

        if (dist1 > dist2) 
        {
            swap(dist1, dist2); 
            swap(child1, child2); 
        }

        if (dist1 == FLOAT_DIST_MAX)
        {
            if (stackPtr == 0) 
                break; 
            else 
                node = stack[--stackPtr];
        }
        else
        {
            node = child1;
            if (dist2 != FLOAT_DIST_MAX)
                stack[stackPtr++] = child2;
        }
    }
}

void BuildBVH()
{
    if (!bvhNode)
    {
        bvhNode = (BVHNode*)_aligned_malloc(sizeof(BVHNode) * N * 2, 64);
    }

    nodesUsed = 2;

    if (!bvhNode)
        return;

    for (int i = 0; i < N; i++) 
        triIdx[i] = i;
    for (int i = 0; i < N; i++)
        tri[i].Centroid = (tri[i].Vertex0 + tri[i].Vertex1 + tri[i].Vertex2) * 0.3333f;

    BVHNode& root = bvhNode[rootNodeIdx];
    root.LeftFirst = 0, root.TriCount = N;
    UpdateNodeBounds(rootNodeIdx);
    {
        Timer t;
        Subdivide(rootNodeIdx);
        printf("BVH (%i nodes) constructed in %.2fms.\n", nodesUsed, t.elapsed() * 1000);
    }
}

void UpdateNodeBounds(uint nodeIdx)
{
    BVHNode& node = bvhNode[nodeIdx];
    node.AABBMin = float3(1e30f);
    node.AABBMax = float3(-1e30f);
    for (uint first = node.LeftFirst, i = 0; i < node.TriCount; i++)
    {
        uint leafTriIdx = triIdx[first + i];
        Tri& leafTri = tri[leafTriIdx];
        node.AABBMin = fminf(node.AABBMin, leafTri.Vertex0);
        node.AABBMin = fminf(node.AABBMin, leafTri.Vertex1);
        node.AABBMin = fminf(node.AABBMin, leafTri.Vertex2);
        node.AABBMax = fmaxf(node.AABBMax, leafTri.Vertex0);
        node.AABBMax = fmaxf(node.AABBMax, leafTri.Vertex1);
        node.AABBMax = fmaxf(node.AABBMax, leafTri.Vertex2);
    }
}

float EvaluateSAH(BVHNode& node, int axis, float pos)
{
    // determine triangle counts and bounds for this split candidate
    AABB leftBox, rightBox;
    int leftCount = 0, rightCount = 0;
    for (uint i = 0; i < node.TriCount; i++)
    {
        Tri& triangle = tri[triIdx[node.LeftFirst + i]];
        if (triangle.Centroid[axis] < pos)
        {
            leftCount++;
            leftBox.Grow(triangle.Vertex0);
            leftBox.Grow(triangle.Vertex1);
            leftBox.Grow(triangle.Vertex2);
        }
        else
        {
            rightCount++;
            rightBox.Grow(triangle.Vertex0);
            rightBox.Grow(triangle.Vertex1);
            rightBox.Grow(triangle.Vertex2);
        }
    }
    float cost = leftCount * leftBox.Area() + rightCount * rightBox.Area();
    return cost > 0 ? cost : FLOAT_DIST_MAX;
}

float FindBestSplitPlane(BVHNode& node, int& outAxis, float& splitPos)
{
    int bestAxis = -1;
    float bestPos = 0;
    float bestCost = FLOAT_DIST_MAX;

    constexpr float PLANE_INTERVALS = 8;
    
    for (int axis = 0; axis < 3; axis++)
    {
        float boundsMin = FLOAT_DIST_MAX;
        float boundsMax = -FLOAT_DIST_MAX;

        for (uint i = 0; i < node.TriCount; ++i)
        {
            Tri& triangle = tri[triIdx[node.LeftFirst + i]];
            boundsMin = min(boundsMin, triangle.Centroid[axis]);
            boundsMax = max(boundsMax, triangle.Centroid[axis]);
        }

        if (boundsMin == boundsMax)
            continue;

        Bin bins[BINS];
        float scale = BINS / (boundsMax - boundsMin);
        for (uint i = 0; i < node.TriCount; ++i)
        {
            Tri& triangle = tri[triIdx[node.LeftFirst + i]];
            int binIdx = min((int)BINS - 1, (int)((triangle.Centroid[axis] - boundsMin) * scale));
            bins[binIdx].TriCount++;
            bins[binIdx].Bounds.Grow(triangle.Vertex0);
            bins[binIdx].Bounds.Grow(triangle.Vertex1);
            bins[binIdx].Bounds.Grow(triangle.Vertex2);
        }

        float leftArea[BINS - 1];
        float rightArea[BINS - 1];
        int leftCount[BINS - 1];
        int rightCount[BINS - 1];

        AABB leftBox;
        AABB rightBox;
        int leftSum = 0;
        int rightSum = 0;

        for (int i = 0; i < BINS - 1; ++i)
        {
            leftSum += bins[i].TriCount;
            leftCount[i] = leftSum;
            leftBox.Grow(bins[i].Bounds);
            leftArea[i] = leftBox.Area();
            rightSum += bins[BINS - 1 - i].TriCount;
            rightCount[BINS - 2 - i] = rightSum;
            rightBox.Grow(bins[BINS - 1 - i].Bounds);
            rightArea[BINS - 2 - i] = rightBox.Area();
        }

        scale = (boundsMax - boundsMin) / BINS;

        for (uint i = 0; i < BINS - 1; ++i)
        {
            float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
            if (planeCost < bestCost)
            {
                bestCost = planeCost;
                splitPos = boundsMin + scale * (i + 1);
                outAxis = axis;
            }
        }
    }

    return bestCost;
}

float CalculateNodeCost(BVHNode& node)
{
    float3 e = node.AABBMax - node.AABBMin; // extent of parent
    float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;
    float cost = node.TriCount * surfaceArea;

    return cost;
}

void Subdivide(uint nodeIdx)
{
    BVHNode& node = bvhNode[nodeIdx];

    // determine split axis using SAH
    int axis;
    float splitPos;
    float splitCost = FindBestSplitPlane(node, axis, splitPos);
    
    float noSplitCost = CalculateNodeCost(node);
    if (splitCost >= noSplitCost) 
        return;

    int i = node.LeftFirst;
    int j = i + node.TriCount - 1;
    while (i <= j)
    {
        if (tri[triIdx[i]].Centroid[axis] < splitPos)
            i++;
        else
            swap(triIdx[i], triIdx[j--]);
    }

    int leftCount = i - node.LeftFirst;
    if (leftCount == 0 || leftCount == node.TriCount) return;

    int leftChildIdx = nodesUsed++;
    int rightChildIdx = nodesUsed++;
    bvhNode[leftChildIdx].LeftFirst = node.LeftFirst;
    bvhNode[leftChildIdx].TriCount = leftCount;
    bvhNode[rightChildIdx].LeftFirst = i;
    bvhNode[rightChildIdx].TriCount = node.TriCount - leftCount;
    node.LeftFirst = leftChildIdx;
    node.TriCount = 0;
    UpdateNodeBounds(leftChildIdx);
    UpdateNodeBounds(rightChildIdx);

    Subdivide(leftChildIdx);
    Subdivide(rightChildIdx);
}

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

    FILE* file = fopen("Assets/bigben.tri", "r");
    for (int t = 0; t < N; t++)
    {
        int ret = fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
            &original[t].Vertex0.x, &original[t].Vertex0.y, &original[t].Vertex0.z,
            &original[t].Vertex1.x, &original[t].Vertex1.y, &original[t].Vertex1.z,
            &original[t].Vertex2.x, &original[t].Vertex2.y, &original[t].Vertex2.z);

        tri[t].Vertex0 = original[t].Vertex0;
        tri[t].Vertex1 = original[t].Vertex1;
        tri[t].Vertex2 = original[t].Vertex2;
    }

    fclose(file);

	BuildBVH();
}

void Animate()
{
    static float r = 0.f;
    constexpr float TWO_PI = 2 * PI;
    if ((r += 0.05f) > TWO_PI)
        r -= TWO_PI;

    float a = sinf(r) * 0.5f;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            float3 o = (&original[i].Vertex0)[j];
            float s = a * (o.y - 0.2f) * 0.2f;
            float x = o.x * cosf(s) - o.y * sinf(s);
            float y = o.x * sinf(s) + o.y * cosf(s);
            (&tri[i].Vertex0)[j] = float3(x, y, o.z);
        }
    }
    
}

void RefitBVH()
{
    for (int i = nodesUsed - 1; i >= 0; --i)
    {
        if (i != 1)
        {
            BVHNode& node = bvhNode[i];
            if (node.IsLeaf())
            {
                UpdateNodeBounds(i);
                continue;
            }

            auto& leftChild = bvhNode[node.LeftFirst];
            auto& rightChild = bvhNode[node.LeftFirst + 1];

            node.AABBMin = fminf(leftChild.AABBMin, rightChild.AABBMin);
            node.AABBMax = fmaxf(leftChild.AABBMax, rightChild.AABBMax);
        }
    }
}

void MyApp::Tick( float deltaTime )
{	
    // clear the screen to black
    screen->Clear(0);

    Timer t;
    Animate();
    RefitBVH();

    float3 p0(-1, 1, 2), p1(1, 1, 2), p2(-1, -1, 2);
	Ray ray;

	constexpr uint WIDTH = SCRWIDTH;
	constexpr uint HEIGHT = SCRHEIGHT;
    constexpr uint TILE_COUNT = 6400;
    constexpr uint TILE_SIZE = 8;

    #pragma omp parallel for schedule(dynamic)
    for (int tile = 0; tile < TILE_COUNT; ++tile)
    {
        int x = tile % 80;
        int y = tile / 80;

        Ray ray;
        ray.Orig = float3(0, 3.5f, -4.5f);
        for (int v = 0; v < TILE_SIZE; ++v)
        {
            for (int u = 0; u < TILE_SIZE; ++u)
            {
                float3 pixelPos = ray.Orig + p0 +
                    (p1 - p0) * ((x * TILE_SIZE + u) / 640.f) +
                    (p2 - p0) * ((y * TILE_SIZE + v) / 640.f);

                ray.Dir = normalize(pixelPos - ray.Orig);
                ray.T = FLOAT_DIST_MAX;
                ray.rD = float3(1 / ray.Dir.x, 1 / ray.Dir.y, 1 / ray.Dir.z);

                IntersectBVH(ray);
                uint c = ray.T < FLOAT_DIST_MAX ? (255 - (int)((ray.T - 4) * 180)) : 0;
                screen->Plot(x * 8 + u, y * 8 + v, c * 0x10101);
            }
        }
    }

	float elapsed = t.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);
}