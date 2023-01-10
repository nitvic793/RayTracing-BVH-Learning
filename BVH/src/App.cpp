#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>

#pragma warning (disable: 4996)

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

#define USE_SSE 1

constexpr uint N = 12582;

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

    float Area()
    {
        float3 e = bmax - bmin; // box extent
        return e.x * e.y + e.y * e.z + e.z * e.x;
    }
};


// application data
Tri tri[N];
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
            if (stackPtr == 0) break; else node = stack[--stackPtr];
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
    bvhNode = (BVHNode*)_aligned_malloc(sizeof(BVHNode) * N * 2, 64);
    if (!bvhNode)
        return;

    for (int i = 0; i < N; i++) 
        triIdx[i] = i;
    for (int i = 0; i < N; i++)
        tri[i].Centroid = (tri[i].Vertex0 + tri[i].Vertex1 + tri[i].Vertex2) * 0.3333f;

    BVHNode& root = bvhNode[rootNodeIdx];
    root.LeftFirst = 0, root.TriCount = N;
    UpdateNodeBounds(rootNodeIdx);
    Timer t;
    Subdivide(rootNodeIdx);
    printf("BVH (%i nodes) constructed in %.2fms.\n", nodesUsed, t.elapsed() * 1000);
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
    return cost > 0 ? cost : 1e30f;
}

void Subdivide(uint nodeIdx)
{
    BVHNode& node = bvhNode[nodeIdx];

    // determine split axis using SAH
    int bestAxis = -1;
    float bestPos = 0, bestCost = 1e30f;
    for (int axis = 0; axis < 3; axis++) for (uint i = 0; i < node.TriCount; i++)
    {
        Tri& triangle = tri[triIdx[node.LeftFirst + i]];
        float candidatePos = triangle.Centroid[axis];
        float cost = EvaluateSAH(node, axis, candidatePos);
        if (cost < bestCost)
            bestPos = candidatePos, bestAxis = axis, bestCost = cost;
    }
    int axis = bestAxis;
    float splitPos = bestPos;
    float3 e = node.AABBMax - node.AABBMin; // extent of parent
    float parentArea = e.x * e.y + e.y * e.z + e.z * e.x;
    float parentCost = node.TriCount * parentArea;
    if (bestCost >= parentCost) return;

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

    FILE* file = fopen("Assets/unity.tri", "r");
    float a, b, c, d, e, f, g, h, i;
    for (int t = 0; t < N; t++)
    {
        int ret = fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
            &a, &b, &c, &d, &e, &f, &g, &h, &i);
        tri[t].Vertex0 = float3(a, b, c);
        tri[t].Vertex1 = float3(d, e, f);
        tri[t].Vertex2 = float3(g, h, i);
    }
    fclose(file);

	BuildBVH();
}

void MyApp::Tick( float deltaTime )
{	
	// clear the screen to black
	screen->Clear( 0 );

	float3 camPos(0, 0, -18);
	float3 p0(-1, 1, 2), p1(1, 1, 2), p2(-1, -1, 2);
	Ray ray;
	Timer t;

	constexpr uint WIDTH = SCRWIDTH;
	constexpr uint HEIGHT = SCRHEIGHT;

	for (uint y = 0; y < HEIGHT; y+=4)
	{
		for (uint x = 0; x < WIDTH; x+=4)
		{
            for (int v = 0; v < 4; v++)
            {
                for (int u = 0; u < 4; u++)
                {
                    ray.Orig = float3(-1.5f, -0.2f, -2.5f);
                    float3 pixelPos = ray.Orig + p0 + (p1 - p0) * ((x + u) / (float)WIDTH) + (p2 - p0) * ((y + v) / (float)HEIGHT);
                    ray.Dir = normalize(pixelPos - ray.Orig);
                    ray.T = FLOAT_DIST_MAX;

                    ray.rD = float3(1 / ray.Dir.x, 1 / ray.Dir.y, 1 / ray.Dir.z);
                    IntersectBVH(ray);

                    uint c = 500 - (int)(ray.T * 42);
                    if (ray.T < FLOAT_DIST_MAX) 
                        screen->Plot(x + u, y + v, c * 0x10101);
                }
            }
		}
	}

	float elapsed = t.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);
}