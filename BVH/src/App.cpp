#include "Common/precomp.h"
#include "App.h"
#include <BVH.h>

#pragma warning (disable: 4996)

using namespace bvh;

TheApp* CreateApp() { return new MyApp(); }

constexpr uint N = 12582;

Tri tri[N];
uint triIdx[N];
BVHNode BVHNodes[2 * N + 1];
uint RootNodeIdx = 0;
uint NodesUsed = 1;

struct AABB
{
	float3 BMax = -FLOAT_MAX;
	float3 BMin = FLOAT_MAX;
};

void LoadTris()
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

	FILE* file = fopen(".\\Assets\\unity.tri", "r");
	float a, b, c, d, e, f, g, h, i;
	for (int t = 0; t < N; t++)
	{
		auto ret = fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
			&a, &b, &c, &d, &e, &f, &g, &h, &i);
		tri[t].Vertex0 = float3(a, b, c);
		tri[t].Vertex1 = float3(d, e, f);
		tri[t].Vertex2 = float3(g, h, i);
	}

	for (int t = 0; t < N; t++)
		triIdx[t] = t;

	fclose(file);
}

void UpdateNodeBounds(uint nodeIdx, BVHNode* nodes, Tri* tris, uint* triIndices, uint& nodesUsed)
{
	auto& node = nodes[nodeIdx];
	node.AABBMin = float3(FLOAT_MAX);
	node.AABBMax = float3(-FLOAT_MAX);

	for (uint first = node.LeftFirst, i = 0; i < node.TriCount; ++i)
	{
		const uint leafTriIdx = triIndices[first + i];
		Tri& leafTri = tris[leafTriIdx];
		node.AABBMin = fminf(node.AABBMin, leafTri.Vertex0);
		node.AABBMin = fminf(node.AABBMin, leafTri.Vertex1);
		node.AABBMin = fminf(node.AABBMin, leafTri.Vertex2);
		node.AABBMax = fmaxf(node.AABBMax, leafTri.Vertex0);
		node.AABBMax = fmaxf(node.AABBMax, leafTri.Vertex1);
		node.AABBMax = fmaxf(node.AABBMax, leafTri.Vertex2);
	}
}

void Subdivide(uint nodeIdx, BVHNode* nodes, Tri* tris, uint* triIndices, uint& nodesUsed)
{
	auto& node = nodes[nodeIdx];
	if (node.TriCount <= 2) 
		return;

	float3 extent = node.AABBMax - node.AABBMin;

	int axis = 0;
	if (extent.y > extent.x) 
		axis = 1;

	if (extent.z > extent[axis])
		axis = 2;

	float splitPos = node.AABBMin[axis] + extent[axis] * 0.5f;

	int i = node.LeftFirst;
	int j = i + node.TriCount - 1;

	while (i <= j)
	{
		if (tris[triIndices[i]].Centroid[axis] < splitPos)
			i++;
		else
			swap(triIndices[i], triIndices[j--]);
	}

	int leftCount = i - node.LeftFirst;
	if (leftCount == 0 || leftCount == node.TriCount)
		return;

	int leftChildIdx = nodesUsed++;
	int rightChildIdx = nodesUsed++;
	
	nodes[leftChildIdx].LeftFirst = node.LeftFirst;
	nodes[leftChildIdx].TriCount = leftCount;
	nodes[rightChildIdx].LeftFirst = i;
	nodes[rightChildIdx].TriCount = node.TriCount - leftCount;

	node.LeftFirst = leftChildIdx;
	node.TriCount = 0;

	UpdateNodeBounds(leftChildIdx, nodes, tris, triIndices, nodesUsed);
	UpdateNodeBounds(rightChildIdx, nodes, tris, triIndices, nodesUsed);

	Subdivide(leftChildIdx, nodes, tris, triIndices, nodesUsed);
	Subdivide(rightChildIdx, nodes, tris, triIndices, nodesUsed);
}

void BuildBVH(Tri* tri, uint* triIndices, const uint triCount, const uint rootNodeIdx, uint& nodesUsed, BVHNode* bvhNodes, uint maxBvhNodeCount)
{
	for (uint i = 0; i < triCount; ++i)
	{
		auto& t = tri[i];
		t.Centroid = (t.Vertex0 + t.Vertex1 + t.Vertex2) * 0.3333f;
	}

	auto& root = bvhNodes[rootNodeIdx];
	root.LeftFirst = 0;
	root.TriCount = triCount;
	UpdateNodeBounds(rootNodeIdx, bvhNodes, tri, triIndices, nodesUsed);
	Subdivide(rootNodeIdx, bvhNodes, tri, triIndices, nodesUsed);
}

constexpr bool IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax)
{
	float tx1 = (bmin.x - ray.Orig.x) / ray.Dir.x, tx2 = (bmax.x - ray.Orig.x) / ray.Dir.x;
	float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
	float ty1 = (bmin.y - ray.Orig.y) / ray.Dir.y, ty2 = (bmax.y - ray.Orig.y) / ray.Dir.y;
	tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
	float tz1 = (bmin.z - ray.Orig.z) / ray.Dir.z, tz2 = (bmax.z - ray.Orig.z) / ray.Dir.z;
	tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
	return tmax >= tmin && tmin < ray.T&& tmax > 0;
}

void IntersectBVH(Ray& ray, const uint nodeIdx, BVHNode* nodes, Tri* tris, uint* triIndices)
{
	auto& node = nodes[nodeIdx];
	if (!IntersectAABB(ray, node.AABBMin, node.AABBMax))
		return;

	if (node.IsLeaf())
	{
		for (uint i = 0; i < node.TriCount; ++i)
			IntersectTri(ray, tri[triIndices[node.LeftFirst + i]]);
	}
	else
	{
		IntersectBVH(ray, node.LeftFirst, nodes, tris, triIndices);
		IntersectBVH(ray, node.LeftFirst + 1, nodes, tris, triIndices);
	}
}

void MyApp::Init()
{
	LoadTris();

	BuildBVH(tri, triIdx, N, RootNodeIdx, NodesUsed, BVHNodes, 2 * N + 1);
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

	for (uint y = 0; y < HEIGHT; ++y)
	{
		for (uint x = 0; x < WIDTH; ++x)
		{
			ray.Orig = float3(-1.5f, -0.2f, -2.5f);
			float3 pixelPos = ray.Orig + p0 +
				(p1 - p0) * (x / (float)WIDTH) +
				(p2 - p0) * (y / (float)HEIGHT);

			ray.Dir = normalize(pixelPos - ray.Orig);
			ray.T = FLOAT_MAX;

			//for (int i = 0; i < N; i++)
			//{
			//	IntersectTri(ray, tri[i]);
			//}

			IntersectBVH(ray, RootNodeIdx, BVHNodes, tri, triIdx);

			uint c = 500 - (int)(ray.T * 42);
			if (ray.T < FLOAT_MAX)
				screen->Plot(x, y, c * 0x10101);
		}
	}

	float elapsed = t.elapsed() * 1000;
	printf("tracing time: %.2fms (%5.2fK rays/s)\n", elapsed, sqr(630) / elapsed);

#if 0

	static Kernel* kernel = 0;			// statics should be members of MyApp of course.
	static Surface bitmap( 512, 512 );	// having them here allows us to disable the OpenCL
	static Buffer* clBuffer = 0;		// demonstration using a single #if 0.
	static int offset = 0;
	if (!kernel)
	{
		// prepare for OpenCL work
		Kernel::InitCL();		
		// compile and load kernel "render" from file "kernels.cl"
		kernel = new Kernel( "cl/kernels.cl", "render" );
		// create an OpenCL buffer over using bitmap.pixels
		clBuffer = new Buffer( 512 * 512, Buffer::DEFAULT, bitmap.pixels );
	}
	// pass arguments to the OpenCL kernel
	kernel->SetArgument( 0, clBuffer );
	kernel->SetArgument( 1, offset++ );
	// run the kernel; use 512 * 512 threads
	kernel->Run( 512 * 512 );
	// get the results back from GPU to CPU (and thus: into bitmap.pixels)
	clBuffer->CopyFromDevice();
	// show the result on screen
	bitmap.CopyTo( screen, 500, 200 );

#endif

}