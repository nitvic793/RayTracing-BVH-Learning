#include <Common/precomp.h>
#include <BVH.h>

#pragma warning (disable: 4996)

namespace bvh
{
    void IntersectTri(Ray& ray, const Tri& tri)
    {
        constexpr float EPSILON = 0.0001f;

		const float3 edge1 = tri.Vertex1 - tri.Vertex0;
		const float3 edge2 = tri.Vertex2 - tri.Vertex0;
		const float3 h = cross(ray.Dir, edge2);
		const float a = dot(edge1, h);

		if (a > -EPSILON && a < EPSILON)
			return; // ray parallel to triangle

		const float f = 1.0f / a;
		const float3 s = ray.Orig - tri.Vertex0;
		const float u = f * dot(s, h);
		if (u < 0 || u > 1) 
			return;

		const float3 q = cross(s, edge1);
		const float v = f * dot(ray.Dir, q);
		if (v < 0 || u + v > 1) 
			return;

		const float t = f * dot(edge2, q);
		if (t > EPSILON)
			ray.T = min(ray.T, t);
    }

	BVH::BVH(const char* triFile, int N) :
        nodesUsed(2)
	{
        FILE* file = fopen(triFile, "r");
        triCount = N;
        tri = new Tri[N];
        for (int t = 0; t < N; t++)
        {
            int ret = fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
                &tri[t].Vertex0.x, &tri[t].Vertex0.y, &tri[t].Vertex0.z,
                &tri[t].Vertex1.x, &tri[t].Vertex1.y, &tri[t].Vertex1.z,
                &tri[t].Vertex2.x, &tri[t].Vertex2.y, &tri[t].Vertex2.z);
        }

        fclose(file);

        bvhNode = (BVHNode*)_aligned_malloc(sizeof(BVHNode) * N * 2, 64);
        triIdx = new uint[N];

        Build();
	}

	void BVH::Build()
	{
        uint rootNodeIdx = 0;
        nodesUsed = 2;

        if (!bvhNode)
            return;

        for (uint i = 0; i < triCount; i++)
            triIdx[i] = i;

        for (uint i = 0; i < triCount; i++)
            tri[i].Centroid = (tri[i].Vertex0 + tri[i].Vertex1 + tri[i].Vertex2) * 0.3333f;

        BVHNode& root = bvhNode[rootNodeIdx];

        root.LeftFirst = 0; 
        root.TriCount = triCount;

        UpdateNodeBounds(rootNodeIdx);
        {
            Timer t;
            Subdivide(rootNodeIdx);
            printf("BVH (%i nodes) constructed in %.2fms.\n", nodesUsed, t.elapsed() * 1000);
        }
	}

	void BVH::Refit()
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

	void BVH::Intersect(Ray& ray)
	{
        uint rootNodeIdx = 0;

        BVHNode* node = &bvhNode[rootNodeIdx];
        BVHNode* stack[64];

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

    float CalculateNodeCost(BVHNode& node)
    {
        float3 e = node.AABBMax - node.AABBMin; // extent of parent
        float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;
        float cost = node.TriCount * surfaceArea;

        return cost;
    }

	void BVH::Subdivide(uint nodeIdx)
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

	void BVH::UpdateNodeBounds(uint nodeIdx)
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

	float BVH::FindBestSplitPlane(BVHNode& node, int& outAxis, float& splitPos)
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
}