#include <Common/precomp.h>
#include <BVH.h>
#include <TLAS.h>

#pragma warning (disable: 4996)

namespace bvh
{
    TLAS::TLAS(BVH* bvhList, int N)
    {
        blas = bvhList;
        blasCount = N;

        tlasNode = (TLASNode*)_aligned_malloc(sizeof(TLASNode) * N * 2, 64);
        nodesUsed = 2;
    }

    void TLAS::Build()
    {
        // assign a TLASleaf node to each BLAS
        tlasNode[2].leftBlas = 0;
        tlasNode[2].AABBMin = float3(-100);
        tlasNode[2].AABBMax = float3(100);
        tlasNode[2].isLeaf = true;
        tlasNode[3].leftBlas = 1;
        tlasNode[3].AABBMin = float3(-100);
        tlasNode[3].AABBMax = float3(100);
        tlasNode[3].isLeaf = true;

        // create a root node over the two leaf nodes
        tlasNode[0].leftBlas = 2;
        tlasNode[0].AABBMin = float3(-100);
        tlasNode[0].AABBMax = float3(100);
        tlasNode[0].isLeaf = false;
    }

    void TLAS::Intersect(Ray& ray)
    {
        TLASNode* node = &tlasNode[0];
        TLASNode* stack[64];

        uint stackPtr = 0;

        while (1)
        {
            if (node->isLeaf)
            {
                blas[node->leftBlas].Intersect(ray);
                if (stackPtr == 0)
                    break;
                else
                    node = stack[--stackPtr];

                continue;
            }

            TLASNode* child1 = &tlasNode[node->leftBlas];
            TLASNode* child2 = &tlasNode[node->leftBlas + 1];

            float dist1 = IntersectAABB(ray, child1->AABBMin, child1->AABBMax);
            float dist2 = IntersectAABB(ray, child2->AABBMin, child2->AABBMax);

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
}