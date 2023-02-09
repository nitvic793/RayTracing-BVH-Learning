#include <Common/precomp.h>
#include <BVH.h>
#include <TLAS.h>

#pragma warning (disable: 4996)

namespace bvh
{
    TLAS::TLAS(BVHInstance* bvhList, int N)
    {
        blas = bvhList;
        blasCount = N;

        tlasNode = (TLASNode*)_aligned_malloc(sizeof(TLASNode) * N * 2, 64);
        nodesUsed = 1;
    }

    void TLAS::Build()
    { 
        int nodeIdx[256];
        int nodeIndices = blasCount;

        nodesUsed = 1; // 0 is root node
        for (uint i = 0; i < blasCount; ++i)
        {
            nodeIdx[i] = nodesUsed;
            tlasNode[nodesUsed].AABBMin = blas[i].bounds.bmin;
            tlasNode[nodesUsed].AABBMax = blas[i].bounds.bmax;
            tlasNode[nodesUsed].BLAS = i;
            tlasNode[nodesUsed].leftRight = 0; // 0 = leaf node
            nodesUsed++;
        }

        // Agglomerative clustering algorithm
        int A = 0;
        int B = FindBestMatch(nodeIdx, nodeIndices, A);
        while (nodeIndices > 1)
        {
            int C = FindBestMatch(nodeIdx, nodeIndices, B);
            if (A == C)
            {
                int nodeIdxA = nodeIdx[A];
                int nodeIdxB = nodeIdx[B];

                TLASNode& nodeA = tlasNode[nodeIdxA];
                TLASNode& nodeB = tlasNode[nodeIdxB];
                TLASNode& newNode = tlasNode[nodesUsed];

                newNode.leftRight = nodeIdxA + (nodeIdxB << 16);
                newNode.AABBMin = fminf(nodeA.AABBMin, nodeB.AABBMin);
                newNode.AABBMax = fmaxf(nodeA.AABBMax, nodeB.AABBMax);
                nodeIdx[A] = nodesUsed++;
                nodeIdx[B] = nodeIdx[nodeIndices - 1];
                B = FindBestMatch(nodeIdx, --nodeIndices, A);
            }
            else
            {
                A = B;
                B = C;
            }
        }

        tlasNode[0] = tlasNode[nodeIdx[A]];
    }

    void TLAS::Intersect(Ray& ray)
    {
        TLASNode* node = &tlasNode[0];
        TLASNode* stack[64];

        uint stackPtr = 0;

        while (1)
        {
            if (node->IsLeaf())
            {
                blas[node->BLAS].Intersect(ray);
                if (stackPtr == 0)
                    break;
                else
                    node = stack[--stackPtr];

                continue;
            }

            TLASNode* child1 = &tlasNode[node->LeftChild()];
            TLASNode* child2 = &tlasNode[node->RightChild()];

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

    int TLAS::FindBestMatch(int* list, int N, int A)
    {
        float smallest = FLOAT_DIST_MAX;
        int bestB = -1;

        for (int B = 0; B < N; ++B)
        {
            if (B != A)
            {
                float3 bmax = fmaxf(tlasNode[list[A]].AABBMax, tlasNode[list[B]].AABBMax);
                float3 bmin = fmaxf(tlasNode[list[A]].AABBMin, tlasNode[list[B]].AABBMin);
                float3 extent = bmax - bmin;
                float surfaceArea = GetSurfaceArea(extent);
                if (surfaceArea < smallest)
                {
                    smallest = surfaceArea;
                    bestB = B;
                }
            }
        }

        return bestB;
    }
}