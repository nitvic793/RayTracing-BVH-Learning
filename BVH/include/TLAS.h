#pragma once

#include <BVH.h>

namespace bvh
{
    struct TLASNode
    {
        float3 AABBMin;
        uint   leftRight; // 2x16 
        float3 AABBMax;
        uint   BLAS;

        constexpr bool IsLeaf() const 
        { 
            return leftRight == 0;
        }

        constexpr uint LeftChild() const
        {
            return leftRight & (0xFFFF);
        }

        constexpr uint RightChild() const
        {
            return leftRight >> 16;
        }
    };

    class TLAS
    {
    public:
        TLAS() = default;
        TLAS(BVH* bvhList, int N);
        void Build();
        void Intersect(Ray& ray);

    private:
        int FindBestMatch(int* list, int N, int A);

    private:
        TLASNode*   tlasNode = nullptr;
        BVH*        blas = nullptr;
        uint        nodesUsed = 1; 
        uint        blasCount = 0;
    };
}