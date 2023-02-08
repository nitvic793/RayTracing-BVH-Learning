#pragma once

#include <BVH.h>

namespace bvh
{
    struct TLASNode
    {
        float3 AABBMin;
        uint   leftBlas;
        float3 AABBMax;
        uint   isLeaf;
    };

    class TLAS
    {
    public:
        TLAS() = default;
        TLAS(BVH* bvhList, int N);
        void Build();
        void Intersect(Ray& ray);
    private:
        TLASNode*   tlasNode = nullptr;
        BVH*        blas = nullptr;
        uint        nodesUsed; 
        uint        blasCount;
    };
}