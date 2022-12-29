#pragma once

namespace bvh
{
    constexpr float FLOAT_MAX = 1e30f;

    struct Tri
    {
        float3 Vertex0;
        float3 Vertex1;
        float3 Vertex2;
        float3 Centroid;
    };

    struct Ray
    {
        float3 Orig;
        float3 Dir;
        float  T;
    };

    struct BVHNode
    {
        float3 AABBMin;
        float3 AABBMax;
        uint   LeftFirst;
        uint   TriCount;
        
        constexpr bool IsLeaf() const
        {
            return TriCount > 0;
        }
    };

    void IntersectTri(Ray& ray, const Tri& tri);
}