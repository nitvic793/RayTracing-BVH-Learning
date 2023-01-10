#pragma once

namespace bvh
{
    constexpr float FLOAT_DIST_MAX = 1e30f;

    struct Tri
    {
        float3 Vertex0;
        float3 Vertex1;
        float3 Vertex2;
        float3 Centroid;
    };

    __declspec(align(64)) struct Ray
    {
        Ray() 
        { 
            O4 = D4 = rD4 = _mm_set1_ps(1); 
            dummy1 = dummy2 = dummy3 = 0.f;
        }

        union 
        { 
            struct { float3 Orig; float dummy1; }; __m128 O4; 
        };

        union 
        {
            struct { float3 Dir; float dummy2; }; __m128 D4; 
        };

        union 
        { 
            struct { float3 rD; float dummy3; }; __m128 rD4; 
        };

        float  T = FLOAT_DIST_MAX;
    };

    struct BVHNode
    {
        union 
        {
            struct  
            {
                float3 AABBMin;
                uint   LeftFirst;
            };

            __m128 AABBMin4;
        };

        union 
        {
            struct 
            {
                float3 AABBMax;
                uint   TriCount;
            };

            __m128 AABBMax4;
        };

        constexpr bool IsLeaf() const
        {
            return TriCount > 0;
        }
    };

    void IntersectTri(Ray& ray, const Tri& tri);
}