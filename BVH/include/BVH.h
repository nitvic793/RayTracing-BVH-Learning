#pragma once

#define USE_SSE 1

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


    class BVH
    {
    public:
        static constexpr uint BINS = 8;

    public:
        BVH() = default;
        BVH(const char* triFile, int N);

        void Build();
        void Refit();
        void Intersect(Ray& ray);

    private:
        void    Subdivide(uint nodeIdx);
        void    UpdateNodeBounds(uint nodeIdx);
        float   FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos);

    private:
        BVHNode*    bvhNode = nullptr;
        Tri*        tri = nullptr;
        uint*       triIdx = nullptr;
        uint        nodesUsed; 
        uint        triCount;
    };
}