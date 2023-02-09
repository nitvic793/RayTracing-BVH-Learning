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


    struct Intersection
    {
        float T = FLOAT_DIST_MAX;		// intersection distance along ray
        float U;
        float V;		// U,V => barycentric coordinates
        uint instPrim;	// instance index (12 bit) and primitive index (20 bit)
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

        Intersection Hit;
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

    void IntersectTri(Ray& ray, const Tri& tri, const uint instPrim);
    float IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax);
    float IntersectAABB_SSE(const Ray& ray, const __m128& bmin4, const __m128& bmax4);

    constexpr float GetSurfaceArea(const float3& extent)
    {
        const auto& e = extent;
        return e.x * e.y + e.y * e.z + e.z * e.x;
    }

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

    class Mesh;

    class BVH
    {
    public:
        static constexpr uint BINS = 8;

    public:
        BVH() = default;
        BVH(Mesh* triMesh);

        void  Build();
        void  Refit();
        void  Intersect(Ray& ray, uint instanceIdx);
        void  SetTransform(const mat4& transform);

        // Public data
        mat4        invTransform;
        BVHNode*    bvhNode = nullptr;
        AABB        bounds;

    private:
        void  Subdivide(uint nodeIdx);
        void  UpdateNodeBounds(uint nodeIdx);
        float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos);

    private:
        uint*       triIdx      = nullptr;
        uint        nodesUsed   = 2; 
        Mesh*       mesh        = nullptr;
    };

    class BVHInstance
    {
    public:
        BVHInstance() = default;
        BVHInstance(BVH* blas, uint index) : bvh(blas), idx(index) { SetTransform(mat4()); }
        void SetTransform(const mat4& transform);
        const mat4& GetTransform() const { return transform; }
        void Intersect(Ray& ray);

    private:
        BVH* bvh = 0;
        mat4 transform;
        mat4 invTransform; // inverse transform
        uint idx;

    public:
        AABB bounds; // in world space
    };

    struct TriEx 
    { 
        float2 uv0; 
        float2 uv1;
        float2 uv2;
        float3 N0;
        float3 N1; 
        float3 N2;
        float dummy; 
    };

    class Mesh
    {
    public:
        Mesh() = default;
        Mesh(const char* objFile, const char* textureFile);

        BVH*    bvh = nullptr;
        Tri*     tri;
        TriEx*   triEx;
        float3* P = nullptr;
        float3* N = nullptr;
        int     triCount = 0;

        Surface* texture = nullptr;
    };
}