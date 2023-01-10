#include <Common/precomp.h>
#include <BVH.h>

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
}