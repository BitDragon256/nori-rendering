/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
    Changed 2024 by Benedict Sondershaus
*/

#include <nori/bvh.h>

using Scalar  = float;
using Vec3    = bvh::v2::Vec<Scalar, 3>;
using BBox    = bvh::v2::BBox<Scalar, 3>;
using Tri     = bvh::v2::Tri<Scalar, 3>;
using Node    = bvh::v2::Node<Scalar, 3>;
using Bvh     = bvh::v2::Bvh<Node>;
using Ray     = bvh::v2::Ray<Scalar, 3>;

NORI_NAMESPACE_BEGIN

Vec3 nori2bvh_vec(const Vector3f& v)
{
    return Vec3(v.x(), v.y(), v.z());
}
Vector3f bvh2nori_vec(const Vec3& v)
{
    return Vector3f(v[0], v[1], v[2]);
}

BVH::BVH()
{

}

void BVH::clear()
{
    m_meshes.clear();
}

void BVH::addMesh(Mesh *mesh)
{
    m_meshes.emplace_back(mesh);
    const auto meshTriangleCount = static_cast<size_t>(mesh->getTriangleCount());
    const auto oldTriangleCount = m_triangles.size();

    m_meshTriangleStart.push_back(oldTriangleCount);

    m_triangles.resize(m_triangles.size() + meshTriangleCount);
    m_boundingBoxes.resize(m_triangles.size());
    m_centers.resize(m_triangles.size());

    m_executor.for_each(0, meshTriangleCount, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i)
        {
            const size_t triIndex = i + oldTriangleCount;

            const MatrixXf &V  = mesh->getVertexPositions();
            const MatrixXu &F  = mesh->getIndices();

            const uint32_t idx0 = F(0, i), idx1 = F(1, i), idx2 = F(2, i);

            m_triangles[triIndex] = Tri(
                nori2bvh_vec(V.col(idx0)),
                nori2bvh_vec(V.col(idx1)),
                nori2bvh_vec(V.col(idx2))
            );
            m_boundingBoxes[triIndex] = m_triangles[triIndex].get_bbox();
            m_centers[triIndex] = m_triangles[triIndex].get_center();
        }
    });
}


void BVH::build()
{
    bvh::v2::DefaultBuilder<Node>::Config config;
    config.quality = bvh::v2::DefaultBuilder<Node>::Quality::High;

    m_bvh = bvh::v2::DefaultBuilder<Node>::build(m_threadPool, m_boundingBoxes, m_centers, config);

    m_precomputedTris.resize(m_triangles.size());
    m_executor.for_each(0, m_triangles.size(), [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i)
        {
            const auto j = m_should_permute ? m_bvh.prim_ids[i] : i;
            m_precomputedTris[i] = m_triangles[j];
        }
    });
}

bool BVH::rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const
{
    Ray bvhRay;
    bvhRay.dir = Vec3(ray.d.x(), ray.d.y(), ray.d.z());
    bvhRay.org = Vec3(ray.o.x(), ray.o.y(), ray.o.z());
    bvhRay.tmin = ray.mint;
    bvhRay.tmax = ray.maxt;

    static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    static constexpr size_t stack_size = 64;
    static constexpr bool use_robust_traversal = false;

    auto prim_id = invalid_id;
    Scalar u, v;

    bvh::v2::SmallStack<Bvh::Index, stack_size> stack;
    m_bvh.intersect<false, use_robust_traversal>(bvhRay, m_bvh.get_root().index, stack,
    [&] (size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (auto hit = PrecomputedTri(m_triangles[i]).intersect(bvhRay))
            {
                prim_id = i;
                std::tie(u, v) = *hit;
            }

            continue;
            const size_t j = m_should_permute ? i : m_bvh.prim_ids[i];
            if (auto hit = m_precomputedTris[j].intersect(bvhRay)) {
                prim_id = i;
                std::tie(u, v) = *hit;
            }
        }
        return prim_id != invalid_id;
    });

    if (prim_id != invalid_id)
    {
        // get mesh
        size_t meshIndex = m_meshes.size() - 1;
        for (size_t i = 0; i < meshIndex; ++i)
        {
            if (m_meshTriangleStart[i+1] > prim_id)
            {
                meshIndex = i;
                break;
            }
        }

        // fill intersection info
        its.t = bvhRay.tmax;
        its.mesh = m_meshes[meshIndex];
        its.prim_idx = prim_id - m_meshTriangleStart[meshIndex];

        const auto sqrtU = std::sqrt(u);
        Vector3f bary = { 1.f - u - v, u, v };

        // vvvvvvvvv COPIED FROM NORI SOURCE vvvvvvvvv
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        const uint32_t idx0 = F(0, its.prim_idx), idx1 = F(1, its.prim_idx), idx2 = F(2, its.prim_idx);
        const Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        // const auto realIntersection = bvhRay.org + bvhRay.dir * bvhRay.tmax;
        // std::cout << its.p.toString() << " <-> " << realIntersection[0] << " " << realIntersection[1] << " " << realIntersection[2] << std::endl;

        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            const Normal3f n0 = N.col(idx0), n1 = N.col(idx1), n2 = N.col(idx2);

            its.shFrame = Frame(
                (bary.x() * n0 +
                 bary.y() * n1 +
                 bary.z() * n2).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }

        return true;
    }

    return false;
}


NORI_NAMESPACE_END
