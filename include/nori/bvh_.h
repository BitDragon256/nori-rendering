/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <bvh/v2/bvh.h>
#include <bvh/v2/vec.h>
#include <bvh/v2/ray.h>
#include <bvh/v2/node.h>
#include <bvh/v2/default_builder.h>
#include <bvh/v2/thread_pool.h>
#include <bvh/v2/executor.h>
#include <bvh/v2/stack.h>
#include <bvh/v2/tri.h>

#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class BVH
{
public:
    BVH();
    virtual ~BVH() = default;
    void clear();

    void addMesh(Mesh *mesh);
    void build();

    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay = false) const;

    /// Return the total number of meshes registered with the BVH
    uint32_t getMeshCount() const { return (uint32_t) m_meshes.size(); }

    /// Return one of the registered meshes
    Mesh *getMesh(uint32_t idx) { return m_meshes[idx]; }

    /// Return one of the registered meshes (const version)
    const Mesh *getMesh(uint32_t idx) const { return m_meshes[idx]; }

    //// Return an axis-aligned bounding box containing the entire tree
    const BoundingBox3f &getBoundingBox() const {
        return m_boundingBox;
    }

private:

    using Scalar  = float;
    using Vec3    = bvh::v2::Vec<Scalar, 3>;
    using BBox    = bvh::v2::BBox<Scalar, 3>;
    using Tri     = bvh::v2::Tri<Scalar, 3>;
    using Node    = bvh::v2::Node<Scalar, 3>;
    using Bvh     = bvh::v2::Bvh<Node>;
    using Ray     = bvh::v2::Ray<Scalar, 3>;

    using PrecomputedTri = bvh::v2::PrecomputedTri<Scalar>;

    std::vector<Mesh*> m_meshes;
    BoundingBox3f m_boundingBox;

    std::vector<size_t> m_meshTriangleStart;

    // BVH lib
    std::vector<Tri> m_triangles;
    std::vector<PrecomputedTri> m_precomputedTris;

    std::vector<BBox> m_boundingBoxes;
    std::vector<Vec3> m_centers;
    Bvh m_bvh;

    bvh::v2::ThreadPool m_threadPool;
    bvh::v2::ParallelExecutor m_executor{ m_threadPool };

    static constexpr bool m_should_permute = true;

};

NORI_NAMESPACE_END
