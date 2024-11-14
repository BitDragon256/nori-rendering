/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <optional>
#include <random>

#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/dpdf.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Intersection data structure
 *
 * This data structure records local information about a ray-triangle intersection.
 * This includes the position, traveled ray distance, uv coordinates, as well
 * as well as two local coordinate frames (one that corresponds to the true
 * geometry, and one that is used for shading computations).
 */
struct Intersection {
    /// Position of the surface intersection
    Point3f p;
    /// Unoccluded distance along the ray
    float t;
    /// UV coordinates, if any
    Point2f uv;
    /// Shading frame (based on the shading normal)
    Frame shFrame;
    /// Geometric frame (based on the true geometry)
    Frame geoFrame;
    /// Pointer to the associated mesh
    const Mesh *mesh = nullptr;
    /// Primitive index in the associated mesh (e.g., triangle id)
    uint32_t prim_idx;

    /// Create an uninitialized intersection record
    Intersection() = default;

    /// Transform a direction vector into the local shading frame
    Vector3f toLocal(const Vector3f &d) const {
        return shFrame.toLocal(d);
    }

    /// Transform a direction vector from local to world coordinates
    Vector3f toWorld(const Vector3f &d) const {
        return shFrame.toWorld(d);
    }

    /// Return a human-readable summary of the intersection record
    std::string toString() const;
};

// helping renames
using TriangleIndices = std::array<uint32_t, 3>;
using TripleCoord = std::array<Point3f, 3>;

/**
 * \brief Triangle mesh
 *
 * This class stores a triangle mesh object and provides numerous functions
 * for querying the individual triangles. Subclasses of \c Mesh implement
 * the specifics of how to create its contents (e.g. by loading from an
 * external file)
 */
class Mesh : public NoriObject {
public:
    /// Release all memory
    virtual ~Mesh();

    /// Initialize internal data structures (called once by the XML parser)
    virtual void activate();

    /// Return the total number of triangles in this shape
    uint32_t getTriangleCount() const { return (uint32_t) m_F.cols(); }

    /// Return the total number of vertices in this shape
    uint32_t getVertexCount() const { return (uint32_t) m_V.cols(); }

    /**
     * \brief Uniformly sample a position on the mesh with
     * respect to surface area. Returns both position and normal
     */
    void samplePosition(const Point2f &sample, Point3f &p, Normal3f &n) const;

    /// Return the surface area of the given triangle
    float surfaceArea(uint32_t index) const;

    /// Return the surface area of the entire mesh
    float totalSurfaceArea() const;

    //// Return an axis-aligned bounding box of the entire mesh
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    //// Return an axis-aligned bounding box containing the given triangle
    BoundingBox3f getBoundingBox(uint32_t index) const;

    //// Return the centroid of the given triangle
    Point3f getCentroid(uint32_t index) const;

    /** \brief Ray-triangle intersection test
     *
     * Uses the algorithm by Moeller and Trumbore discussed at
     * <tt>http://www.acm.org/jgt/papers/MollerTrumbore97/code.html</tt>.
     *
     * Note that the test only applies to a single triangle in the mesh.
     * An acceleration data structure like \ref BVH is needed to search
     * for intersections against many triangles.
     *
     * \param index
     *    Index of the triangle that should be intersected
     * \param ray
     *    The ray segment to be used for the intersection query
     * \param t
     *    Upon success, \a t contains the distance from the ray origin to the
     *    intersection point,
     * \param u
     *   Upon success, \c u will contain the 'U' component of the intersection
     *   in barycentric coordinates
     * \param v
     *   Upon success, \c v will contain the 'V' component of the intersection
     *   in barycentric coordinates
     * \return
     *   \c true if an intersection has been detected
     */
    bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const;

    /// Return a reference to the vertex positions
    const MatrixXf &getVertexPositions() const { return m_V; }

    /// Return a reference to the vertex normals (there might be none)
    const MatrixXf &getVertexNormals() const { return m_N; }

    /// Return a reference to the texture coordinates (there might be none)
    const MatrixXf &getVertexTexCoords() const { return m_UV; }

    /// Return a reference to the triangle vertex index list
    const MatrixXu &getIndices() const { return m_F; }

    /// Is this mesh an area emitter?
    bool isEmitter() const { return m_emitter != nullptr; }

    /// Return a pointer to an attached area emitter instance (or \c nullptr if there is none)
    Emitter *getEmitter() { return m_emitter; }

    /// Return a pointer to an attached area emitter instance (const version)
    const Emitter *getEmitter() const { return m_emitter; }

    /// Return a pointer to the BSDF associated with this mesh
    const BSDF *getBSDF() const { return m_bsdf; }

    /// Register a child object (e.g. a BSDF) with the mesh
    virtual void addChild(NoriObject *child);

    /// Return the name of this mesh
    const std::string &getName() const { return m_name; }

    /// Return a human-readable summary of this instance
    std::string toString() const;

    /**
     * \brief Return the type of object (i.e. Mesh/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EMesh; }

protected:
    /// Create an empty mesh
    Mesh();

    float m_cachedArea{ 0.f };

    std::random_device dev;
    mutable std::mt19937 m_rng{ dev() };
    mutable std::uniform_real_distribution<float> m_uniformDistribution;

    // return a uniformly distributed value in the interval [0,1)
    [[nodiscard]] float uniformRandomValue() const;

    // map the unit triangle (0,0)--(1,0)--(1,1) to barycentric coordinates
    [[nodiscard]] Point3f barycentric(const Point2f& p) const;

    [[nodiscard]] Point3f apply_barycentric(const Point3f& barycentricPos, const TripleCoord& coords) const;

    [[nodiscard]] TriangleIndices triangleIndices(uint32_t triangleIndex) const;
    [[nodiscard]] TripleCoord triangle(uint32_t triangleIndex) const;
    [[nodiscard]] TripleCoord normals(uint32_t triangleIndex) const;


protected:
    std::string   m_name;       ///< Identifying name
    MatrixXf      m_V;          ///< Vertex positions
    MatrixXf      m_N;          ///< Vertex normals
    MatrixXf      m_UV;         ///< Vertex texture coordinates
    MatrixXu      m_F;          ///< Faces
    BSDF*         m_bsdf    {}; ///< BSDF of the surface
    Emitter*      m_emitter {}; ///< Associated emitter, if any
    BoundingBox3f m_bbox;       ///< Bounding box of the mesh
    DiscretePDF   m_distr;      ///< Discrete distribution for choosing triangles
};

NORI_NAMESPACE_END
