/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <array>
#include <numeric>

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() = default;

Mesh::~Mesh() = default;

void Mesh::activate() {

    /* Create a discrete distribution for sampling triangles
     * with respect to their surface area (use the existing \c m_distr)
     */

    const auto triCount = getTriangleCount();
    m_distr.reserve(triCount);
    for (uint32_t i = 0; i < triCount; ++i)
    {
        const auto area = surfaceArea(i);
        m_distr.append(area);
        m_cachedArea += area;
    }

    m_distr.normalize();

    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }

    if(m_emitter) {
        m_emitter->setParent(this);
    }
}

/// uniformly sample a position on the mesh
void Mesh::samplePosition(const Point2f &sample, Point3f &p, Normal3f &n) const {
    /* Uniformly sample a position on the mesh
     * Use \ref Warp::squareToUniformTriangle to sample a point on the triangle
     * Then, convert to barycentric coordinates and choose a point
     * on the actual triangle using the barycentric coordinates.
     */

    const auto uniTriPos = Warp::squareToUniformTriangle(sample);
    const auto barycentricPos = barycentric(uniTriPos);

    const auto triIndex = m_distr.sample(uniformRandomValue());

    p = apply_barycentric(barycentricPos, triangle(static_cast<uint32_t>(triIndex)));
    n = apply_barycentric(barycentricPos, normals(static_cast<uint32_t>(triIndex)));
}

/// returns the total surface area of the mesh
float Mesh::totalSurfaceArea() const {
    return m_cachedArea;
}

/// returns the surface area of an individual triangle
float Mesh::surfaceArea(uint32_t index) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

BoundingBox3f Mesh::getBoundingBox(uint32_t index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(uint32_t index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}

void Mesh::addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Mesh: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter: {
                Emitter *emitter = static_cast<Emitter *>(obj);
                if (m_emitter)
                    throw NoriException(
                        "Mesh: tried to register multiple Emitter instances!");
                m_emitter = emitter;
            }
            break;

        default:
            throw NoriException("Mesh::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

float Mesh::uniformRandomValue() const
{
    return m_uniformDistribution(m_rng);
}

Point3f Mesh::barycentric(const Point2f &p) const
{
    const auto sqrtU = std::sqrt(p.x());

    return { (1.f - sqrtU), (1 - p.y()) * sqrtU, p.y() * sqrtU };
}

Point3f Mesh::apply_barycentric(const Point3f& barycentricPos, const TripleCoord &coords) const
{
    return barycentricPos.x() * coords[0] + barycentricPos.y() * coords[1] + barycentricPos.z() * coords[2];
}

TriangleIndices Mesh::triangleIndices(const uint32_t triangleIndex) const
{
    return {
        m_F(0, triangleIndex),
        m_F(1, triangleIndex),
        m_F(2, triangleIndex)
    };
}

TripleCoord Mesh::triangle(const uint32_t triangleIndex) const
{
    const auto indices = triangleIndices(triangleIndex);
    return {
        m_V.col(indices[0]),
        m_V.col(indices[1]),
        m_V.col(indices[2])
    };
}

TripleCoord Mesh::normals(const uint32_t triangleIndex) const
{
    if (!m_N.count())
    {
        const auto tri = triangle(triangleIndex);
        const auto normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalized();
        return { normal, normal, normal };
    }
    const auto indices = triangleIndices(triangleIndex);
    return {
        m_N.col(indices[0]),
        m_N.col(indices[1]),
        m_N.col(indices[2])
    };
}

std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null")
    );
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}


NORI_NAMESPACE_END
