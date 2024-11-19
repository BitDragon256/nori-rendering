/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>
#include <cmath>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformTriangle(const Point2f &sample) {
    // sample points uniformly on the unit triangle (0,0) -- (1,0) -- (1,1)
    if (sample.y() >= sample.x())
        return { -sample.x() + 1.f, -sample.y() + 1.f };
    return sample;
}

float Warp::squareToUniformTrianglePdf(const Point2f &p) {
    // return the PDF for points sampled on the unit triangle (or the PDF for those outside of it!)
    if (p.y() <= p.x())
        return 2.f;
    return 0.f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    // TODO: Exercise 5.1 c): Sample points uniformly on the unit disk with center (0,0) and radius 1

    const float theta = sample.x() * M_PI * 2.f;
    const float r = std::sqrtf(sample.y());

    return { r * std::cos(theta), r * std::sin(theta) };
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    // TODO: Exercise 5.1 c): Return the PDF for points sampled on the unit disk (or outside of it)

    if (p.norm() <= 1.f)
        return INV_PI;
    return 0.f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    const float cosTheta = 1.0f - 2.0f * sample.x();
    const float sinTheta = std::sqrt(std::max(0.0f, 1.0f-cosTheta*cosTheta));

    const float sinPhi = std::sin(2.0f * M_PI * sample.y());
    const float cosPhi = std::cos(2.0f * M_PI * sample.y());

    return Vector3f(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    (void) v; // unused
    return INV_FOURPI;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    const float cosTheta = sample.x();
    const float sinTheta = std::sqrt(std::max(0.0f, 1.0f-cosTheta*cosTheta));

    const float sinPhi = std::sin(2.0f * M_PI * sample.y());
    const float cosPhi = std::cos(2.0f * M_PI * sample.y());

    return Vector3f(cosPhi * sinTheta, sinPhi * sinTheta, cosTheta);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if (v.z() < 0.0f)
        return 0.0f;
    return INV_TWOPI;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    // TODO: Exercise 5.1 d): Sample directions on the cosine weighted hemisphere
    const auto disk = squareToUniformDisk(sample);
    return { disk.x(), disk.y(), std::sqrtf(1.f - disk.norm() * disk.norm()) };
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    // TODO: Exercise 5.1 d): Return the PDF for samples on the cosine weighted hemisphere (or outside of it)
    return std::max(0.f, v.z()) * INV_PI;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    // TODO: Exercise 10.1 a): Sample directions according to the Beckmann distribution
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    // TODO: Exercise 10.1 a) Return the PDF of directions sampled according to the Beckmann distribution (or invalid directions)
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
