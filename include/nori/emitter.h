/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct EmitterQueryRecord {

    /**
     * position in the scene which will be illuminated by the emitter (in world space)
     * (usually the current intersection's position in the integrator)
     */
    Point3f p {0.0f};

    /// sampled position on the emitter (in world space)
    Point3f ep {0.0f};

    /// distance between p and ep (in world space)
    float distance {0.0f};

    /**
     * the sampled direction in which the emitter emits light (in the world frame)
     * (normalized direction from ep towards p in world space)
     */
    Vector3f ws_wi {0.0f};

    /**
     * If filled correctly, the following equality should hold: (up to small numeric errors)
     * p == ep+ws_wi*distance
     */

    /**
     * the sampled direction in which the emitter emits light (in the local frame)
     * (direction from ep to p in the local space at ep)
     */
    Vector3f wi {0.0f};

    /// Measure associated with the sample.
    EMeasure measure {EUnknownMeasure};

    /// index of the sampled emitter in the scene's emitter list
    uint32_t eidx {-1U};

    /**
     * \brief Create a new record for \b sampling a distant Emitter or for \b evaluating the PDF of such a sample.
     *
     * For sampling, only a point is necessary and the remaining values should be filled in by the emitter.
     * For evaluating the PDF, all values need to be given.
     */
    EmitterQueryRecord(Point3f p, Point3f ep={}, float distance={}, Vector3f ws_wi={}, Vector3f wi={}, EMeasure measure=EUnknownMeasure, uint32_t eidx=-1U)
        : p(p), ep(ep), distance(distance), ws_wi(ws_wi), wi(wi), measure(measure), eidx(eidx) {}

};


/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    virtual ~Emitter() override = default;

    /**
     * \brief Evaluate the emitted radiance for a given intersection
     * The vector \c wi gives the direction towards the camera in the Emitter's local frame
     * \return Radiance emitted towards the query point.
     */
    virtual Color3f eval(Vector3f wi) const = 0;

    /**
     * \brief Sample the emitter for direct illumination and fill in the values of the EmitterQueryRecord eRec.
     * \return Radiance emitted towards the query point divided by the pdf.
     * I.e., \c eval(eRec) divided by \c pdfDirect(eRec).
     * Often, this can be computed more efficiently than calling these functions explicitly.
     *
     * \details Given a query point p, use \c Emitter::sampleDirect to generate a sample and fill the following values:
     *  ep, ws_wi, wi, distance, measure
     */
    virtual Color3f sampleDirect(EmitterQueryRecord &eRec, Point2f sample) const = 0;

    /**
     * \brief Compute the probability density of sampling the direction \c -eRec.ws_wi when at point \c p,
     * in relation to all possible directions that could be sampled when sampling this emitter.
     *
     * \return The probability density in solid angle if \c eRec.measure is set to \c ESolidAngle (continuous set of directions).
     * \return The discrete probability of the sample if \c eRec.measure is set to \c EDiscrete (countable set of directions).
     */
    virtual float pdfDirect(const EmitterQueryRecord &eRec) const = 0;

    /**
     * \brief Sample a photon ray for photon mapping
     * \return Power of the emitter in Watts
     */
    virtual Color3f samplePhoton(Ray3f &ray, Sampler *sampler) const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.)
     * provided by this instance
     */
    EClassType getClassType() const override { return EEmitter; }

    /// index of the emitter in the scene's emitter list
    uint32_t idx {-1U};
};

NORI_NAMESPACE_END
