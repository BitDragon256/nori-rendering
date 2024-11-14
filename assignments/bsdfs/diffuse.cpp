/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/texture.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class Diffuse : public BSDF {
public:
    Diffuse(const PropertyList &propList) {
        m_albedo = propList.getColor("albedo", Color3f(0.5f));
    }

    /**
     * \brief Evaluate the BSDF for a pair of directions and measure
     * specified in \code bRec
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     * \return
     *     The BSDF value, evaluated for each color channel
     */
    Color3f eval(const BSDFQueryRecord &bRec) const override {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
                || Frame::cosTheta(bRec.wi) <= 0
                || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        /* The BRDF is simply the albedo / pi */
        if (m_texture)
            return m_texture->eval(bRec.uv)*INV_PI;

        return m_albedo * INV_PI;
    }

    /**
     * \brief Compute the probability of sampling \c bRec.wo
     * (conditioned on \c bRec.wi).
     *
     * This method provides access to the probability density that
     * is realized by the \ref sample() method.
     *
     * \param bRec
     *     A record with detailed information on the BSDF query
     *
     * \return
     *     A probability density value expressed with respect
     *     to the specified measure
     */
    float pdf(const BSDFQueryRecord &bRec) const override {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
                || Frame::cosTheta(bRec.wi) <= 0
                || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;


        /* Importance sampling density wrt. solid angles:
           cos(theta) / pi.

           Note that the directions in 'bRec' are in local coordinates,
           so Frame::cosTheta() actually just returns the 'z' component.
         */
        return INV_PI * Frame::cosTheta(bRec.wo);
    }

    /**
     * \brief Sample the BSDF and return the importance weight (i.e. the
     * value of the BSDF * cos(theta_o) divided by the probability density
     * of the sample with respect to solid angles).
     *
     * \param bRec    A BSDF query record
     * \param sample  A uniformly distributed sample on \f$[0,1]^2\f$
     *
     * \return The BSDF value divided by the probability density of the sample,
     *         multiplied by the cosine factor associated with the outgoing direction.
     *         A zero value means that sampling failed.
     */
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        /* Each BSDF sample() function has to set the following parameters in the BSDFQueryRecord
         * bRec.mesure  - Can be ESolidAngle or EDiscrete
         * bRec.wo      - Outgoing direction in local coordinate space
         */

        bRec.measure = ESolidAngle;

        /* Warp a uniformly distributed sample on [0,1]^2
           to a direction on a cosine-weighted hemisphere */
        bRec.wo = Warp::squareToCosineHemisphere(sample);

        /* eval() / pdf() * cos(theta) = albedo. There
           is no need to call these functions. */
        if (m_texture)
            return m_texture->eval(bRec.uv);

        return m_albedo;
    }

    void addChild(NoriObject *child)  override {
        m_texture = dynamic_cast<Texture2D*>(child);
    }

    /* Only important for PhotonMapping */
    bool isDiffuse() const override {
        return true;
    }

    /// Return a human-readable summary
    std::string toString() const override {
        return tfm::format(
                    "Diffuse[\n"
                    "  albedo = %s\n"
                    "]", (m_texture ? m_texture->toString() : m_albedo.toString()));
    }

    EClassType getClassType() const  override { return EBSDF; }
private:
    Color3f m_albedo;
    Texture2D* m_texture = nullptr;
};

NORI_REGISTER_CLASS(Diffuse, "diffuse");
NORI_NAMESPACE_END
