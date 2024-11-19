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
 * \brief Dielectric / Lambertian BRDF model
 */
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        m_etaExt = propList.getFloat("extIOR", 1.0f);
        m_etaInt = propList.getFloat("intIOR", 1.5f);
    }

    Color3f eval(const BSDFQueryRecord &bRec) const override
    {
        const auto wr = reflect(bRec.wo);

        // with the dirac-delta function only either the one or the other side of the equation can be returned

        float etaO, etaT;
        if (bRec.wo == reflect(bRec.wi))
        {
            // reflect
            etaO = m_etaExt;
            etaT = m_etaExt;

            return {
                fresnelDielectric(Frame::cosTheta(bRec.wi), m_etaExt, m_etaInt) / Frame::cosTheta(bRec.wi)
            };
        }
        else if (bRec.wo == refract(bRec.wi, m_etaExt / m_etaInt))
        {
            // refract
            etaO = m_etaExt;
            etaT = m_etaInt;

            return {
                std::pow(etaO / etaT, 2.f) * (1.f - fresnelDielectric(Frame::cosTheta(bRec.wi), m_etaExt, m_etaInt)) / Frame::cosTheta(bRec.wi)
            };
        }
        return { 0.f };
    }

    float pdf(const BSDFQueryRecord &bRec) const override {
        if (bRec.measure != EDiscrete || bRec.wi != reflect(bRec.wo))
            return 0.0f;

        return 1.f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        bRec.measure = EDiscrete;

        const auto wr = bRec.wi;
        const float f = fresnelDielectric(Frame::cosTheta(wr), m_etaExt, m_etaInt);

        if (sample.x() >= f)
        {
            // reflect
            bRec.wo = reflect(bRec.wi);
        }
        else
        {
            // refract
            bRec.wo = refract(bRec.wi, m_etaExt / m_etaInt);
        }

        return eval(bRec);
    }

    void addChild(NoriObject *child)  override {
        m_texture = dynamic_cast<Texture2D*>(child);
    }


    /// Return a human-readable summary
    std::string toString() const override {
        return "42";
    }

    EClassType getClassType() const  override { return EBSDF; }
private:
    float m_etaExt, m_etaInt;
    Texture2D* m_texture = nullptr;

    static float col_norm_sq(Color3f c)
    {
        return c.x() * c.x() + c.y() * c.y() + c.z() * c.z();
    }

    static float fresnel(Vector3f wi, float eta, float k)
    {
        const auto etaSq = col_norm_sq(eta);
        const auto kSq = col_norm_sq(k);

        const auto sinTh = Frame::sinTheta(wi);
        const auto sinThSq = Frame::sinTheta2(wi);

        const auto cosTh = Frame::cosTheta(wi);
        const auto cosThSq = cosTh * cosTh;

        const auto tanTh = Frame::tanTheta(wi);
        const auto tanThSq = tanTh * tanTh;

        const auto inner_term = etaSq - kSq - sinThSq;
        const auto sqrt_term = std::sqrt(inner_term * inner_term + 4.f * etaSq * kSq);

        const auto aSq = 0.5f * ( sqrt_term + inner_term );
        const auto bSq = 0.5f * ( sqrt_term - inner_term );

        const auto a = std::sqrt(aSq);

        const auto Rs = (aSq + bSq - 2.f * a * cosTh + cosThSq) / (aSq + bSq + 2.f * a * cosTh + cosThSq);
        const auto Rp = Rs * (aSq + bSq - 2.f * a * sinTh * tanTh + sinThSq * tanThSq) / (aSq + bSq + 2.f * a * sinTh * tanTh + sinThSq * tanThSq);

        return (Rs + Rp) * 0.5f;
    }

    static Vector3f reflect(Vector3f v)
    {
        return { -v.x(), -v.y(), v.z() };
    }

    static Vector3f refract(Vector3f v, float refr)
    {
        return refr * v + Vector3f{ 0.f, 0.f, refr * Frame::cosTheta(v) - std::sqrt(1.f - Frame::sinTheta2(v)) };
    }
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
