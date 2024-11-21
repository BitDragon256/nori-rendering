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

    Color3f eval(const BSDFQueryRecord &bRec) const override {
        const auto wr = reflect(bRec.wo);

        // with the dirac-delta function only either the one or the other side of the equation can be returned

        float etaO, etaT;
        if (bRec.wo == reflect(bRec.wi))
        {
            // reflect
            etaO = m_etaExt;
            etaT = m_etaExt;

            return {
                fresnelDielectric(Frame::cosTheta(bRec.wi), m_etaExt, m_etaInt) / std::abs(Frame::cosTheta(bRec.wi))
            };
        }
        else if (bRec.wo == refract(bRec.wi, m_etaExt, m_etaInt))
        {
            // refract
            if (bRec.wi.z() >= 0.f) {
                etaO = m_etaExt;
                etaT = m_etaInt;
            } else {
                etaO = m_etaInt;
                etaT = m_etaExt;
            }

            return {
                std::pow(etaO / etaT, 2.f) * (1.f - fresnelDielectric(Frame::cosTheta(bRec.wi), etaO, etaT)) / std::abs(Frame::cosTheta(bRec.wi)) * M_PI
            };
        }
        return { 0.f };
    }

    float pdf(const BSDFQueryRecord &bRec) const override {
        if (bRec.measure != EDiscrete || (bRec.wi != reflect(bRec.wo) && bRec.wi != refract(bRec.wo, m_etaExt, m_etaInt)))
            return 0.f;

        return 1.f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {

        /*
        bRec.measure = EDiscrete;

        float cosThetaI = Frame::cosTheta(bRec.wi);
        const bool entering = cosThetaI > 0;

        // Determine IORs depending on whether the ray is entering or exiting
        const auto etaI = entering ? m_etaExt : m_etaInt;
        const auto etaT = entering ? m_etaInt : m_etaExt;


        const auto reflectionDirection = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z()); ;

        const auto cosTheta = Frame::cosTheta(bRec.wi); // cos(theta_i)
        const auto reflectionProbability = fresnelDielectric(cosTheta, m_etaExt, m_etaInt);
        if(sample.x() < reflectionProbability)
        {
            // reflection case
            bRec.wo = reflectionDirection;
            return {0.0f};
        }


        // refraction case
        const auto eta = etaI / etaT;
        const auto sinThetaI2 = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
        const auto sinThetaT2 = eta * eta * sinThetaI2;

        // Check for total internal reflection
        if (sinThetaT2 >= 1.0f)
            return { 0.0f };

        auto cosThetaT = std::sqrt(1.0f - sinThetaT2);
        if (!entering)
            cosThetaT = -cosThetaT;

        // Calculate refracted direction
        bRec.wo = eta * -bRec.wi + (eta * cosThetaI - cosThetaT) * Vector3f(0, 0, 1.f);

        // Scale by the transmission factor (1 - Fresnel)
        const auto cosThetaO = Frame::cosTheta(bRec.wo);
        const auto transmission = (1 - reflectionProbability) * (etaT * etaT) / (etaI * etaI);

        return { transmission / std::abs(cosThetaO) };
        */

        ///*

        bRec.measure = EDiscrete;

        auto n1 = m_etaExt;
        auto n2 = m_etaInt;
        if (bRec.wi.z() < 0)
            std::swap(n1, n2);

        const auto wr = bRec.wi;
        const float f = fresnelDielectric(Frame::cosTheta(wr), n1, n2);

        if (sample.x() < f)
        {
            // reflect
            bRec.wo = reflect(bRec.wi);
        }
        else
        {
            // refract
            bRec.wo = refract(bRec.wi, n1, n2);
            // bRec.wo = -bRec.wi;
        }

        return eval(bRec);

        //*/
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

    static Vector3f refract(Vector3f v, float n1, float n2)
    {
        v = -v;
        const auto proj = Vector3f(v.x(), v.y(), 0.f);

        std::swap(n1, n2);

        // return (v - proj * n1 / n2).normalized();

        return n1 / n2 * proj + Vector3f(0.f, 0.f, std::sqrt(1.f - (n1 / n2 * proj).squaredNorm()));
    }
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
