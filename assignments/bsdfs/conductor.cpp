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
 * \brief Conductor / Lambertian BRDF model
 */
class Conductor : public BSDF {
public:
    Conductor(const PropertyList &propList) {
        m_eta = propList.getColor("eta", Color3f(0.5f));
        m_k = propList.getColor("k", Color3f(1.0f));
    }

    Color3f eval(const BSDFQueryRecord &bRec) const override
    {
        const auto wr = reflect(bRec.wo);

        if (wr != bRec.wi)
            return {0.f};

        return Color3f{
            fresnel(wr, m_eta.x(), m_k.x()),
            fresnel(wr, m_eta.y(), m_k.y()),
            fresnel(wr, m_eta.z(), m_k.z())
        };
    }

    float pdf(const BSDFQueryRecord &bRec) const override {
        if (bRec.measure != EDiscrete || bRec.wi != reflect(bRec.wo))
            return 0.0f;

        return 1.f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        bRec.measure = EDiscrete;
        bRec.wo = reflect(bRec.wi);

        return eval(bRec);
    }

    void addChild(NoriObject *child)  override {
        m_texture = dynamic_cast<Texture2D*>(child);
    }


    /// Return a human-readable summary
    std::string toString() const override {
        return tfm::format(
                    "Conductor[\n"
                    "  eta = %s\n"
                    "]", (m_texture ? m_texture->toString() : m_eta.toString()));
    }

    EClassType getClassType() const  override { return EBSDF; }
private:
    Color3f m_eta;
    Color3f m_k;
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
};

NORI_REGISTER_CLASS(Conductor, "conductor");
NORI_NAMESPACE_END
