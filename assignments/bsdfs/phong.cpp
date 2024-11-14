/*
    This file is part of Nori, a simple educational ray tracer
    Phong BSDF

    Copyright (c) 2021 by Lukas Ruppert
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

Vector3f reflect(const Vector3f& v) {
    return {-v.x(), -v.y(), v.z()};
}

/// Phong BRDF
class Phong : public BSDF {
public:
    Phong(const PropertyList &p) {
        m_specular = p.getColor("specular", Color3f(0.5f));
        m_diffuse  = p.getColor("diffuse", Color3f(0.35f));
        m_exponent = p.getFloat("exponent", 30.0f);

        m_spec_sampling_rate = m_specular.mean()/(m_specular.mean()+m_diffuse.mean());
    }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord & bRec) const override {
        if (Frame::cosTheta(bRec.wi) <= 0.0f || Frame::cosTheta(bRec.wo) <= 0.0f)
            return {0.0f};
        const float alpha = bRec.wo.dot(reflect(bRec.wi));
        const float spec_pdf = alpha > 0.0f ? std::pow(alpha, m_exponent)*(m_exponent+1.0f)*INV_TWOPI : 0.0f;
        return m_diffuse*INV_PI + m_specular*spec_pdf;
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord & bRec) const override {
        if (bRec.measure != ESolidAngle || Frame::cosTheta(bRec.wi) <= 0.0f || Frame::cosTheta(bRec.wo) <= 0.0f)
            return 0.0f;

        const float alpha = bRec.wo.dot(reflect(bRec.wi));
        const float spec_pdf = alpha > 0.0f ? std::pow(alpha, m_exponent)*(m_exponent+1.0f)*INV_TWOPI : 0.0f;
        const float diff_pdf = Warp::squareToCosineHemispherePdf(bRec.wo);

        return diff_pdf+(spec_pdf-diff_pdf)*m_spec_sampling_rate;
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        if (Frame::cosTheta(bRec.wi) <= 0.0f)
            return 0.0f;

        const bool sample_spec = sample.y() < m_spec_sampling_rate;
        if (sample_spec) {
            const Point2f spec_sample {sample.x(), sample.y()/m_spec_sampling_rate};
            const Point2f disk_sample = Warp::squareToUniformDisk(spec_sample);
            // in the uniform disk, the radius is randomly distributed according to sqrt(uniform_0_1)
            // we can reuse it as a uniform random number if we square it again
            const float radius_sample = disk_sample.squaredNorm();
            if (radius_sample > 0.0f) {
                float cosTheta = std::pow(radius_sample, 1.0f/(m_exponent+1.0f));
                // sin(theta)/radius of the disk_sample
                float sinThetaDivRadius = std::sqrt((1.0f-cosTheta*cosTheta)/radius_sample);
                bRec.wo = Vector3f{sinThetaDivRadius*disk_sample.x(), sinThetaDivRadius*disk_sample.y(), cosTheta};

                // center Phong lobe around reflection direction
                const Frame reflection_frame {reflect(bRec.wi)};
                bRec.wo = reflection_frame.toWorld(bRec.wo);
            }
            else
                bRec.wo = reflect(bRec.wi);
        }
        else {
            const Point2f diff_sample {sample.x(), (sample.y()-m_spec_sampling_rate)/(1.0f-m_spec_sampling_rate)};
            bRec.wo = Warp::squareToCosineHemisphere(diff_sample);
        }

        bRec.eta = 1.0f;
        bRec.measure = ESolidAngle;

        if (Frame::cosTheta(bRec.wo) <= 0.0f)
            return 0.0f;

        return eval(bRec)*Frame::cosTheta(bRec.wo)/pdf(bRec);
    }

    std::string toString() const override {
        return tfm::format(
                    "Phong[\n"
                    "  m_specular = %s,\n"
                    "  m_diffuse = %s,\n"
                    "  m_exponent = %s\n"
                    "]",
                    m_specular.toString(), m_diffuse.toString(), m_exponent);
    }

private:
    Color3f m_specular;
    Color3f m_diffuse;
    float   m_exponent;

    float m_spec_sampling_rate;
};

NORI_REGISTER_CLASS(Phong, "phong");
NORI_NAMESPACE_END
