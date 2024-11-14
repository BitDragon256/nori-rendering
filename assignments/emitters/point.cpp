#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter {
public:
    PointLight(const PropertyList &propList) {
        m_power = propList.getColor("power");
        m_intensity = m_power/(4.0f*M_PI);
        m_toWorld = propList.getTransform("toWorld");
    }

    Color3f eval(Vector3f wi) const override {
        /* A point light source can not be evaluated,
         * since we can only evaluate emitters for a given intersection.
         * However, there is no way we will ever intersect a point light.
         */
        return Color3f(0.0f);
    }

    Color3f sampleDirect(EmitterQueryRecord &eRec, Point2f sample) const override {
        eRec.ep = m_toWorld * Point3f(0.0f);
        eRec.measure = EDiscrete;

        eRec.ws_wi = (eRec.p - eRec.ep);
        eRec.distance = eRec.ws_wi.norm();
        eRec.ws_wi.normalize();
        eRec.wi = Vector3f(0, 0, 1.f);

        // attenuation function as in Cem Yuksel. Point light attenuation without singularity. In Special Interest Group on Computer Graphics and Interactive Techniques Conference Talks, SIGGRAPH ’20, New York, NY, USA, 2020. Association for Computing Machinery.
        constexpr float radiusSquared = 2.f * 2.f;
        const float attenuation = 2.f / (radiusSquared) * (1.f - eRec.distance / std::sqrt(eRec.distance * eRec.distance + radiusSquared));

        return m_intensity * attenuation;
    }

    float pdfDirect(const EmitterQueryRecord &eRec) const override {
        /* Assuming a valid sample, the discrete probability is always one, since there is only one possible direction.
         * The solid angle covered by this single direction is zero.
         */
        return (eRec.measure == EDiscrete) ? 1.0f : 0.0f;
    }

    Color3f samplePhoton(Ray3f &ray, Sampler *sampler) const override {
        // implementing this for the point light is not part of the exercises
        throw NoriException("PointLight::samplePhoton() is not yet implemented!");
    }

    /* Return a human-readable summary */
    std::string toString() const override {

        std::ostringstream oss;
            oss << "PointLight[" << endl
                << " position = " << (m_toWorld*Point3f(0.0f, 0.0f, 0.0f)).toString() << endl
                << " power = " << m_power.toString() << endl
                << "]";
        return oss.str();

    }

private:
    Color3f m_intensity;
    Color3f m_power;
    Transform m_toWorld;
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END
