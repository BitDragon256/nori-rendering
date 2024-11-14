#include <nori/emitter.h>
#include <nori/bitmap.h>
#include <nori/frame.h>
#include <nori/mesh.h>
#include <nori/warp.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class AreaLight: public Emitter {
public:
    AreaLight(const PropertyList &propList) {
        m_radiance = propList.getColor("radiance");
    }

    void setParent(NoriObject* obj) override {
        // This will be called by \ref Mesh::activate().
        m_parent = dynamic_cast<Mesh*>(obj);
    }

    Color3f sampleDirect(EmitterQueryRecord &eRec, Point2f sample) const override {
        // Sample a direction depending on the query point \c eRec.p.

        Normal3f normal;
        m_parent->samplePosition(sample, eRec.ep, normal);
        eRec.measure = ESolidAngle;

        eRec.ws_wi = (eRec.p - eRec.ep);
        eRec.distance = eRec.ws_wi.norm();
        eRec.ws_wi.normalize();
        eRec.wi = Frame(normal).toLocal(eRec.ws_wi);

        return eval(eRec.wi) / pdfDirect(eRec);
    }

    float pdfDirect(const EmitterQueryRecord &eRec) const override {
        // Return the PDF of the sample given in \c eRec.
        /* Return the PDF in solid angle or the discrete probability according to the \c eRec.measure. */
        if (eRec.measure == ESolidAngle)
            return 1.f / m_parent->totalSurfaceArea() * std::powf(eRec.distance, 2.f) / Frame::cosTheta(eRec.wi);
        else
            return 1.f / m_parent->totalSurfaceArea();
    }

    Color3f eval(Vector3f wi) const override {
        // Return the emitted radiance of the given sample.
        if (Frame::cosTheta(wi) > 0.f)
            return m_radiance;
        return { 0.f };
    }

    Color3f samplePhoton(Ray3f &ray, Sampler* sampler) const override {
        // TODO: Exercise 7.1 a): sample a photon ray
        throw NoriException("AreaLight::samplePhoton() is not yet implemented!");
    }

    std::string toString() const override {

        std::ostringstream oss;
        oss << "AreaLight[" << endl
                /* Create output! */
                << "]";
        return oss.str();

    }

private:

    Color3f m_radiance;
    Mesh* m_parent;

};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END
