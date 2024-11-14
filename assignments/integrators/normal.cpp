/*
 * normal.cpp
 *
 *  Created on: Oct 26, 2016
 *      Author: herholz
 */


#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief NormalIntegrator
 */
class NormalIntegrator : public Integrator {
public:
    NormalIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        Intersection info;
        if (!scene->rayIntersect(cameraRay, info))
            return {0.f};

        const auto normalizedNormal = info.toWorld(Vector3f(0.f, 0.f, 1.f)) / 2.f + Vector3f(.5f);
        return {normalizedNormal.x(), normalizedNormal.y(), normalizedNormal.z()};
    }

    std::string toString() const override {
        std::ostringstream oss;
        oss << "NormalIntegrator[" << endl
            << "]";
        return oss.str();
    }
};

NORI_REGISTER_CLASS(NormalIntegrator, "normal");
NORI_NAMESPACE_END
