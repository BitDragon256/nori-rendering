/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2012 by Wenzel Jakob and Steve Marschner.
*/

#include <execution>
#include <numeric>

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

Color3f operator*(Color3f a, Color3f b)
{
    return { a.x() * b.x(), a.y() * b.y(), a.z() * b.z() };
}
Color3f operator*(Color3f a, float v)
{
    return { a.x() * v, a.y() * v, a.z() * v };
}

/**
 * \brief DirectLightIntegrator
 */
class DirectLightIntegrator : public Integrator {
public:
    DirectLightIntegrator(const PropertyList &propList){

    }

    /* Li is called N times, where N stands for the amount of samples
     * which are set in the XML file (check the sampleCount parameter)
     *
     * (have a look at assignments/integrators/depth.cpp for an example)
     */
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        // evaluate ray
        Intersection hitInfo;
        if (!scene->rayIntersect(cameraRay, hitInfo))
            return {0.f};

        const auto bsdf = hitInfo.mesh->getBSDF();
        const auto localCRD = hitInfo.toLocal(-cameraRay.d);

        auto startColor = Color3f(0.f);
        if (hitInfo.mesh->isEmitter())
        {
            startColor = hitInfo.mesh->getEmitter()->eval(localCRD);
        }

        // calulate emitter influence
        const auto& emitters = scene->getEmitters();
        return std::transform_reduce(
            emitters.cbegin(), emitters.cend(),
            startColor,
            std::plus<>(),
            [scene, hitInfo, bsdf, sampler, localCRD](const std::unique_ptr<Emitter>& emitter) -> Color3f {
                if (hitInfo.mesh->getEmitter() == emitter.get())
                    return { 0.f };

                EmitterQueryRecord emitterSampleInfo{ hitInfo.p };
                const auto emitterSample = emitter->sampleDirect(emitterSampleInfo, sampler->next2D() );

                if (scene->rayIntersect({ emitterSampleInfo.p, -emitterSampleInfo.ws_wi, Epsilon, emitterSampleInfo.distance * (1.f - Epsilon) }))
                    return { 0.f };

                const BSDFQueryRecord bsdfQueryRecord{ emitterSampleInfo.wi, localCRD, EMeasure::ESolidAngle, hitInfo.uv };
                const auto bsdfEval = bsdf->eval(bsdfQueryRecord);
                return bsdfEval * emitterSample * std::abs(Frame::cosTheta(hitInfo.toLocal(emitterSampleInfo.ws_wi)));
            }
        );
    }

    std::string toString() const override {
        std::ostringstream oss;
        oss << "DirectLightIntegrator[" << endl
            << "]";
        return oss.str();
    }

};

NORI_REGISTER_CLASS(DirectLightIntegrator, "direct");
NORI_NAMESPACE_END
