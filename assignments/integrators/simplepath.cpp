/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2012 by Wenzel Jakob and Steve Marschner.
*/

#include <numeric>
#include <optional>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/scene.h>
#include <nori/warp.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN
    /**
 * \brief SimplePathIntegrator
 */
class SimplePathIntegrator : public Integrator {
public:
    SimplePathIntegrator(const PropertyList &propList){
        m_maxBounces = propList.getInteger("maxBounces", 10);
    }

    struct RayBounce
    {
        Intersection intersection;
        Vector3f dir;
    };

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        // TODO: Exercise 3.3: Implement the simple path tracer
        auto throughput = Color3f(1.0f);
        auto radiance = Color3f(0);
        Ray3f currentRay = cameraRay;

        for (int bounce = 0; bounce < m_maxBounces; ++bounce) {
            Intersection its;
            if (scene->rayIntersect(currentRay, its)) {
                const Emitter *emitter = its.mesh->getEmitter();
                Vector3f wi = its.toLocal(-currentRay.d).normalized();
                Vector3f wo = Warp::squareToUniformHemisphere(sampler->next2D());
                float pdf = Warp::squareToUniformHemispherePdf(wo);

                if (emitter) {
                    radiance += throughput * emitter->eval(wi);
                }

                BSDFQueryRecord bsdfRec(wi, wo, ESolidAngle, its.uv);
                Color3f bsdfColor = its.mesh->getBSDF()->eval(bsdfRec);
                float cosTheta = Frame::cosTheta(wo);
                throughput *= (bsdfColor * cosTheta) / pdf;
                currentRay = Ray3f(its.p, its.toWorld(wo));
            } else {
                break;
            }
        }

        return radiance;
    }

    /* "verry verry complex"
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        Color3f accumulatedRadiance{ 1.f };

        auto ray = cameraRay;

        std::vector<RayBounce> bounces(static_cast<size_t>(m_maxBounces+1));
        auto bounceIt = bounces.begin();

        while (bounceIt != bounces.end())
        {
            const auto oldD = ray.d;
            if (const auto potBounce = bounce_ray(ray, scene, sampler))
            {
                *bounceIt++ = potBounce.value();
            }
            else
                break;
        }

        Color3f light(1.f);
        bool firstHit = true;

        while (bounceIt != bounces.begin())
        {
            --bounceIt;

            const auto& [hitInfo, dir] = *bounceIt;
            const auto bsdf = hitInfo.mesh->getBSDF();
            const auto outgoingRayDir = hitInfo.toLocal(-dir);
            auto bsdfEval = Color3f(0.f);

            // return Color3f(hitInfo.shFrame.n.array()) * .5f + Color3f(.5f);

            // return static_cast<float>(bounceIt - bounces.begin()) / static_cast<float>(m_maxBounces);

            if (!firstHit)
            {
                const auto incomingRayDir = hitInfo.toLocal((bounceIt+1)->dir);
                const BSDFQueryRecord bsdfQueryRecord{ outgoingRayDir, incomingRayDir, EMeasure::ESolidAngle, hitInfo.uv };
                bsdfEval = bsdf->eval(bsdfQueryRecord) / Warp::squareToUniformHemispherePdf(incomingRayDir) * Frame::cosTheta(incomingRayDir);
                // bsdfEval += scene_radiance(scene, sampler, bsdf, hitInfo, incomingRayDir);
            }
            else
            {
                // bsdfEval = scene_radiance(scene, sampler, bsdf, hitInfo, outgoingRayDir);
                firstHit = false;
            }


            light = emittance(hitInfo.mesh, outgoingRayDir) + bsdfEval * light;
        }

        return light;
    }
    */

    std::string toString() const override {
        std::ostringstream oss;
            oss << "SimplePathIntegrator[" << endl
                << " maxBounces = " << m_maxBounces << endl
                << "]";
        return oss.str();
    }
private:
    int m_maxBounces;

    // returns the emittance of a mesh and 0 if it is no emitter
    Color3f emittance(const Mesh* mesh, const Vector3f& localDir) const
    {
        if (mesh->isEmitter())
            return mesh->getEmitter()->eval(localDir);
        return { 0.f };
    }

    Vector3f random_bounce_dir(Sampler* sampler) const
    {
        // return Vector3f{ 0.f, 0.f, 1.f }.normalized();
        // return Vector3f( sampler->next1D(), sampler->next1D(), 1.f + sampler->next1D() ).normalized();
        return Warp::squareToUniformHemisphere(sampler->next2D());
    }

    // shoots a ray into the scene, prepares the ray for the next bounce and returns the intersection
    std::optional<RayBounce> bounce_ray(Ray3f& ray, const Scene* scene, Sampler* sampler) const
    {
        Intersection hitInfo;

        if (!scene->rayIntersect(ray, hitInfo))
            return {};

        const auto oldD = ray.d;

        ray.d = random_bounce_dir(sampler);
        ray.d = hitInfo.toWorld(ray.d);
        ray.o = hitInfo.p + hitInfo.shFrame.n;

        return RayBounce { hitInfo, oldD };
    }

    // calculates the radiance of all emitters in the scene shining on the given point
    Color3f scene_radiance(const Scene* scene, Sampler* sampler, const BSDF* bsdf, const Intersection& hitInfo, const Vector3f& incomingRayDir) const
    {
        const auto& emitters = scene->getEmitters();
        return std::transform_reduce(
            emitters.cbegin(), emitters.cend(),
            Color3f(0.f),
            std::plus<>(),
            [scene, hitInfo, bsdf, sampler, incomingRayDir](const std::unique_ptr<Emitter>& emitter) -> Color3f {
                if (hitInfo.mesh->getEmitter() == emitter.get())
                    return { 0.f };

                EmitterQueryRecord emitterSampleInfo{ hitInfo.p };
                const auto emitterSample = emitter->sampleDirect(emitterSampleInfo, sampler->next2D() );

                if (scene->rayIntersect({ emitterSampleInfo.p, -emitterSampleInfo.ws_wi, Epsilon, emitterSampleInfo.distance * (1.f - Epsilon) }))
                    return { 0.f };

                const BSDFQueryRecord bsdfQueryRecord{ emitterSampleInfo.wi, incomingRayDir, EMeasure::ESolidAngle, hitInfo.uv };
                const auto bsdfEval = bsdf->eval(bsdfQueryRecord);
                return bsdfEval * emitterSample * std::abs(Frame::cosTheta(hitInfo.toLocal(emitterSampleInfo.ws_wi)));
            }
        );
    }

};

NORI_REGISTER_CLASS(SimplePathIntegrator, "simplepath");
NORI_NAMESPACE_END
