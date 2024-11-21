//
// Created by Benedict on 11/19/2024.
//

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
class BSDFIntegrator : public Integrator {
public:
    BSDFIntegrator(const PropertyList &propList){
        m_maxBounces = propList.getInteger("maxBounces", 10);
        m_rrMinBounces = propList.getInteger("rrMinBounces", 3);
    }

    struct RayBounce
    {
        Intersection intersection;
        Vector3f dir;
    };

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        auto throughput = Color3f(1.0f);
        auto radiance = Color3f(0);
        Ray3f currentRay = cameraRay;

        for (int bounce = 0; bounce < m_maxBounces; ++bounce) {
            Intersection its;
            if (scene->rayIntersect(currentRay, its)) {
                const Emitter *emitter = its.mesh->getEmitter();
                Vector3f wi = its.toLocal(-currentRay.d).normalized();

                BSDFQueryRecord bsdfRec(wi, its.uv);

                auto bsdfColor = its.mesh->getBSDF()->sample(bsdfRec, sampler->next2D());

                float probabilityToDie = std::max(0.01f, Vector3f(bsdfColor.x(), bsdfColor.y(), bsdfColor.z()).norm());
                if (bounce < m_rrMinBounces)
                    probabilityToDie = 1.f;

                if (emitter) {
                    radiance += throughput * emitter->eval(wi);
                }

                if (bounce >= m_rrMinBounces)
                    if (sampler->next1D() > probabilityToDie)
                        break;

                throughput *= bsdfColor / probabilityToDie;
                currentRay = Ray3f(its.p, its.toWorld(bsdfRec.wo));
            } else {
                break;
            }
        }

        return radiance;
    }

    std::string toString() const override {
        std::ostringstream oss;
            oss << "SimplePathIntegrator[" << endl
                << " maxBounces = " << m_maxBounces << endl
                << "]";
        return oss.str();
    }
private:
    int m_maxBounces;
    int m_rrMinBounces;
    const float m_probabilityToDie = 0.1f;

};

NORI_REGISTER_CLASS(BSDFIntegrator, "bsdfpath");
NORI_NAMESPACE_END
