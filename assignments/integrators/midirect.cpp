/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2012 by Wenzel Jakob and Steve Marschner.
*/

#include <execution>
#include <numeric>
#include <ranges>
#include <algorithm>

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief MISDIntegrator
 */
class MISDIntegrator : public Integrator {
public:
    MISDIntegrator(const PropertyList &propList){

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
        const auto wi = hitInfo.toLocal(-cameraRay.d).normalized();

        auto startColor = Color3f(0.f);
        if (hitInfo.mesh->isEmitter())
        {
            startColor = hitInfo.mesh->getEmitter()->eval(wi);
        }

        auto accColor = Color3f(0.f);
        constexpr auto sampleCount = 1;
        for (size_t i = 0; i < sampleCount; ++i)
        {
            auto misColor = Color3f(0.f);

            // shoot ray against emitter
            {
                EmitterQueryRecord emitterSampleInfo{ hitInfo.p };
                const auto emitterColor = scene->sampleEmitterDirect(emitterSampleInfo, sampler->next2D());
                if (emitterColor.isZero())
                    goto end_emitter_eval;

                const auto emitterPdf = scene->pdfEmitterDirect(emitterSampleInfo);

                const auto isShadowed = scene->rayIntersect({ hitInfo.p, -emitterSampleInfo.ws_wi, Epsilon, emitterSampleInfo.distance * (1.f - Epsilon) });

                if (isShadowed || emitterPdf <= 0.f)
                    goto end_emitter_eval;

                const auto wo = hitInfo.toLocal(-emitterSampleInfo.ws_wi);
                BSDFQueryRecord bsdfQueryRecord(
                    wi,
                    wo,
                    ESolidAngle
                ); // UV is not needed here

                auto bsdfPdf= bsdf->pdf(bsdfQueryRecord);
                auto bsdfColor = bsdf->eval(bsdfQueryRecord);

                float misWeight;
                if (emitterSampleInfo.measure == EDiscrete)
                    misWeight = 1.f;
                else
                    misWeight = weighting_heuristic(emitterPdf, { bsdfPdf, emitterPdf });
                misColor += misWeight * emitterColor * bsdfColor * std::abs(Frame::cosTheta(wo));
            }
            end_emitter_eval:

            // shoot ray with BSDF
            {
                BSDFQueryRecord bsdfQueryRecord(wi, hitInfo.uv);
                const auto bsdfColor = bsdf->sample(bsdfQueryRecord, sampler->next2D());
                if (bsdfColor.isZero())
                    goto end_bsdf_eval;

                const auto bsdfPdf = bsdf->pdf(bsdfQueryRecord);

                Intersection emitterHitInfo;
                const auto intersected = scene->rayIntersect({ hitInfo.p, hitInfo.toWorld(bsdfQueryRecord.wo) }, emitterHitInfo);

                if (bsdfPdf <= 0.f || !intersected || !emitterHitInfo.mesh-> isEmitter())
                    goto end_bsdf_eval;

                EmitterQueryRecord emitterQueryRecord(
                    hitInfo.p,
                    emitterHitInfo.p,
                    emitterHitInfo.t,
                    hitInfo.toWorld(wi),
                    wi,
                    bsdfQueryRecord.measure,
                    emitterHitInfo.mesh->getEmitter()->idx
                );
                auto emitterColor = emitterHitInfo.mesh->getEmitter()->eval(wi);
                auto emitterPdf = scene->pdfEmitterDirect(emitterQueryRecord);

                auto cosTheta = std::abs(Frame::cosTheta(wi));

                auto misWeight = weighting_heuristic(bsdfPdf, { bsdfPdf, emitterPdf });
                misColor += misWeight * emitterColor * bsdfColor;
            }
            end_bsdf_eval:

            accColor += misColor;
        }

        return accColor / sampleCount + startColor;
    }

    std::string toString() const override {
        std::ostringstream oss;
        oss << "MISDIntegrator[" << endl
            << "]";
        return oss.str();
    }

private:

    using pdfvec = const std::vector<float>&;

static float weighting_heuristic(float pdf, pdfvec pdfs)
{
    return power_wh(pdf, pdfs);
}

static float uniform_wh(float pdf, pdfvec pdfs)
{
    const auto sum = std::transform_reduce(pdfs.cbegin(), pdfs.cend(), 0.f, std::plus<>(), [](float v){ return v > 0 ? 1.f : 0.f; });
    if (!isnormal(sum))
        return 0.f;
    return (pdf > 0 ? 1.f : 0.f) / sum;
}
static float balance_wh(float pdf, pdfvec pdfs)
{
    return pdf / std::reduce(pdfs.cbegin(), pdfs.cend(), 0.f, std::plus<>());
}
static float power_wh(float pdf, pdfvec pdfs)
{
    return pdf*pdf / std::transform_reduce(pdfs.cbegin(), pdfs.cend(), 0.f, std::plus<>(), [](float v){ return v*v; });
}

};

NORI_REGISTER_CLASS(MISDIntegrator, "midirect");
NORI_NAMESPACE_END
