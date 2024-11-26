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
        for (size_t i = 0; i < 1; ++i)
        {
            auto misColor = Color3f(0.f);

            // shoot ray against emitter
            if (false)
            {
                EmitterQueryRecord emitterSampleInfo{ hitInfo.p };
                auto emitterColor = scene->sampleEmitterDirect(emitterSampleInfo, sampler->next2D());
                auto emitterPdf = 0.f;

                if (!scene->rayIntersect({ emitterSampleInfo.p, -emitterSampleInfo.ws_wi, Epsilon, emitterSampleInfo.distance * (1.f - Epsilon) }))
                {
                    emitterPdf = scene->pdfEmitterDirect(emitterSampleInfo);
                    BSDFQueryRecord bsdfQueryRecord(
                        emitterSampleInfo.wi,
                        -hitInfo.toLocal(cameraRay.d),
                        ESolidAngle
                    ); // UV is not needed here

                    auto bsdfColor = bsdf->eval(bsdfQueryRecord) * std::max(std::abs(Frame::cosTheta(bsdfQueryRecord.wo)), 0.0001f);
                    auto bsdfPdf= bsdf->pdf(bsdfQueryRecord);

                    // std::cout << Frame::cosTheta(bsdfQueryRecord.wo) << " " << emitterPdf << " " << bsdfPdf << std::endl;

                    if (emitterPdf != 0.f && Frame::cosTheta(emitterSampleInfo.wi) > 0)
                        misColor += emitterColor * bsdfColor * weighting_heuristic(emitterPdf, { emitterPdf, bsdfPdf }) / emitterPdf;
                    if (bsdfPdf != 0.f)
                        misColor += emitterColor * bsdfColor * weighting_heuristic(bsdfPdf, { emitterPdf, bsdfPdf }) / bsdfPdf;
                }
            }

            // shoot ray with BSDF
            if (true)
            {
                BSDFQueryRecord bsdfQueryRecord(wi, hitInfo.uv);
                auto bsdfColor = bsdf->sample(bsdfQueryRecord, sampler->next2D());// * Frame::cosTheta(wi);

                if (Intersection emitterHitInfo; scene->rayIntersect({ hitInfo.p, hitInfo.toWorld(bsdfQueryRecord.wo) }, emitterHitInfo) && emitterHitInfo.mesh->isEmitter())
                {
                    auto bsdfPdf = bsdf->pdf(bsdfQueryRecord);

                    EmitterQueryRecord emitterQueryRecord(
                        hitInfo.p,
                        emitterHitInfo.p,
                        emitterHitInfo.t,
                        hitInfo.toWorld(wi),
                        bsdfQueryRecord.wo,
                        bsdfQueryRecord.measure,
                        emitterHitInfo.mesh->getEmitter()->idx
                    );
                    auto emitterColor = emitterHitInfo.mesh->getEmitter()->eval(wi);
                    auto emitterPdf = scene->pdfEmitterDirect(emitterQueryRecord);

                    // std::cout << emitterPdf << std::endl;

                    if (emitterPdf != 0.f)
                        misColor += bsdfColor * emitterColor * weighting_heuristic(emitterPdf, { emitterPdf, bsdfPdf }) / emitterPdf;
                    if (bsdfPdf != 0.f)
                        misColor += bsdfColor * emitterColor * weighting_heuristic(bsdfPdf, { emitterPdf, bsdfPdf }) / bsdfPdf;
                }
            }

            accColor += misColor;// * 0.5f;
        }

        return accColor + startColor;
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
        return balance_wh(pdf, pdfs);
    }

    static float uniform_wh(float pdf, pdfvec pdfs)
    {
        return static_cast<float>(pdf > 0) / std::transform_reduce(pdfs.cbegin(), pdfs.cend(), 0.f, std::plus<>(), [](float v){ return static_cast<float>(v > 0); });
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
