/*
    This file is part of Nori, a simple educational ray tracer
    Rendermanager for Metropolis Light Transport implementations

    Copyright (c) 2020 by Lukas Ruppert
*/

#include <nori/rendermanager.h>
#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <nori/mlt.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <thread>

NORI_NAMESPACE_BEGIN

class MLTRenderManager : public RenderManager {
public:
    MLTRenderManager() = default;
    MLTRenderManager(const PropertyList &propList)
        : m_pssmlt{
              propList.getInteger("chainlength", 10000),
              propList.getInteger("startup", 1000),
              propList.getFloat("newChain", 0.1f)} {
    }

    void start_render(Scene *scene, ImageBlock& result) override {
        /* Do the following in parallel and asynchronously */
        render_thread = std::thread([this, scene, &result] {
            const Camera *camera = scene->getCamera();
            const Vector2i& outputSize = camera->getOutputSize();
            Integrator *integrator = scene->getIntegrator();
            integrator->preprocess(scene);

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            // try to match the given number of samples per pixel, overshoot by completing the last iteration
            const int chainlength = m_pssmlt.getChainlength();
            const int numIterations = static_cast<int>((outputSize.prod()*static_cast<long>(scene->getSampler()->getSampleCount())+chainlength-1)/chainlength);
            const tbb::blocked_range<int> range(0, numIterations);

            auto map = [&](const tbb::blocked_range<int> &range)
            {
                /// Clone of the scene's sampler for the current thread
                thread_local std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());

                /// Histogram of accepted states
                thread_local HistogramImage histogram(outputSize, camera->getReconstructionFilter());

                // iterate over individual Markov Chains
                // update the result image at the end of each iteration
                for (int iteration=range.begin(); iteration<range.end(); ++iteration)
                {
                    // Initialize the sampler with a different state per thread and iteration
                    {
                        ImageBlock temp(Vector2i(0), nullptr);
                        temp.setOffset(Point2i(range.end(), iteration));
                        sampler->prepare(temp);
                        sampler->generate();
                    }

                    // hand control over to delegate function
                    m_pssmlt.processMarkovChain(scene, integrator, sampler.get(), histogram);

                    // inform the histogram about the number of samples
                    histogram.incrementSampleCount(chainlength);

                    // update the result image immediately if there are enough samples for the mean brightness
                    float meanBrightness {0.0f};
                    bool meanBrightnessValid {false};
                    std::tie(meanBrightness, meanBrightnessValid) = m_pssmlt.getMeanBrightness();
                    if (meanBrightnessValid)
                        histogram.updateResult(result, meanBrightness);
                }

                // final update in the assigned sub-range - in case the previous updates could not be performed
                // this may compute additional samples to estimate the mean brightness
                histogram.updateResult(result, m_pssmlt.estimateMeanBrightness(scene, integrator, sampler.get()));
            };

            /// Uncomment the following line for single threaded rendering
            //map(range);

            /// Default: parallel rendering
            tbb::parallel_for(range, map);

            cout << "done. (took " << timer.elapsedString() << ")" << endl;
        });
    }

    std::string toString() const override {
        return tfm::format(
            "MLTRenderManager[]"
        );
    }

private:
    PSSMLT m_pssmlt;
};

NORI_REGISTER_CLASS(MLTRenderManager, "mlt");
NORI_NAMESPACE_END
