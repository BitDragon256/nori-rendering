/*
    This file is part of Nori, a simple educational ray tracer
    Progressive Rendermanager

    Copyright (c) 2020 by Lukas Ruppert
    Copyright (c) 2017 by Christoph Kreisl
*/

#include <nori/rendermanager.h>
#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <thread>

NORI_NAMESPACE_BEGIN

class ProgressiveRenderManager : public RenderManager {
public:
    ProgressiveRenderManager() = default;
    ProgressiveRenderManager(const PropertyList &propList) { }

    void start_render(Scene *scene, ImageBlock& result) override {
        /* Do the following in parallel and asynchronously */
        render_thread = std::thread([scene, &result] {
            const Camera *camera = scene->getCamera();
            Vector2i outputSize = camera->getOutputSize();
            scene->getIntegrator()->preprocess(scene);

            /* Create a block generator (i.e. a work scheduler) */
            BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            tbb::blocked_range<int> range(0, blockGenerator.getBlockCount());

            uint32_t processedSPP = 0;

            auto map = [&](const tbb::blocked_range<int> &range) {
                /* Allocate memory for a small image block to be rendered
                   by the current thread */
                ImageBlock block(Vector2i(NORI_BLOCK_SIZE),
                        camera->getReconstructionFilter());

                /* Create a clone of the sampler for the current thread */
                std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());

                for (int i=range.begin(); i<range.end(); ++i) {
                    /* Request an image block from the block generator */
                    blockGenerator.next(block);

                    /* Manipulate the block offset to achieve different initialization for each sample */
                    const Point2i offset = block.getOffset();
                    block.setOffset(Point2i(offset.x()+offset.y()*result.cols(), processedSPP));

                    /* Inform the sampler about the block to be rendered */
                    sampler->prepare(block);

                    /* Restore correct block offset */
                    block.setOffset(offset);

                    /* Render all contained pixels */
                    renderBlock(scene, sampler.get(), block, processedSPP);

                    /* The image block has been processed. Now add it to
                       the "big" block that represents the entire image */
                    result.put(block);
                }
            };

            for (processedSPP = 0; processedSPP < scene->getSampler()->getSampleCount(); ++processedSPP) {
                /// Uncomment the following line for single threaded rendering
                //map(range);

                /// Default: parallel rendering
                tbb::parallel_for(range, map);

                /// reset block generator
                blockGenerator.setBlockCount(outputSize, NORI_BLOCK_SIZE);

                std::cout << "rendered sample " << processedSPP << "\n";
            }

            cout << "done. (took " << timer.elapsedString() << ")" << endl;
        });
    }

    std::string toString() const override {
        return tfm::format(
            "ProgressiveRenderManager[]"
        );
    }

private:

    static void renderBlock(const Scene *scene, Sampler *sampler, ImageBlock &block, uint32_t sampleCount) {
        const Camera *camera = scene->getCamera();
        const Integrator *integrator = scene->getIntegrator();

        Point2i offset = block.getOffset();
        Vector2i size  = block.getSize();

        /* Clear the block contents */
        block.clear();

        /* For each pixel and pixel sample sample */
        for (int y=0; y<size.y(); ++y) {
            for (int x=0; x<size.x(); ++x) {

                /* call before pixel gets sampled */
                sampler->generate();

                /* advance over previous samples */
                for (uint32_t i=0; i<sampleCount; ++i)
                    sampler->advance();

                Point2f pixelSample = Point2f((float) (x + offset.x()), (float) (y + offset.y())) + sampler->next2D();
                Point2f apertureSample = sampler->next2D();

                /* Sample a ray from the camera */
                Ray3f ray;
                Color3f value = camera->sampleRay(ray, pixelSample, apertureSample);

                /* Compute the incident radiance */
                value *= integrator->Li(scene, sampler, ray);

                /* Store in the image block */
                block.put(pixelSample, value);
            }
        }
    }

};


NORI_REGISTER_CLASS(ProgressiveRenderManager, "progressive");
NORI_NAMESPACE_END
