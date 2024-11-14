/*
    This file is part of Nori, a simple educational ray tracer
    EMCA Interface

    Copyright (c) 2021 by Lukas Ruppert
*/

#include <nori/rendermanager.h>
#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/bsdf.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <thread>

#include <nori/emcadataapi.h>
#include <emca/renderinterface.h>
#include <emca/emcaserver.h>

NORI_NAMESPACE_BEGIN

class EMCARenderManager final : public RenderManager, emca::RenderInterface {
public:
    EMCARenderManager() = default;
    EMCARenderManager(const PropertyList &propList) { }

    ~EMCARenderManager() override = default;

    // begin Nori interface

    void start_render(Scene *scene, ImageBlock& result) override {

        this->scene = scene;
        this->resultImage = &result;

        auto emca = EMCADataApi::getInstance();
        emca->configureMeshMapping(scene->getMeshes());

        // Start EMCA server instead of rendering something right away.
        render_thread = std::thread([this](){
            emca::EMCAServer server(this, EMCADataApi::getInstance());
            server.run();
        });

    }

    std::string toString() const override {
        return tfm::format(
            "EMCARenderManager[]"
        );
    }

    // end Nori interface

    void setOutputFileName(const std::string& filename) override {
        output_filename = filename;
    }



    // begin EMCA interface

    void renderImage() override {
        // clear the result in case the image has been rendered before
        resultImage->clear();

        // afterwards, just perform the regular rendering steps

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

                /* Render all contained pixels */
                renderBlock(scene, sampler.get(), block, processedSPP);

                /* The image block has been processed. Now add it to
                   the "big" block that represents the entire image */
                resultImage->put(block);
            }
        };

        for (processedSPP = 0; processedSPP < scene->getSampler()->getSampleCount(); ++processedSPP) {
            /// Uncomment the following line for single threaded rendering
            //map(range);

            /// Default: parallel rendering
            tbb::parallel_for(range, map);

            /// reset block generator
            blockGenerator.setBlockCount(outputSize, NORI_BLOCK_SIZE);
        }

        cout << "done. (took " << timer.elapsedString() << ")" << endl;

        if (!output_filename.empty()) {
            /* Now turn the rendered image block into
                a properly normalized bitmap */
            std::unique_ptr<Bitmap> bitmap(resultImage->toBitmap());

            /* Save using the OpenEXR format */
            bitmap->saveEXR(output_filename);

            /* Save tonemapped (sRGB) output using the PNG format */
            bitmap->savePNG(output_filename);
        }
    }

    void renderPixel(uint32_t x, uint32_t y) override {
        auto emca = EMCADataApi::getInstance();
        std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());

        for (uint32_t i=0; i < sampler->getSampleCount(); ++i) {
            emca->setPathIdx(i);
            renderPath(scene, sampler.get(), Point2f{float(x), float(y)}, i);
        }
    }

    std::string getRendererName() const override {
        return "Nori (specialized for CG Tuebingen, 2021)";
    }

    std::string getSceneName() const override {
        // TODO: pass through the proper scene name
        return "Nori Scene";
    }

    uint32_t getSampleCount() const override {
        const Sampler* sampler = scene->getSampler();
        return static_cast<uint32_t>(sampler->getSampleCount());
    }

    void setSampleCount(size_t sampleCount) override {
        // TODO: support setting the sample count
        (void)sampleCount;
        cerr << "setting sample count is not yet supported - doing nothing." << endl;
    }

    emca::Camera getCameraData() const override {
        const Camera* camera = scene->getCamera();
        auto [pos, dir, up, tnear, tfar, fov] = camera->getPerspectiveCameraParameters();
        return {{pos.x(), pos.y(), pos.z()}, {dir.x(), dir.y(), dir.z()}, {up.x(), up.y(), up.z()}, tnear, tfar, fov};
    }

    std::vector<emca::Mesh> getMeshData() const override {
        std::vector<emca::Mesh> meshData;

        const auto& meshes = scene->getMeshes();
        meshData.reserve(meshes.size());

        for (const auto& mesh : meshes) {
            emca::Mesh mesh_data;

            mesh_data.vertices.resize(mesh->getVertexCount());
            mesh_data.triangles.resize(mesh->getTriangleCount());

            // manually copy vertex-by-vertex and face-by-face, since we cannot assume how Eigen stores the data internally
            for (size_t i=0; i<mesh_data.vertices.size(); ++i) {
                Point3f vertex = mesh->getVertexPositions().col(i);
                mesh_data.vertices.at(i) = {vertex.x(), vertex.y(), vertex.z()};
            }
            for (size_t i=0; i<mesh_data.triangles.size(); ++i) {
                uint32_t a = mesh->getIndices()(0, i);
                uint32_t b = mesh->getIndices()(1, i);
                uint32_t c = mesh->getIndices()(2, i);
                mesh_data.triangles.at(i) = {a, b, c};
            }

            const BSDF* bsdf = mesh->getBSDF();
            BSDFQueryRecord bRec({0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, ESolidAngle);
            Color3f diffColor = bsdf->eval(bRec) * M_PI;
            Color3f specColor {bsdf->isDiffuse() ? 0.0f : 1.0f};

            mesh_data.specularColor = {specColor.r(), specColor.g(), specColor.b()};
            mesh_data.diffuseColor = {diffColor.r(), diffColor.g(), diffColor.b()};

            try {
                mesh_data.surfaceArea = mesh->totalSurfaceArea();
            }
            catch (const NoriException&) {
                // don't break if this is not implemented yet
            }

            meshData.emplace_back(std::move(mesh_data));
        }

        cout << meshData.size() << " mehses converted for use in EMCA" << endl;

        return meshData;
    }

    std::string getRenderedImagePath() const override {
        return output_filename+".exr";
    }

    // end EMCA interface

private:
    Scene* scene {nullptr};
    ImageBlock* resultImage {nullptr};

    std::string output_filename;

    std::tuple<Point2f, Color3f> renderPath(const Scene *scene, Sampler *sampler, const Point2f& pixel, uint32_t sampleNumber)  {
        const Camera *camera = scene->getCamera();
        const Integrator *integrator = scene->getIntegrator();

        // just used to pass a per-pixel offset to the sampler
        ImageBlock dummyBlock(Vector2i(0), nullptr);
        dummyBlock.setOffset(Point2i(pixel.x()+sampleNumber*resultImage->rows()*resultImage->cols(), pixel.y()+sampleNumber*resultImage->rows()*resultImage->cols()));
        sampler->prepare(dummyBlock);

        /* call before pixel gets sampled */
        sampler->generate();

        /* advance over previous samples */
        for (uint32_t i=0; i<sampleNumber; ++i)
            sampler->advance();

        Point2f pixelSample = pixel + sampler->next2D();
        Point2f apertureSample = sampler->next2D();

        /* Sample a ray from the camera */
        Ray3f ray;
        Color3f value = camera->sampleRay(ray, pixelSample, apertureSample);

        /* Compute the incident radiance */
        const Color3f Li = integrator->Li(scene, sampler, ray);

        auto emca = EMCADataApi::getInstance();
        emca->setFinalEstimate(Li);

        value *= Li;

        return {pixelSample, value};
    }

    void renderBlock(const Scene *scene, Sampler *sampler, ImageBlock &block, uint32_t sampleCount) {
        Point2i offset = block.getOffset();
        Vector2i size  = block.getSize();

        /* Clear the block contents */
        block.clear();

        /* For each pixel and pixel sample sample */
        for (int y=0; y<size.y(); ++y) {
            for (int x=0; x<size.x(); ++x) {

                const Point2f pixel = Point2f((float) (x + offset.x()), (float) (y + offset.y()));
                const auto [pixelSample, value] = renderPath(scene, sampler, pixel, sampleCount);

                /* Store in the image block */
                block.put(pixelSample, value);
            }
        }
    }

};

// declared in emcadataapi.h
std::unique_ptr<EMCADataApi> EMCADataApi::emca;

void EMCADataApi::addHeatmapData(const Mesh *mesh, uint32_t primIndex, const Point3f& p, const Color3f& value, float weight) {
    if (!heatmap.isCollecting())
        return;

    // discard samples which are not associated to a shape
    if (mesh == nullptr) {
        std::cerr << "discarding sample without shape" << std::endl;
        return;
    }

    auto it = mesh_to_id.find(mesh);
    // discard samples on unknown shapes (this should never happen!)
    if (it == mesh_to_id.end()) {
        if (mesh_to_id.empty())
            std::cerr << "mapping from mesh to object ids not configured" << std::endl;
        std::cerr << "discarding sample from unknown mesh" << std::endl;
        return;
    }
    const uint32_t mesh_id = it->second;

    // discard samples on shapes that do not provide trianlge ids (for now)
    if (primIndex == -1U) {
        // TODO: perform ray intersection on a proxy mesh
        return;
    }

    heatmap.addSample(mesh_id, emca::Point3f{p.x(), p.y(), p.z()}, primIndex, emca::Color4f(value[0], value[1], value[2]), weight);
}

void EMCADataApi::configureMeshMapping(const std::vector<std::unique_ptr<Mesh>> &meshes) {
    for (size_t i=0; i<meshes.size(); ++i)
        mesh_to_id.emplace(meshes.at(i).get(), i);
}

NORI_REGISTER_CLASS(EMCARenderManager, "emca");
NORI_NAMESPACE_END
