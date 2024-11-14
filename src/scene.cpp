/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/scene.h>
#include <nori/bitmap.h>
#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/camera.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

Scene::Scene(const PropertyList &) {}
Scene::~Scene() = default;

void Scene::activate() {
    m_bvh->build();

    if (!m_integrator)
        throw NoriException("No integrator was specified!");
    if (!m_camera)
        throw NoriException("No camera was specified!");

    if (!m_sampler) {
        /* Create a default (independent) sampler */
        m_sampler.reset(static_cast<Sampler*>(
            NoriObjectFactory::createInstance("independent", PropertyList())));
    }

    if (!m_rendermanager) {
        /* Create default rendermanager (progressive) */
        m_rendermanager.reset(static_cast<RenderManager *>(
            NoriObjectFactory::createInstance("progressive", PropertyList())));
    }

    // register all BSDFs and Emitters with the scene
    for (const auto& mesh : m_meshes) {
        m_bsdfs.emplace_back(mesh->getBSDF());
        if (mesh->isEmitter()) {
            Emitter* emitter = mesh->getEmitter();
            emitter->idx = static_cast<uint32_t>(m_emitters.size());
            m_emitters.emplace_back(emitter);
        }
    }

    cout << endl;
    cout << "Configuration: " << toString() << endl;
    cout << endl;
}

/// Sample a random emitter for direct illumination
Color3f Scene::sampleEmitterDirect(EmitterQueryRecord &eRec, Point2f sample) const {
    eRec.eidx = static_cast<uint32_t>(sample.x()*static_cast<float>(m_emitters.size()));
    sample.x() = sample.x()*static_cast<float>(m_emitters.size())-static_cast<float>(eRec.eidx);

    Color3f result = m_emitters.at(eRec.eidx)->sampleDirect(eRec, sample);
    result *= static_cast<float>(m_emitters.size());

    return result;
}

/// Return the PDF for a given direct illumination sample
float Scene::pdfEmitterDirect(const EmitterQueryRecord &eRec) const {
    return m_emitters.at(eRec.eidx)->pdfDirect(eRec) / static_cast<float>(m_emitters.size());
}

/// Sample a random emitter for photon mapping
Color3f Scene::sampleEmitterPhoton(Ray3f &ray, Sampler* sampler) const {
    uint32_t eidx = static_cast<uint32_t>(sampler->next1D()*static_cast<float>(m_emitters.size()));

    Color3f result = m_emitters.at(eidx)->samplePhoton(ray, sampler);
    result *= static_cast<float>(m_emitters.size());

    return result;
}

void Scene::addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
        case EMesh: {
                Mesh *mesh = static_cast<Mesh *>(obj);
                m_bvh->addMesh(mesh);
                m_meshes.emplace_back(mesh);
            }
            break;

        case EEmitter: {
                Emitter *emitter = static_cast<Emitter *>(obj);
                emitter->setParent(this);
                emitter->idx = static_cast<uint32_t>(m_emitters.size());
                m_emitters.emplace_back(emitter);
            }

            break;

        case ESampler:
            if (m_sampler)
                throw NoriException("There can only be one sampler per scene!");
            m_sampler.reset(static_cast<Sampler *>(obj));
            break;

        case ECamera:
            if (m_camera)
                throw NoriException("There can only be one camera per scene!");
            m_camera.reset(static_cast<Camera *>(obj));
            break;

        case EIntegrator:
            if (m_integrator)
                throw NoriException("There can only be one integrator per scene!");
            m_integrator.reset(static_cast<Integrator *>(obj));
            break;

        case ERenderManager:
            if(m_rendermanager)
                throw NoriException("There can only be one render manager per scene");
            m_rendermanager.reset(static_cast<RenderManager *>(obj));
            break;

        default:
            throw NoriException("Scene::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
    }
}

std::string Scene::toString() const {
    std::string meshes;
    for (size_t i=0; i<m_meshes.size(); ++i) {
        meshes += std::string("  ") + indent(m_meshes[i]->toString(), 2);
        if (i + 1 < m_meshes.size())
            meshes += ",";
        meshes += "\n";
    }

    return tfm::format(
        "Scene[\n"
        "  rendermanager = %s, \n"
        "  integrator = %s,\n"
        "  sampler = %s\n"
        "  camera = %s,\n"
        "  meshes = {\n"
        "  %s  },\n"
        "  envmap = %s\n"
        "]",
        indent(m_rendermanager->toString()),
        indent(m_integrator->toString()),
        indent(m_sampler->toString()),
        indent(m_camera->toString()),
        indent(meshes, 2),
        indent(m_envmap ? m_envmap->toString() : std::string("null"))
    );
}

NORI_REGISTER_CLASS(Scene, "scene");
NORI_NAMESPACE_END
