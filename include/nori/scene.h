/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/bvh.h>
#include <nori/dpdf.h>
#include <nori/rendermanager.h>

#include <memory>

NORI_NAMESPACE_BEGIN

/**
 * \brief Main scene data structure
 *
 * This class holds information on scene objects and is responsible for
 * coordinating rendering jobs. It also provides useful query routines that
 * are mostly used by the \ref Integrator implementations.
 */
class Scene : public NoriObject {
public:
    /// Construct a new scene object
    Scene(const PropertyList &);

    /// Release all memory
    virtual ~Scene() override;

    /// Return a pointer to the scene's BVH
    const BVH *getBVH() const { return m_bvh.get(); }

    /// Return a pointer to the scene's integrator
    const Integrator *getIntegrator() const { return m_integrator.get(); }

    /// Return a pointer to the scene's integrator
    Integrator *getIntegrator() { return m_integrator.get(); }

    /// Return a pointer to the scene's camera
    const Camera *getCamera() const { return m_camera.get(); }

    /// Return a pointer to the scene's sample generator (const version)
    /// Note that this serves only as a template that is cloned into each thread before usage.
    /// Do not use this sampler to generate samples directly.
    const Sampler *getSampler() const { return m_sampler.get(); }

    /// Return a reference to an array containing all meshes
    const std::vector<std::unique_ptr<Mesh>> &getMeshes() const { return m_meshes; }

    /// Return a pointer to the used render manager
    RenderManager *getRenderManager() { return m_rendermanager.get(); }

    /// Return a pointer to a vector containing all emitters
    const std::vector<std::unique_ptr<Emitter>> &getEmitters() const { return m_emitters; }

    /// Set the scene's environment map
    void setEnvMap(Emitter* envmap) { m_envmap = envmap; }

    /// Sample a random emitter for direct illumination
    Color3f sampleEmitterDirect(EmitterQueryRecord &eRec, Point2f sample) const;

    /// Return the scene-wide PDF for a given direct illumination sample
    float pdfEmitterDirect(const EmitterQueryRecord &eRec) const;

    /// Sample a random emitter for photon mapping
    Color3f sampleEmitterPhoton(Ray3f &ray, Sampler *sampler) const;

    /// Get the environment emitter
    const Emitter *getEnvironmentMap() const { return m_envmap; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene
     * and return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum
     *    extent information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray, Intersection &its) const {
        return m_bvh->rayIntersect(ray, its, false);
    }

    /**
     * \brief Intersect a ray against all triangles stored in the scene
     * and \a only determine whether or not there is an intersection.
     *
     * This method much faster than the other ray tracing function,
     * but the performance comes at the cost of not providing any
     * additional information about the detected intersection
     * (not even its position).
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum
     *    extent information
     *
     * \return \c true if an intersection was found
     */
    bool rayIntersect(const Ray3f &ray) const {
        Intersection its; /* Unused */
        return m_bvh->rayIntersect(ray, its, true);
    }

    /// \brief Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const {
        return m_bvh->getBoundingBox();
    }

    /**
     * \brief Inherited from \ref NoriObject::activate()
     *
     * Initializes the internal data structures (kd-tree,
     * emitter sampling data structures, etc.)
     */
    void activate() override;

    /// Add a child object to the scene (meshes, integrators etc.)
    void addChild(NoriObject *obj) override;

    /// Return a string summary of the scene (for debugging purposes)
    std::string toString() const override;

    EClassType getClassType() const override { return EScene; }
private:
    std::vector<std::unique_ptr<Mesh>> m_meshes;
    std::unique_ptr<Integrator> m_integrator;
    std::unique_ptr<const Sampler> m_sampler;
    std::unique_ptr<Camera> m_camera;
    std::unique_ptr<BVH> m_bvh {new BVH{}};
    std::vector<std::unique_ptr<Emitter>> m_emitters;
    std::vector<std::unique_ptr<const BSDF>> m_bsdfs;
    Emitter *m_envmap = nullptr;
    std::unique_ptr<RenderManager> m_rendermanager;
};

NORI_NAMESPACE_END
