/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

#include <fstream>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);

        // path tracing parameters
        m_maxBounces   = props.getInteger("maxBounces", 10);
        m_rrMinBounces = props.getInteger("rrMinBounces", 3);
    }

    void preprocess(const Scene *scene) override {

        /* Create a sample generator for the preprocess step */
        std::unique_ptr<Sampler> sampler =
                std::unique_ptr<Sampler>(static_cast<Sampler*>(NoriObjectFactory::createInstance("independent", PropertyList())));

        /* Reserve memory for the photon map */
        m_photonMap.reserve(m_photonCount+m_photonCount/8); // allow for some overshoot

        /* Estimate a default photon radius */
        if (m_photonRadius == 0.0f)
            m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

        // TODO: Exercise 7.2: Implement the photon mapper

        /* Dummy gathering step: just add a single photon */
        m_photonMap.push_back(Photon(
            Point3f(0, 0, 0),  // Position
            Vector3f(0, 0, 1), // Direction
            Color3f(1, 2, 3)   // Power
        ));

        /* Build the photon map */
        m_photonMap.build();

        /* Now let's do a lookup to see if it worked */
        std::vector<uint32_t> results;
        m_photonMap.search(Point3f(0, 0, 0) /* lookup position */, m_photonRadius, results);

        for (uint32_t i : results) {
            const Photon &photon = m_photonMap[i];
            cout << "Found photon!" << endl;
            cout << " Position  : " << photon.getPosition().toString() << endl;
            cout << " Power     : " << photon.getPower().toString() << endl;
            cout << " Direction : " << photon.getDirection().toString() << endl;
        }
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        // TODO: Exercise 7.2: Implement the photon mapper
        throw NoriException("PhotonMapper::Li() is not yet implemented!");
    }

    std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:

    int m_maxBounces;
    int m_rrMinBounces;

    int m_photonCount;
    float m_photonRadius;
    PhotonMap m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
