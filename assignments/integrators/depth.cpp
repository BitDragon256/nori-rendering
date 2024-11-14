/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2012 by Wenzel Jakob and Steve Marschner.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief DepthIntegrator
 **/
class DepthIntegrator : public Integrator {
public:
    DepthIntegrator(const PropertyList &propList){
        /* Loading necessary parameters from the XML file */
        m_minDepth = propList.getFloat("minDepth", 0.0f);
        m_maxDepth = propList.getFloat("maxDepth", 1000.0f);
    }

    /* Li is called N times per pixel, where N stands for the amount of samples
     * which are set in the XML file (check the sampleCount parameter)
     */
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &cameraRay) const override {
        Intersection its;

        /* Check if there is an intersection with an object in the scene.
         * If yes, the parameters of the Intersection record will be initialized.
         * Otherwise, we did not hit anything. In this case, we return 0.
         */
        if (!scene->rayIntersect(cameraRay, its))
            return Color3f(0.0);

        /* Check the Intersection struct for more information about the parameters
         * (include/nori/mesh.h)
         */
        return Color3f(1.0f - (its.t - m_minDepth) / (m_maxDepth - m_minDepth));
    }

    // Information which will be printed in the console before Nori starts its rendering job
    std::string toString() const override {
        std::ostringstream oss;
            oss << "DepthIntegrator[" << endl
                << " minDepth = " << m_minDepth << endl
                << " maxDepth = " << m_maxDepth << endl
                << "]";
        return oss.str();

    }
private:
    // Nori uses the Hungarian notation, so member variables start with "m_"
    float m_minDepth;
    float m_maxDepth;
};

NORI_REGISTER_CLASS(DepthIntegrator, "depth");
NORI_NAMESPACE_END
