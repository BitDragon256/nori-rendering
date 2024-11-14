#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/mlt.h>

NORI_NAMESPACE_BEGIN

/// Perturb the sample value using a box mutation
void PSSMLT::boxPerturbation(Sampler *sampler, float &value) {
    // Exercise 8.2: Implement the box perturbation
    throw NoriException("PSSMLT::boxPerturbation() is not yet implemented!");
}

/// Perturb the sample value using a smooth mutation
void PSSMLT::smoothPerturbation(Sampler *sampler, float &value) {
    // Exercise 8.3: Implement the smooth perturbation
    throw NoriException("PSSMLT::smoothPerturbation() is not yet implemented!");
}

/// Process the Markov Chain. Mutate for \param chainlength steps
void PSSMLT::processMarkovChain(const Scene* scene, Integrator* integrator, Sampler* sampler, HistogramImage &histogram) {
    // TODO: Exercise 8.2, 8.3: Implement the Markov Chain

    const Point2i outputSize = scene->getCamera()->getOutputSize();

    /// Current and tentative MCMC states
    ReplaySampler currentState(sampler), tentativeState(sampler);
    Color3f currentContribution, tentativeContribution;

    throw NoriException("PSSMLT::processMarkovChain() is not yet implemented!");
    // silence unused variable warning
    (void) outputSize;
}

/// Compute the contribution of a given Markov Chain state
Color3f PSSMLT::computeContribution(const Scene *scene, Integrator *integrator, ReplaySampler &state) const {
    //TODO: Exercise 8.4: Implement PSSMLT

    // return the testpattern
    return testpattern(state.getPixelSample());
}

NORI_NAMESPACE_END
