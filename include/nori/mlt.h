/*
    This file is part of Nori, a simple educational ray tracer
    Utilities for Metropolis Light Transport implementations

    Copyright (c) 2020 by Lukas Ruppert
*/

/* =======================================================================
     This file contains classes for Metropolis Light Transport.
 * ======================================================================= */

# pragma once

#include <nori/block.h>
#include <nori/rfilter.h>
#include <nori/sampler.h>
#include <atomic>
#include <numeric>
#include <functional>

NORI_NAMESPACE_BEGIN

/**
 * \brief Extension of the image block to be used as a histogram in MLT-style applications.
 */
class HistogramImage : public ImageBlock {
public:
    HistogramImage(const Vector2i &size, const ReconstructionFilter *filter)
        : ImageBlock(size, filter) {

        // compute 1D filter weight based on precomputation in ImageBlock
        float filterWeight = std::accumulate(&m_filter[0], &m_filter[NORI_FILTER_RESOLUTION], 0.0f);
        // compute filterWeight^2 times 4 quadrants, normalize by filter resolution and radius
        filterWeight *= m_filterRadius*2.0f/static_cast<float>(NORI_FILTER_RESOLUTION);
        filterWeight *= filterWeight;

        // normalize by filter weight and image size
        m_normalization = filterWeight/static_cast<float>(m_size.prod());
    }

    // use the regular \ref ImageBlock::put(const Point2f &pos, const Color3f &value) function to add samples

    /// Increment the number of samples contained in the histogram
    void incrementSampleCount(const float numEntries) {
        m_entries += numEntries;
    }

    /// Accumulate the histogram data into the \param result image
    /// make sure that the \param mean is accurate
    void updateResult(ImageBlock& result, const float mean=1.0f) {
        const float weight = m_normalization*m_entries/mean;

        if (weight > 0.0f)
        {
            for (long i=0; i<size(); ++i)
                coeffRef(i).w() = weight;

            result.put(*this);
            clear();
        }

        m_entries = 0.0f;
    }

private:
    /// normalization terms for filter weight and image size
    float m_normalization;
    float m_entries {0.0f};
};

/**
 * \brief A sampler which will replay its internal state after each call to generate() or advance()
 * The internal state is automatically extended on demand using the provided fallback sampler.
 *
 * The internal state can be reset to a new random state using \ref newChain()
 * The internal state can be perturbed using \ref perturb()
 */
class ReplaySampler : public Sampler {
public:
    ReplaySampler(Sampler* fallbackSampler)
        : Sampler(), m_fallbackSampler(fallbackSampler)
    {
        m_fallbackSampler->generate();
        m_samples.reserve(32);
        m_sampleCount = 0;
    }

    std::unique_ptr<Sampler> clone() const override {
        return std::make_unique<ReplaySampler>(*this);
    }
    void prepare(const ImageBlock & /*block*/) override {}
    /// Prepare for a new pixel
    void generate()  override {
        m_fallbackSampler->advance();
        // precompute 8 dimensions
        accessSample(7);
        m_dimension = 1;
    }
    /// Advance to the next path within the pixel
    void advance()   override { m_dimension = 1; }
    /// Return the next 1D sample (and skip one)
    float   next1D() override { return accessSample(++m_dimension)[0]; }
    /// Return the next 2D sample
    Point2f next2D() override { return accessSample(++m_dimension);    }

    std::string toString() const override {
        return "ReplaySampler";
    }

    /// Return the pixel sample
    const Point2f& getPixelSample()    const { return m_samples.at(0); }
    /// Return the aperture sample
    const Point2f& getApertureSample() const { return m_samples.at(1); }

    /// Generate a new chain
    void newChain() {
        // cut the chain to reduce future perturbation effort
        m_samples.clear();
        // prepare for the next path
        generate();
    }

    /// Perturb the existing samples
    /// call like this:
    /// ReplaySampler::perturb([](Sampler* sampler, float& value) { value = sampler->next1D(); });
    void perturb(std::function<void(Sampler*, float&)> perturbationFunction) {
        for (Point2f& sample : m_samples)
        {
            perturbationFunction(m_fallbackSampler, sample.x());
            perturbationFunction(m_fallbackSampler, sample.y());

            if (sample.x() < 0.0f || sample.x() >= 1.0f) [[unlikely]]
                throw std::runtime_error("perturbed sample x is outside the allowed [0,1) range.");
            if (sample.y() < 0.0f || sample.y() >= 1.0f) [[unlikely]]
                throw std::runtime_error("perturbed sample y is outside the allowed [0,1) range.");
        }
        // prepare for the next path
        generate();
    }

    /// Swap the samples with another ReplaySampler
    void swapSamples(ReplaySampler& other) {
        m_samples.swap(other.m_samples);
    }

    /// Copy the samples from another ReplaySampler
    void copySamples(const ReplaySampler& other) {
        m_samples = other.m_samples;
    }

    /// Returns the current amount of samples (this will grow on demand)
    size_t getSize() const { return m_samples.size(); }

private:
    /// fallback sampler to extend the current samples as needed
    Sampler* m_fallbackSampler;
    /// internal sample storage (grows on demand)
    std::vector<Point2f> m_samples;
    /// current dimension. always starting after the aperture sample
    uint32_t m_dimension {1};

    /// access an existing sample or extend the samples to satisfy the request
    inline Point2f& accessSample(const size_t index) {
        if (index >= m_samples.size()) {
            const ptrdiff_t currentSize = static_cast<ptrdiff_t>(m_samples.size());
            // always generate 4 samples at once
            m_sampleCount = (index/4)*4+4;
            m_samples.resize(m_sampleCount);
            std::generate(m_samples.begin()+currentSize, m_samples.end(), [this]() -> Point2f {return m_fallbackSampler->next2D();});

            if (m_sampleCount == 100) {
                // Warn at 100 samples - might still be valid usage in rare cases.
                std::cerr << "WARNING: requested the 100th sample in the ReplaySampler.\n"
                             "Unless you are computing very long paths, there is probably an issue with your implementation.\n"
                             "Make sure to call ReplaySampler::advance() after processing each path\n"
                             "and ReplaySampler::generate() before processing each pixel.\n"
                          << std::flush;
            }
            if (m_sampleCount == 400) {
                // At 400 samples, something definitely went wrong - throw an exception.
                throw NoriException("Requested the 400th sample in the ReplaySampler.\nThere is most likely an isssue in your implementation.");
            }
        }
        return m_samples.at(index);
    }
};

/**
 * \brief Pirmary Sample Space Metropolis Light Ttransport implementation template
 * The provided implementation can already perform \ref newChain mutations
 * and will automatically estimate the mean image brightness for you.
 * In combination with the provided \ref MLTRenderManager,
 * you only need to implement the perturbation functions and the MCMC core loop.
 */
class PSSMLT
{
public:
    PSSMLT() = default;
    PSSMLT(int chainlength, int startup, float newChain)
        : m_chainlength{chainlength}, m_startup{startup}, m_newChain{newChain} {}

    // MCMC core loop and evaluation of contribution -- you will have to implement these

    /// Process the Markov Chain. Mutate for \param chainlength steps
    void processMarkovChain(const Scene *scene, Integrator *integrator, Sampler *sampler, HistogramImage &histogram);
    /// Compute the contribution for a specific state of the Markov Chain
    Color3f computeContribution(const Scene *scene, Integrator *integrator, ReplaySampler& state) const;

    // sample perturbation functions -- you will have to implement these

    /// Perturbs the \param value by applying a box mutation.
    /// Used by \ref perturbChainBox
    static void boxPerturbation(Sampler *sampler, float &value);
    /// Perturbs the \param value by applying a smooth mutation.
    /// Used by \ref perturbChainSmooth
    static void smoothPerturbation(Sampler *sampler, float &value);

    // from here on, the implementation is provided

    /// Compute a new chain and return its contribution.
    /// Also updates the global estimate of the mean brightness
    [[nodiscard]] Color3f newChain(const Scene *scene, Integrator *integrator, ReplaySampler& state) {
        state.newChain();
        const Color3f contribution = computeContribution(scene, integrator, state);
        // update mean contribution
        if (contribution.isValid())
        {
            // this section is probably a bit overengineered, don't worry about it too much :)

            thread_local BrightnessEstimate localEstimate;
            // update thread local estimate
            localEstimate += contribution.mean();

            // try to update the global estimate once a few local estimates have been accumulated
            if (localEstimate.count >= 16)
            {
                BrightnessEstimate current = m_brightnessSumAndCount.load();
                const BrightnessEstimate updated{current.sumBrightness+localEstimate.sumBrightness, current.count+localEstimate.count};
                if (std::atomic_compare_exchange_weak(&m_brightnessSumAndCount, &current, updated))
                    localEstimate.clear();
            }
        }
        return contribution;
    }

    /// Perturb the chain using box mutations and return its contribution
    [[nodiscard]] Color3f perturbChainBox(const Scene *scene, Integrator *integrator, ReplaySampler& state) const {
        state.perturb(boxPerturbation);
        return computeContribution(scene, integrator, state);
    }

    /// Perturb the chain using smooth mutations and return its contribution
    [[nodiscard]] Color3f perturbChainSmooth(const Scene *scene, Integrator *integrator, ReplaySampler& state) const {
        state.perturb(smoothPerturbation);
        return computeContribution(scene, integrator, state);
    }

    /// Return the mean brightness and a boolean indicating whether it should be considered valid
    std::pair<float, bool> getMeanBrightness() const {
        const BrightnessEstimate current = m_brightnessSumAndCount.load(std::memory_order_relaxed);
        return std::make_pair(current.getMean(), current.isValid());
    }

    /// Returns the mean brightness.
    /// Additional samples are computed as necessary until the required confidence level is reached.
    float estimateMeanBrightness(const Scene *scene, Integrator *integrator, Sampler* sampler) {
        ReplaySampler temp(sampler);
        float meanBrightness;
        bool meanBrightnessValid;
        while(true)
        {
            std::tie(meanBrightness, meanBrightnessValid) = getMeanBrightness();
            if (meanBrightnessValid)
                return meanBrightness;

            // update estimate as needed -- compute a few samples at a time
            for (int i=0; i<16; ++i)
                (void) newChain(scene, integrator, temp);
        }
    }

    /// Return the number of state mutations made per Markov Chain
    int getChainlength() const { return m_chainlength; }

private:
    /// number of steps each Markov Chain should take
    int m_chainlength {10000};
    /// number of additional initial steps to discard
    int m_startup      {1000};
    /// ratio of new chain mutations
    float m_newChain   {0.1f};

    // small helper struct for easy atomic update
    struct BrightnessEstimate {
        float sumBrightness {0.0f};
        int count {0};

        void operator+=(float brightness) {
            sumBrightness += brightness;
            ++count;
        }

        void clear() {
            *this = BrightnessEstimate{};
        }

        float getMean() const { return sumBrightness/count; }

        bool isValid() const {
            const int requiredSamples = 100000; // this should be sufficient in most cases
            return count >= requiredSamples;
        }

        BrightnessEstimate() noexcept {}
        BrightnessEstimate(float sumBrightness, int count)
            : sumBrightness{sumBrightness}, count{count} {}
    };
    std::atomic<BrightnessEstimate> m_brightnessSumAndCount;

    // 2D testpattern to test the implementation without using the nested integrator
    static Color3f testpattern(const Point2f& pixelSample)
    {
        const uint32_t x = static_cast<uint32_t>(pixelSample.x()*8);
        const uint32_t y = static_cast<uint32_t>(pixelSample.y()*8);

        Color3f result {0.0f};

        if (((x>>1)^(y>>1))&1)
            result += Color3f(0.0f, 0.0f, 1.0f-sqrt(pixelSample.prod()));
        if ((~(x^y))&1)
            result += Color3f(pixelSample.x(), pixelSample.y(), 0.0f);
        else
            result += (pixelSample-Point2f(1.0f)).prod()*0.25f;

        const float falloff = exp(-128.0f*(pixelSample-Point2f(0.25f, 0.25f)).squaredNorm())
                + exp(-128.0f*(pixelSample-Point2f(0.25f, 0.75f)).squaredNorm())
                + exp(-128.0f*(pixelSample-Point2f(0.75f, 0.25f)).squaredNorm())
                + exp(-128.0f*(pixelSample-Point2f(0.75f, 0.75f)).squaredNorm());

        return result*falloff;
    }
};

NORI_NAMESPACE_END
