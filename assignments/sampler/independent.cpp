/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/sampler.h>
#include <nori/block.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

/**
 * Independent sampling - returns independent uniformly distributed
 * random numbers on <tt>[0, 1)x[0, 1)</tt>.
 *
 * This class is essentially just a wrapper around the pcg32 pseudorandom
 * number generator. For more details on what sample generators do in
 * general, refer to the \ref Sampler class.
 */
class Independent : public Sampler {
public:
    Independent(const PropertyList &propList) {
        m_sampleCount = static_cast<size_t>(propList.getInteger("sampleCount", 1));
    }

    std::unique_ptr<Sampler> clone() const override {
        return std::make_unique<Independent>(*this);
    }

    void prepare(const ImageBlock &block) override {
        m_random.seed(
            block.getOffset().x(),
            block.getOffset().y()
        );
    }

    void generate() override { /* No-op for this sampler */ }
    void advance()  override { /* No-op for this sampler */ }

    float next1D() override {
        return m_random.nextFloat();
    }

    Point2f next2D() override {
        return Point2f(
            m_random.nextFloat(),
            m_random.nextFloat()
        );
    }

    std::string toString() const override {
        return tfm::format("Independent[sampleCount=%i]", m_sampleCount);
    }
protected:
    Independent() = default;

private:
    pcg32 m_random;
};

NORI_REGISTER_CLASS(Independent, "independent");
NORI_NAMESPACE_END
