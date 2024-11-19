#include <numeric>
#include <nori/sampler.h>
#include <nori/block.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

class Halton : public Sampler {
public:
    Halton(const PropertyList &propList) {
        m_sampleCount = static_cast<size_t>(propList.getInteger("sampleCount", 1));
    }

    std::unique_ptr<Sampler> clone() const override {
        return std::make_unique<Halton>(*this);
    }

    void prepare(const ImageBlock &block) override {
        /* noop */
        m_random.seed(
            block.getOffset().x(),
            block.getOffset().y()
        );
    }

    void generate() override
    {
        m_pathDepth = 0;
        m_sample = 0;
    }
    void advance()  override
    {
        ++m_pathDepth;
        m_sample = 0;
    }

float next1D() override {
    const size_t sample = m_sample;
    ++m_sample;
    if (sample >= 4)
        return m_random.nextFloat();
    return static_cast<float>(radical_inverse(m_pathDepth, prime_numbers[halton_1d[sample]]));
}

Point2f next2D() override {
    const size_t sample = m_sample;
    ++m_sample;
    if (sample >= 4)
        return Point2f(
            m_random.nextFloat(),
            m_random.nextFloat()
        );
    return {
        static_cast<float>(radical_inverse(m_pathDepth, prime_numbers[halton_2d_a[sample]])),
        static_cast<float>(radical_inverse(m_pathDepth, prime_numbers[halton_2d_b[sample]]))
    };
}

    std::string toString() const override {
        return tfm::format("Halton[sampleCount=%i]", m_sampleCount);
    }
protected:
    Halton() = default;

private:
    size_t m_sample{ 0 };
    size_t m_pathDepth{ 0 };
const size_t prime_numbers[12] = {
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37
};
const size_t halton_2d_a[4] = { 1, 3, 5, 7 };
const size_t halton_2d_b[4] = { 2, 4, 6, 8 };
const size_t halton_1d[4] = { 9, 10, 11, 12 };

static double radical_inverse(size_t i, const size_t base)
{
    const auto dbase = static_cast<double>(base);
    double inverse = 0.0;
    double factor = 1.0 / dbase;

    while (i > 0) {
        inverse += static_cast<double>(i % base) * factor;
        i /= base;
        factor /= dbase;
    }

    return inverse;
}

    pcg32 m_random;
};

NORI_REGISTER_CLASS(Halton, "halton");
NORI_NAMESPACE_END
