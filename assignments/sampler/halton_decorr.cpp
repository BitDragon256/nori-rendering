#include <numeric>
#include <nori/sampler.h>
#include <nori/block.h>
#include <pcg32.h>

NORI_NAMESPACE_BEGIN

class HaltonDecorr : public Sampler {
public:
    HaltonDecorr(const PropertyList &propList) {
        m_sampleCount = static_cast<size_t>(propList.getInteger("sampleCount", 1));
    }

    std::unique_ptr<Sampler> clone() const override {
        return std::make_unique<HaltonDecorr>(*this);
    }

    void prepare(const ImageBlock &block) override {
        m_random.seed(
            block.getOffset().x(),
            block.getOffset().y()
        );
    }

    void generate() override
    {
        m_randomVec.clear();
        for (int dim = 0; dim < 4; ++dim)
            m_randomVec.push_back(m_random.nextFloat());
        m_pathDepth = 0;
        m_dimension = 0;
    }
    void advance()  override
    {
        ++m_pathDepth;
        m_dimension = 0;
    }

float next1D() override {
    const size_t sample = m_dimension;
    ++m_dimension;
    if (sample >= 4)
        return m_random.nextFloat();
    return std::fmodf(static_cast<float>(m_randomVec[sample] + radical_inverse(m_pathDepth, prime_numbers[halton_1d[sample]])), 1.f);
}

Point2f next2D() override {
    const size_t sample = m_dimension;
    ++m_dimension;
    if (sample >= 4)
        return Point2f(
            m_random.nextFloat(),
            m_random.nextFloat()
        );
    return {
        std::fmodf(static_cast<float>(m_randomVec[sample] + radical_inverse(m_pathDepth, prime_numbers[halton_2d_a[sample]])), 1.f),
        std::fmodf(static_cast<float>(m_randomVec[sample] + radical_inverse(m_pathDepth, prime_numbers[halton_2d_b[sample]])), 1.f)
    };
}

    std::string toString() const override {
        return tfm::format("HaltonDecorr[sampleCount=%i]", m_sampleCount);
    }
protected:
    HaltonDecorr() = default;

private:
    size_t m_dimension{ 0 };
    size_t m_pathDepth{ 0 };
    const size_t prime_numbers[12] = {
        2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37
    };
    const size_t halton_2d_a[4] = { 1, 3, 5, 7 };
    const size_t halton_2d_b[4] = { 2, 4, 6, 8 };
    const size_t halton_1d[4] = { 9, 10, 11, 12 };

    std::vector<float> m_randomVec;

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

NORI_REGISTER_CLASS(HaltonDecorr, "halton_decorr");
NORI_NAMESPACE_END
