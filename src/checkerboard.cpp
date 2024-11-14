#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class CheckerboardTexture2D : public Texture2D {
public:
    CheckerboardTexture2D(const PropertyList &propList)
        : Texture2D(propList) {
        m_color0 = propList.getColor("color0",Color3f(0.0f));
        m_color1 = propList.getColor("color1",Color3f(1.0f));
    }

    Color3f eval(const Point2f &uv) const override {
        const Point2f warpedUV = applyUVScaleAndOffset(uv);
        const int x = warpedUV.x()*2.0f;
        const int y = warpedUV.y()*2.0f;

        if ((x^y)&1)
            return m_color1;
        return m_color0;
    }

    std::string toString() const override {
        std::ostringstream oss;
        oss << "Checkerboard[" << endl
            << " color0 = " << m_color0.toString() << endl
            << " color1 = " << m_color1.toString() << endl
            << "]";
        return oss.str();
    }

private:
    Color3f m_color0;
    Color3f m_color1;
};

NORI_REGISTER_CLASS(CheckerboardTexture2D, "checkerboard");
NORI_NAMESPACE_END
