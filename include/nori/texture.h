#pragma once

#include <nori/object.h>
#include <nori/color.h>
#include <nori/vector.h>

NORI_NAMESPACE_BEGIN

class Texture2D : public NoriObject {
public:

    Texture2D(const PropertyList &propList) {
        m_uvoffset = Point2f(propList.getFloat("uoffset", 0.0f), propList.getFloat("voffset", 0.0f));
        const float uvscale = propList.getFloat("uvscale", 1.0f);
        m_uvscale = Vector2f(propList.getFloat("uscale", uvscale), propList.getFloat("vscale", uvscale));
    }

    virtual Color3f eval(const Point2f &uv) const = 0;

    EClassType getClassType() const { return ETexture; }

protected:
    inline Point2f applyUVScaleAndOffset(const Point2f &uv) const {
        return uv.cwiseProduct(m_uvscale)+m_uvoffset;
    }

    ///offsets in the uv space
    Point2f m_uvoffset;

    ///scale in the uvspace
    Vector2f m_uvscale;
};

NORI_NAMESPACE_END
