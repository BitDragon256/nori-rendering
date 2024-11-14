#include <nori/texture.h>
#include <nori/bitmap.h>

#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN

class BitmapTexture2D : public Texture2D {

public:
    BitmapTexture2D(const PropertyList &propList) : Texture2D(propList) {
        m_scale = propList.getColor("scale", Color3f(1.0));

        // resolve the local file name
        filesystem::path filename = getFileResolver()->resolve(propList.getString("filename"));

        m_filename = filename.str();

        m_bitmap = new Bitmap(m_filename);
        m_res = Point2i(m_bitmap->cols(), m_bitmap->rows());
    }

    Color3f eval(const Point2f &uv) const {
        const Point2f finalUV = applyUVScaleAndOffset(uv);

        // compute pixel coordinates
        int x = ((int(finalUV[0]*m_res[0]) % m_res[0]) + m_res[0]) % m_res[0];
        int y = ((int(finalUV[1]*m_res[1]) % m_res[1]) + m_res[1]) % m_res[1];

        //flip y coordinate because uv starts at bottom left
        //and xy starts at top left
        y = m_res[1]-(y+1);

        //the bitmap is stored in row-major format
        return m_bitmap->coeff(y, x)*m_scale;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "BitmapTexture[" << endl
            << " filename = " << m_filename << endl
            << "]";
        return oss.str();
    }

private:
    std::string m_filename;
    Bitmap *m_bitmap;
    Point2i m_res;
    Color3f m_scale;

};

NORI_REGISTER_CLASS(BitmapTexture2D, "bitmaptexture");

NORI_NAMESPACE_END
