#include <nori/emitter.h>
#include <nori/bitmap.h>
#include <nori/scene.h>
#include <nori/frame.h>
#include <nori/mesh.h>
#include <nori/warp.h>
#include <nori/photon.h>
#include <nori/dpdf.h>

#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN

class Envmap : public Emitter {
public:
    Envmap(const PropertyList &propList) {
        // load the toWorld transformation and compute the inverse
        m_toWorld = propList.getTransform("toWorld", Transform());
        m_toLocal = m_toWorld.inverse();

        // load the environment map
        const filesystem::path filename = getFileResolver()->resolve(propList.getString("filename"));
        m_bitmap = std::make_unique<Bitmap>(filename.str());

        // TODO: Exercise 9.2: prepare your importance sampling distribution
    }

    void setParent(NoriObject* obj) override {
        if (obj->getClassType() == EScene)
        {
            Scene* scene = static_cast<Scene*>(obj);
            scene->setEnvMap(this);
        }
    }

    Color3f sampleDirect(EmitterQueryRecord &eRec, Point2f sample) const override {
        if (!m_bitmap)
            return Color3f(0.0f);

        if constexpr (!m_useImportanceSampling)
        {
            /* When using uniform sphere sampling, coordinate frame transformations become redundant,
             * Otherwise, we would need to account for rotations of the environment map in the \c m_toWorld transformation
             */
            eRec.wi = eRec.ws_wi = Warp::squareToUniformSphere(sample);

            // TODO: Exercise 9.1: no importance sampling
            throw NoriException("Envmap::sampleDirect() is not yet implemented!");

            // set these values to something reasonable and remove the exception
            eRec.ep = Point3f(-1.0f);
            eRec.distance = -1.0f;
            eRec.measure = EUnknownMeasure;
            return eval(eRec.ws_wi)/pdfDirect(eRec);
        }
        else
        {
            // TODO: Exercise 9.2: importance sampling

            /* When you use importance sampling, you will need to transform your sampled direction.
             * Note that the provided test scenes for this exercise do not apply any transformation.
             */

            // choose an outgoing direction using importance sampling
            Vector3f localDir;


            /* Transform the local direction vector back to the world coordinate system
             * and flip the outward-pointing direction to an incident direction
             */
            eRec.ws_wi = m_toWorld*Vector3f(-localDir);

            // also return eval()/pdf() -- you may not need to call those functions
            throw NoriException("Envmap::sampleDirect() is not yet implemented!");
        }
    }

    float pdfDirect(const EmitterQueryRecord &eRec) const override {
        // you will have to adjust this when implementing importance sampling
        if constexpr (!m_useImportanceSampling)
        {
            return Warp::squareToUniformSpherePdf(eRec.ws_wi);
        }
        else
        {
            // TODO: Exercise 9.2: return the importance sampling PDF in solid angle
            throw NoriException("Envmap::pdfDirect() is not yet implemented!");
        }
    }

    Color3f eval(Vector3f wi) const override {
        if (!m_bitmap)
            return Color3f(0.0f);

        const int width  = m_bitmap->cols();
        const int height = m_bitmap->rows();

        /* Flip the incident direction into an outward-pointing direction
         * and transform it into the local coordinate system of the envmap.
         * (y = up, x = right, z = front)
         */
        const Vector3f localDir = m_toLocal*Vector3f(-wi);

        // TODO: Exercise 9.1: evaluate the environment map

        throw NoriException("Envmap::eval() is not yet implemented!");
        // silence unused variable warnings
        (void)width, (void)height, (void)localDir;
    }

    Color3f samplePhoton(Ray3f &ray, Sampler* sampler) const override {
        // implementing this for the envmap is not part of the exercises
        throw NoriException("Envmap::samplePhoton() is not yet implemented!");
    }

    std::string toString() const override {

        std::ostringstream oss;
        oss << "Envmap[" << endl
            << "]";
        return oss.str();

    }

private:
    Transform m_toWorld;
    Transform m_toLocal;
    std::unique_ptr<Bitmap> m_bitmap;

    // TODO: Exercise 9.2: importance sampling: switch this to true
    static constexpr bool m_useImportanceSampling = false;
};

NORI_REGISTER_CLASS(Envmap, "envmap");
NORI_NAMESPACE_END
