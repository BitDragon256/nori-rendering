/*
    This file is part of Nori, a simple educational ray tracer
    EMCA Interface

    Copyright (c) 2020 by Lukas Ruppert
*/

#pragma once

#include <nori/common.h>
#include <nori/vector.h>
#include <nori/color.h>
#include <emca/dataapi.h>

#include <cassert>
#include <memory>

NORI_NAMESPACE_BEGIN

class EMCADataApi final : public emca::DataApi {
private:
    static std::unique_ptr<EMCADataApi> emca;
public:
    static EMCADataApi* getInstance() {
        if (!emca.get())
            emca.reset(new EMCADataApi());

        return emca.get();
    }

    ~EMCADataApi() override = default;

    // overload functions with Nori-specific data types
    void setPathOrigin(const Point3f& p) {
        emca::DataApi::setPathOrigin({p.x(), p.y(), p.z()});
    }
    void setIntersectionPos(const Point3f& p) {
        emca::DataApi::setIntersectionPos({p.x(), p.y(), p.z()});
    }
    void setNextEventEstimationPos(const Point3f& p, bool visible) {
        emca::DataApi::setNextEventEstimationPos({p.x(), p.y(), p.z()}, visible);
    }
    void setIntersectionEstimate(const Color3f& c) {
        emca::DataApi::setIntersectionEstimate({c[0], c[1], c[2]});
    }
    void setIntersectionEmission(const Color3f& c) {
        emca::DataApi::setIntersectionEmission({c[0], c[1], c[2]});
    }
    void setFinalEstimate(const Color3f& c) {
        emca::DataApi::setFinalEstimate({c[0], c[1], c[2]});
    }

    // provide the generic interface
    using emca::DataApi::addIntersectionData;


    // support for Nori 2D types
    template <typename T, typename std::enable_if<std::is_same<T, Point2i>::value || std::is_same<T, Point2f>::value || std::is_same<T, Vector2i>::value || std::is_same<T, Vector2f>::value, int>::type = 0>
    void addIntersectionData(const std::string& s, const T& p) {
        emca::DataApi::addIntersectionData(s, p.x(), p.y());
    }

    // support for Nori 3D types
    template <typename T, typename std::enable_if<std::is_same<T, Point3i>::value || std::is_same<T, Point3f>::value || std::is_same<T, Vector3i>::value || std::is_same<T, Vector3f>::value, int>::type = 1>
    void addIntersectionData(const std::string& s, const T& p) {
        emca::DataApi::addIntersectionData(s, p.x(), p.y(), p.z());
    }

    // support for Nori Color
    void addIntersectionData(const std::string& s, const Color3f& c) {
        emca::DataApi::addIntersectionData(s, c[0], c[1], c[2], 1.0f);
    }

    // provide the generic interface
    using emca::DataApi::addPathData;


    // support for Nori 2D types
    template <typename T, typename std::enable_if<std::is_same<T, Point2i>::value || std::is_same<T, Point2f>::value || std::is_same<T, Vector2i>::value || std::is_same<T, Vector2f>::value, int>::type = 0>
    void addPathData(const std::string& s, const T& p) {
        emca::DataApi::addPathData(s, p.x, p.y);
    }

    // support for Nori 3D types
    template <typename T, typename std::enable_if<std::is_same<T, Point3i>::value || std::is_same<T, Point3f>::value || std::is_same<T, Vector3i>::value || std::is_same<T, Vector3f>::value, int>::type = 1>
    void addPathData(const std::string& s, const T& p) {
        emca::DataApi::addPathData(s, p.x, p.y, p.z);
    }

    // support for Nori Color
    void addPathData(const std::string& s, const Color3f& c) {
        emca::DataApi::addPathData(s, c[0], c[1], c[2], 1.0f);
    }

    /**
     * @brief addHeatmapData records information to be displayed in 3D heatmaps
     * @param shape     intersected shape
     * @param primIndex intersected triangle
     * @param value     value to record at the location
     */
    void addHeatmapData(const Mesh *mesh, uint32_t primIndex, const Point3f& p, const Color3f& value, float weight=1.0f);
    void configureMeshMapping(const std::vector<std::unique_ptr<Mesh>>& meshes);

private:
    std::unordered_map<const Mesh*, uint32_t> mesh_to_id;
};

NORI_NAMESPACE_END
