/*
    This file is part of Nori, a simple educational ray tracer
    Rendermanager for progressive rendering

    Copyright (c) 2020 by Lukas Ruppert
    Copyright (c) 2017 by Christoph Kreisl
*/

#pragma once

#include <nori/object.h>
#include <thread>

NORI_NAMESPACE_BEGIN

class RenderManager : public NoriObject {
public:
    virtual void start_render(Scene *scene, ImageBlock& result) = 0;
    void join() {render_thread.join();}

    EClassType getClassType() const { return ERenderManager; }

    // set the output filename of the image (needed for EMCA Interface)
    virtual void setOutputFileName(const std::string& /* unused */) {}

protected:
    std::thread render_thread;
};

NORI_NAMESPACE_END
