/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <tbb/task_scheduler_init.h>
#include <filesystem/resolver.h>
#include <thread>

using namespace nori;

static int threadCount = -1;
static bool gui = true;

static void render(Scene *scene, const std::string &filename) {
    const Camera *camera = scene->getCamera();
    Vector2i outputSize = camera->getOutputSize();

    /* Allocate memory for the entire output image and clear it */
    ImageBlock result(outputSize, camera->getReconstructionFilter());
    result.clear();

    /* Create a window that visualizes the partially rendered result */
    NoriScreen *screen = nullptr;
    if (gui) {
        try {
            nanogui::init();
            screen = new NoriScreen(result);
        } catch (const std::runtime_error& error) {
            gui = false;
            std::cout << error.what() << std::endl
                        << "Continuing without GUI." << std::endl;
        }
    }

    /* Do the following in parallel and asynchronously */
    std::thread render_thread([&] {
        tbb::task_scheduler_init init(threadCount);

        /* Determine the filename of the output bitmap */
        std::string outputName = filename;
        size_t lastdot = outputName.find_last_of(".");
        if (lastdot != std::string::npos)
            outputName.erase(lastdot, std::string::npos);

        /* Get RenderManager progressive (default) or blockwise for rendering the scene */
        RenderManager* renderManager = scene->getRenderManager();
        /* Provide additional information to the render manager (for EMCA) */
        renderManager->setOutputFileName(filesystem::path::getcwd().str()+"/"+outputName);
        /* Start rendering the scene */
        renderManager->start_render(scene, result);

#ifndef __APPLE__
        // change the window title to show that rendering is in progress
        // (not allowed on Mac, since only the main thread may access the GUI)
        if (screen)
            screen->set_caption("Nori (rendering...)");
#endif

        /* Wait for the render thread to finish */
        renderManager->join();

        /* Now turn the rendered image block into
            a properly normalized bitmap */
        std::unique_ptr<Bitmap> bitmap(result.toBitmap());

        /* Save using the OpenEXR format */
        bitmap->saveEXR(outputName);

        /* Save tonemapped (sRGB) output using the PNG format */
        bitmap->savePNG(outputName);

#ifndef __APPLE__
        // change the window title to show that rendering has finished
        // (not allowed on Mac, since only the main thread may access the GUI)
        if (screen)
            screen->set_caption("Nori (done)");
#endif
    });

    /* Enter the application main loop */
    if (gui)
        nanogui::mainloop(50.f);

    /* Shut down the user interface */
    render_thread.join();

    if (gui) {
        delete screen;
        nanogui::shutdown();
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "Syntax: " << argv[0] << " <scene.xml> [--no-gui] [--threads N]" <<  endl;
        return -1;
    }

    std::string sceneName = "";
    std::string exrName = "";

    // std::string sceneFilePath = R"(Z:\PC\FortKnoxTheSecond\Studium\PI\Rendering\exercices\nori\scenes\ex03\cbox\cbox-direct.xml)";

    // cornell box
    std::string sceneFilePath = R"(Z:\PC\FortKnoxTheSecond\Studium\PI\Rendering\exercices\nori\scenes\ex03\cbox\cbox-path.xml)";

    // modified cornell box
    // std::string sceneFilePath = R"(Z:\PC\FortKnoxTheSecond\Studium\PI\Rendering\exercices\nori\scenes\ex03\cbox\cbox-path2.xml)";

    // sibenik
    // std::string sceneFilePath = R"(Z:\PC\FortKnoxTheSecond\Studium\PI\Rendering\exercices\nori\scenes\ex03\sibenik\sibenik-path.xml)";

    //std::cout << "Enter File: ";
    //std::cin >> sceneFilePath;

    sceneFilePath.erase(std::remove(sceneFilePath.begin(), sceneFilePath.end(), '\"'), sceneFilePath.end());

    for (int i = 1; i < argc; ++i) {
        std::string token(argv[i]);
        if (token == "-t" || token == "--threads") {
            if (i+1 >= argc) {
                cerr << "\"--threads\" argument expects a positive integer following it." << endl;
                return -1;
            }
            threadCount = atoi(argv[i+1]);
            i++;
            if (threadCount <= 0) {
                cerr << "\"--threads\" argument expects a positive integer following it." << endl;
                return -1;
            }

            continue;
        }
        else if (token == "--no-gui") {
            gui = false;
            continue;
        }

        // filesystem::path path(argv[i]);
        filesystem::path path(sceneFilePath);

        try {
            if (path.extension() == "xml") {
                // sceneName = argv[i];
                sceneName = sceneFilePath;

                /* Add the parent directory of the scene file to the
                   file resolver. That way, the XML file can reference
                   resources (OBJ files, textures) using relative paths */
                getFileResolver()->prepend(path.parent_path());
            } else if (path.extension() == "exr") {
                /* Alternatively, provide a basic OpenEXR image viewer */
                // exrName = argv[i];
                exrName = sceneFilePath;
            } else {
                cerr << "Fatal error: unknown file \"" << argv[i]
                     << "\", expected an extension of type .xml or .exr" << endl;
            }
        } catch (const std::exception &e) {
            cerr << "Fatal error: " << e.what() << endl;
            return -1;
        }
    }

    if (exrName !="" && sceneName !="") {
        cerr << "Both .xml and .exr files were provided. Please only provide one of them." << endl;
        return -1;
    }
    else if (exrName == "" && sceneName == "") {
        cerr << "Please provide the path to a .xml (or .exr) file." << endl;
        return -1;
    }
    else if (exrName != "") {
        if (!gui) {
            cerr << "Flag --no-gui was set. Please remove it to display the EXR file." << endl;
            return -1;
        }
        try {
            Bitmap bitmap(exrName);
            ImageBlock block(Vector2i((int) bitmap.cols(), (int) bitmap.rows()), nullptr);
            block.fromBitmap(bitmap);
            nanogui::init();
            NoriScreen *screen = new NoriScreen(block);
            nanogui::mainloop(50.f);
            delete screen;
            nanogui::shutdown();
        } catch (const std::exception &e) {
            cerr << e.what() << endl;
            return -1;
        }
    }
    else { // sceneName != ""
        if (threadCount < 0) {
            threadCount = tbb::task_scheduler_init::automatic;
        }
        try {
            std::unique_ptr<NoriObject> root(loadFromXML(sceneName));
            /* When the XML root object is a scene, start rendering it .. */
            if (root->getClassType() == NoriObject::EScene)
                render(static_cast<Scene *>(root.get()), sceneName);
        } catch (const std::exception &e) {
            cerr << e.what() << endl;
            return -1;
        }
    }

    return 0;
}
