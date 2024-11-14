cmake_minimum_required (VERSION 3.8...3.17)
project(nori
  DESCRIPTION
    "Nori"
  LANGUAGES
    CXX C
)

add_subdirectory(ext ext_build)

include_directories(
  # Nori include files
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # tinyformat string formatting library
  ${TFM_INCLUDE_DIR}
  # Eigen linear algebra library
  SYSTEM ${EIGEN_INCLUDE_DIR}
  # OpenEXR high dynamic range bitmap library
  SYSTEM ${OPENEXR_INCLUDE_DIRS}
  # Intel Thread Building Blocks
  SYSTEM ${TBB_INCLUDE_DIR}
  # Pseudorandom number generator
  ${PCG32_INCLUDE_DIR}
  # PugiXML parser
  ${PUGIXML_INCLUDE_DIR}
  # Helper functions for statistical hypothesis tests
  ${HYPOTHESIS_INCLUDE_DIR}
  # GLFW library for OpenGL context creation
  SYSTEM ${GLFW_INCLUDE_DIR}
  # GLEW library for accessing OpenGL functions
  SYSTEM ${GLEW_INCLUDE_DIR}
  # NanoVG drawing library
  SYSTEM ${NANOVG_INCLUDE_DIR}
  # NanoGUI user interface library
  SYSTEM ${NANOGUI_INCLUDE_DIR}
  SYSTEM ${NANOGUI_EXTRA_INCS}
  # Portable filesystem API
  SYSTEM ${FILESYSTEM_INCLUDE_DIR}
  # STB Image Write
  SYSTEM ${STB_IMAGE_WRITE_INCLUDE_DIR}
)

# The following lines build the main executable.
# Make sure to create all additional files in the assignments folder.
# Then, CMake will automatically detect them and configure the build process accordingly.
# Starting with CMake 3.12, it will re-scan the folder before every build and reconfigure if anything changes.
# With older versions of CMake, you will have to run it yourself when you add a new file.

file(GLOB_RECURSE ASSIGNMENT_SOURCES CONFIGURE_DEPENDS "assignments/*")

add_executable(nori

  # Header files
  include/nori/rendermanager.h
  include/nori/bbox.h
  include/nori/bitmap.h
  include/nori/block.h
  include/nori/mlt.h
  include/nori/bsdf.h
  include/nori/bvh.h
  include/nori/camera.h
  include/nori/color.h
  include/nori/common.h
  include/nori/dpdf.h
  include/nori/frame.h
  include/nori/integrator.h
  include/nori/emitter.h
  include/nori/mesh.h
  include/nori/object.h
  include/nori/parser.h
  include/nori/proplist.h
  include/nori/ray.h
  include/nori/rfilter.h
  include/nori/sampler.h
  include/nori/scene.h
  include/nori/texture.h
  include/nori/timer.h
  include/nori/transform.h
  include/nori/vector.h
  include/nori/warp.h

  # Source code files
  src/bitmap.cpp
  src/bitmaptexture.cpp
  src/checkerboard.cpp
  src/block.cpp
  src/blockwise.cpp
  src/bvh.cpp
  src/chi2test.cpp
  src/common.cpp
  src/gui.cpp
  src/main.cpp
  src/mltrendermanager.cpp
  src/obj.cpp
  src/object.cpp
  src/parser.cpp
  src/photon.cpp
  src/progressive.cpp
  src/proplist.cpp
  src/rfilter.cpp
  src/scene.cpp
  src/ttest.cpp

  # ASSIGNMENT_SOURCES
  ${ASSIGNMENT_SOURCES}

)

if(EMCA_ENABLED)
  target_sources(nori
    PRIVATE
    include/nori/emcadataapi.h
    src/emca.cpp)
  target_link_libraries(nori emca)
endif()

add_definitions(${NANOGUI_EXTRA_DEFS})

file(GLOB ASSIGNMENT_WARPTEST_SOURCES CONFIGURE_DEPENDS "assignments/sampler/halton*.cpp" "assignments/bsdfs/roughconductor.cpp")

# The following lines build the warping test application
add_executable(warptest
  include/nori/warp.h
  assignments/util/warp.cpp
  src/warptest.cpp
  src/object.cpp
  src/proplist.cpp
  src/common.cpp
  ${ASSIGNMENT_WARPTEST_SOURCES}
)

# BVH library
add_subdirectory(ext/bvh)

if (WIN32)
  target_link_libraries(nori tbb_static pugixml IlmImf nanogui ${NANOGUI_EXTRA_LIBS} bvh zlibstatic)
else()
  target_link_libraries(nori tbb_static pugixml IlmImf nanogui ${NANOGUI_EXTRA_LIBS} bvh)
endif()

target_link_libraries(warptest tbb_static nanogui ${NANOGUI_EXTRA_LIBS})

# Force colored output for the ninja generator
if (CMAKE_GENERATOR STREQUAL "Ninja")
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcolor-diagnostics")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
  elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
  endif()
endif()

target_compile_features(warptest PRIVATE cxx_std_17)
target_compile_features(nori PRIVATE cxx_std_20)

# vim: set et ts=2 sw=2 ft=cmake nospell:
