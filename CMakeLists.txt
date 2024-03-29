# TODO: many of the best practices described here
# (https://www.slideshare.net/DanielPfeifer1/cmake-48475415) are violated
# in this file. Would be nice to address some of these.

CMAKE_MINIMUM_REQUIRED ( VERSION 3.0.0 )

PROJECT ( PBRT-V3 )

# For sanitizers
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

OPTION(PBRT_FLOAT_AS_DOUBLE "Use 64-bit floats" OFF)

IF (PBRT_FLOAT_AS_DOUBLE)
  ADD_DEFINITIONS ( -D PBRT_FLOAT_AS_DOUBLE )
ENDIF()

OPTION(PBRT_SAMPLED_SPECTRUM "Use SampledSpectrum rather than RGBSpectrum" OFF)

IF (PBRT_SAMPLED_SPECTRUM)
  ADD_DEFINITIONS ( -D PBRT_SAMPLED_SPECTRUM )
ENDIF()

ENABLE_TESTING()
SET (CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/ext/openexr/OpenEXR")
  message(FATAL_ERROR "The OpenEXR submodule directory is missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by running \"git submodule update --init --recursive\"")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/ext/glog/cmake")
  message(FATAL_ERROR "The glog submodule directory is missing! "
    "You probably did not clone the project with --recursive, or you first checked out "
    "pbrt before it was added. It is possible to recover by running "
    "\"git submodule update --init --recursive\"")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/ext/ptex/src")
  message(FATAL_ERROR "The ptex submodule directory is missing! "
    "You probably did not clone the project with --recursive, or you first checked out "
    "pbrt before it was added. It is possible to recover by running "
    "\"git submodule update --init --recursive\"")
endif()

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/ext/zlib/doc")
  message(FATAL_ERROR "The zlib submodule directory is missing! "
    "You probably did not clone the project with --recursive, or you first checked out "
    "pbrt before it was added. It is possible to recover by running "
    "\"git submodule update --init --recursive\"")
endif()

FIND_PACKAGE ( Sanitizers )
FIND_PACKAGE ( Threads )

IF(CMAKE_BUILD_TYPE MATCHES RELEASE)
  ADD_DEFINITIONS (-DNDEBUG)
ENDIF()

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

###########################################################################
# Annoying compiler-specific details

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion-null")
ELSEIF(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register")
ELSEIF(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

  FIND_PROGRAM(XIAR xiar)
  IF(XIAR)
    SET(CMAKE_AR "${XIAR}")
  ENDIF(XIAR)
  MARK_AS_ADVANCED(XIAR)

  FIND_PROGRAM(XILD xild)
  IF(XILD)
    SET(CMAKE_LINKER "${XILD}")
  ENDIF(XILD)
  MARK_AS_ADVANCED(XILD)

  # ICC will default to -fp-model fast=1, which performs value-unsafe optimizations which will
  # cause pbrt_test to fail. For safety, -fp-model precise is explicitly set here by default.
  set(FP_MODEL "precise" CACHE STRING "The floating point model to compile with.")
  set_property(CACHE FP_MODEL PROPERTY STRINGS "precise" "fast=1" "fast=2")

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fp-model ${FP_MODEL}")
ENDIF()

IF(MSVC)
  ADD_DEFINITIONS (/D _CRT_SECURE_NO_WARNINGS)
ENDIF()

INCLUDE (CheckIncludeFiles)

CHECK_INCLUDE_FILES ( alloca.h HAVE_ALLOCA_H )
IF ( HAVE_ALLOCA_H )
  ADD_DEFINITIONS ( -D PBRT_HAVE_ALLOCA_H )
ENDIF ()

CHECK_INCLUDE_FILES ( memory.h HAVE_MEMORY_H )
IF ( HAVE_MEMORY_H )
  ADD_DEFINITIONS ( -D PBRT_HAVE_MEMORY_H )
ENDIF ()

###########################################################################
# Check for various C++11 features and set preprocessor variables or
# define workarounds.

INCLUDE (CheckCXXSourceCompiles)
INCLUDE (CheckCXXSourceRuns)

CHECK_CXX_SOURCE_COMPILES (
  "int main() { float x = 0x1p-32f; }"
  HAVE_HEX_FP_CONSTANTS )
IF ( HAVE_HEX_FP_CONSTANTS )
  ADD_DEFINITIONS ( -D PBRT_HAVE_HEX_FP_CONSTANTS )
ENDIF ()

CHECK_CXX_SOURCE_COMPILES (
  "int main() { int x = 0b101011; }"
  HAVE_BINARY_CONSTANTS )
IF ( HAVE_BINARY_CONSTANTS )
  ADD_DEFINITIONS ( -D PBRT_HAVE_BINARY_CONSTANTS )
ENDIF ()

CHECK_CXX_SOURCE_COMPILES (
  "int main() { constexpr int x = 0; }"
  HAVE_CONSTEXPR )
IF ( HAVE_CONSTEXPR )
  ADD_DEFINITIONS ( -D PBRT_HAVE_CONSTEXPR )
  ADD_DEFINITIONS ( -D PBRT_CONSTEXPR=constexpr )
ELSE ()
  ADD_DEFINITIONS ( -D PBRT_CONSTEXPR=const )
ENDIF ()

CHECK_CXX_SOURCE_COMPILES (
  "struct alignas(32) Foo { char x; }; int main() { }"
  HAVE_ALIGNAS )
IF ( HAVE_ALIGNAS )
  ADD_DEFINITIONS ( -D PBRT_HAVE_ALIGNAS )
ENDIF ()

CHECK_CXX_SOURCE_COMPILES (
  "int main() { int x = alignof(double); }"
  HAVE_ALIGNOF )
IF ( HAVE_ALIGNOF )
  ADD_DEFINITIONS ( -D PBRT_HAVE_ALIGNOF )
ENDIF ()

CHECK_CXX_SOURCE_RUNS ( "
#include <signal.h>
#include <string.h>
#include <sys/time.h>
void ReportProfileSample(int, siginfo_t *, void *) { }
int main() {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = ReportProfileSample;
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPROF, &sa, NULL);
    static struct itimerval timer;
    return setitimer(ITIMER_PROF, &timer, NULL) == 0 ? 0 : 1;
}
" HAVE_ITIMER )
IF ( HAVE_ITIMER )
  ADD_DEFINITIONS ( -D PBRT_HAVE_ITIMER )
ENDIF()

CHECK_CXX_SOURCE_COMPILES ( "
class Bar { public: Bar() { x = 0; } float x; };
struct Foo { union { int x[10]; Bar b; }; Foo() : b() { } };
int main() { Foo f; }
" HAVE_NONPOD_IN_UNIONS )
IF ( HAVE_NONPOD_IN_UNIONS )
  ADD_DEFINITIONS ( -D PBRT_HAVE_NONPOD_IN_UNIONS )
ENDIF ()

CHECK_CXX_SOURCE_COMPILES ( "
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
int main() {
   int fd = open(\"foo\", O_RDONLY);
   struct stat s;
   fstat(fd, &s);
   size_t len = s.st_size;
   void *ptr = mmap(0, len, PROT_READ, MAP_FILE | MAP_SHARED, fd, 0);
   munmap(ptr, len);
}
" HAVE_MMAP )
if ( HAVE_MMAP )
  ADD_DEFINITIONS ( -D PBRT_HAVE_MMAP )
ENDIF ()

########################################
# noinline

CHECK_CXX_SOURCE_COMPILES (
"__declspec(noinline) void foo() { }
int main() { }"
HAVE_DECLSPEC_NOINLINE )

CHECK_CXX_SOURCE_COMPILES (
"__attribute__((noinline)) void foo() { }
int main() { }"
HAVE_ATTRIBUTE_NOINLINE )

IF ( HAVE_ATTRIBUTE_NOINLINE )
  ADD_DEFINITIONS ( -D "PBRT_NOINLINE=__attribute__\\(\\(noinline\\)\\)" )
ELSEIF ( HAVE_DECLSPEC_NOINLINE )
  ADD_DEFINITIONS ( -D "PBRT_NOINLINE=__declspec(noinline)" )
ELSE ()
  ADD_DEFINITIONS ( -D PBRT_NOINLINE )
ENDIF ()

########################################
# Aligned memory allocation

CHECK_CXX_SOURCE_COMPILES ( "
#include <malloc.h>
int main() { void * ptr = _aligned_malloc(1024, 32); }
" HAVE__ALIGNED_MALLOC )

CHECK_CXX_SOURCE_COMPILES ( "
#include <stdlib.h>
int main() {
  void *ptr;
  posix_memalign(&ptr, 32, 1024);
} " HAVE_POSIX_MEMALIGN )

CHECK_CXX_SOURCE_COMPILES ( "
#include <malloc.h>
int main() {
    void *ptr = memalign(32, 1024);
} " HAVE_MEMALIGN )

IF ( HAVE__ALIGNED_MALLOC )
  ADD_DEFINITIONS ( -D PBRT_HAVE__ALIGNED_MALLOC )
ELSEIF ( HAVE_POSIX_MEMALIGN )
  ADD_DEFINITIONS ( -D PBRT_HAVE_POSIX_MEMALIGN )
ELSEIF ( HAVE_MEMALIGN )
  ADD_DEFINITIONS ( -D PBRTHAVE_MEMALIGN )
ELSE ()
  MESSAGE ( SEND_ERROR "Unable to find a way to allocate aligned memory" )
ENDIF ()

########################################
# thread-local variables

CHECK_CXX_SOURCE_COMPILES ( "
#ifdef __CYGWIN__
// Hack to work around https://gcc.gnu.org/bugzilla/show_bug.cgi?id=64697
#error \"No thread_local on cygwin\"
#endif  // __CYGWIN__
thread_local int x; int main() { }
" HAVE_THREAD_LOCAL )

CHECK_CXX_SOURCE_COMPILES ( "
__declspec(thread) int x; int main() { }
" HAVE_DECLSPEC_THREAD )

CHECK_CXX_SOURCE_COMPILES ( "
__thread int x; int main() { }
" HAVE___THREAD )

IF ( HAVE_THREAD_LOCAL )
  ADD_DEFINITIONS ( -D PBRT_THREAD_LOCAL=thread_local )
ELSEIF ( HAVE___THREAD )
  ADD_DEFINITIONS ( -D PBRT_THREAD_LOCAL=__thread )
ELSEIF ( HAVE_DECLSPEC_THREAD )
  ADD_DEFINITIONS ( -D "PBRT_THREAD_LOCAL=__declspec(thread)" )
ELSE ()
  MESSAGE ( SEND_ERROR "Unable to find a way to declare a thread-local variable")
ENDIF ()

###########################################################################
# zlib

FIND_PACKAGE ( ZLIB )
IF(NOT ZLIB_FOUND)
  # Build zlib
  SET(ZLIB_BUILD_STATIC_LIBS ON CACHE BOOL " " FORCE)
  SET(ZLIB_BUILD_SHARED_LIBS OFF CACHE BOOL " " FORCE)
  ADD_SUBDIRECTORY(src/ext/zlib)

  SET(ZLIB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/ext/zlib")
  SET(ZLIB_LIBRARY zlibstatic)
  SET_PROPERTY(TARGET zlibstatic PROPERTY FOLDER "ext")
  INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR} "${CMAKE_CURRENT_BINARY_DIR}/src/ext/zlib")
ENDIF()
INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIRS})

###########################################################################
# OpenEXR

SET(ILMBASE_NAMESPACE_VERSIONING OFF CACHE BOOL " " FORCE)
SET(OPENEXR_NAMESPACE_VERSIONING OFF CACHE BOOL " " FORCE)
SET(OPENEXR_BUILD_SHARED_LIBS    OFF CACHE BOOL " " FORCE)
SET(ILMBASE_BUILD_SHARED_LIBS    OFF CACHE BOOL " " FORCE)

ADD_SUBDIRECTORY(src/ext/openexr)

SET_PROPERTY(TARGET IexMath eLut toFloat b44ExpLogTable dwaLookups IlmThread Half Iex Imath IlmImf PROPERTY FOLDER "ext")

INCLUDE_DIRECTORIES (
  src/ext/openexr/IlmBase/Imath
  src/ext/openexr/IlmBase/Half
  src/ext/openexr/IlmBase/Iex
  src/ext/openexr/OpenEXR/IlmImf
  ${CMAKE_CURRENT_BINARY_DIR}/src/ext/openexr/IlmBase/config
  ${CMAKE_CURRENT_BINARY_DIR}/src/ext/openexr/OpenEXR/config
)
IF(WIN32)
  SET(OPENEXR_LIBS IlmImf Imath Half ${ZLIB_LIBRARY})
ELSE()
  SET(OPENEXR_LIBS IlmImf Imath Half)
ENDIF()

###########################################################################
# glog

SET(WITH_GFLAGS OFF CACHE BOOL "Use gflags")
SET(BUILD_SHARED_LIBS OFF CACHE BOOL " " FORCE)
IF(WIN32)
  ADD_DEFINITIONS( -D GOOGLE_GLOG_DLL_DECL= )
ENDIF()
ADD_SUBDIRECTORY(src/ext/glog)
SET_PROPERTY(TARGET glog logging_unittest demangle_unittest utilities_unittest stl_logging_unittest PROPERTY FOLDER "ext")
INCLUDE_DIRECTORIES (
  src/ext/glog/src
  ${CMAKE_CURRENT_BINARY_DIR}/src/ext/glog
)

###########################################################################
# ptex

# work around https://github.com/wdas/ptex/issues/28
IF ( CMAKE_BUILD_TYPE )
  STRING ( TOLOWER ${CMAKE_BUILD_TYPE} LOWER_BUILD_TYPE )
  SET ( ENV{FLAVOR} ${LOWER_BUILD_TYPE} )
ENDIF ()
SET(PTEX_BUILD_SHARED_LIBS OFF CACHE BOOL " " FORCE)

SET(CMAKE_MACOSX_RPATH 1)
IF ( WIN32 )
  ADD_DEFINITIONS ( /D PTEX_STATIC)
ENDIF ()
ADD_SUBDIRECTORY(src/ext/ptex)
SET_PROPERTY(TARGET Ptex_static ptxinfo halftest ftest rtest wtest PROPERTY FOLDER "ext")
INCLUDE_DIRECTORIES ( src/ext/ptex/src/ptex )

###########################################################################
# Protobuf

ADD_SUBDIRECTORY(src/messages)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/src/messages)

###########################################################################
# On to pbrt...

SET ( PBRT_CORE_SOURCE
  src/core/api.cpp
  src/core/bssrdf.cpp
  src/core/camera.cpp
  src/core/efloat.cpp
  src/core/error.cpp
  src/core/fileutil.cpp
  src/core/film.cpp
  src/core/filter.cpp
  src/core/floatfile.cpp
  src/core/geometry.cpp
  src/core/imageio.cpp
  src/core/integrator.cpp
  src/core/interaction.cpp
  src/core/interpolation.cpp
  src/core/light.cpp
  src/core/lightdistrib.cpp
  src/core/lowdiscrepancy.cpp
  src/core/material.cpp
  src/core/medium.cpp
  src/core/memory.cpp
  src/core/microfacet.cpp
  src/core/parallel.cpp
  src/core/paramset.cpp
  src/core/parser.cpp
  src/core/primitive.cpp
  src/core/progressreporter.cpp
  src/core/quaternion.cpp
  src/core/reflection.cpp
  src/core/sampler.cpp
  src/core/sampling.cpp
  src/core/scene.cpp
  src/core/shape.cpp
  src/core/sobolmatrices.cpp
  src/core/spectrum.cpp
  src/core/stats.cpp
  src/core/texture.cpp
  src/core/transform.cpp
  )

SET ( PBRT_CORE_HEADERS
  src/core/api.h
  src/core/bssrdf.h
  src/core/camera.h
  src/core/efloat.h
  src/core/error.h
  src/core/fileutil.h
  src/core/film.h
  src/core/filter.h
  src/core/floatfile.h
  src/core/geometry.h
  src/core/imageio.h
  src/core/integrator.h
  src/core/interaction.h
  src/core/interpolation.h
  src/core/light.h
  src/core/lowdiscrepancy.h
  src/core/material.h
  src/core/medium.h
  src/core/memory.h
  src/core/microfacet.h
  src/core/mipmap.h
  src/core/parallel.h
  src/core/paramset.h
  src/core/parser.h
  src/core/pbrt.h
  src/core/primitive.h
  src/core/progressreporter.h
  src/core/quaternion.h
  src/core/reflection.h
  src/core/rng.h
  src/core/sampler.h
  src/core/sampling.h
  src/core/scene.h
  src/core/shape.h
  src/core/sobolmatrices.h
  src/core/spectrum.h
  src/core/stats.h
  src/core/stringprint.h
  src/core/texture.h
  src/core/transform.h
  )

FILE ( GLOB PBRT_SOURCE
  src/ext/*
  src/accelerators/*
  src/cameras/*
  src/filters/*
  src/integrators/*
  src/lights/*
  src/materials/*
  src/samplers/*
  src/shapes/*
  src/textures/*
  src/media/*
  src/cloud/*
  src/cloud/ptex/*
  src/messages/*
  src/util/*
 )

INCLUDE_DIRECTORIES ( src )
INCLUDE_DIRECTORIES ( src/core )
INCLUDE_DIRECTORIES ( src/include )

# Visual Studio source folders
SOURCE_GROUP (core REGULAR_EXPRESSION src/core/.*)
SOURCE_GROUP (ext REGULAR_EXPRESSION src/ext/.*)
SOURCE_GROUP (accelerators REGULAR_EXPRESSION src/accelerators/.*)
SOURCE_GROUP (cameras REGULAR_EXPRESSION src/cameras/.*)
SOURCE_GROUP (filters REGULAR_EXPRESSION src/filters/.*)
SOURCE_GROUP (integrators REGULAR_EXPRESSION src/integrators/.*)
SOURCE_GROUP (lights REGULAR_EXPRESSION src/lights/.*)
SOURCE_GROUP (materials REGULAR_EXPRESSION src/materials/.*)
SOURCE_GROUP (samplers REGULAR_EXPRESSION src/samplers/.*)
SOURCE_GROUP (shapes REGULAR_EXPRESSION src/shapes/.*)
SOURCE_GROUP (textures REGULAR_EXPRESSION src/textures/.*)
SOURCE_GROUP (media REGULAR_EXPRESSION src/media/.*)

###########################################################################
# pbrt libraries and executables

ADD_LIBRARY ( pbrt STATIC
  ${PBRT_YACC_LEX_SOURCE}
  ${PBRT_CORE_SOURCE}
  ${PBRT_CORE_HEADERS}
  ${PBRT_SOURCE}
  )
ADD_SANITIZERS ( pbrt )

TARGET_INCLUDE_DIRECTORIES ( pbrt PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/src/include"
  "${CMAKE_CURRENT_LIST_DIR}/src/code"
  )

# A non-exhaustive but pretty representative set..
# Note that we work-around shoddy c++11 support in MSVC2013
# (constexpr, etc.), so don't test for that stuff here
SET ( PBRT_CXX11_FEATURES
  cxx_auto_type
  cxx_explicit_conversions
  cxx_lambdas
  cxx_nullptr
  cxx_range_for
  cxx_static_assert
)
TARGET_COMPILE_FEATURES ( pbrt PRIVATE ${PBRT_CXX11_FEATURES} )

IF (WIN32)
  # Avoid a name clash when building on Visual Studio
  SET_TARGET_PROPERTIES ( pbrt PROPERTIES OUTPUT_NAME libpbrt )
ENDIF()

INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)

INCLUDE(FindLZ4)
FIND_PACKAGE(LZ4 REQUIRED)

FIND_LIBRARY(TCMALLOC_LIB tcmalloc tcmalloc_minimal)
IF(NOT TCMALLOC_LIB)
  message(FATAL_ERROR "tcmalloc not found")
ENDIF()

SET(ALL_PBRT_LIBS
  pbrt
  pbrt_messages
  ${CMAKE_THREAD_LIBS_INIT}
  ${OPENEXR_LIBS}
  glog
  Ptex_static
  ${LZ4_LIBRARY}
  ${ZLIB_LIBRARY}
  ${Protobuf_LIBRARIES}
)

# Main renderer
ADD_EXECUTABLE ( pbrt_exe src/main/pbrt.cpp )
ADD_SANITIZERS ( pbrt_exe )

SET_TARGET_PROPERTIES ( pbrt_exe PROPERTIES OUTPUT_NAME pbrt )
TARGET_COMPILE_FEATURES ( pbrt_exe PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_exe ${ALL_PBRT_LIBS} -fno-builtin-malloc
  -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free ${TCMALLOC_LIB} )

# Tools
ADD_EXECUTABLE ( bsdftest src/tools/bsdftest.cpp )
ADD_SANITIZERS ( bsdftest )
TARGET_COMPILE_FEATURES ( bsdftest PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( bsdftest ${ALL_PBRT_LIBS} )

ADD_EXECUTABLE ( imgtool src/tools/imgtool.cpp )
ADD_SANITIZERS ( imgtool )
TARGET_COMPILE_FEATURES ( imgtool PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( imgtool ${ALL_PBRT_LIBS} )

ADD_EXECUTABLE ( obj2pbrt src/tools/obj2pbrt.cpp )
ADD_SANITIZERS ( obj2pbrt )

ADD_EXECUTABLE ( cyhair2pbrt src/tools/cyhair2pbrt.cpp )
ADD_SANITIZERS ( cyhair2pbrt )

# Static version of pbrt
ADD_EXECUTABLE ( pbrt_static_exe src/main/pbrt.cpp )
ADD_SANITIZERS ( pbrt_static_exe )

SET_TARGET_PROPERTIES ( pbrt_static_exe PROPERTIES OUTPUT_NAME pbrt-static )
TARGET_COMPILE_FEATURES ( pbrt_static_exe PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_static_exe ${ALL_PBRT_LIBS} dl z unwind lzma
                                        -static -Wl,-allow-multiple-definition
                                        -Wl,--whole-archive -lpthread
                                        -Wl,--no-whole-archive -s )

## Cloud rendering
# pbrt-do
ADD_EXECUTABLE ( pbrt_do src/cloud/do.cpp )
ADD_SANITIZERS ( pbrt_do )

SET_TARGET_PROPERTIES ( pbrt_do PROPERTIES OUTPUT_NAME "pbrt-do" )
TARGET_COMPILE_FEATURES ( pbrt_do PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_do ${ALL_PBRT_LIBS} )

# pbrt-genrays
ADD_EXECUTABLE ( pbrt_genrays src/cloud/genrays.cpp )
ADD_SANITIZERS ( pbrt_genrays )

SET_TARGET_PROPERTIES ( pbrt_genrays PROPERTIES OUTPUT_NAME "pbrt-genrays" )
TARGET_COMPILE_FEATURES ( pbrt_genrays PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_genrays ${ALL_PBRT_LIBS} )

# pimage_test
ADD_EXECUTABLE ( pimage_test src/cloud/pimage_test.cpp )
ADD_SANITIZERS ( pimage_test )

SET_TARGET_PROPERTIES ( pimage_test PROPERTIES OUTPUT_NAME "pimage_test" )
TARGET_COMPILE_FEATURES ( pimage_test PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pimage_test ${ALL_PBRT_LIBS} )

# pbrt-treelet-info
ADD_EXECUTABLE ( pbrt_treelet_info src/cloud/treelet-info.cpp )
ADD_SANITIZERS ( pbrt_treelet_info )

SET_TARGET_PROPERTIES ( pbrt_treelet_info PROPERTIES OUTPUT_NAME "pbrt-treelet-info" )
TARGET_COMPILE_FEATURES ( pbrt_treelet_info PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_treelet_info ${ALL_PBRT_LIBS} )

# pbrt-ptexbench
ADD_EXECUTABLE ( pbrt_ptexbench src/cloud/ptexbench.cpp )
ADD_SANITIZERS ( pbrt_ptexbench )

SET_TARGET_PROPERTIES ( pbrt_ptexbench PROPERTIES OUTPUT_NAME "pbrt-ptexbench" )
TARGET_COMPILE_FEATURES ( pbrt_ptexbench PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_ptexbench ${ALL_PBRT_LIBS} )

# pbrt-ptexpand
ADD_EXECUTABLE ( pbrt_ptexpand src/cloud/ptexpand.cpp )
ADD_SANITIZERS ( pbrt_ptexpand )

SET_TARGET_PROPERTIES ( pbrt_ptexpand PROPERTIES OUTPUT_NAME "pbrt-ptexpand" )
TARGET_COMPILE_FEATURES ( pbrt_ptexpand PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_ptexpand ${ALL_PBRT_LIBS} )

# print-scene-summary
ADD_EXECUTABLE ( print_scene_summary src/cloud/print-scene-summary.cpp )
ADD_SANITIZERS ( print_scene_summary )

SET_TARGET_PROPERTIES ( print_scene_summary PROPERTIES OUTPUT_NAME "print-scene-summary" )
TARGET_COMPILE_FEATURES ( print_scene_summary PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( print_scene_summary ${ALL_PBRT_LIBS} )

# convert-treelet
ADD_EXECUTABLE ( convert_treelet src/cloud/convert-treelet.cpp )
ADD_SANITIZERS ( convert_treelet )

SET_TARGET_PROPERTIES ( convert_treelet PROPERTIES OUTPUT_NAME "convert-treelet" )
TARGET_COMPILE_FEATURES ( convert_treelet PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( convert_treelet ${ALL_PBRT_LIBS} -fno-builtin-malloc
  -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free ${TCMALLOC_LIB} )

# print-manifest
ADD_EXECUTABLE ( print_manifest src/cloud/print-manifest.cpp )
ADD_SANITIZERS ( print_manifest )

SET_TARGET_PROPERTIES ( print_manifest PROPERTIES OUTPUT_NAME "print-manifest" )
TARGET_COMPILE_FEATURES ( print_manifest PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( print_manifest ${ALL_PBRT_LIBS} )

# extract-instances
ADD_EXECUTABLE ( extract_instances src/cloud/extract-instances.cpp )
ADD_SANITIZERS ( extract_instances )

SET_TARGET_PROPERTIES ( extract_instances PROPERTIES OUTPUT_NAME "extract-instances" )
TARGET_COMPILE_FEATURES ( extract_instances PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( extract_instances ${ALL_PBRT_LIBS} )

# print-proxy-header
ADD_EXECUTABLE ( print_proxy_header src/cloud/print-proxy-header.cpp )
ADD_SANITIZERS ( print_proxy_header )

SET_TARGET_PROPERTIES ( print_proxy_header PROPERTIES OUTPUT_NAME "print-proxy-header" )
TARGET_COMPILE_FEATURES ( print_proxy_header PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( print_proxy_header ${ALL_PBRT_LIBS} )

# write-proxy-header
ADD_EXECUTABLE ( write_proxy_header src/cloud/write-proxy-header.cpp )
ADD_SANITIZERS ( write_proxy_header )

SET_TARGET_PROPERTIES ( write_proxy_header PROPERTIES OUTPUT_NAME "write-proxy-header" )
TARGET_COMPILE_FEATURES ( write_proxy_header PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( write_proxy_header ${ALL_PBRT_LIBS} )

# dump-bboxes
ADD_EXECUTABLE ( dump_bboxes src/cloud/dump-bboxes.cpp )
ADD_SANITIZERS ( dump_bboxes )

SET_TARGET_PROPERTIES ( dump_bboxes PROPERTIES OUTPUT_NAME "dump-bboxes" )
TARGET_COMPILE_FEATURES ( dump_bboxes PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( dump_bboxes ${ALL_PBRT_LIBS} )

# Unit test

FILE ( GLOB PBRT_TEST_SOURCE
  src/tests/*.cpp
  src/tests/gtest/*.cc
  )

ADD_EXECUTABLE ( pbrt_test ${PBRT_TEST_SOURCE} )
ADD_SANITIZERS ( pbrt_test )
TARGET_COMPILE_FEATURES ( pbrt_test PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( pbrt_test ${ALL_PBRT_LIBS} )

ADD_TEST ( pbrt_unit_test pbrt_test )

FILE ( GLOB UNIT_TEST_SOURCE
  src/tests/util/*.cpp
  src/tests/gtest/*.cc
  )

ADD_EXECUTABLE ( unit_test ${UNIT_TEST_SOURCE} )
ADD_SANITIZERS ( unit_test )
TARGET_COMPILE_FEATURES ( unit_test PRIVATE ${PBRT_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( unit_test ${ALL_PBRT_LIBS} )

ADD_TEST ( util_unit_test unit_test )


# Installation

INSTALL ( TARGETS
  pbrt_exe
  bsdftest
  imgtool
  obj2pbrt
  cyhair2pbrt
  DESTINATION
  bin
  )

INSTALL ( TARGETS
  pbrt
  DESTINATION
  lib
  )
