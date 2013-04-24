#####################################################################
# Setup
#####################################################################
cmake_minimum_required(VERSION 2.8)
project(strx CXX C)
set(strx_VERSION_MAJOR 0)
set(strx_VERSION_MINOR 3)
add_definitions(-DVERSION="${strx_VERSION_MAJOR}.${strx_VERSION_MINOR}")

# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

# Append -O2 optimization flag for Debug builds
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O2")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O2")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

########################################################################
# Compiler specific setup
########################################################################
include(GrMiscUtils) #compiler flag check

if(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
    #http://gcc.gnu.org/wiki/Visibility
    GR_ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-fvisibility=hidden HAVE_VISIBILITY_HIDDEN)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
    GR_ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wsign-compare HAVE_WARN_SIGN_COMPARE)
    GR_ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wall HAVE_WARN_ALL)
    GR_ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wextra HAVE_WARN_ALL)
    GR_ADD_CXX_COMPILER_FLAG_IF_AVAILABLE(-Wno-uninitialized HAVE_WARN_NO_UNINITIALIZED)
endif(CMAKE_COMPILER_IS_GNUCXX)

include(GrBoost)

find_package(GnuradioRuntime)
find_package(GnuradioBlocks)
find_package(UHD)
find_package(GnuradioUHD)
find_package(GnuradioFilter)
find_package(GnuradioFft)
find_package(GnuradioAnalog)
find_package(GnuradioDigital)
find_package(GnuradioAudio)

if (NOT GNURADIO_RUNTIME_FOUND)
    message(FATAL_ERROR "gnuradio-runtime 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_BLOCKS_FOUND)
    message(FATAL_ERROR "gnuradio-blocks 3.7 or later is required to compile strx")
endif()

if (NOT UHD_FOUND)
    message(FATAL_ERROR "UHD is required to compile strx")
endif()

if (NOT GNURADIO_UHD_FOUND)
    message(FATAL_ERROR "gnuradio-uhd 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_FILTER_FOUND)
    message(FATAL_ERROR "gnuradio-filter 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_FFT_FOUND)
    message(FATAL_ERROR "gnuradio-fft 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_ANALOG_FOUND)
    message(FATAL_ERROR "gnuradio-analog 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_DIGITAL_FOUND)
    message(FATAL_ERROR "gnuradio-digital 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_AUDIO_FOUND)
    message(FATAL_ERROR "gnuradio-audio 3.7 or later is required to compile strx")
endif()


########################################################################
# Setup the include and linker paths
########################################################################
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
    ${GNURADIO_BLOCKS_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ${GNURADIO_UHD_INCLUDE_DIRS}
    ${GNURADIO_FILTER_INCLUDE_DIRS}
    ${GNURADIO_FFT_INCLUDE_DIRS}
    ${GNURADIO_ANALOG_INCLUDE_DIRS}
    ${GNURADIO_DIGITAL_INCLUDE_DIRS}
    ${GNURADIO_AUDIO_INCLUDE_DIRS}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${GNURADIO_RUNTIME_LIBRARY_DIRS}
    ${GNURADIO_BLOCKS_LIBRARY_DIRS}
    ${UHD_LIBRARY_DIRS}
    ${GNURADIO_UHD_LIBRARY_DIRS}
    ${GNURADIO_FILTER_LIBRARY_DIRS}
    ${GNURADIO_FFT_LIBRARY_DIRS}
    ${GNURADIO_ANALOG_LIBRARY_DIRS}
    ${GNURADIO_DIGITAL_LIBRARY_DIRS}
    ${GNURADIO_AUDIO_LIBRARY_DIRS}
)

set(gr_link_libs
    ${Boost_LIBRARIES}
    ${GNURADIO_RUNTIME_LIBRARIES}
    ${GNURADIO_BLOCKS_LIBRARIES}
    ${UHD_LIBRARIES}
    ${GNURADIO_UHD_LIBRARIES}
    ${GNURADIO_FILTER_LIBRARIES}
    ${GNURADIO_FFT_LIBRARIES}
    ${GNURADIO_ANALOG_LIBRARIES}
    ${GNURADIO_DIGITAL_LIBRARIES}
    ${GNURADIO_AUDIO_LIBRARIES}
)

set(strx_HDRS
    strx/receiver.h
    strx/strx_source_c.h
    strx/strx_source_c_impl.h
)

set(strx_SRCS
    strx/receiver.cpp
    strx/strx.cpp
    strx/strx_source_c_impl.cpp
)

add_executable(strx ${strx_SRCS})
target_link_libraries(strx ${gr_link_libs})

# Correlator & decoder
add_executable(correlator decoder/correlator.c decoder/viterbi.h)
