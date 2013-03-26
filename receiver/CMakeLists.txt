#####################################################################
# Setup
#####################################################################
cmake_minimum_required(VERSION 2.8)
project(strx CXX C)
set(strx_VERSION_MAJOR 0)
set(strx_VERSION_MINOR 1)
add_definitions(-DVERSION="\"${VERSION_MAJOR}"."${VERSION_MINOR}"\")

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

find_package(Gruel)
find_package(GnuradioCore)
find_package(GnuradioBlocks)
find_package(GnuradioUHD)
find_package(GnuradioFilter)
find_package(GnuradioAnalog)
find_package(GnuradioDigital)
find_package(GnuradioAudio)

if (NOT GRUEL_FOUND)
    message(FATAL_ERROR "gruel 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_CORE_FOUND)
    message(FATAL_ERROR "gnuradio-core 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_BLOCKS_FOUND)
    message(FATAL_ERROR "gnuradio-blocks 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_UHD_FOUND)
    message(FATAL_ERROR "gnuradio-uhd 3.7 or later is required to compile strx")
endif()

if (NOT GNURADIO_FILTER_FOUND)
    message(FATAL_ERROR "gnuradio-filter 3.7 or later is required to compile strx")
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
    ${GRUEL_INCLUDE_DIRS}
    ${GNURADIO_CORE_INCLUDE_DIRS}
    ${GNURADIO_CORE_BLOCKS_DIRS}
    ${GNURADIO_UHD_INCLUDE_DIRS}
    ${GNURADIO_FILTER_INCLUDE_DIRS}
    ${GNURADIO_ANALOG_INCLUDE_DIRS}
    ${GNURADIO_DIGITAL_INCLUDE_DIRS}
    ${GNURADIO_AUDIO_INCLUDE_DIRS}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${GRUEL_LIBRARY_DIRS}
    ${GNURADIO_CORE_LIBRARY_DIRS}
    ${GNURADIO_CORE_BLOCKS_DIRS}
    ${GNURADIO_UHD_LIBRARY_DIRS}
    ${GNURADIO_FILTER_LIBRARY_DIRS}
    ${GNURADIO_ANALOG_LIBRARY_DIRS}
    ${GNURADIO_DIGITAL_LIBRARY_DIRS}
    ${GNURADIO_AUDIO_LIBRARY_DIRS}
)

set(gr_link_libs
    ${Boost_LIBRARIES}
    ${GRUEL_LIBRARIES}
    ${GNURADIO_CORE_LIBRARIES}
    ${GNURADIO_BLOCKS_LIBRARIES}
    ${GNURADIO_UHD_LIBRARIES}
    ${GNURADIO_FILTER_LIBRARIES}
    ${GNURADIO_ANALOG_LIBRARIES}
    ${GNURADIO_DIGITAL_LIBRARIES}
    ${GNURADIO_AUDIO_LIBRARIES}
)

#set(strx_HDRS
#)

set(strx_SRCS
    src/strx.cpp
)

add_executable(strx ${strx_SRCS})
target_link_libraries(strx ${gr_link_libs})

# Correlator & decoder
add_executable(correlator decoder/correlator.c)
