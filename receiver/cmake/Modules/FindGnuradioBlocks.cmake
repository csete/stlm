INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(GNURADIO_BLOCKS gnuradio-blocks>=3.7)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GNURADIO_BLOCKS DEFAULT_MSG GNURADIO_BLOCKS_LIBRARIES GNURADIO_BLOCKS_INCLUDE_DIRS)
MARK_AS_ADVANCED(GNURADIO_BLOCKS_LIBRARIES GNURADIO_BLOCKS_INCLUDE_DIRS)
