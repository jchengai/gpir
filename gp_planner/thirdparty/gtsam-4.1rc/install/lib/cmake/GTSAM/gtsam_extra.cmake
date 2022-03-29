# Extra CMake definitions for GTSAM

# All version variables are handled by GTSAMConfigVersion.cmake, except these
# two below. We continue to set them here in case someone uses them
set (GTSAM_VERSION_NUMERIC 40100)
set (GTSAM_VERSION_STRING "4.1.0")

set (GTSAM_USE_TBB 1)
set (GTSAM_DEFAULT_ALLOCATOR TBB)

if("")
  list(APPEND GTSAM_CYTHON_INSTALL_PATH "")
  list(APPEND GTSAM_EIGENCY_INSTALL_PATH "")
endif()
