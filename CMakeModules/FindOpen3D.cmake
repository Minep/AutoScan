if (DEFINED O3D_DIR)
    set(O3D_ROOTDIR ${O3D_DIR})
else()
    message(FATAL_ERROR "O3d directory not set")
endif()

set(O3D_INCLUDE_DIR "${O3D_ROOTDIR}/include" "${O3D_ROOTDIR}/include/open3d/3rdparty")

find_library(O3D_LIBRARY NAMES Open3D libOpen3D
             PATHS ${O3D_ROOTDIR} PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Open3D DEFAULT_MSG
        O3D_LIBRARY O3D_INCLUDE_DIR)

mark_as_advanced(O3D_INCLUDE_DIR O3D_LIBRARY)

set(Open3D_LIBRARIES ${O3D_LIBRARY})
set(Open3D_INCLUDE_DIRS ${O3D_INCLUDE_DIR})