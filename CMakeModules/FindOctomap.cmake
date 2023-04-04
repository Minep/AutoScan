if (DEFINED OCTOMAP_DIR)
    set(OCTOMAP_ROOT_DIR ${OCTOMAP_DIR})
else()
    message(FATAL_ERROR "Octomap directory not set")
endif()

set(OCTOMAP_INCLUDE_DIR "${OCTOMAP_ROOT_DIR}/include")

find_library(OCTOMAP_LIBARY NAMES octomap liboctomap
             PATHS ${OCTOMAP_ROOT_DIR} PATH_SUFFIXES lib)
find_library(OCTOMATH_LIBARY NAMES octomath liboctomath
             PATHS ${OCTOMAP_ROOT_DIR} PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Octomap DEFAULT_MSG
        OCTOMAP_LIBARY OCTOMAP_INCLUDE_DIR OCTOMATH_LIBARY)

mark_as_advanced(OCTOMAP_INCLUDE_DIR OCTOMAP_LIBARY)

set(Octomap_LIBRARIES ${OCTOMAP_LIBARY} ${OCTOMATH_LIBARY})
set(Octomap_INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIR})