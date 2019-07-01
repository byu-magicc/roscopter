get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/geometry-targets.cmake)
get_filename_component(geometry_INCLUDE_DIRS "${SELF_DIR}/../../include/geometry" ABSOLUTE)
