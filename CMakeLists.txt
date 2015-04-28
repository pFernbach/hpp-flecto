CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME hpp-flecto)
SET(PROJECT_DESCRIPTION
  "Implementation of elastic rod manipulation in hpp"
)
SET(PROJECT_URL "")


SET(${PROJECT_NAME}_HEADERS
  include/hpp/flecto/flexible-validation.hh
)

SET(CXX_DISABLE_WERROR 1)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()


SETUP_PROJECT()
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_DOC_DEPENDENCY("hpp-core >= 3")
# Add dependency toward hpp-FLECTO library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("hpp-flecto")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
