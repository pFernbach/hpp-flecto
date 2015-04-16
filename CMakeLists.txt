 CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
 INCLUDE(cmake/base.cmake)

 SET(PROJECT_NAME hpp-flecto)
 SET(PROJECT_DESCRIPTION
   "Implementation of elastic rod manipulation in hpp"
 )
 SET(PROJECT_URL "")

 SETUP_PROJECT()
 ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")

 # Create and install executable running the corba server
 ADD_EXECUTABLE (hpp-flecto-server
   src/server.cc
 )
 # Link executable with hpp-corbaserver library
 PKG_CONFIG_USE_DEPENDENCY (hpp-flecto-server hpp-corbaserver)
 # Install executable
 INSTALL (TARGETS hpp-flecto-server DESTINATION ${CMAKE_INSTALL_BINDIR})
 SETUP_PROJECT_FINALIZE()
