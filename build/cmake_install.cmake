# Install script for directory: /home/tangming/ImageBasedModellingEduV1.0

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tangming/ImageBasedModellingEduV1.0/build/core/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/util/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/features/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/sfm/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/mvs/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/examples/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/surface/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/texturing/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/3rdParty/mrf/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/3rdParty/coldet/cmake_install.cmake")
  include("/home/tangming/ImageBasedModellingEduV1.0/build/3rdParty/gco/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tangming/ImageBasedModellingEduV1.0/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
