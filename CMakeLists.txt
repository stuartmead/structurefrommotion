#Copyright (c) 2015, Stuart Mead - Risk Frontiers, Macquarie University
#All rights reserved.
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#      
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#      
#    * Neither the name of the copyright holder nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
#ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  For further information, contact:
#    Stuart Mead
#    Risk Frontiers
#    Dept. of Environmental Sciences
#    Macquarie University
#    North Ryde NSW 2109


cmake_minimum_required(VERSION 2.6)
project(STRUCTUREFROMMOTION)
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    find_path(WORKSPACE_CMAKE_ROOT Workspace_CMake_Root.txt
              HINTS ${CMAKE_SOURCE_DIR} ENV WORKSPACE_CMAKE_ROOT
              DOC "Directory containing the Workspace_CMake_Root.txt file")
    if (NOT WORKSPACE_CMAKE_ROOT)
        message(FATAL_ERROR "Please set WORKSPACE_CMAKE_ROOT to the directory containing the file Workspace_CMake_Root.txt")
    endif()
    include( ${WORKSPACE_CMAKE_ROOT}/CMakeLists.txt NO_POLICY_SCOPE )
endif()
if (USE_QT5)
	find_package(Qt5Core)
	find_package(Qt5Widgets)
	set(QT_LIBRARIES Qt5::Core;Qt5::Widgets)
else()
	find_package(Qt4 COMPONENTS QtCore QtGui)
	include(${QT_USE_FILE)
endif()

add_subdirectory(brisk)

include_directories(${STRUCTUREFROMMOTION_SOURCE_DIR} )
if (NOT ${STRUCTUREFROMMOTION_SOURCE_DIR} STREQUAL ${STRUCTUREFROMMOTION_BINARY_DIR})
    include_directories(${STRUCTUREFROMMOTION_BINARY_DIR})
endif()

set(STRUCTUREFROMMOTION_PLUGIN_VERSION 0.1.0)
string(REGEX MATCH "^[0-9]+" STRUCTUREFROMMOTION_PLUGIN_SOVERSION ${STRUCTUREFROMMOTION_PLUGIN_VERSION})

if (USE_QT5)
    find_package(Qt5Core)
    find_package(Qt5Widgets)
	find_package(Qt5Gui)
    set(QT_LIBRARIES Qt5::Core;Qt5::Widgets;Qt5::Gui)
else()
    find_package(Qt4 COMPONENTS QtCore QtGui)
    include(${QT_USE_FILE})
endif()

set(OpenCV_DIR "" CACHE PATH "THe location of the build directory in your OpenCV directory")
find_package(OpenCV REQUIRED)

set(HEADERS
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/filesindirectory.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/matchfeatures.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/wsmat.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/calibratecamera.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/briskdetection.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/siftdetection.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/structurefrommotionplugin_api.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/structurefrommotionplugin.h
)

set(INSTALL_HEADERS
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/filesindirectory.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/matchfeatures.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/wsmat.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/calibratecamera.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/briskdetection.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/siftdetection.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/structurefrommotionplugin_api.h
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/structurefrommotionplugin.h
)

set(MOC_HEADERS
)

set(SOURCES
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/filesindirectory.cpp
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/matchfeatures.cpp
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/wsmat.cpp
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/calibratecamera.cpp
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/briskdetection.cpp
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/siftdetection.cpp
    ${STRUCTUREFROMMOTION_SOURCE_DIR}/src/structurefrommotionplugin.cpp
)

# The next line is used by the simple application generator wizard
# add_subdirectory(${STRUCTUREFROMMOTION_SOURCE_DIR}/Application)

# The below line can be used to import sub-directories
# include( ${STRUCTUREFROMMOTION_SOURCE_DIR}/SubDir/CMakeLists.txt )

qtx_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

add_definitions(-DSTRUCTUREFROMMOTION_PLUGIN_VERSION=${STRUCTUREFROMMOTION_PLUGIN_VERSION})

include_directories(${OpenCV_INCLUDE_DIRS} ${STRUCTUREFROMMOTION_SOURCE_DIR}/brisk/thirdparty/agast/include)

add_library(structurefrommotionplugin ${SOURCES} ${HEADERS} ${MOC_SOURCES})
target_link_libraries(structurefrommotionplugin meshplugin renderingplugin workspace brisk agast ${OpenCV_LIBS} ${QT_LIBRARIES})

set_target_properties(structurefrommotionplugin PROPERTIES
    DEFINE_SYMBOL RF_EXPORT
    VERSION ${STRUCTUREFROMMOTION_PLUGIN_VERSION}
    SOVERSION ${STRUCTUREFROMMOTION_PLUGIN_SOVERSION}
)

setTargetOutputDirectory(structurefrommotionplugin ${CSIRO_INSTALL_AREA}/lib/Plugins)
#configure_file(pkg-structurefrommotionplugin.cmake ${CSIRO_INSTALL_AREA}/cmake/Exports/pkg-structurefrommotionplugin.cmake @ONLY)

# Copy our install headers into the install directory so that others can build against our plugin.
foreach(inFile ${INSTALL_HEADERS})
    string(REGEX REPLACE "(${STRUCTUREFROMMOTION_SOURCE_DIR}/)(.*)" "${CSIRO_INSTALL_AREA}/include/StructureFromMotion/\\2" outFile "${inFile}")
    configure_file(${inFile} ${outFile} COPYONLY)
endforeach(inFile)
