cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
     cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

file(GLOB AGAST_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

file(GLOB AGAST_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/include/agast/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/agast)

#build the library dynamic and static versions
IF(BRISK_BUILD_SHARED)
    add_library(agast SHARED ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})
    target_link_libraries(agast ${OpenCV_LIBS} )
ELSE(BRISK_BUILD_SHARED)
    add_library(agast STATIC ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})
    target_link_libraries(agast ${OpenCV_LIBS} )
ENDIF(BRISK_BUILD_SHARED)
