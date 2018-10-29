cmake_minimum_required(VERSION 2.8)

project(slam_one)


set( CMAKE_CXX_COMPILER "g++" )
#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11  -O3" )
message( ${CMAKE_CXX_FLAGS})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

######################################libs###########################################
include_directories( "/usr/include/eigen3" )
#sophus

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
#opencv
find_package( OpenCV 3.3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#g2o
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
#
Find_Package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
###
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
#set the lib
set( THIRED_LIBS
        ${Sophus_LIBRARIES}
        ${OpenCV_LIBS}
        ${CHOLMOD_LIBRARIES}
        g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_solver_cholmod
        ${Pangolin_LIBRARIES})

#####################################################################################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
####################################################################################


