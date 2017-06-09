cmake_minimum_required(VERSION 2.4.6)
#########################################################
### USER CONFIG #########################################
#########################################################
set(USE_SIFT_GPU        1) 
set(ENV{SIFT_GPU_MODE}	2) #CUDA = 1, GLSL = 2
set(ENV{SIFT_GPU_CUDA_PATH}	/usr/local/cuda)	
set(USE_GICP_BIN		0)
set(USE_GICP_CODE		0)
set(USE_PCL_ICP		  0)
set(USE_GL2PS       0) #For printing the 3D Window to PostScript - requires installation of additional library: libgl2ps-dev
IF ("${ROS_PARALLEL_JOBS}" EQUAL "")
  set(ROS_PARALLEL_JOBS 1)
endif ("${ROS_PARALLEL_JOBS}" EQUAL "")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#SET(CMAKE_CXX_FLAGS "-O3 -fPIC -std=c++0x")
#For annotated profiling with perf: 
# SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -std=c++0x")
SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -std=c++0x")
# SET(CMAKE_CXX_FLAGS "-ggdb -O3 -std=c++0x")

IF (${USE_GL2PS})
  add_definitions(-DGL2PS)
ENDIF (${USE_GL2PS})
#is used for compiling the libraries 
set(USE_GICP 0)
IF (${USE_GICP_BIN} OR ${USE_GICP_CODE})
	set(USE_GICP 1)
ENDIF (${USE_GICP_BIN} OR ${USE_GICP_CODE})
#########################################################
#########################################################


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
####################################################
# ROS
####################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
#message(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp image_transport cv_bridge sensor_msgs 
             geometry_msgs visualization_msgs std_msgs tf message_generation message_filters rosbag rosconsole)

project(rgbdslam)

#add_definitions(-DROSCONSOLE_SEVERITY_INFO)

add_service_files(DIRECTORY srv FILES rgbdslam_ros_ui.srv rgbdslam_ros_ui_b.srv rgbdslam_ros_ui_f.srv rgbdslam_ros_ui_s.srv) 
generate_messages()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs 
  DEPENDS message_runtime std_msgs 
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/lib)
# link_libraries(rgbdslam ${catkin_LIBRARIES})

####################################################
# Required 3rd party
####################################################
# PCL #######################
set(PCL_DIR "/home/davidz/work/3rdlibs/pcl_1.7.1_src/pcl-pcl-1.7.1/build")
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# Octomap ###################
find_package(octomap 1.6 REQUIRED)
include_directories(BEFORE ${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# g2o #######################
set(G2O_ROOT "/home/davidz/work/3rdlibs/g2o/install")
# set(G2O_INCLUDE_DIR "${G2O_DIR}/include")
# set(G2O_LIBRARIES "${G2O_DIR}/lib")
find_package(G2O REQUIRED)
set(G2O_INCLUDE_DIRS "${G2O_ROOT}/include")
include_directories(${G2O_INCLUDE_DIRS})
set(G2O_LIBRARY_DIRS "${G2O_ROOT}/lib")
link_directories(${G2O_LIBRARY_DIRS}) 

# g2o_ext_csparse, if the suitesparse has already been installed, this library will be conflict with the libcsparse
set(G2O_LIBS g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse g2o_stuff g2o_core g2o_csparse_extension  g2o_solver_eigen g2o_solver_dense)
set(G2O_LIBRARIES ${G2O_LIBS} cholmod cxsparse)
# link_libraries(${G2O_LIBRARIES})
message("G2o-libraries "${G2O_LIBRARIES})

# for linking to libcxsparse2.2.3
link_directories("/usr/lib/i386-linux-gnu")
link_directories("/usr/lib/x86_64-linux-gnu")

# Qt ########################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

# OpenMP ########################
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# This is necessary as all ui files etc will get dumped in the bottom of then binary directory. 
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/external/gicp/ann_1.1.2/include/ANN)

#get_directory_property(clean ADDITIONAL_MAKE_CLEAN_FILES)
#SET(clean external/siftgpu/linux/bin/libsiftgpu.so)
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${clean}")
 	
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})


SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})
#SET(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE TRUE)


#Only possible when not subscribing to point cloud topic. Possibly dangerous anyway, but halves storage space per point cloud
#ADD_DEFINITIONS(-DRGB_IS_4TH_DIM) 
#ADD_DEFINITIONS(-DDO_FEATURE_OPTIMIZATION)
#ADD_DEFINITIONS(-DHEMACLOUDS) 
ADD_DEFINITIONS(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

#############################
# SiftGPU ###################
#############################
IF (${USE_SIFT_GPU})
  add_subdirectory(external)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/SiftGPU/src)
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/external/SiftGPU/bin)	
  
  add_definitions(-DUSE_SIFT_GPU)
  add_definitions(-DSIFT_GPU_MODE=$ENV{SIFT_GPU_MODE})
ENDIF (${USE_SIFT_GPU})

#############################
# GICP ######################
#############################
IF (${USE_GICP})
	SET(gicp_build_path ${CMAKE_SOURCE_DIR}/external/gicp/ann_1.1.2)
  FILE(MAKE_DIRECTORY ${gicp_build_path}/lib/)
  MESSAGE("\n------------------------------------------------------------------\n")
	MESSAGE("Compiling GICP...")
	EXECUTE_PROCESS(
	    COMMAND make ${ROS_PARALLEL_JOBS} linux-g++
	    WORKING_DIRECTORY ${gicp_build_path}
	    RESULT_VARIABLE gicp_return
    )
    MESSAGE("\n------------------------------------------------------------------\n")
	
	#on error
	IF (NOT ${gicp_return} EQUAL 0)
		MESSAGE(FATAL_ERROR "GICP cannot be compiled. Returned: ${gicp_return}")
	ENDIF (NOT ${gicp_return} EQUAL 0)
	
	FILE(COPY ${gicp_build_path}/lib/libANN.a DESTINATION ${CMAKE_SOURCE_DIR}/lib/)
ENDIF (${USE_GICP})

IF (${USE_GICP_BIN})
	add_definitions(-DUSE_ICP_BIN)
ENDIF (${USE_GICP_BIN})

IF (${USE_GICP_CODE})
	add_definitions(-DUSE_ICP_CODE) 
ENDIF (${USE_GICP_CODE})
IF (${USE_PCL_ICP})
  add_definitions(-DUSE_PCL_ICP) 
ENDIF (${USE_PCL_ICP})



#############################
# OpenCV ####################
#############################
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#############################
# Eigen ####################
#############################
# find_package(Eigen)
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

##############################################################################
# Some QT Magic (MOC Creation)
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/qtros.h src/openni_listener.h src/qt_gui.h src/graph_manager.h src/glviewer.h src/ros_service_ui.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})  

##############################################################################
# Sources to Compile
##############################################################################
SET(ADDITIONAL_SOURCES src/gicp-fallback.cpp src/qtros.cpp  src/openni_listener.cpp src/qt_gui.cpp src/node.cpp src/glviewer.cpp src/parameter_server.cpp src/ros_service_ui.cpp src/misc.cpp src/landmark.cpp src/loop_closing.cpp src/ColorOctomapServer.cpp src/scoped_timer.cpp src/icp.cpp src/matching_result.cpp)
#GraphManager Files
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/graph_manager.cpp src/graph_mgr_io.cpp src/graph_manager2.cpp)
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/transformation_estimation_euclidean.cpp src/transformation_estimation.cpp src/graph_manager2.cpp src/header.cpp)
#Computation of meaningful covariances
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/covariance_estimation.cpp) # src/edge_labeler.cpp)
#Detector Threshold Adapter
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/feature_adjuster.cpp src/aorb.cpp src/features.cpp)

IF (${USE_SIFT_GPU})
 	SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/sift_gpu_wrapper.cpp)
ENDIF (${USE_SIFT_GPU})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS)

include_directories(external/ /usr/include/suitesparse/)
include_directories(external/SiftGPU/include)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW) ## the libraries libB will not be affected by the full_path /path/to/libA 
endif(COMMAND cmake_policy)

#set libs
#SET(G2O_LIBS cholmod g2o_core g2o_stuff g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse cxsparse g2o_incremental)
#SET(G2O_LIBS ${G2O_LIBRARIES} cholmod cxsparse)
#SET(LIBS_LINK GL GLU -lgl2ps ${G2O_LIBS} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})
#Use specific version of cxsparse, to avoid linker warning about possible conflicts between versions 2.2.3 (linked from g2o) and 3.1.0
# SET(LIBS_LINK GL GLU cholmod /usr/lib/libcxsparse.so.2.2.3 ${G2O_LIBRARIES} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})
message("OPENCV_LIBS: ${OpenCV_LIBS}")
SET(LIBS_LINK GL GLU  ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} ${G2O_LIBRARIES})

#-lboost_signals -lrt -loctomap -loctomap_server -loctomath)

IF (${USE_SIFT_GPU})
 	SET(LIBS_LINK ${LIBS_LINK} siftgpu )
ENDIF (${USE_SIFT_GPU})
IF (${USE_GL2PS})
 	SET(LIBS_LINK ${LIBS_LINK} -lgl2ps)
ENDIF (${USE_GL2PS})
IF (${USE_GICP})
 	SET(LIBS_LINK ${LIBS_LINK} gicp ANN gsl gslcblas)
ENDIF (${USE_GICP})

##############################################################################
# Binaries
##############################################################################


# add_executable(rgbdslam src/main.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${ADDITIONAL_SOURCES})
# add_dependencies(rgbdslam rgbdslam_gencpp)
# target_link_libraries(rgbdslam ${LIBS_LINK} ${catkin_LIBRARIES})

 add_library(rgbdslam_v2 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${ADDITIONAL_SOURCES})
 add_dependencies(rgbdslam_v2 rgbdslam_gencpp)
#link libraries
 target_link_libraries(rgbdslam_v2 ${LIBS_LINK} ${catkin_LIBRARIES})

# add_executable 
 add_executable(rgbdslam src/main.cpp)
 target_link_libraries(rgbdslam rgbdslam_v2) # ${LIBS_LINK})

IF (${USE_GICP})
  set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fpermissive)
  add_library(gicp external/gicp/bfgs_funcs.cpp external/gicp/gicp.cpp external/gicp/optimize.cpp external/gicp/scan.cpp external/gicp/transform.cpp)
ENDIF (${USE_GICP})

# test g2o compilation 
# add_executable(g2o_exp src/g2o_in_rgbdslam.cpp ) #${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${ADDITIONAL_SOURCES} )
# target_link_libraries(g2o_exp ${LIBS_LINK} ${catkin_LIBRARIES})

#debug g2o experiment 
# QT4_WRAP_CPP(QT_g2o_MOC_HPP src/g2o_graph_manager.h)  

#  src/g2o_graph_mgr_io.cpp 
# add_executable(g2o_exp src/g2o_in_rgbdslam.cpp 
#              src/g2o_graph_manager.cpp src/g2o_graph_manager2.cpp 
#              src/g2o_parameter_server.cpp
#               ${QT_g2o_MOC_HPP})
# target_link_libraries(g2o_exp rgbdslam_v2 ${LIBS_LINK} ${catkin_LIBRARIES})
# target_link_libraries(g2o_exp ${G2O_LIBRARIES} ${catkin_LIBRARIES})

###############################################################################
## Bag Viewer
###############################################################################
#
#file(GLOB QT_FORMS2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/qtros.h src/bagloader.h src/qt_gui.h src/glviewer.h)
#
#QT4_ADD_RESOURCES(QT_RESOURCES_CPP2 ${QT_RESOURCES2})
#QT4_WRAP_UI(QT_FORMS_HPP2 ${QT_FORMS2})
#QT4_WRAP_CPP(QT_MOC_HPP2 ${QT_MOC2})  
#
#
#SET(ADDITIONAL_SOURCES2 src/result-bag-viewer.cpp src/bagloader.cpp src/qtros.cpp  src/qt_gui.cpp src/glviewer.cpp src/parameter_server.cpp src/misc.cpp src/ColorOctomapServer.cpp src/scoped_timer.cpp)
#add_executable(bagviewer ${QT_SOURCES2} ${QT_RESOURCES_CPP2} ${QT_FORMS_HPP2} ${QT_MOC_HPP2} ${ADDITIONAL_SOURCES2})
#target_link_libraries(bagviewer ${LIBS_LINK})
# 
