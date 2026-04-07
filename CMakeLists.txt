cmake_minimum_required(VERSION 2.8.3)
project(m2cs_player)

add_definitions(-std=c++17)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE release)
endif ()

find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  rosbag
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
  pcl_msgs
  camera_info_manager
  tf
)
set(CMAKE_AUTOMOC ON)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

#set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)
#find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  m2cs_player
  CATKIN_DEPENDS 
    roscpp rospy 
    std_msgs 
    geometry_msgs 
    image_transport 
    cv_bridge  
    pcl_ros
    pcl_conversions  
    pcl_msgs
    camera_info_manager
    tf
  # DEPENDS 
  #   Eigen
  #   PCL
  #   OpenCV
)

set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set (qt_lib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set (qt_lib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/ROSThread.h)
set (qt_lib_ui  ${SRC_DIR}/mainwindow.ui)
set (qt_bin_src ${SRC_DIR}/main.cpp)

#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${m2cs_player_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
  include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

qt5_wrap_ui (qt_lib_ui_moc ${qt_lib_ui})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

###########
## Build ##
###########

add_executable(${PROJECT_NAME} ${qt_lib_src} 
                               ${qt_lib_hdr}
                               ${qt_bin_src} 
                               ${SHADER_RSC_ADDED} 
                               ${qt_lib_ui_moc})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} 
                                      ${QT_LIBRARIES}  
                                      ${OPENGL_LIBRARIES} 
                                      ${BOOST_CUSTOM_LIBS} 
                                      ${Eigen_LIBRARIES}
                                      ${PCL_LIBRARIES} 
                                      ${OpenCV_LIBRARIES}
                                      Qt5::Widgets 
                                      Qt5::Gui 
                                      Qt5::OpenGL 
                                      GL)
