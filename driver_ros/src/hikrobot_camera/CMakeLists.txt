cmake_minimum_required(VERSION 2.8.3)

project(hikrobot_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package(
 
)

find_package(OpenCV 4.5.4 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  /home/tjurm/Downloads/HikVision-Camera-Tool-master/camera/hikvision/include
)

LINK_DIRECTORIES(
  /home/tjurm/Downloads/HikVision-Camera-Tool-master/camera/hikvision/64
)

add_executable(${PROJECT_NAME}
src/hikrobot_camera.cpp)

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   GCBase_gcc421_v3_0
   MvCameraControl
   GenApi_gcc421_v3_0
   MVGigEVisionSDK
   log4cpp_gcc421_v3_0
   MVRender
   Log_gcc421_v3_0
   MvUsb3vTL
   MathParser_gcc421_v3_0
   NodeMapData_gcc421_v3_0
   MediaProcess
   XmlParser_gcc421_v3_0
   X11
 )
