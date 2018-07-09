set(PROJECT_NAME lightweight_slam)
cmake_minimum_required(VERSION 2.8)
project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(G2O REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)

set(Astra_SDK_DIR /home/gingerli/Tools/AstraSDK-v2.0.9-beta3-Linux/)
set(Astra_INCLUDE_DIR ${Astra_SDK_DIR}/include/)
set(Astra_LIBS 
    ${Astra_SDK_DIR}/lib/libastra.so
    ${Astra_SDK_DIR}/lib/libastra_core.so
    ${Astra_SDK_DIR}/lib/libastra_core_api.so
)

set(OPENNI2_INCLUDE_DIR /home/gingerli/Tools/OpenNI2-ORBBEC/Linux/OpenNI-Linux-x64-2.3/Include)
set(OPENNI2_LIBRARIES
  /home/gingerli/Tools/OpenNI2-ORBBEC/Linux/OpenNI-Linux-x64-2.3/Redist/libOpenNI2.so
  /home/gingerli/Tools/OpenNI2-ORBBEC/Linux/OpenNI-Linux-x64-2.3/Redist/OpenNI2/Drivers/libOniFile.so
  /home/gingerli/Tools/OpenNI2-ORBBEC/Linux/OpenNI-Linux-x64-2.3/Redist/OpenNI2/Drivers/liborbbec.so
  /home/gingerli/Tools/OpenNI2-ORBBEC/Linux/OpenNI-Linux-x64-2.3/Redist/OpenNI2/Drivers/libPSLink.so
)

include_directories(
  ${OPENNI2_INCLUDE_DIR}
  ${Astra_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src/camera/
)

set(PROJECT_SOURCE_FILES  
  ${PROJECT_SOURCE_DIR}/src/camera/AstraRgbdCamera.cpp
  ${PROJECT_SOURCE_DIR}/src/camera/OpenNI2Camera.cpp
  ${PROJECT_SOURCE_DIR}/src/main.cpp
)

Message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
Message("G2O: Include: ${G2O_INCLUDE_DIR} Libraries: ${G2O_LIBRARIES}")
Message("Ceres: ${CERES_INCLUDE_DIRS}")
Message("Eigen: ${EIGEN3_INCLUDE_DIR}")
Message("OpenCV: Include: ${OpenCV_INCLUDE_DIRS} Libraries: ${OpenCV_LIBS}")
Message("Boost: ${Boost_INCLUDE_DIR}")
Message("Pangolin: ${Pangolin_INCLUDE_DIR}")
#Message("OpenNI2: ${OPENNI2_INCLUDE_DIR}")
message(STATUS "Astra_SDK_DIR = ${Astra_SDK_DIR}")
message(STATUS "Astra_INCLUDE_DIR = ${Astra_INCLUDE_DIR}")
message(STATUS "Astra_LIBS = ${Astra_LIBS}")

add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${Pangolin_LIBRARIES} ${OPENNI2_LIBRARIES} ${Astra_LIBS})

add_executable(slam_executor src/main.cpp)
target_link_libraries(slam_executor ${PROJECT_NAME})