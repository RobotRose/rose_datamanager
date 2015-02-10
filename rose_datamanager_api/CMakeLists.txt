cmake_minimum_required(VERSION 2.8.3)
project(rose_datamanager_api)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC")

find_package(catkin REQUIRED
	COMPONENTS
	luctor_classes
	roscomm
	roscpp
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rose_datamanager_api
  CATKIN_DEPENDS roscomm luctor_classes 
  DEPENDS roscpp
)

add_library(rose_datamanager_api SHARED src/rose_datamanager_api.cpp )

target_link_libraries(rose_datamanager_api ${catkin_LIBRARIES})