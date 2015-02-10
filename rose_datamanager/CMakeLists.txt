cmake_minimum_required(VERSION 2.8.3)
project(datamanager)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS
	luctor_classes
	message_generation
	message_runtime
	roscpp
	std_msgs
	sql_database
)

catkin_package(
   	INCLUDE_DIRS 
   		include
#  	LIBRARIES rose_postgresql_database
	CATKIN_DEPENDS 
		luctor_classes
  	DEPENDS 
  		roscpp 
  		std_msgs 
  		sql_database
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( datamanager 	src/datamanager_node.cpp
								src/datamanager.cpp
								src/database_table_base.cpp
)

target_link_libraries( datamanager ${catkin_LIBRARIES} )