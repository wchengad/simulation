cmake_minimum_required(VERSION 2.8.3)
project(virtual_uav)


find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	rospy
	cv_bridge
	std_msgs
	message_generation
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /usr/local/include)

add_message_files(
	FILES
	FlightControlData.msg
	GlobalPosition.msg
	LocalPosition.msg
)

generate_messages(
	 DEPENDENCIES
	 std_msgs
)


catkin_package(
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	/usr/include
	/usr/local/include
	/usr/include/opencv
	/usr/include/opencv2
	${PROJECT_SOURCE_DIR}
	src/common/
	src/tracking/
)

link_directories(
		/usr/local/lib)

aux_source_directory(${PROJECT_SOURCE_DIR}/sdk POSE_ESTIMATE_PROJECT_SOURCE)


add_executable(virtual_uav_node src/main.cpp
	${PROJECT_SOURCE_DIR}/src/common/common_utility.cpp
	${PROJECT_SOURCE_DIR}/src/common/network.cpp
	)



target_link_libraries(virtual_uav_node
	libzmq.so 
	usb-1.0 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

