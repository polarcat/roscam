cmake_minimum_required(VERSION 3.5)

project(roscam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(PRINT_FPS)
	add_definitions(-DPRINT_FPS)
endif(PRINT_FPS)

include_directories(include)

file(GLOB SOURCES
    include/*.h
    src/*.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} v4l2)

ament_target_dependencies(${PROJECT_NAME}
	rclcpp sensor_msgs rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

# ros2 looks for package executables under /usr/lib/${PROJECT_NAME}/
install(TARGETS
	${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
