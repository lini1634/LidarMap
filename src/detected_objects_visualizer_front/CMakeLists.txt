cmake_minimum_required(VERSION 2.8.3)
project(detected_objects_visualizer_front)

find_package(catkin REQUIRED COMPONENTS
        
        roscpp
        autoware_msgs
        geometry_msgs
        sensor_msgs
        tf
        visualization_msgs
        cv_bridge
        image_transport
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        autoware_msgs
        geometry_msgs
        sensor_msgs
        tf
        visualization_msgs
        cv_bridge
        image_transport
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

#Rviz Marker visualization
add_executable(visualize_detected_objects_front
        include/visualize_detected_objects.h
        src/visualize_detected_objects_main.cpp
        src/visualize_detected_objects.cpp
        )

target_include_directories(visualize_detected_objects_front PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        include
        )

target_link_libraries(visualize_detected_objects_front
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_dependencies(visualize_detected_objects_front
        ${catkin_EXPORTED_TARGETS}
        )


#Image Rect Visualization
add_executable(visualize_rects_front
        include/visualize_rects.h
        src/visualize_rects_main.cpp
        src/visualize_rects.cpp
        )

target_include_directories(visualize_rects_front PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        include
        )

target_link_libraries(visualize_rects_front
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_dependencies(visualize_rects_front
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS
        visualize_detected_objects_front visualize_rects_front
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
