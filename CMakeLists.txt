cmake_minimum_required(VERSION 3.0.2)
project(dogrun-circle)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dogrun
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dogrun-circle src/dogrun_circle.cpp)
target_link_libraries(dogrun-circle
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(dogrun-yolo src/sokudotyousei.cpp)
target_link_libraries(dogrun-yolo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(meat-dogrun src/meat_dogrun.cpp)
target_link_libraries(meat-dogrun
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(house-dogrun src/house_dogrun.cpp)
target_link_libraries(house-dogrun
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)