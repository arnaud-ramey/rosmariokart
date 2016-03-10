add_executable(rosmariokart.exe rosmariokart.cpp
  ../include/rosmariokart/rosmariokart.h)
target_link_libraries(rosmariokart.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(mip_animations.exe mip_animations.cpp)
target_link_libraries(mip_animations.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(sumo_animations.exe sumo_animations.cpp)
target_link_libraries(sumo_animations.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})
