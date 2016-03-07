add_executable(rosmariokart.exe rosmariokart.cpp
 ../include/rosmariokart/rosmariokart.h)
target_link_libraries(rosmariokart.exe ${catkin_LIBRARIES} ${OpenCV_LIBS})
