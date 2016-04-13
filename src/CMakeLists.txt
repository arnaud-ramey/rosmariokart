add_executable(rosmariokart.exe rosmariokart.cpp
  ../include/rosmariokart/rosmariokart.h
  ../include/rosmariokart/sdl_utils.h
  ../include/rosmariokart/timer.h
)
target_link_libraries(rosmariokart.exe ${catkin_LIBRARIES}
  ${SDL2_LIBRARY} SDL2_gfx SDL2_image SDL2_ttf SDL2_mixer)

add_executable(mip_animations.exe mip_animations.cpp)
target_link_libraries(mip_animations.exe ${catkin_LIBRARIES})

add_executable(sumo_animations.exe sumo_animations.cpp)
target_link_libraries(sumo_animations.exe ${catkin_LIBRARIES})
