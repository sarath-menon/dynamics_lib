# ###############################################################################
# # Library of structural elements
# ###########################################################

add_library(structural_lib STATIC
arm/arm.cpp
quadcopter_frame/quadcopter_frame.cpp
quadcopter_frame_plus/quadcopter_frame_plus.cpp
quadcopter_frame_x/quadcopter_frame_x.cpp
)

target_include_directories(structural_lib PUBLIC 
arm
quadcopter_frame
quadcopter_frame_plus
quadcopter_frame_x
)

target_link_libraries(structural_lib PRIVATE
primitives_lib
)

