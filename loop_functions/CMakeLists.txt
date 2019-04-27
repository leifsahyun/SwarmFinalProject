# Compile MPGA library (generic library for multi-process genetic algorithm)
add_library(mpga SHARED
  mpga.h mpga.cpp
  mpga_loop_functions.h mpga_loop_functions.cpp)
target_link_libraries(mpga
  argos3core_simulator)
if(NOT APPLE)
target_link_libraries(mpga rt)
endif(NOT APPLE)

# Compile phototaxis example
add_library(experiment1_loop_functions SHARED
  experiment1_loop_functions.h experiment1_loop_functions.cpp)

target_link_libraries(experiment1_loop_functions
  #footbot_nn
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  mpga)
