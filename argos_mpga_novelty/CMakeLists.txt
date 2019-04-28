add_executable(mpga_novelty main.cpp)
target_link_libraries(mpga_novelty
  footbot_nn
  mpga_novelty_loop_functions
  argos3core_simulator)
