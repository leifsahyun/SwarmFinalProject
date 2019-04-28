add_executable(experiment_1 main.cpp)
target_link_libraries(experiment_1
  #footbot_nn
  mpga
  experiment1_loop_functions
  argos3core_simulator)
