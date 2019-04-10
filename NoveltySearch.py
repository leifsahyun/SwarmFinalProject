'''
Implements the novelty search algorithm for swarm behavior discovery.
Assumes the robots have two motors, a 1-bit mutable state, and a line-of-sight sensor that can
detect robots and walls and see the state of other robots. Each generation, this program produces
a population of robot controllers and outputs them to a file. An oracle script is activated that
produces a file containing arbitrary length feature vectors based on global swarm behavior. The
novelty search uses these feature vectors to evaluate the novelty of each controller for evolution.
Author: Leif Sahyun
04/10/2019
'''
import numpy.random as rng
import csv
import subprocess

pop_size = 100
num_gens = 10
oracle = "bash oracle.sh"

class Controller:
    def __init__(self, vector = [
        # each robot response is a tuple of (left wheel speed, right wheel speed, probability to change state)
        # robot responses when robot is in state 0
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        # robot responses when robot is in state 1
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        ]):
        self.vector = vector

    def get_response(this, state, sensor_value):
        # state is 0 or 1
        # sensor_value is 0-3
        if state==0:
            return self.vector[sensor_value]
        else:
            return self.vector[sensor_value+4]

    def __repr__(self):
        return str(self.vector)

population = [ Controller(
    list((rng.uniform(), rng.uniform(), rng.uniform()) for i in range(8))
    ) for j in range(pop_size) ]

for generation in range(num_gens):
    with open('population.csv', 'w') as output:
        writer = csv.writer(output)
        for controller in population:
            writer.writerow(controller.vector)
    
    subprocess.run(oracle)
    
    with open('features.csv', 'r') as input:
        # evaluate the feature results of the population, generate new generation
