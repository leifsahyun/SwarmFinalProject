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
import math

pop_size = 100
num_gens = 10
oracle = "bash oracle.sh"
default_k = 15

# represents a controller genome in the population
class Controller:
    def __init__(self, vector = [
        # each robot response is a tuple of (left wheel speed, right wheel speed, probability to change state)
        # robot responses when robot is in state 0
        0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
        # robot responses when robot is in state 1
        0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0
        ]):
        self.vector = vector
        self.novelty = -1

    def get_response(this, state, sensor_value):
        # state is 0 or 1
        # sensor_value is 0-3
        index = 3*sensor_value
        if state==0:
            return (self.vector[index], self.vector[index+1], self.vector[index+2])
        else:
            return (self.vector[12+index], self.vector[12+index+1], self.vector[12+index+2])

    def __repr__(self):
        return str(self.vector)

# computes the distance between two feature vectors
def dist(feature1, feature2):
    total = 0
    for i in range(len(feature1)):
        total = total + math.pow(feature1[i] - feature2[i], 2.0)
    return math.sqrt(total)

# gets the k nearest neighbors of a feature vector in a list
def get_k_neighbors(k, feature, feature_list):
    neighbors = sorted(feature_list, key=lambda n: dist(feature, n))
    return neighbors[:k]

# writes the current population to a file
def export_population(population):
    with open('population.csv', 'w', newline='') as output_file:
        writer = csv.writer(output_file)
        for controller in population:
            writer.writerow(controller.vector)
    
# reads the feature vectors from a file
def import_features():
    features = []
    with open('features.csv', 'r', newline='') as input_file:
        # evaluate the feature results of the population, generate new generation
        reader = csv.reader(input_file)
        for row in reader:
            features.append(list(map(float,row)))
    return features
    
# evaluates the features, associating novelty values with the population
def eval_features(population, features, archive):
    for i in range(len(features)):
        k_neighbors = get_k_neighbors(default_k, features[i], archive)
        novelty = 1.0/default_k * sum(map(lambda n: dist(features[i], n), k_neighbors))
        population[i].novelty = novelty

# runs the novelty search        
def run():
    archive = []
    population = [ Controller(
        list(rng.uniform() for i in range(24))
        ) for j in range(pop_size) ]
    for generation in range(num_gens):
        print("Generation", generation)
        export_population(population)
        subprocess.check_call(oracle)
        features = import_features()
        archive.extend(features)
        eval_features(population, features, archive)
        for c in population:
            print(c.novelty, end=', ')
        # genetic algorithm implementation goes here

if __name__=="__main__":
    run()
