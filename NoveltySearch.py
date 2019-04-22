'''
Implements the novelty search algorithm for swarm behavior discovery.
Assumes the robots have two motors, a 1-bit mutable state, and a line-of-sight sensor that can
detect robots and walls and see the state of other robots. Each generation, this program produces
a population of robot controllers and outputs them to a file. An oracle script is activated that
produces a file containing arbitrary length feature vectors based on global swarm behavior. The
novelty search uses these feature vectors to evaluate the novelty of each controller for evolution.
Author: Leif Sahyun
04/22/2019
Dependencies:
 - Numpy
 - DEAP
 - Matplotlib
'''
import numpy.random as rng
import numpy
import csv
import subprocess
import math
from deap import base, creator, tools
import matplotlib.pyplot as plt

pop_size = 10
num_gens = 100
oracle = "bash oracle.sh"
default_k = 15
controller_size = 24
crossover_probability = 0.5
mutation_probability = 0.2

# computes the distance between two feature vectors
def dist(feature1, feature2):
    total = 0
    for i in range(len(feature1)):
        total = total + math.pow(feature1[i] - feature2[i], 2.0)
    return math.sqrt(total)

# gets the k nearest neighbors of a feature vector in the archive
def get_k_neighbors(k, feature, archive):
    neighbors = sorted(archive, key=lambda n: dist(feature, n.features))
    return neighbors[:k]

# writes the current population to a file
def export_population(population):
    with open('population.csv', 'w', newline='') as output_file:
        writer = csv.writer(output_file)
        for controller in population:
            writer.writerow(controller)
    
# reads the feature vectors from a file
def import_features():
    features = []
    with open('features.csv', 'r', newline='') as input_file:
        # evaluate the feature results of the population, generate new generation
        reader = csv.reader(input_file)
        for row in reader:
            features.append(list(map(float,row)))
    return features

# associates feature vectors with the population
def associate_features(population, features):
    for i in range(len(population)):
        population[i].features = features[i]
    
# evaluates the features, associating novelty values with the population
def eval_features(population, archive):
    for individual in population:
        k_neighbors = get_k_neighbors(default_k, individual.features, archive)
        novelty = 1.0/default_k * sum(map(lambda n: dist(individual.features, n.features), k_neighbors))
        individual.novelty = novelty

def analyze(archive):
    for i in range(24):        
        elements = list(map(lambda c: c.features[i], archive))
        plt.hist(elements, 20)#to_plot)
        plt.show()

# runs the novelty search        
def run():
    # DEAP population creation
    toolbox = base.Toolbox()
    creator.create("Controller", list, novelty=0.0, features=list)
    toolbox.register("individual", tools.initRepeat,
                     container=creator.Controller, func=rng.uniform, n=controller_size)
    toolbox.register("population", tools.initRepeat,
                     container=list, func=toolbox.individual)
    pop = toolbox.population(n=pop_size)
    archive = []
    # DEAP toolbox registration
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.1)
    toolbox.register("select", tools.selTournament, tournsize=3, fit_attr='novelty')
    # actual novelty search
    for generation in range(num_gens):
        print("Generation", generation)
        # use the oracle to find population feature vectors
        export_population(pop)
        subprocess.check_call(oracle)
        features = import_features()
        associate_features(pop, features)
        # add the current generation to the archive for comparison
        archive.extend(pop)
        # evaluate novelty
        eval_features(pop, archive)
        # create new generation with DEAP genetic toolbox
        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))
        # pairing every other child in offspring for possible crossover
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if rng.uniform() < crossover_probability:
                toolbox.mate(child1, child2)
        # mutate offspring
        for mutant in offspring:
            if rng.uniform() < mutation_probability:
                toolbox.mutate(mutant)
            # reset novelty and features
            mutant.novelty = 0.0
            mutant.features = []
        # update the population
        pop[:] = offspring
    # display results
    analyze(archive)

if __name__=="__main__":
    run()
