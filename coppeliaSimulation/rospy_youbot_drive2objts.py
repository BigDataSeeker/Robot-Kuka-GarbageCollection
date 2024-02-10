#!/usr/bin/env python3
import copy
import math
import sys
sys.path.append('/home/maxim/.local/lib/python3.8/site-packages')
import rospy
from std_msgs.msg import Float32,Bool,Float32MultiArray
import random
import pandas as pd
import operator
import numpy as np
import copy
import time


class City:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def distance(self, city):
        xDis = abs(self.x - city.x)
        yDis = abs(self.y - city.y)
        distance = np.sqrt((xDis ** 2) + (yDis ** 2))
        return distance

    def __repr__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"


class Fitness:
    def __init__(self, route):
        self.route = route
        self.distance = 0
        self.fitness = 0.0

    def routeDistance(self):
        if self.distance == 0:
            pathDistance = 0
            for i in range(0, len(self.route)):
                fromCity = self.route[i]
                toCity = None
                if i + 1 < len(self.route):
                    toCity = self.route[i + 1]
                else:
                    toCity = self.route[0]
                pathDistance += fromCity.distance(toCity)
            self.distance = pathDistance
        return self.distance

    def routeFitness(self):
        if self.fitness == 0:
            self.fitness = 1 / float(self.routeDistance())
        return self.fitness


def createRoute(cityList):
    route = random.sample(cityList[1:], len(cityList) - 1)
    return [cityList[0]] + route


def initialPopulation(popSize, cityList):
    population = []

    for i in range(0, popSize):
        population.append(createRoute(cityList))

    return population


def rankRoutes(population):
    fitnessResults = {}
    for i in range(0, len(population)):
        fitnessResults[i] = Fitness(population[i]).routeFitness()
    return sorted(fitnessResults.items(), key=operator.itemgetter(1), reverse=True)


def selection(popRanked, eliteSize):
    selectionResults = []
    df = pd.DataFrame(np.array(popRanked), columns=["Index", "Fitness"])
    df['cum_sum'] = df.Fitness.cumsum()
    df['cum_perc'] = 100 * df.cum_sum / df.Fitness.sum()

    for i in range(0, eliteSize):
        selectionResults.append(popRanked[i][0])
    for i in range(0, len(popRanked) - eliteSize):
        pick = 100 * random.random()
        for i in range(0, len(popRanked)):
            if pick <= df.iat[i, 3]:
                selectionResults.append(popRanked[i][0])
                break
    return selectionResults


def matingPool(population, selectionResults):
    matingpool = []
    for i in range(0, len(selectionResults)):
        index = selectionResults[i]
        matingpool.append(population[index])
    return matingpool


def breed(parent1, parent2):
    child = []
    childP1 = []
    childP2 = []

    geneA = int(random.random() * len(parent1[1:]))
    geneB = int(random.random() * len(parent1[1:]))

    startGene = min(geneA, geneB)
    endGene = max(geneA, geneB)

    for i in range(startGene, endGene):
        childP1.append(parent1[1:][i])

    childP2 = [item for item in parent2[1:] if item not in childP1]

    child = [parent1[0]] + childP1 + childP2
    return child


def breedPopulation(matingpool, eliteSize):
    children = []
    length = len(matingpool) - eliteSize
    pool = random.sample(matingpool, len(matingpool))

    for i in range(0, eliteSize):
        children.append(matingpool[i])

    for i in range(0, length):
        child = breed(pool[i], pool[len(matingpool) - i - 1])
        children.append(child)
    return children


def mutate(individual, mutationRate):
    for swapped in range(1, len(individual)):
        if (random.random() < mutationRate):
            swapWith = int(random.random() * len(individual))
            if swapWith == 0:
                swapWith += 1

            city1 = individual[swapped]
            city2 = individual[swapWith]

            individual[swapped] = city2
            individual[swapWith] = city1
    return individual


def mutatePopulation(population, mutationRate):
    mutatedPop = []

    for ind in range(0, len(population)):
        mutatedInd = mutate(population[ind], mutationRate)
        mutatedPop.append(mutatedInd)
    return mutatedPop


def nextGeneration(currentGen, eliteSize, mutationRate):
    popRanked = rankRoutes(currentGen)
    selectionResults = selection(popRanked, eliteSize)
    matingpool = matingPool(currentGen, selectionResults)
    children = breedPopulation(matingpool, eliteSize)
    nextGeneration = mutatePopulation(children, mutationRate)
    return nextGeneration


def geneticAlgorithmPlot(population, popSize, eliteSize, mutationRate, generations):
    pop = initialPopulation(popSize, population)
    progress = []
    progress.append(1 / rankRoutes(pop)[0][1])

    for i in range(0, generations):
        pop = nextGeneration(pop, eliteSize, mutationRate)
        progress.append(1 / rankRoutes(pop)[0][1])

    plt.plot(progress)
    plt.ylabel('Distance')
    plt.xlabel('Generation')
    plt.show()


def geneticAlgorithm(population, popSize, eliteSize, mutationRate, generations):
    pop = initialPopulation(popSize, population)
    print("Initial distance: " + str(1 / rankRoutes(pop)[0][1]))

    for i in range(0, generations):
        pop = nextGeneration(pop, eliteSize, mutationRate)

    print("Final distance: " + str(1 / rankRoutes(pop)[0][1]))
    bestRouteIndex = rankRoutes(pop)[0][0]
    bestRoute = pop[bestRouteIndex]
    return bestRoute

#points = geneticAlgorithm(population=pointList, popSize=3, eliteSize=2, mutationRate=0.55, generations=50)

def current_time(msg):
    c_time =  msg.data
    global current_bowl_idx
    global bowls_order2drive
    rospy.loginfo(f'Current bowl idx is {current_bowl_idx}')

    if current_bowl_idx == 0:
        bowls_order2drive = copy.deepcopy(bowls_pos)

    rel_bowl_pos = [bowls_order2drive[0].x - youbot_pos.data[0],bowls_order2drive[0].y - youbot_pos.data[1],bowls_order2drive[0].z - youbot_pos.data[2]]
    rospy.loginfo(f'Current bowl relative to youbot position - {rel_bowl_pos}')
    angle_dir_diff = -1*math.atan2(rel_bowl_pos[0], rel_bowl_pos[1])
    target_gl_youbot_dir = angle_dir_diff
    rospy.loginfo(f'Global target direction is {target_gl_youbot_dir}')

    for_speed = 0
    side_speed = 0
    rot_speed = math.copysign(0.5,youbot_ori.data[2] - target_gl_youbot_dir)

    if abs(target_gl_youbot_dir - youbot_ori.data[2]) < 0.03:
        rot_speed = 0
        for_speed = 0.6 * (math.sqrt(rel_bowl_pos[0]**2+rel_bowl_pos[1]**2) - 0.3)
        if (math.sqrt(rel_bowl_pos[0]**2+rel_bowl_pos[1]**2) < 0.6):
            current_bowl_idx += 1
            if len(bowls_order2drive) == 1:
                rospy.loginfo('Stop! All objects are visited')
                pub_for_vel.publish(0)
                pub_side_vel.publish(0)
                pub_rot_vel.publish(0)
                exit()
            bowls_order2drive = geneticAlgorithm(population=bowls_order2drive, popSize=4, eliteSize=2, mutationRate=0.55, generations=50)[1:]


    #pub_rot_arm_mov.publish(arm_rot_spd)
    #pub_for_arm_mov.publish(arm_for_spd)
    rospy.loginfo(f'Current target velocity - {for_speed,side_speed,rot_speed}')
    pub_for_vel.publish(for_speed)
    pub_side_vel.publish(side_speed)
    pub_rot_vel.publish(rot_speed)


def get_youbot_pos(msg):
    global youbot_pos
    youbot_pos.data[0] = msg.data[0]
    youbot_pos.data[1] = msg.data[1]
    youbot_pos.data[2] = msg.data[2]
    bowls_pos[0].x = msg.data[0]
    bowls_pos[0].y = msg.data[1]
    bowls_pos[0].z = msg.data[2]
    #rospy.loginfo(f'Current youbot position - {youbot_pos}')

def get_bowl0_pos(msg):
    global bowl0_pos
    bowls_pos[1].x= msg.data[0]
    bowls_pos[1].y = msg.data[1]
    bowls_pos[1].z = msg.data[2]
    #rospy.loginfo(f'Current bowl position - {bowl_pos}')

def get_bowl1_pos(msg):
    global bowl1_pos
    bowls_pos[2].x = msg.data[0]
    bowls_pos[2].y = msg.data[1]
    bowls_pos[2].z = msg.data[2]
    #rospy.loginfo(f'Current bowl position - {bowl_pos}')

def get_bowl2_pos(msg):
    global bowl0_pos
    bowls_pos[3].x= msg.data[0]
    bowls_pos[3].y = msg.data[1]
    bowls_pos[3].z = msg.data[2]
    #rospy.loginfo(f'Current bowl position - {bowl_pos}')

def get_bowl3_pos(msg):
    global bowl0_pos
    bowls_pos[4].x= msg.data[0]
    bowls_pos[4].y = msg.data[1]
    bowls_pos[4].z = msg.data[2]
    #rospy.loginfo(f'Current bowl position - {bowl_pos}')

def get_youbot_ori(msg):
    global youbot_ori
    youbot_ori.data[0] = msg.data[0]
    youbot_ori.data[1] = msg.data[1]
    youbot_ori.data[2] = msg.data[2]
    rospy.loginfo(f'Current youbot_orientation - {youbot_ori}')

if __name__ == '__main__':
    rospy.init_node('youbot_driver')
    rospy.loginfo("youbot_driver has been started")
    rate = rospy.Rate(50)
    current_bowl_idx = 0

    bowls_order2drive = None

    youbot_pos = Float32MultiArray()
    youbot_pos.data.append(0.001)
    youbot_pos.data.append(0.001)
    youbot_pos.data.append(0.001)
    bowls_pos = [City(0,0,0),City(0,0,0),City(0,0,0),City(0,0,0),City(0,0,0)]
    youbot_ori = Float32MultiArray()
    youbot_ori.data.append(0.001)
    youbot_ori.data.append(0.001)
    youbot_ori.data.append(0.001)

    pub_for_vel = rospy.Publisher("froward_Speed",Float32,queue_size = 10)
    pub_side_vel = rospy.Publisher("side_Speed", Float32, queue_size=10)
    pub_rot_vel = rospy.Publisher("rot_Speed", Float32, queue_size=10)
    #pub_rot_arm_mov = rospy.Publisher("rot_arm_mov", Float32, queue_size=10)
    #pub_for_arm_mov = rospy.Publisher("for_arm_mov", Float32, queue_size=10)
    youbot_pos_sub = rospy.Subscriber('youBot_position', Float32MultiArray, callback=get_youbot_pos)
    bowl0_pos_sub = rospy.Subscriber('bowl0_position', Float32MultiArray, callback= get_bowl0_pos)
    bowl1_pos_sub = rospy.Subscriber('bowl1_position', Float32MultiArray, callback=get_bowl1_pos)
    bowl2_pos_sub = rospy.Subscriber('bowl2_position', Float32MultiArray, callback=get_bowl2_pos)
    bowl3_pos_sub = rospy.Subscriber('bowl3_position', Float32MultiArray, callback=get_bowl3_pos)
    youbot_ori_sub = rospy.Subscriber('youBot_orientation', Float32MultiArray, callback=get_youbot_ori)
    time_sub = rospy.Subscriber('simTime', Float32, callback=current_time)

    rospy.spin()