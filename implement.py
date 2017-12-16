import numpy as np
import random
import time
import openravepy
from random import randint
import matplotlib.pyplot as plt

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

import Queue as Q

class Compare(object):
    def __init__(self, cost, config):
        self.cost = cost
        self.config = config
        return
    def __cmp__(self, other):
        return cmp(self.cost, other.cost) #find the smallest cost

STEP = 0.1
limits = [[-10.0, -10.0], [10.0, 10.0]]

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def is_collision(env, robot, config):
    with env:
        robot.SetActiveDOFValues(config)
    waitrobot(robot)
    return env.CheckCollision(robot)

def getNeighbors(location, step = STEP):
    neighbors = []
    neighbors.append([location[0] + step, location[1], 0.05])
    neighbors.append([location[0] - step, location[1], 0.05])
    neighbors.append([location[0] + step, location[1] + step, 0.05])
    neighbors.append([location[0] + step, location[1] - step, 0.05])
    neighbors.append([location[0] - step, location[1] + step, 0.05])
    neighbors.append([location[0] - step, location[1] - step, 0.05])
    neighbors.append([location[0], location[1] + step, 0.05])
    neighbors.append([location[0], location[1] - step, 0.05])
    return neighbors

def getTwoNeighbors(location, step = STEP):
    neighbors = []
    neighbors.append([location[0] + 2 * step, location[1] + step, 0.05])
    neighbors.append([location[0] - 2 * step, location[1] + step, 0.05])
    neighbors.append([location[0] + 2 * step, location[1] - step, 0.05])
    neighbors.append([location[0] - 2 * step, location[1] - step, 0.05])
    neighbors.append([location[0] - step, location[1] + 2 * step, 0.05])
    neighbors.append([location[0] - step, location[1] - 2 * step, 0.05])
    neighbors.append([location[0] + step, location[1] + 2 * step, 0.05])
    neighbors.append([location[0] + step, location[1] - 2 * step, 0.05])
    neighbors.append([location[0] + 2 * step, location[1], 0.05])
    neighbors.append([location[0] - 2 * step, location[1], 0.05])
    neighbors.append([location[0] + 2 * step, location[1] + 2 * step, 0.05])
    neighbors.append([location[0] + 2 * step, location[1] - 2 * step, 0.05])
    neighbors.append([location[0] - 2 * step, location[1] + 2 * step, 0.05])
    neighbors.append([location[0] - 2 * step, location[1] - 2 * step, 0.05])
    neighbors.append([location[0], location[1] + 2 * step, 0.05])
    neighbors.append([location[0], location[1] - 2 * step, 0.05])
    return neighbors

def check_whether_out_of_bound(point, m = 10, n = 10):
    if (point[0] < -10.0 or point[0] > 10.0 or point[1] < -10.0 or point[1] > 10.0):
        return True
    return False


def Move(env, robot, true_location, heading, step = STEP):
    abnormal_prob = 0.1
    true_location2 = np.array(true_location)
    heading = np.array(heading)
    target_location = true_location2
    target_location[0] = true_location2[0] + heading[0] * step
    target_location[1] = true_location2[1] + heading[1] * step

    if (random.uniform(0, 1) < abnormal_prob):
        neighbors = getNeighbors(true_location2)
        neighbor = neighbors[np.random.randint(0, 8)]
        while(neighbor[0] == target_location[0] and neighbor[1] == target_location[1]):
            neighbor = neighbors[np.random.randint(0, 8)]
        target_location = neighbor
        # target_location.append(0.05)

    if (is_collision(env, robot, target_location) or check_whether_out_of_bound(target_location)):
        return true_location, True
    else:
        return target_location, False

def Move2(env, robot, true_location, heading, step = STEP):
    true_location2 = np.array(true_location)
    heading = np.array(heading)
    target_location = true_location2
    target_location[0] = true_location2[0] + heading[0]
    target_location[1] = true_location2[1] + heading[1]

    if (is_collision(env, robot, target_location) or check_whether_out_of_bound(target_location)):
        return true_location, True
    else:
        return target_location, False

def Move3(env, robot, true_location, heading, step = STEP):
    true_location2 = np.array(true_location)
    heading = np.array(heading)
    target_location = true_location2
    target_location[0] = true_location2[0] + heading[0]
    target_location[1] = true_location2[1] + heading[1]

    if (is_collision(env, robot, target_location) or check_whether_out_of_bound(target_location)):
        return target_location, True
    else:
        return target_location, False

def Sense(true_location):
    problist = [true_location]
    prob = [0.005]
    randstep = random.uniform(0, 1)
    neighbors = getNeighbors(true_location, randstep)
    prob.extend((np.ones(8) * 0.05625).tolist())
    twoneighbors = getTwoNeighbors(true_location, randstep)
    prob.extend((np.ones(16) * 0.025).tolist())
    problist.extend(neighbors)
    problist.extend(twoneighbors)

    prob = prob / sum(prob)
    point = np.random.choice(1+8+16, 1, p = prob)
    return problist[point[0]]

heading = [[1, 0], [0, 1], [-1, 0], [0, -1], 
           [1, 1], [-1, 1], [1, -1], [-1, -1]]

def Sense_with_noise(true_location, noise_x, noise_y):
    sensed_position = []
    sensed_position.append(np.random.normal(0, noise_x) + true_location[0])
    sensed_position.append(np.random.normal(0, noise_y) + true_location[1])
    sensed_position.append(true_location[2])

    return sensed_position

def Sense2(env, robot, true_location, M):
    distance = []
    if M > 1000:
        return distance
    s = 0.5
    location = array(true_location)

    
    global heading
    for h in heading:
        step = array([h[0] * s, h[1] * s, 0.0])
        start = 0.0
        # nextlocation = location + start * step
        while(not is_collision(env, robot, location + start * step)):
            if (check_whether_out_of_bound(location + start * step)):
                break
            start += 1.0
        distance.append(start * s)

    for i in range(8):
        if random.uniform(0, 1) < 0.1:
            distance[i] *= random.uniform(1, 1.5)
    return distance

def plot_sense_distances(env, points, distances, true_location):
    global heading
    points_array = []
    location = array(true_location)
    for i in range(8):
        h = heading[i]
        for s in arange(0.0, distances[i], 0.02):
            step = array([h[0] * s, h[1] * s, 0.0])
            points_array.append(location + step)

    points.append(env.plot3(points= array(points_array),
                            pointsize=2.0,
                            colors=array((0,1,0))))

def PickRandomHeading():
    global heading
    return heading[np.random.randint(8)]


def initial_sampling(env, robot, xlimits = [-10, 10], ylimits = [-10, 10], M = 100):
    points = []
    for i in range(M):
        x = random.uniform(xlimits[0], xlimits[1])
        y = random.uniform(ylimits[0], ylimits[1])
        config = [x, y, 0.05]
        if (not is_collision(env, robot, config)):
            points.append(config)
        
    return points

def get_possibility(env, robot, location, heading, is_wall):
    p = 1
    neighbors = getNeighbors(location)
    walls = 0
    available_neighbors = []
    for i in neighbors:
        config = i
        config.append(0.05)
        # print config
        if (is_collision(env, robot, config) or check_whether_out_of_bound(config)):
            walls += 1
        else:
            available_neighbors.append(config)
    if (is_wall):
        return (8.0 - walls) / 8.0
    else:
        return 1.0 / 8.0


def generate_possibility(env, robot, heading, is_wall, X):
    weight = array(zeros((1, len(X))))[0]
    for i in range(len(X)):
        p = get_possibility(env, robot, X[i], heading, is_wall)
        weight[i] = p
        # print p
    return weight

def check_whether_adjacent(point, secondpoint, step = STEP):
    if ((abs(secondpoint[0] - point[0]) <= step and abs(secondpoint[1] - point[1]) <= step)):
        return True
    return False

def check_whether_two_adjacent(point, secondpoint, step = STEP):
    if ((abs(secondpoint[0] - point[0]) <= 2 * step and abs(secondpoint[1] - point[1]) <= 2 * step)):
        return True
    return False

def chech_same_points(point, secondpoint):
    if ((abs(secondpoint[0] - point[0]) <= 0.000001 and abs(secondpoint[1] - point[1]) <= 0.00001)):
        return True
    return False

def distance(point, secondpoint):
    return sqrt((point[0] - secondpoint[0]) ** 2 + (point[1] - secondpoint[1]) ** 2)

def calculate_posibility(position, sensed_position, true_distances, xm_distances, is_wall, xm_is_wall, step = STEP):
    diff = 1.0
    if (len(xm_distances) > 0):
        true_distances = array(true_distances)
        xm_distances = array(xm_distances)
        diff = abs(1.0 / (sum(abs(true_distances - xm_distances) ** 2) + 0.00001))
        # print diff
        return diff

    if is_wall == 1:
        if xm_is_wall == 1:
            
            # if (configDistance1(position, sensed_position) < 0.1):
            #     return 1.2
            return 1.0 * diff
        else:
            return 0.005 * diff
    else:
        if xm_is_wall == 0:
            return 1.0 * diff
            # if (configDistance1(position, sensed_position) < 0.1):
            #     return 1.1
        else:
            return 0.005 * diff


NOISE = 0.4
def find_noise_helper(env, robot, point, noise, point_index = [1, 0]):
    i = 0.0
    step = 0.05
    # if (noise < i):
    #     step *= -1
    while (abs(i - noise) > 0.05):
        i += step
        if (is_collision(env, robot, [point[0] + i * point_index[0], point[1] + i * point_index[1], 0.05])):
            return i * point_index[0] + i * point_index[1]
    return noise

def find_noise(env, robot, point, noise = [-1 * NOISE, NOISE, -1 * NOISE, NOISE]):
    noise[0] = find_noise_helper(env, robot, point, noise[0], point_index = [-1, 0])
    noise[1] = find_noise_helper(env, robot, point, noise[1], point_index = [1, 0])
    noise[2] = find_noise_helper(env, robot, point, noise[2], point_index = [0, -1])
    noise[3] = find_noise_helper(env, robot, point, noise[3], point_index = [0, 1])
    return noise




def add_noise(point, noise = [-1 * NOISE, NOISE, -1 * NOISE, NOISE]):
    newpoint = point
    n = random.uniform(noise[0], noise[1])
    newpoint[0] += n
    n = random.uniform(noise[2], noise[3])
    newpoint[1] += n
    # while(check_whether_out_of_bound(newpoint)):
    #     n = random.uniform(-1 * noise, noise)
    #     newpoint[0] = point[0] + n
    #     n = random.uniform(-1 * noise, noise)
    #     newpoint[1] = point[1] + n

    return newpoint

def configDistance1(A, B):
    sum = 0.0
    for i in range(2):
        sum += (A[i] - B[i]) ** 2
    return sqrt(sum)

def Resampling(env, robot, Xt, Weight):
    # print Weight
    # Xt = array(Xt)
    point = np.random.choice(len(Xt), len(Xt), p = Weight)
    # print point
    newXt = {}
    poorpoint = {}
    cloud = {}
    temp = Xt[point[0]]
    cloud[tuple(Xt[point[0]])] = [1, [temp[0], temp[1], temp[2]]]
    # print "Xt = ", Xt
    for i in range(len(Xt)):
        poor = True
        for j in cloud:
            # print j, "  ", cloud[j][1][0], " ", cloud[j][1][1]
            if configDistance1(tuple(Xt[point[i]]), j) < 1.1:
                # print j, "  ", cloud[j][1][0], " ", cloud[j][1][1]
                cloud[j][0] += 1
                cloud[j][1][0] += Xt[point[i]][0]
                cloud[j][1][1] += Xt[point[i]][1]
                # print j, "  ", cloud[j][1][0], " ", cloud[j][1][1]
                poor = False
                break

        if poor:
            # print "poor: ", Xt[point[i]], " i = ", i, "  ", point[i]
            temp = Xt[point[i]]
            cloud[tuple(Xt[point[i]])] = [1, [temp[0], temp[1], temp[2]]]

        newXt[tuple(Xt[point[i]])] = Weight[point[i]]

    for config in cloud:
        # print cloud[config][1][0], "   ", cloud[config][0]
        cloud[config][1][0] /= float(cloud[config][0])
        cloud[config][1][1] /= float(cloud[config][0])

    result = []
    for index in newXt:
        if index in cloud and cloud[index][0] < 15:
            # noise = find_noise(env, robot, index)
            # print "noise = ", noise
            num = int(newXt[index] * 2 * len(Xt) * log(2000.0 / sqrt(len(Xt))))
            # if (num < 5):
            #     num = 5
            if (num > 20):
                num = 20
            # print num
            for j in range(num):
                add_point = add_noise(array(index))
                if (not check_whether_out_of_bound(add_point)):
                    result.append(add_point)
            for j in range(num):
                add_point = add_noise(array(cloud[index][1]))
                if (not check_whether_out_of_bound(add_point)):
                    result.append(add_point)
        else:
            if (not check_whether_out_of_bound(index)):
                result.append(array(index))
    return result


def check_final_points_cloud(Xt):
    meanx = mean(array(Xt)[:, 0])
    meany = mean(array(Xt)[:, 1])
    meanconfig = [meanx, meany, 0.05]
    result = False
    for i in Xt:
        if configDistance1(meanconfig, i) > 0.8:
            return result, meanconfig

    result = True
    return result, meanconfig



def Astar(env, robot, startconfig, goalconfig):
    step = 0.25
    d = mat([[step, 0, 0],
             [0, step, 0],
             [0, 0, 1]])
    connect8 = mat([[1, 0, 0], [0, 1, 0], 
                    [-1, 0, 0], [0, -1, 0],
                    [1, 1, 0], [-1, 1, 0],
                    [1, -1, 0], [-1, -1, 0]])

    visited = {} #all visited configs
    queue = Q.PriorityQueue()
    costs = {}
    costs[tuple(startconfig)] = 0
    h = sqrt((startconfig[0] - goalconfig[0]) ** 2 + (startconfig[1] - goalconfig[1]) ** 2 + 
             (min(abs(startconfig[2] - goalconfig[2]), 2 * pi - abs(startconfig[2] - goalconfig[2]))) ** 2)

    queue.put(Compare(h, startconfig))
    parents = {}
    parents[tuple(startconfig)] = tuple(startconfig)

    points = []
    visitpoints = {}
    error = 0.3
    result = []
    finalcost = 0

    while not queue.empty():
        current = queue.get()
        currentconfig = current.config
        currentcost = costs[tuple(currentconfig)]
        visited[tuple(currentconfig)] = current.cost

        plot = array(currentconfig)
        plot[2] = 0.05
        visitpoints[tuple(plot)] = 1

        if (abs(currentconfig[0] - goalconfig[0]) < error and
            abs(currentconfig[1] - goalconfig[1]) < error):
            a = tuple(currentconfig)
            finalcost = costs[a]
            b = parents[a]
            while a != b:
                result.insert(0, list(a))
                a = b
                b = parents[a]
            break

        for i in connect8:
            nextconfig = (mat(currentconfig) + i * d).tolist()[0]

            if tuple(nextconfig) in visited:
                continue
            
            with env:
                robot.SetActiveDOFValues(nextconfig)

            if (env.CheckCollision(robot)):
                plot = array(nextconfig)
                plot[2] = 0.05
                if tuple(plot) not in visitpoints:
                    points.append(env.plot3(points= plot,
                                    pointsize=3.0,
                                    colors=array((1,0,0))))
                visitpoints[tuple(plot)] = 1
                continue

            plot = array(nextconfig)
            plot[2] = 0.05
            # if tuple(plot) not in visitpoints:
            #     points.append(env.plot3(points= plot,
            #                     pointsize=3.0,
            #                     colors=array((0,0,1))))
            visitpoints[tuple(plot)] = 1

            # heuristic
            h = sqrt((nextconfig[0] - goalconfig[0]) ** 2 + (nextconfig[1] - goalconfig[1]) ** 2 + 
                 (min(abs(nextconfig[2] - goalconfig[2]), 2 * pi - abs(nextconfig[2] - goalconfig[2]))) ** 2)
            # action cost
            c = sqrt((nextconfig[0] - currentconfig[0]) ** 2 + (nextconfig[1] - currentconfig[1]) ** 2 + 
                 (min(abs(nextconfig[2] - currentconfig[2]), 2 * pi - abs(nextconfig[2] - currentconfig[2]))) ** 2)

            queue.put(Compare(currentcost + c + h, nextconfig))
            costs[tuple(nextconfig)] = currentcost + c
            parents[tuple(nextconfig)] = tuple(currentconfig)
            visited[tuple(nextconfig)] = currentcost + c + h

    plot = array(result)
    plot[:, 2] = 0.05
    plot = list(plot)
    return plot


def demo_analysis(particle_time_list, KF_time_list, particle_err_list, KF_err_list, particle_hit_count, KF_hit_count, count):
    print ""
    print "Analysis:"

    print "Particle filter results invalid probablity:", particle_hit_count/count
    print "Kalman filter results invalid probability:", KF_hit_count/count
    
    plt.figure()
    plt.plot([i for i in range(len(particle_time_list))], particle_time_list, 'bo-')
    plt.xlabel('Iteration')
    plt.ylabel('Time')
    plt.title('Computation Time vs Iteration (b: particle)')
    plt.show()

    plt.figure()
    plt.plot([i for i in range(len(KF_time_list))], KF_time_list, 'go-')
    plt.xlabel('Iteration')
    plt.ylabel('Time')
    plt.title('Computation Time vs Iteration (g: kalman)')
    plt.show()

    #print particle_err_list, KF_err_list

    plt.figure()
    plt.plot([i for i in range(len(particle_err_list))], particle_err_list, 'bo-')
    KF_err_list = np.squeeze(np.asarray(KF_err_list))
    plt.plot([i for i in range(len(KF_err_list))], KF_err_list, 'go-')
    plt.xlabel('Iteration')
    plt.ylabel('Error')
    plt.title('Estimation Error vs Iteration (b: particle, g:kalman)')
    plt.show()





