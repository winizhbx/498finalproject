#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import Queue as Q
import implement
import random
from random import randint

class Compare(object):
    def __init__(self, cost, config):
        self.cost = cost
        self.config = config
        return
    def __cmp__(self, other):
        return cmp(self.cost, other.cost) #find the smallest cost

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint',
                      'r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'') 
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    # env.Load('pr2test2.env.xml')
    env.Load('easytest.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        goalconfig = [2.6,-1.3,-pi/2]

    start = time.clock()

        #### YOUR CODE HERE ####

    # d = mat([[0.1, 0, 0],
    #          [0, 0.1, 0],
    #          [0, 0, pi/2]])
    step = 0.3
    d = mat([[step, 0],
             [0, step]])
    # startconfig = mat([-3.4, -1.4, 0])
    # connect4 = mat([[1, 0, 0], [0, 1, 0], [0, 0, 1], 
    #                 [-1, 0, 0], [0, -1, 0], [0, 0, -1]])
    # connect8 = mat([[1, 0, 0], [0, 1, 0], [0, 0, 1], 
    #                 [-1, 0, 0], [0, -1, 0], [0, 0, -1],
    #                 [1, 1, 0], [0, 1, 1], [1, 0, 1],
    #                 [1, -1, 0], [0, 1, -1], [1, 0, -1],
    #                 [-1, 1, 0], [0, -1, 1], [-1, 0, 1],
    #                 [-1, -1, 0], [0, -1, -1], [-1, 0, -1],
    #                 [-1, 1, 1], [1, -1, 1], [1, 1, -1],
    #                 [-1, -1, 1], [1, -1, -1], [-1, 1, -1],
    #                 [1, 1, 1], [-1, -1, -1]])
    connect8 = mat([[1, 0], [0, 1], 
                    [-1, 0], [0, -1],
                    [1, 1], [-1, 1],
                    [1, -1], [-1, -1]])

    M = 2000 # initial M
    points = []
    Xt_1 = implement.initial_sampling(env, robot, M = M)
    M = len(Xt_1)
    print "M = ", M
    # print X0
    points.append(env.plot3(points= array(Xt_1),
                                pointsize=5.0,
                                colors=array((1,1,0))))

    t = 0
    position = [-8.5, -8.5, 0.05]
    points.append(env.plot3(points= position,
                                pointsize=5.0,
                                colors=array((1,0,0))))

    print position

    count = 0
    heading = implement.PickRandomHeading()

    target_position = implement.initial_sampling(env, robot, M = M)[0]
    points.append(env.plot3(points= position,
                                pointsize=5.0,
                                colors=array((1,0,0))))
    points.append(env.plot3(points= array(target_position),
                                pointsize=5.0,
                                colors=array((0,0,0))))
    # path = implement.RRT(env, robot, position, target_position)
    path = [[1, 1, 1, 1]]
    
    
    raw_input("Press enter to exit...")
    path_index = 0
    heading = array([-1, -1, 0]) * step
    AAA = 1
    while(True):
        # t += 1
        is_wall = False

        if (count % 30 == 29):
            target_position = implement.initial_sampling(env, robot, M = M)[0]
            points.append(env.plot3(points= array(target_position),
                                pointsize=5.0,
                                colors=array((0,0,0))))
            path = implement.Astar(env, robot, position, target_position)
            points.append(env.plot3(points= array(path),
                                        pointsize=5.0,
                                        colors=array((1,1,1))))
            AAA += 1
            heading = array(implement.PickRandomHeading()) * step
        else:
            position, is_wall = implement.Move2(env, robot, position, heading)

        if AAA % 2 == 0:
            heading = [path[0][0] - position[0], path[0][1] - position[1]]
            position = path[0]
            del path[0]
        # position, is_wall = implement.Move(env, robot, position, heading)
        print "heading = ", heading
        
        sensed_position = implement.Sense(position)
        true_distances = implement.Sense2(env, robot, position, M)
        print "distances = ", true_distances
        print "position = ", position, " is_wall = ", is_wall
        Xt = []
        Weight = []
        for i in range(M):
            xm, xm_is_wall = implement.Move2(env, robot, Xt_1[i], heading)
            xm_distances = implement.Sense2(env, robot, Xt_1[i], M)
            w = implement.calculate_posibility(xm, sensed_position, true_distances, xm_distances, is_wall, xm_is_wall)
            Weight.append(w)
            Xt.append(xm)

        # posibility = implement.generate_possibility(env, robot, heading, is_wall, Xt_1)
        # print "posibility = ", posibility
        Weight = Weight / sum(Weight)
        # print "Weight = ", Weight
        Xt_1 = implement.Resampling(env, robot, Xt, Weight)
        M = len(Xt_1)
        raw_input("Press enter to exit...")
        del points[:]

        points.append(env.plot3(points= array(Xt_1),
                                pointsize=5.0,
                                colors=array((1,1,0))))
        points.append(env.plot3(points= position,
                                pointsize=5.0,
                                colors=array((1,0,0))))
        points.append(env.plot3(points= array(sensed_position),
                                pointsize=5.0,
                                colors=array((0,1,1))))
        if AAA % 2 == 0:
            points.append(env.plot3(points= array(path),
                                            pointsize=5.0,
                                            colors=array((1,1,1))))
        if is_wall and AAA % 2 == 1:
            heading = array(implement.PickRandomHeading()) * step

        if (len(path) < 1 and AAA % 2 == 0):
            target_position = implement.initial_sampling(env, robot, M = M)[0]
            points.append(env.plot3(points= array(target_position),
                                pointsize=5.0,
                                colors=array((0,0,0))))
            path = implement.Astar(env, robot, position, target_position)
            points.append(env.plot3(points= array(path),
                                        pointsize=5.0,
                                        colors=array((1,1,1))))
            # if change == False:
            #     change = True
            # else:
            #     change = False
        count += 1
        print "count = ", count
        # if count > 20:
        #     break
        check_final, mean_config = implement.check_final_points_cloud(Xt_1)
        if (check_final):
            print "Final position find: ", mean_config
            points.append(env.plot3(points= array(mean_config),
                                pointsize=10.0,
                                colors=array((1,0,0))))
            implement.is_collision(env, robot, mean_config)
            break



        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.

        #### Draw the X and Y components of the configurations explored by your algorithm
    path = [] #put your final path in this variable


        #### END OF YOUR CODE ###
        
    end = time.clock()
    print "Time: ", end - start

    # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")

