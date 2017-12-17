#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import Queue as Q
import implement
import random
from random import randint
import numpy as np

from KF import *
from key_detect import *

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


def particle_kidnapped_robot(env_option=0, use_EKF=0):
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    if env_option == 2:
        env.Load('pr2test2.env.xml')
    elif env_option == 1:
        env.Load('easytest.xml')
    else:
        env.Load('empty.xml')
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

    step = 0.25
    d = mat([[step, 0],
             [0, step]])
    connect8 = mat([[1, 0], [0, 1], 
                    [-1, 0], [0, -1],
                    [1, 1], [-1, 1],
                    [1, -1], [-1, -1]])


    M = 3000 # initial M
    points = []
    Xt_1 = implement.initial_sampling(env, robot, M = M)
    M = len(Xt_1)
    # print "M = ", M
    # print X0
    points.append(env.plot3(points= array(Xt_1),
                                pointsize=5.0,
                                colors=array((1,1,0))))

    t = 0
    position = [-8.5, -8.5, 0.05]
    # points.append(env.plot3(points= position,
    #                             pointsize=5.0,
    #                             colors=array((1,0,0))))

    # print position

    count = 0
    heading = implement.PickRandomHeading()

    target_position = implement.initial_sampling(env, robot, M = M)[0]
    points.append(env.plot3(points= position,
                                pointsize=5.0,
                                colors=array((1,0,0))))
    # points.append(env.plot3(points= array(target_position),
    #                             pointsize=5.0,
    #                             colors=array((0,0,0))))
    # path = implement.RRT(env, robot, position, target_position)
    path = [[1, 1, 1, 1]]
    
    time.sleep(0.5)
    print("Red point: current true location.")
    print("Yellow points: possible locations.")
    print("White points: Astar path.")
    print("Green lines shows the noisy sonar measurements.")
    time.sleep(0.5)
    raw_input("Press enter to start:")
    path_index = 0
    heading = array([-1, -1, 0]) * step
    AAA = 1
    while(True):
        # t += 1
        is_wall = False

        if (count % 15 == 14):
            target_position = implement.initial_sampling(env, robot, M = 1)[0]
            points.append(env.plot3(points= array(target_position),
                                pointsize=5.0,
                                colors=array((0,0,0))))
            path = implement.Astar(env, robot, position, target_position)
            AAA += 1
            heading = array(implement.PickRandomHeading()) * step
        else:
            pre_position = position
            if AAA % 2 == 1:
                position, is_wall = implement.Move2(env, robot, position, heading)
                motion = np.array(position) - np.array(pre_position)
                print "motion", motion

        if AAA % 2 == 0:
            heading = [path[0][0] - position[0], path[0][1] - position[1]]
            pre_position = position
            position = path[0]
            del path[0]
            motion = np.array(position) - np.array(pre_position)
            print "motion", motion

        
        sensed_position = implement.Sense_with_noise(position, 1, 1)
        true_distances = implement.Sense2(env, robot, position, M)

        # print "distances = ", true_distances
        # print "position = ", position, " is_wall = ", is_wall
        Xt = []
        Weight = []
        particle_start = time.clock()
        for i in range(M):
            xm, xm_is_wall = implement.Move3(env, robot, Xt_1[i], heading)
            xm_distances = implement.Sense2(env, robot, Xt_1[i], M)
            w = implement.calculate_posibility(xm, sensed_position, true_distances, xm_distances, is_wall, xm_is_wall)
            Weight.append(w)
            Xt.append(xm)
        # posibility = implement.generate_possibility(env, robot, heading, is_wall, Xt_1)
        # print "posibility = ", posibility
        Weight = Weight / sum(Weight)
        # print "Weight = ", Weight
        Xt_1 = implement.Resampling(env, robot, Xt, Weight)
        particle_end = time.clock()
        M = len(Xt_1)
        del points[:]
        points.append(env.plot3(points= position,
                                pointsize=5.0,
                                colors=array((1,0,0))))
        if len(true_distances) > 1:
            implement.plot_sense_distances(env, points, true_distances, position)

        points.append(env.plot3(points= array(Xt_1),
                                pointsize=5.0,
                                colors=array((1,1,0))))
        # points.append(env.plot3(points= array(sensed_position),
        #                         pointsize=5.0,
        #                         colors=array((0,1,1))))
        if AAA % 2 == 0 and len(path) > 1:
            points.append(env.plot3(points= array(path),
                                            pointsize=5.0,
                                            colors=array((1,1,1))))


        if is_wall and AAA % 2 == 1:
            heading = array(implement.PickRandomHeading()) * step

        if (len(path) < 1 and AAA % 2 == 0):
            target_position = implement.initial_sampling(env, robot, M = 1)[0]
            points.append(env.plot3(points= array(target_position),
                                pointsize=5.0,
                                colors=array((0,0,0))))
            path = implement.Astar(env, robot, position, target_position)
            points.append(env.plot3(points= array(path),
                                        pointsize=5.0,
                                        colors=array((1,1,1))))

        count += 1
        print "Times = ", count
        print "True position: ", position
        check_final, mean_config = implement.check_final_points_cloud(Xt_1)
        if (check_final):
            print "Final position find: ", mean_config
            print "Error = ", sqrt(sum(abs(array(mean_config) - array(position)) ** 2))
            # particle_err_list.append(sqrt(sum(abs(array(mean_config) - array(position)) ** 2)))
            # KF_err_list.append(sqrt(sum(abs(array(KF_x) - array(position)) ** 2)))
            points.append(env.plot3(points= array(mean_config),
                                pointsize=10.0,
                                colors=array((1,0,0))))
            implement.is_collision(env, robot, mean_config)
            break

        # set back to current location
        with env:
            robot.SetActiveDOFValues(position)
        waitrobot(robot)

    raw_input("CLOSE THE WINDOW FIRST. Press enter to exit this demo:")
    print "Return to main menu..."


def particle_KF_demo(env_option=0, use_EKF=0):

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    if env_option == 2:
        env.Load('pr2test2.env.xml')
    elif env_option == 1:
        env.Load('easytest.xml')
    else:
        env.Load('empty.xml')
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

    ## For analysis:
    particle_time_list = []
    KF_time_list = []
    particle_hit_count = 0.
    KF_hit_count = 0.
    particle_err_list = []
    KF_err_list = []

    start = time.clock()

    step = 0.25
    d = mat([[step, 0],
             [0, step]])
    connect8 = mat([[1, 0], [0, 1], 
                    [-1, 0], [0, -1],
                    [1, 1], [-1, 1],
                    [1, -1], [-1, -1]])

    position = [-8.5, -8.5, 0.05]
    # points.append(env.plot3(points= position,
    #                             pointsize=5.0,
    #                             colors=array((1,0,0))))

    
    heading = array([-1, -1, 0])
    points = []
    points2 = []
    points3 = []

    ## KF init
    x = np.matrix('0. 0.').T 
    P = np.matrix(np.eye(2))*1000 # initial uncertainty
    R = np.array([[1,0],[0,1]])
    pre_position = position
    x[0][0] = pre_position[0]
    x[1][0] = pre_position[1]
    Q = np.eye(2)*0.01
    ## KF init ends

    count = 0
    AAA = 1
    path = [[1, 1, 1, 1]]


    ## particle init
    M = 300 # initial M
    Xt_1 = implement.initial_sampling(env, robot, M = M)
    M = len(Xt_1)
    points3.append(env.plot3(points= array(Xt_1),
                                pointsize=5.0,
                                colors=array((1,1,0))))

    time.sleep(0.5)
    print("Red points: true locations.")
    print("Blue points: sensed locations.")
    print("Yellow points: Particle filter possible locations.")
    print("Purple points: Kalman filter predict locations.")
    print("White points: Astar path.")
    time.sleep(0.5)
    raw_input("Press enter to start:")
    while(True):
        # t += 1
        is_wall = False
        print "\n"
        print "Time: ", count
        print "true_position: ", position
        pre_position = position

        step = implement.PickRandomStep()
        curheading = heading

        # position, is_wall = implement.Move2(env, robot, position, curheading)
        # motion = np.array(position) - np.array(pre_position)
        # print "motion: ", motion

        if (count % 30 == 29):
            AAA += 1
            if AAA % 2 == 0:
                target_position = implement.initial_sampling(env, robot, M = 1)[0]
                points2.append(env.plot3(points= array(target_position),
                                    pointsize=5.0,
                                    colors=array((0,0,0))))
                path = implement.Astar(env, robot, position, target_position, step = 0.2)
                del points2[:]
                points2.append(env.plot3(points= array(path),
                                                pointsize=5.0,
                                                colors=array((1,1,1))))
            else:
                heading = array(implement.PickRandomHeading())

        if AAA % 2 == 0:
            step = 1.0
            heading = [path[0][0] - position[0], path[0][1] - position[1]]
            pre_position = position
            if random.uniform(0, 1) > 0.08:
                print "error"
                position = path[0]
                del path[0]
            motion = np.array(position) - np.array(pre_position)
            print "motion", motion
        else:
            pre_position = position
            position, is_wall = implement.Move_with_error(env, robot, position, heading, step = step, error = 0.08)
            motion = np.array(position) - np.array(pre_position)
            print "motion", motion
        
        sensed_position = implement.Sense_with_noise(position, 1, 1)

        ## particle filter
        Xt = []
        Weight = []
        particle_start = time.clock()
        for i in range(M):
            # xm, xm_is_wall = implement.Move3(env, robot, Xt_1[i], heading)
            xm = Xt_1[i]
            if AAA % 2 == 0:
                # print "heading = ", heading
                # print "step = ", step
                # print xm
                xm, xm_is_wall = implement.Move_with_error2(env, robot, Xt_1[i], heading, step = step, error = 0.08)
                # print xm
            else:
                xm, xm_is_wall = implement.Move_with_error(env, robot, Xt_1[i], heading, step = step, error = 0.08)
            w = implement.calculate_posibility2(xm, sensed_position)
            Weight.append(w)
            Xt.append(xm)
        # posibility = implement.generate_possibility(env, robot, heading, is_wall, Xt_1)
        # print "posibility = ", posibility
        Weight = Weight / sum(Weight)
        Xt_1 = implement.Resampling2(env, robot, Xt, Weight)
        particle_end = time.clock()
        M = len(Xt_1)
        del points3[:]
        points3.append(env.plot3(points= array(Xt_1),
                                pointsize=5.0,
                                colors=array((1,1,0))))
        check_final, mean_config = implement.check_final_points_cloud(Xt_1)
        ## particle filter end

        ## KF filter
        if use_EKF == 0:
            KF_start = time.clock()
            meas = sensed_position[:2]
            x, P = kalman(x, P, meas, R, Q=Q, motion=np.matrix([motion[0]+np.random.normal(0, 0.1),motion[1]+np.random.normal(0, 0.1)]).T,
                F = np.matrix('''
                            1. 0.;
                            0. 1.
                            '''),
                H = np.matrix('''
                            1. 0.;
                            0. 1.'''))
            KF_end = time.clock()

        else:
            KF_start = time.clock()
            meas = np.sqrt(sensed_position[0]**2 + sensed_position[1]**2)
            x, P = extend_kalman(x, P, meas, R=np.array([[40]]), motion=np.array([[motion[0],motion[1]]]).T, Q=Q)
            KF_end = time.clock()
        
        KF_x = []
        KF_x.append(x[0][0])
        KF_x.append(x[1][0])
        KF_x.append(sensed_position[2])
        points.append(env.plot3(points= array(KF_x),
                                pointsize=5.0,
                                colors=array((0.8,0,0.8))))
        if use_EKF == 0:
            points.append(env.plot3(points= array(sensed_position),
                                pointsize=5.0,
                                colors=array((0,0,1))))
        else:
            handles = []
            handles.append(env.drawlinestrip(points=array((sensed_position,(0,0,0.5))),
                                           linewidth=5.0,
                                           colors=array((1,0,0,1))))

        ## KF filter ends

        ## set heading
        if is_wall and AAA % 2 == 1:
            heading = array(implement.PickRandomHeading())


        ## set plots on windows
        del points2[:]
        if AAA % 2 == 0 and len(path) > 1:
            points2.append(env.plot3(points= array(path),
                                            pointsize=5.0,
                                            colors=array((1,1,1))))

        

        if (len(path) < 1 and AAA % 2 == 0):
            target_position = implement.initial_sampling(env, robot, M = 1)[0]
            points.append(env.plot3(points= array(target_position),
                                pointsize=5.0,
                                colors=array((0,0,0))))
            path = implement.Astar(env, robot, position, target_position, step = 0.2)
            del points2[:]
            points2.append(env.plot3(points= array(path),
                                        pointsize=5.0,
                                        colors=array((1,1,1))))

        points.append(env.plot3(points= array(position),
                                pointsize=5.0,
                                colors=array((1,0,0))))

        ## For analysis
        particle_time_list.append((particle_end - particle_start))
        KF_time_list.append((KF_end - KF_start))
        
        # if implement.is_collision(env, robot, mean_config) or implement.check_whether_out_of_bound(mean_config, m = 10, n = 10):
        #     particle_hit_count += 1
        if implement.is_collision(env, robot, KF_x) or implement.check_whether_out_of_bound(KF_x, m = 10, n = 10):
            KF_hit_count += 1 

        particle_err_list.append(sqrt(sum(abs(array(mean_config) - array(position)) ** 2)))
        KF_err_list.append(sqrt(sum(abs(array(KF_x) - array(position)) ** 2)))

        # set back to current location
        with env:
            robot.SetActiveDOFValues(position)
        waitrobot(robot)

        # end iteration
        count += 1
        time.sleep(0.1)
        if (count > 100):
            break
        
    end = time.clock()
    print "Time: ", end - start
    print KF_hit_count, particle_hit_count
    implement.demo_analysis(particle_time_list, KF_time_list, particle_err_list, KF_err_list, particle_hit_count, KF_hit_count, count)

    waitrobot(robot)

    raw_input("CLOSE THE WINDOW FIRST. Press enter to exit this demo:")
    print "Return to main menu..."


if __name__ == "__main__":

    command = 's'
    while(1):

        if command != 'y':
            command = display_menu(0)
        quit = 0
        
        if command == '1':
            demo_kalman_xy()
            raw_input("Press enter to continue a EKF demo...")
            
            demo_extend_kalman_xy()
            raw_input("Press enter and back to main manu...")
            command = display_menu(1)
            if command == 'q':
                print("Quit!")
                break

        if command == '2' or command == 'y':
            room_option = display_menu(2)
            if room_option == 'q':
                print("Quit!")
                break

            start_option = display_menu(5)
            if start_option == 'q':
                print("Quit!")
                break

            if start_option == '0':
                KF_option = display_menu(3)
                if KF_option == 'q':
                    print("Quit!")
                    break

                particle_KF_demo(room_option, KF_option)

                command = display_menu(4)
                if command == 'q':
                    print("Quit!")
                    break

                while command == 'y':
                    room_option = display_menu(2)
                    if room_option == 'q':
                        quit = 1 
                        break

                    KF_option = display_menu(3)
                    if KF_option == 'q':
                        quit = 1
                        break

                    particle_KF_demo(room_option, KF_option)

                    command = display_menu(4)
                    if command == 'q':
                        quit = 1
                        break

            elif start_option == '1':
                particle_kidnapped_robot(room_option)

                command = display_menu(4)
                if command == 'q':
                    print("Quit!")
                    break
                elif command == 'y':
                    continue


            


        if command == 'q' or quit == 1:
            print("Quit!")
            break
