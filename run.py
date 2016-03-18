#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from RRTPlanner import RRTPlanner
from RRTConnectPlanner import RRTConnectPlanner

def main(robot, planning_env, planner, shorten, times):

    raw_input('Press any key to begin planning')

    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
        goal_config = numpy.array([ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
    else:
        goal_config = numpy.array([2.0, -0.8])
    dists = []
    num_vs = []
    p_times = []
    s_times = []
    s_dists = []
    for i in range(times):
        plan, tot_dist, num_vertices, planning_time = planner.Plan(start_config, goal_config)
        if shorten == True:
            plan_short, shortening_time, shortened_dist = planning_env.ShortenPath(plan, planner.visualize, planner.robot)
            s_times.append(shortening_time)
            s_dists.append(shortened_dist)
        dists.append(tot_dist)
        num_vs.append(num_vertices)
        p_times.append(planning_time)
    
    print "Average Distance:", numpy.mean(dists)
    print "Average Number of Vertices:", numpy.mean(num_vs)
    print "Average Planning Times:", numpy.mean(p_times)
    if shorten == True:
        s_dists = [x - y for x,y in zip(dists, s_dists)]
        print "Average Shortenning Times:", numpy.mean(s_times)
        print "Average Shortenning Distance:", numpy.mean(s_dists)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='rrt',
                        help='The planner to run (rrt or rrtconnect)')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-o', '--optimize', type=str, default='no',
                        help='Apply Path shortening')
    parser.add_argument('-t', '--times', type=int, default=10,
                        help='How many times to run planner')
    

    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 2 Viewer')

    # First setup the environment and the robot
    visualize = args.visualize
    if args.robot == 'herb':
        robot = HerbRobot(env, args.manip)
        planning_env = HerbEnvironment(robot)
        #visualize = False
    elif args.robot == 'simple':
        robot = SimpleRobot(env)
        planning_env = SimpleEnvironment(robot)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    # Next setup the planner
    if args.planner == 'rrt':
        planner = RRTPlanner(planning_env, robot, visualize=visualize)
    elif args.planner == 'rrtconnect':
        planner = RRTConnectPlanner(planning_env, robot, visualize=visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)

    if args.optimize == 'no':
        main(robot, planning_env, planner, False, args.times)
    else:
        main(robot, planning_env, planner, True, args.times)

    import IPython
    IPython.embed()

        
    
