import numpy
import time
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, robot, visualize):
        self.robot = robot
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        start_time = time.time()
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        #if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
        #    self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        # TODO Check for direct Extend
        self.planning_env.SetGoalParameters(goal_config, 0.2)
        goalReached = False
        while goalReached == False:
            random_config = self.planning_env.GenerateRandomConfiguration()
            nearest_vid, nearest_vertex = tree.GetNearestVertex(random_config)
            new_configs = self.planning_env.Extend(nearest_vertex, random_config)
            # print new_configs
            if new_configs != None and new_configs != []:
                last_vid = nearest_vid
                # for new_config in new_configs: #[new_configs[-1]]:
                new_config = new_configs[-1]
                new_vid = tree.AddVertex(new_config)
                tree.AddEdge(last_vid, new_vid)
                last_vid = new_vid
                # self.planning_env.PlotEdge(nearest_vertex, new_config)
                d = self.planning_env.ComputeDistance(new_config, goal_config)
                if d < epsilon:
                    goalReached = True
                    goal_vid = new_vid
        curr_vid = goal_vid
        while curr_vid != tree.GetRootId():
            plan.append(tree.vertices[curr_vid])
            curr_vid = tree.edges[curr_vid]
        plan.append(tree.vertices[curr_vid])
        plan.reverse()
        tot_dist = self.planning_env.comp_totDist(plan)
        num_vertices = self.planning_env.comp_totVertices(tree)
        planning_time = time.time() - start_time

        if self.visualize == True:
            traj = self.robot.ConvertPlanToTrajectory(plan)
            self.robot.ExecuteTrajectory(traj)

        # plan.append(start_config)
        # plan.append(goal_config)
        # print plan
        return plan, tot_dist, num_vertices, planning_time
