import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        self.planning_env.SetGoalParameters(goal_config, 0.2)
        goalReached = False
        while goalReached == False:
            random_config = self.planning_env.GenerateRandomConfiguration()
            nearest_vid, nearest_vertex = tree.GetNearestVertex(random_config)
            new_configs = self.planning_env.Extend(nearest_vertex, random_config)
            if new_configs != None and new_configs != []:
                for new_config in [new_configs[-1]]:
                    new_vid = tree.AddVertex(new_config)
                    tree.AddEdge(nearest_vid, new_vid)
                    # self.planning_env.PlotEdge(nearest_vertex, new_config)
                    # d = self.planning_env.ComputeDistance(new_config, goal_config)
                    if numpy.array_equal(new_config, goal_config):
                        goalReached = True
                        goal_vid = new_vid
        curr_vid = goal_vid
        while curr_vid != tree.GetRootId():
            plan.append(tree.vertices[curr_vid])
            curr_vid = tree.edges[curr_vid]
        plan.append(tree.vertices[curr_vid])
        plan.reverse()
        # plan.append(start_config)
        # plan.append(goal_config)
        print plan
        return plan
