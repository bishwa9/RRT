import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.001):
    	# epsilon = 0.2 # remove if using herb
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        goalReached = False
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        #self.planning_env.SetGoalParameters(goal_config,0.2)
		
        while goalReached == False:
            f_random_config = self.planning_env.GenerateRandomConfiguration()
            # r_random_config = self.planning_env.GenerateRandomConfiguration()
            r_random_config = f_random_config
            f_nearest_vid, f_nearest_vertex = ftree.GetNearestVertex(f_random_config)
            r_nearest_vid, r_nearest_vertex = rtree.GetNearestVertex(r_random_config)
            f_new_configs = self.planning_env.Extend(f_nearest_vertex, f_random_config)
            r_new_configs = self.planning_env.Extend(r_nearest_vertex, r_random_config)

            if f_new_configs != None:
                f_last_vid = f_nearest_vid
                for f_new_config in f_new_configs: #[new_configs[-1]]:
                    f_new_vid = ftree.AddVertex(f_new_config)
                    ftree.AddEdge(f_last_vid, f_new_vid)
                    f_last_vid = f_new_vid
                    # self.planning_env.PlotEdge(f_nearest_vertex, f_new_config)	#remove for herb 
                    r_nearest2f_vid, r_nearest2f_vertex = rtree.GetNearestVertex(f_new_config)
                    d = self.planning_env.ComputeDistance(f_new_config, r_nearest2f_vertex)
                    if d < epsilon:
                        goalReached = True
                        f_closest_vid = f_new_vid
                        r_closest_vid = r_nearest2f_vid

            if r_new_configs != None and goalReached != True:
                r_last_vid = r_nearest_vid
                for r_new_config in r_new_configs: #[new_configs[-1]]:
                    r_new_vid = rtree.AddVertex(r_new_config)
                    rtree.AddEdge(r_last_vid, r_new_vid)
                    r_last_vid = r_new_vid
                    # self.planning_env.PlotEdge(r_nearest_vertex, r_new_config)	#remove for herb 
                    f_nearest2r_vid, f_nearest2r_vertex = ftree.GetNearestVertex(r_new_config)
                    d = self.planning_env.ComputeDistance(r_new_config, f_nearest2r_vertex)
                    if d < epsilon:
                        goalReached = True
                        f_closest_vid = f_nearest2r_vid
                        r_closest_vid = r_new_vid

        curr_vid = f_closest_vid
        while curr_vid != ftree.GetRootId():
            plan.append(ftree.vertices[curr_vid])
            curr_vid = ftree.edges[curr_vid]
        plan.append(ftree.vertices[curr_vid])
        plan.reverse()
        
        plan_temp = []
        curr_vid = r_closest_vid
        while curr_vid != rtree.GetRootId():
            plan_temp.append(rtree.vertices[curr_vid])
            curr_vid = rtree.edges[curr_vid]
        plan_temp.append(rtree.vertices[curr_vid])
        plan.extend(plan_temp)
        
        # print plan
        #plan.append(start_config)
        #plan.append(goal_config)
        return plan