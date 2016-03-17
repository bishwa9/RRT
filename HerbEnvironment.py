import numpy
import scipy.spatial
import time

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def collision_pt(self, dof_values):
        # ensure pt is not colliding with robot or table
        table = self.robot.GetEnv().GetKinBody('conference_table')
        with self.robot.GetEnv():
            self.robot.SetDOFValues(dof_values, self.robot.GetActiveDOFIndices(), checklimits=True)
        return self.robot.GetEnv().CheckCollision(self.robot,table) or self.robot.GetEnv().CheckCollision(self.robot,self.robot)

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        p_checker = numpy.random.random_sample()
        if p_checker < self.p:
            return self.goal_config
        while True:
            config = [numpy.random.uniform(lower_limits[i], upper_limits[i], 1)[0] for i in range(len(config))]
            # for i in range(config):
            #     config[i] = numpy.random.uniform(lower_limits[i], upper_limits[i], 1)[0]
            if self.collision_pt(config) == False:
                break;
        return numpy.array(config)

    
    def ComputeDistance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return scipy.spatial.distance.euclidean(start_config, end_config)


    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        d = 0
        delta_d = 0.05
        xy_ = numpy.ones((1,len(self.robot.GetActiveDOFIndices())))
        while d <= 1:
            # print d
            xy = [numpy.add( (1-d)*start_config, d*end_config )]
            # print d*end_config
            xy_ = numpy.append(xy_, xy, axis=0)
            d += delta_d
            if( self.collision_pt( xy[0] ) ):
                return None
        return xy_[1:,:];
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        start_t = time.time()
        curr_path = path
        start_i = 0
        out = 0
        iters = 20000
        for iter_num in range(0,iters):
            if out == 1:
                break
            for i in range(start_i, len(curr_path)):
                if out == 1:
                    break
                for j in range(len(curr_path)-1, i+1, -1):
                    if self.ComputeDistance(curr_path[i], curr_path[j]) > 0.01:
                        new_configs = self.Extend(curr_path[i], curr_path[j])
                        if new_configs != None:
                            # print 'Connecting checking', i, 'to', j, 'as', new_configs
                            curr_path = numpy.append( numpy.append(curr_path[0:i+1], new_configs[1:], axis=0), curr_path[j:], axis=0)
                            start_i = start_i + 1
                            break
                    if(time.time() - start_t > timeout):
                        print 'Timeout'
                        out = 1
                        break
        print time.time() - start_t
        return curr_path