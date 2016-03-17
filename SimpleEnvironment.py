import numpy
import matplotlib.pyplot as pl
import scipy.spatial
import time

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def collision_pt(self, pt):
        # ensure robot at pt is not colliding with table
        table = self.robot.GetEnv().GetKinBody('conference_table')
        trans = numpy.array([[1, 0, 0, pt[0]], [0, 1, 0, pt[1]], [0, 0, 1, 0], [0, 0, 0, 1]])
        with self.robot.GetEnv():
            self.robot.SetTransform(trans)
        return self.robot.GetEnv().CheckCollision(self.robot,table)
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits

        # generate a number E [0,1]
        p_checker = numpy.random.random_sample()
        if p_checker < self.p:
            return self.goal_config
        while True:
            # generate two random points [0,1]
            config[0] = numpy.random.uniform(lower_limits[0], upper_limits[0], 1)[0]
            config[1] = numpy.random.uniform(lower_limits[1], upper_limits[1], 1)[0]
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
        #   a start configuration to a target configuration
        #
        d = 0
        delta_d = 0.1
        xy_ = numpy.ones((1,2))
        while d <= 1:
            xy = [numpy.add( (1-d)*start_config, d*end_config )]
            xy_ = numpy.append(xy_, xy, axis=0)
            d += delta_d
            if( self.collision_pt( xy[0] ) ):
                return None
        return xy_[1:,:];

        # walk with a quantized distance from start_config to end_config
        #diff = end_config - start_config
        #angle = numpy.arctan2(diff[1], diff[0]) # radians
        # dir_ = numpy.sign(diff)
        # if( set(dir_) == set( [0., 0.] ) ):
        #     return None # start = goal

        #uniform_dist = 0.1
        #total_dist = self.ComputeDistance( start_config, end_config )

        #pts_ = numpy.linspace(0, total_dist, num=10) # points expressed in local coordinate frame
        #xy_ = numpy.zeros((numpy.shape(pts_)[0], 2))
        #num_ = 0
        #for point in pts_:
        #    xy = numpy.add(start_config, numpy.array([point * numpy.cos(angle), point * numpy.sin(angle)]))
        #    xy_[num_,:] = xy
        #    num_ += 1
        #    if( self.collision_pt( xy ) ):
        #        return None
        #return xy_;

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        start_t = time.time()
        curr_path = path
        start_i = 0
        iters = 20000
        out = 0
        for iter_num in range(0,iters):
            start_i = 0
            if out == 1:
                break
            for i in range(start_i, len(curr_path)):
                if out == 1:
                    break
                for j in range(len(curr_path)-1, i+1, -1):
                    if self.ComputeDistance(curr_path[i], curr_path[j]) > 0.8:
                        new_configs = self.Extend(curr_path[i], curr_path[j])
                        if new_configs != None and not numpy.array_equal(new_configs, numpy.array([0,0])):
                            # print 'Connecting checking', i, 'to', j, 'as', new_configs
                            curr_path = numpy.append( numpy.append(curr_path[0:i+1], new_configs[1:], axis=0), curr_path[j:], axis=0)
                            start_i = start_i + 1
                            break
                    if(time.time() - start_t > timeout):
                        print 'Timeout'
                        out = 1
                        break
        print time.time() - start_t
        # Plot New shortened path
        self.InitializePlot(path[-1])
        self.PlotEdge(curr_path[0], curr_path[1])
        for i in range(1, len(curr_path)):
            self.PlotEdge(curr_path[i-1], curr_path[i])
        return curr_path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

