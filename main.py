from RRT import RRT
from SimpleEnvironment import SimpleEnvironment
import numpy

simple_env = SimpleEnvironment()
rrt = RRT(simple_env)
start_config = numpy.array([-4,-4])
goal_config = numpy.array([4,4])
rrt.plan(start_config, goal_config)
a = raw_input("Press a key to continue ...")
