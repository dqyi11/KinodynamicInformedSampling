import numpy
import matplotlib.pyplot as plt

class SimpleEnvironment:
    def __init__(self):
        self.boundry_limit = [[-5.,-5.],[5.,5.]]
        self.box_obstacles = []
        self.box_obstacles.append([[-3,-3],[3,3]])

 # p = goal sampling probability
    def set_goal_parameters(self, goal_config, p):
        self.p = p
        self.goal_config = goal_config

    def is_in_collision(self, pt):
        for box_obstacle in self.box_obstacles:
            if (pt[0] >= box_obstacle[0][0] and
                pt[0] <= box_obstacle[1][0] and
                pt[1] >= box_obstacle[0][1] and
                pt[1] <= box_obstacle[1][1]):
                return True
        return False

    def generate_random_configuration(self):
        config = [0] * 2
        lower_limits, upper_limits = self.boundry_limit
        p_checker = numpy.random.random_sample()
        if p_checker < self.p:
            return self.goal_config
        while True:
            config[0] = numpy.random.uniform(lower_limits[0],
                                             upper_limits[0], 1)[0]
            config[1] = numpy.random.uniform(lower_limits[1],
                                             upper_limits[1], 1)[0]
            if self.is_in_collision(config) == False:
                break
        return numpy.array(config)

    def compute_distace(self, start_config, end_config):
        return numpy.linalg.norm(start_config-end_config)

    def plot_initialize(self, start_config, goal_config):
        plt.plot(start_config[0], start_config[1], "o")
        plt.plot(goal_config[0], goal_config[1], "o")
        plt.interactive(True)

    def plot_add_edge(self, start_config, end_config):
        plt.plot([start_config[0], end_config[0]],
                 [start_config[1], end_config[1]], 'k-')

    def plot_show(self):
        plt.show()
