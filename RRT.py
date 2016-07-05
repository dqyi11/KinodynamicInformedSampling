from Tree import Tree

class RRT:
    def __init__(self, planning_env):
        self.planning_env = planning_env

    def plan(self, start_config, goal_config, epsilon=0.01):
        self.planning_env.plot_initialize(start_config, goal_config)
        self.planning_env.plot_show()
        tree = Tree(self.planning_env, start_config)
        plan = []
        self.planning_env.set_goal_parameters(goal_config, 0.1)
        goal_reached = False
        while goal_reached == False:
            random_config = self.planning_env.generate_random_configuration()
            nearest_config_id, nearest_config = tree.get_nearest_vertex(
                                                random_config)
            new_config = tree.extend(nearest_config, random_config, 0.1)
            if new_config.size > 0:
                new_config_id = tree.add_vertex(new_config)
                tree.add_edge(nearest_config_id, new_config_id)
                self.planning_env.plot_add_edge(nearest_config, new_config)
                if (self.planning_env.compute_distace(new_config,
                                     goal_config) < epsilon):
                    goal_reached = True
        path = []
        path.append(goal_config)
        current_config_id = new_config_id
        #TODO: handle abstraction in a better way
        while current_config_id != 0:
            path.insert(0, tree.vertices[current_config_id])
            current_config_id = tree.edges[current_config_id]
        path.insert(0, start_config)
        print path
        return plan
