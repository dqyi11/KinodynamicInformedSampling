import tree

class RRT:
  def __init__(self, planning_env):
    self.planning_env = planning_env

  def plan(self, start_config, goal_config, epsilon=0.0001):
    tree = Tree(self.planning_env, goal_config)
    plan = []
    self.planning_env.set_goal_parameters(goal_config, 0.2)
    goal_reached = false
    while goal_reached == False:

