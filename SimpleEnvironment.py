class SimpleEnvironmrnt:
  def __init__(self):
    self.boundry_limit = [[-5.,-5.],[5.,5.]]
    #TODO: add collision objects`

 # p = goal sampling probability
  def set_goal_parameters(self, goal_config, p):
    self.p = p
    self.goal_config = goal_config

  def is_in_collision(self, pt):
    #TODO: implement collision detection
    return 0

  def generate_random_configuration(self):
    config = [0] * 2
    lower_limits, upper_limits = self.boundry_limit
    p_checker = numpy.random.random_sample()
    if p_checker < self.p:
      return self.goal_config
    while True:
      config[0] = numpy.random.uniform(lower_limits[0], upper_limits[0], 1)[0]
      config[1] = numpy.random.uniform(lower_limits[1], upper_limits[1], 1)[0]
      if self.is_in_collision(config) == False:
        break
    return numpy.array(config)

  def compute_distace(self, start_config, end_config):
    #TODO: implement
    return
