class Tree:
  def __init__(self, planning_env, start_config):
    self.planning_env = planning_env
    self.vertices = []
    self.vertices.append(start_config)
    self.edges = dict()

  def add_vertex(self, config):
    vid = len(self.vertices)
    self.vertices.append(config)
    return vid

  def add_edge(self, sid, eid):
     self.edges[eid] = sid

  def get_nearest_vertex(self, config):
    #TODO:implement
    dists = []

  def total_distance(self, path):
    total_distance = 0
    num = len(path)
    for i in range(num-1):
      total_distance += self.planning_env.compute_distace(path[i],path[i+1])
    return total_distace

  def total_vertices(self):
    return len(self.vertices)
