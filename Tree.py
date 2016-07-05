import numpy

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

    def get_parent(self, n_id):
        return self.edges[n_id]

    def get_nearest_vertex(self, config):
        min_dist = float("inf")
        min_dist_vertex_id = 0
        for vertex_id in range(len(self.vertices)):
            dist = self.planning_env.compute_distace(self.vertices[vertex_id],
                        config)
            if dist < min_dist:
                min_dist = dist
                min_dist_vertex_id = vertex_id
        return min_dist_vertex_id, self.vertices[min_dist_vertex_id]

    def total_distance(self, path):
        total_distance = 0
        num = len(path)
        for i in range(num-1):
            total_distance += self.planning_env.compute_distace(path[i],
                                                                path[i+1])
        return total_distace

    def total_vertices(self):
        return len(self.vertices)

    def extend(self, start_config, goal_config, dist):
        #extend by dist in direction goal_config
        dist_total = self.planning_env.compute_distace(start_config,
                                                       goal_config)
        dist_ratio = dist/dist_total
        if dist_ratio > 1:
            dist_ratio = 1
        new_config = start_config + dist_ratio*(goal_config-start_config)
        if self.planning_env.is_in_collision(new_config):
            return numpy.array([])
        return new_config
