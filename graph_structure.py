import enum

from matplotlib import pyplot as plt

from scene_computings import *

plt.ion()


def plot_edge(edge):
    # plt.clf()
    plt.xlim(-5., 5.)
    plt.ylim(-5., 5.)
    plt.plot([edge.vert1.coords.x, edge.vert2.coords.x],
             [edge.vert1.coords.y, edge.vert2.coords.y], 'o', linestyle='-')
    plt.draw()
    plt.gcf().canvas.flush_events()


class VertexType(enum.Enum):
    deadlock = 1
    corner = 2
    triple_fork = 3
    cross = 4


class Edge:
    def __init__(self, vert1, vert2=-1, length=-1):
        self.vert1 = vert1
        self.vert2 = vert2
        self.length = length
        self.checked = False

    @classmethod
    def undefined_edge(cls):
        return cls(-1, -1, -1)


class Vertex:
    def __init__(self,
                 coords,
                 front_edge, right_edge, back_edge, left_edge,
                 vertex_type,
                 id):
        self.coords = coords
        self.edges = [front_edge, right_edge, back_edge, left_edge]
        self.vertex_type = vertex_type
        self.id = id

    @classmethod
    def undefined_vertex(cls):
        return cls(Point(np.NaN, np.NaN), *([-1] * 6))