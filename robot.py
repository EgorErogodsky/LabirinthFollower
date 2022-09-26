# TODO: Организовать файловую структуру
import time
import math
import enum
from zmqRemoteApi import RemoteAPIClient


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class VertexType(enum.Enum):
    deadlock = 1
    corner = 2
    triple_fork = 3
    cross = 4


class Moving(enum.IntEnum):
    standing = 0
    front = 1
    right = 2
    back = 3
    left = 4


class Edge:
    def __init__(self, vert1, vert2=-1, length=-1):
        self.vert1 = vert1
        self.vert2 = vert2
        self.length = length

    @classmethod
    def undefined_edge(cls):
        return cls(-1, -1, -1)


class Vertex:
    def __init__(self,
                 x, y,
                 front_edge, right_edge, back_edge, left_edge,
                 vertex_type):
        self.coords = (x, y)

        self.edges = [front_edge, right_edge, back_edge, left_edge]

        self.vertex_type = vertex_type


class Robot:
    def __init__(self, sim, robot_name):
        global vertices
        self.name = robot_name
        self.sim = sim
        self.sensors = {'front': self.sim.getObject('/' + self.name + '/FRONTSENS'),
                        'right': self.sim.getObject('/' + self.name + '/RIGHTSENS'),
                        'back': self.sim.getObject('/' + self.name + '/BACKSENS'),
                        'left': self.sim.getObject('/' + self.name + '/LEFTSENS')}

        # self.frontSensor = self.sim.getObject('/'+self.name+'/FRONTSENS')
        # self.backSensor = self.sim.getObject('/' + self.name + '/BACKSENS')
        # self.leftSensor = self.sim.getObject('/' + self.name + '/LEFTSENS')
        # self.rightSensor = self.sim.getObject('/' + self.name + '/RIGHTSENS')
        self.current_edge = Edge.undefined_edge()
        self.moving = Moving.standing

    def get_sensor_data(self, sensor):
        sensor_output = ['result',
                         'distance',
                         'detectedPoint',
                         'detectedObjectHandle',
                         'detectedSurfaceNormalVector']
        out = self.sim.readProximitySensor(sensor)
        return {sensor_output[i]: sim.readProximitySensor(sensor)[i] for i in range(len(sensor_output))}

    def is_in_vertex(self):
        # TODO: Разнести проверку на вершину и добавление вершины в разные методы
        def add_vertex(vertex_type):
            nonlocal results

            add = True
            coords_0 = self.get_coords()
            for vertex in vertices:
                coords_i = vertex.coords
                if distance(*coords_0, *coords_i) <= 0.25:
                    add = False
                    break
            if add:
                vertex = Vertex(*self.get_coords(), *([-1]*4), vertex_type)
                vertex_edges = []
                for result in results:
                    if result:
                        vertex_edges.append(Edge(vertex, -1))
                    else:
                        vertex_edges.append(-1)
                    if self.moving > 0:
                        if self.current_edge.vert2 == -1:
                            self.current_edge.vert2 = vertex
                        vertex_edges[self.moving - 1] = self.current_edge
                vertex.edges = vertex_edges
                vertices.append(vertex)
                for vertex_edge in vertex_edges:
                    if type(vertex_edge) == Edge:
                        edges.append(vertex_edge)

        results = [self.get_sensor_data(sensor)['result'] for sensor in self.sensors.values()]
        if results.count(1) == 3:
            add_vertex(VertexType.deadlock)
        elif results.count(1) == 1:
            add_vertex(VertexType.triple_fork)
        elif results.count(1) == 4:
            add_vertex(VertexType.cross)
        elif (results != [1, 0, 1, 0]) \
                and (results != [0, 1, 0, 1]) \
                and (results.count(1) == 2):
            add_vertex(VertexType.corner)
        else:
            return False
        return True

    def get_coords(self):
        position = self.sim.getObjectPosition(self.sim.getObject('/youBot'), -1)
        return position[0], position[1]


client = RemoteAPIClient()
sim = client.getObject('sim')
vertices = []
edges = []

client.setStepping(False)

sim.startSimulation()
while (t := sim.getSimulationTime()) < 20:
    # s = f'Simulation time: {t:.2f} [s]'
    # print(s)
    robot = Robot(sim, 'youBot')
    print(robot.is_in_vertex())
    # client.step()
    # backSens = sim.getObject('/youBot/BACKSENS')
    # result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(
    #     backSens)
    # print(result, distance, detectedObjectHandle, detectedSurfaceNormalVector)
    if t > 10:
        exit()
sim.stopSimulation()
