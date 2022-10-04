# TODO: Организовать файловую структуру
import enum
import math
import random

import numpy as np

from zmqRemoteApi import RemoteAPIClient


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_neighbour_cell_center(object_coords, side):
    x, y = object_coords
    x1, x2 = np.floor(x), np.ceil(x)
    y1, y2 = np.floor(y), np.ceil(y)
    x_c, y_c = (x2 + x1) / 2, (y2 + y1) / 2
    match side:
        case 0:  # front
            x_n, y_n = x_c, y_c + 1
        case 1:  # right
            x_n, y_n = x_c + 1, y_c
        case 2:  # back
            x_n, y_n = x_c, y_c - 1
        case 3:  # left
            x_n, y_n = x_c - 1, y_c
        case _:
            raise ValueError('Неверно задана сторона!')
    return x_n, y_n


class VertexType(enum.Enum):
    deadlock = 1
    corner = 2
    triple_fork = 3
    cross = 4


class Moving(enum.IntEnum):
    front = 0
    right = 1
    back = 2
    left = 3
    standing = 5


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

    @classmethod
    def undefined_vertex(cls):
        return cls(*([-1] * 7))


class Robot:
    _destination_point = (np.NaN, np.NaN)

    def __init__(self, sim, robot_name):
        global vertices
        self.name = robot_name
        self.sim = sim
        self._sensors = {'front': self.sim.getObject('/' + self.name + '/FRONTSENS'),
                         'right': self.sim.getObject('/' + self.name + '/RIGHTSENS'),
                         'back': self.sim.getObject('/' + self.name + '/BACKSENS'),
                         'left': self.sim.getObject('/' + self.name + '/LEFTSENS')}
        self.current_edge = Edge.undefined_edge()
        self.current_vertex = Vertex.undefined_vertex()
        self.moving = Moving.standing
        _ = self._get_sensor_data(self._sensors['front'])
        _ = self._get_sensor_data(self._sensors['right'])
        _ = self._get_sensor_data(self._sensors['back'])
        _ = self._get_sensor_data(self._sensors['left'])

    def _get_sensor_data(self, sensor):
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
                    self.current_vertex = vertex
                    break
            if add:
                vertex = Vertex(*self.get_coords(), *([-1] * 4), vertex_type)
                vertex_edges = []
                for result in results:
                    if result:
                        vertex_edges.append(-1)
                    else:
                        vertex_edges.append(Edge(vertex, -1))
                if self.moving < 4:
                    if self.current_edge.vert2 == -1:
                        self.current_edge.vert2 = vertex
                    if (self.moving == 0) or (self.moving == 1):
                        vertex_edges[self.moving + 2] = self.current_edge
                    else:
                        vertex_edges[self.moving - 2] = self.current_edge
                vertex.edges = vertex_edges
                vertices.append(vertex)
                self.current_vertex = vertex
                for vertex_edge in vertex_edges:
                    if type(vertex_edge) == Edge:
                        edges.append(vertex_edge)
            self.current_edge = Edge.undefined_edge()

        results = [self._get_sensor_data(sensor)['result'] for sensor in self._sensors.values()]
        if results.count(1) == 3:
            add_vertex(VertexType.deadlock)
        elif results.count(1) == 1:
            add_vertex(VertexType.triple_fork)
        elif results.count(1) == 0:
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

    def _choose_path(self):
        if self.is_in_vertex():
            present_edges = [edge for edge in self.current_vertex.edges
                             if edge != -1]
            # Рандомный выбор
            # TODO: Заменить на выбор по алгоритму обхода лабиринта
            chosen_edge = random.choice(present_edges)
            if chosen_edge.vert2 != -1:
                self._destination_point = chosen_edge.vert2.coords
            else:
                self.moving = self.current_vertex.edges.index(chosen_edge)
                self._destination_point = get_neighbour_cell_center(self.get_coords(),
                                                                    self.moving)
            return chosen_edge, self._destination_point
        else:
            raise Exception("Выбор пути должен производиться только на ветвлении!")

    def _set_movement(self, forward_back_vel, left_right_vel, rotation_vel):
        wheel_joints = [sim.getObject('/' + self.name + '/rollingJoint_fl'),
                        sim.getObject('/' + self.name + '/rollingJoint_rl'),
                        sim.getObject('/' + self.name + '/rollingJoint_rr'),
                        sim.getObject('/' + self.name + '/rollingJoint_fr')]
        sim.setJointTargetVelocity(wheel_joints[0], -forward_back_vel - left_right_vel - rotation_vel)
        sim.setJointTargetVelocity(wheel_joints[1], -forward_back_vel + left_right_vel - rotation_vel)
        sim.setJointTargetVelocity(wheel_joints[2], -forward_back_vel - left_right_vel + rotation_vel)
        sim.setJointTargetVelocity(wheel_joints[3], -forward_back_vel + left_right_vel + rotation_vel)

    def move(self):
        if self.is_in_vertex():
            chosen_edge, self._destination_point = self._choose_path()
        else:
            self._destination_point = get_neighbour_cell_center(self.get_coords(), self.moving)
        if self._destination_point[0] != np.NaN and self._destination_point[1] != np.NaN:
            forward_back_vel = 3 * (self._destination_point[1] - self.get_coords()[1])
            left_right_vel = self._destination_point[0] - self.get_coords()[0]
            self._set_movement(forward_back_vel, left_right_vel, 0)
        while distance(*self.get_coords(), *self._destination_point) > 0.1:
            print()
        self._set_movement(0, 0, 0)


client = RemoteAPIClient()
sim = client.getObject('sim')
vertices = []
edges = []

client.setStepping(False)

sim.startSimulation()
# while (t := sim.getSimulationTime()) < 20:
# s = f'Simulation time: {t:.2f} [s]'
# print(s)
robot = Robot(sim, 'youBot')
# print(robot.is_in_vertex())
if sim.getSimulationTime() > 1:
    while 1:
        robot.move()
    # client.step()
    # backSens = sim.getObject('/youBot/BACKSENS')
    # result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(
    #     backSens)
    # print(result, distance, detectedObjectHandle, detectedSurfaceNormalVector)
    # if t > 10:
    #     exit()
# sim.stopSimulation()
