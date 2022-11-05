import random

import networkx as nx

from graph_structure import *
from zmqRemoteApi import RemoteAPIClient


class Moving(enum.IntEnum):
    front = 0
    right = 1
    back = 2
    left = 3
    standing = 5


class Robot:
    SPEED_BOOST = 5
    _destination_point = (np.NaN, np.NaN)

    def __init__(self, sim, robot_name):
        self.g = nx.Graph()
        def init_sensors():
            _ = self._get_sensor_data(self._sensors['front'])
            _ = self._get_sensor_data(self._sensors['right'])
            _ = self._get_sensor_data(self._sensors['back'])
            _ = self._get_sensor_data(self._sensors['left'])

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
        init_sensors()

    def _get_sensor_data(self, sensor):
        sensor_output = ['result',
                         'distance',
                         'detectedPoint',
                         'detectedObjectHandle',
                         'detectedSurfaceNormalVector']
        return {sensor_output[i]: sim.readProximitySensor(sensor)[i] for i in range(len(sensor_output))}

    def _get_wall_detection(self):
        return [self._get_sensor_data(sensor)['result'] for sensor in self._sensors.values()]

    def is_in_vertex(self):
        results = self._get_wall_detection()
        if results.count(1) == 3:
            vertex_type = VertexType.deadlock
        elif results.count(1) == 1:
            vertex_type = VertexType.triple_fork
        elif results.count(1) == 0:
            vertex_type = VertexType.cross
        elif (results != [1, 0, 1, 0]) \
                and (results != [0, 1, 0, 1]) \
                and (results.count(1) == 2):
            vertex_type = VertexType.corner
        else:
            return False, None
        return True, vertex_type

    def _add_vertex(self, vertex_type):
        add = True

        results = self._get_wall_detection()
        coords_0 = self.get_coords()

        for vertex in vertices:
            coords_i = vertex.coords
            if coords_0.distance(coords_i) <= 0.3:
                add = False
                self.current_vertex = vertex

                if self.moving < 4:
                    if self.current_edge.vert2 == -1:
                        self.current_edge.vert2 = self.current_vertex
                        plot_edge(self.current_edge)
                    if (self.moving == 0) or (self.moving == 1):
                        self.current_vertex.edges[self.moving + 2] = self.current_edge
                    else:
                        self.current_vertex.edges[self.moving - 2] = self.current_edge
                break

        if add:
            vertex = Vertex(get_cell_center(self.get_coords()), *([-1] * 4), vertex_type, len(vertices))
            vertex_edges = []
            for result in results:
                if result:
                    vertex_edges.append(-1)
                else:
                    vertex_edges.append(Edge(vertex, -1))
            if self.moving < 4:
                if self.current_edge.vert2 == -1:
                    self.current_edge.vert2 = vertex
                    plot_edge(self.current_edge)
                if (self.moving == 0) or (self.moving == 1):
                    vertex_edges[self.moving + 2] = self.current_edge
                else:
                    vertex_edges[self.moving - 2] = self.current_edge
            vertex.edges = vertex_edges
            vertices.append(vertex)
            self.current_vertex = vertex
            for vertex_edge in vertex_edges:
                if (type(vertex_edge) == Edge) and (vertex_edge not in edges):
                    edges.append(vertex_edge)
        self.current_edge = Edge.undefined_edge()

    def get_coords(self):
        position = self.sim.getObjectPosition(self.sim.getObject('/youBot'), -1)
        return Point(position[0], position[1])

    def get_orientation(self):
        angles = sim.getObjectOrientation(self.sim.getObject('/youBot'), -1)
        return angles[0], angles[1]

    def _choose_path(self):
        if self.is_in_vertex()[1]:
            present_edges = [edge for edge in self.current_vertex.edges
                             if edge != -1 and not edge.checked]

            if len(present_edges) > 0:
                # Рандомный выбор
                # выбирается если есть неисследованные рёбра и отмечает выбранное
                chosen_edge = random.choice([i for i in present_edges if i!=-1])
                chosen_edge.checked = True
            else:
                # TODO: Отладить движение по Дейкстре
                #self.g = nx.Graph()
                self.g.add_weighted_edges_from(adjacency_list)
                print("**", adjacency_list)
                print(self.current_vertex.edges)
                chosen_edge = min([nx.dijkstra_path(self.g, str(self.current_vertex.id), str(v.id)) for v in vertices if
                                   v.id != self.current_vertex.id and False in
                                   [edge.checked for edge in v.edges if edge != -1]])

                v1 = vertices[int(chosen_edge[0])]
                v2 = vertices[int(chosen_edge[1])]
                for edge in v1.edges:
                    if type(edge) == Edge:
                        if v2 in (edge.vert1, edge.vert2):
                            chosen_edge = edge

                chosen_edge.checked = True

            self.moving = self.current_vertex.edges.index(chosen_edge)
            self._destination_point = get_neighbour_cell_center(self.get_coords(),
                                                                self.moving)
            chosen_edge.length = 1
            self.current_edge = chosen_edge
            return chosen_edge
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

    def _move(self):
        global adjacency_list

        in_vertex, vertex_type = self.is_in_vertex()
        if in_vertex:
            self._add_vertex(vertex_type)
        adjacency_list = [(str(edge.vert1.id), str(edge.vert2.id), edge.length) for edge in edges
                          if (edge.vert1 != -1) and (edge.vert2 != -1)]
        print(adjacency_list)

        if in_vertex:
            print([edge.checked for edge in self.current_vertex.edges if edge != -1])
            self._choose_path()
        else:
            self._destination_point = get_neighbour_cell_center(self.get_coords(), self.moving)
            self.current_edge.length += 1

        if (self._destination_point.x != np.NaN) and (self._destination_point.y != np.NaN):
            destination_vector = Vector(self.get_coords(), self._destination_point)
            alpha = self.get_orientation()[1] - Vector(Point(0, 0), Point(0, 1)).angle(destination_vector)

            forward_back_vel = abs(destination_vector.length() * np.cos(alpha))
            left_right_vel = abs(destination_vector.length() * np.sin(alpha))

            destination_angle = Vector(Point(0, 0), Point(0, 1)).signed_angle(destination_vector) - \
                                self.get_orientation()[1]

            if destination_angle > np.pi:
                destination_angle = -(2 * np.pi - destination_angle)
            elif destination_angle < -np.pi:
                destination_angle = 2 * np.pi + destination_angle
            # print(destination_angle)
            if np.pi / 2 <= destination_angle < np.pi:
                forward_back_vel = -forward_back_vel
            elif -np.pi <= destination_angle < -np.pi / 2:
                forward_back_vel = -forward_back_vel
                left_right_vel = -left_right_vel
            elif -np.pi / 2 <= destination_angle < 0:
                left_right_vel = -left_right_vel

            self._set_movement(self.SPEED_BOOST * forward_back_vel, self.SPEED_BOOST * left_right_vel, 0)

    def start(self):
        while 1:
            self._move()
            while self.get_coords().distance(self._destination_point) > 0.2:
                # print(self._destination_point.x, self._destination_point.y)
                continue
            self._set_movement(0, 0, 0)


client = RemoteAPIClient()
sim = client.getObject('sim')
vertices = []
edges = []
# Матрица смежности
adjacency_list = []

client.setStepping(False)

sim.startSimulation()
# while (t := sim.getSimulationTime()) < 20:
# s = f'Simulation time: {t:.2f} [s]'
# print(s)
robot = Robot(sim, 'youBot')
if sim.getSimulationTime() > 1:
    robot.start()
