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
    SPEED_BOOST = 1
    _destination_point = (np.NaN, np.NaN)

    def __init__(self, sim, robots_names):
        self.g = nx.Graph()

        def init_sensors(idx):
            _ = self._get_sensor_data(self._sensors[idx]['front'])
            _ = self._get_sensor_data(self._sensors[idx]['right'])
            _ = self._get_sensor_data(self._sensors[idx]['back'])
            _ = self._get_sensor_data(self._sensors[idx]['left'])

        global vertices
        self.names = robots_names
        self._destination_point = [(np.NaN, np.NaN) for n in self.names]
        self.sim = sim
        self._sensors = []
        for name in self.names:
            self._sensors += [{'front': self.sim.getObject('/' + name + '/FRONTSENS'),
                             'right': self.sim.getObject('/' + name + '/RIGHTSENS'),
                             'back': self.sim.getObject('/' + name + '/BACKSENS'),
                             'left': self.sim.getObject('/' + name + '/LEFTSENS')}]
        self.current_edges = [Edge.undefined_edge() for i in self.names]
        self.current_vertexes = [Vertex.undefined_vertex() for i in self.names]
        self.movings = [Moving.standing for i in self.names]
        [init_sensors(idx) for idx in range(len(self.names))]


    def _get_sensor_data(self, sensor):
        sensor_output = ['result',
                         'distance',
                         'detectedPoint',
                         'detectedObjectHandle',
                         'detectedSurfaceNormalVector']
        return {sensor_output[i]: sim.readProximitySensor(sensor)[i] for i in range(len(sensor_output))}

    def _get_wall_detection(self, idx):
        return [self._get_sensor_data(sensor)['result'] for sensor in self._sensors[idx].values()]

    def is_in_vertex(self, robot_idx):
        results = self._get_wall_detection(robot_idx)
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

    def _add_vertex(self, vertex_type, idx):
        def connect_vertex(vertex):
            if self.movings[idx] < 4:
                if self.current_edges[idx].vert2 == -1:
                    self.current_edges[idx].vert2 = vertex
                    self.current_edges[idx].length = self.current_edges[idx].vert1.coords.distance(self.current_edges[idx].vert2.coords)
                    plot_edge(self.current_edges[idx])
                if (self.movings[idx] == 0) or (self.movings[idx] == 1):
                    vertex.edges[self.movings[idx] + 2] = self.current_edges[idx]
                else:
                    vertex.edges[self.movings[idx] - 2] = self.current_edges[idx]
            return vertex

        add = True

        results = self._get_wall_detection(idx)
        coords_0 = self.get_coords(idx)

        for vertex in vertices:
            coords_i = vertex.coords
            if coords_0.distance(coords_i) <= 0.3:
                add = False
                self.current_vertexes[idx] = vertex
                self.current_vertexes[idx] = connect_vertex(self.current_vertexes[idx])
                break

        if add:
            vertex = Vertex(get_cell_center(self.get_coords(idx)), *([-1] * 4), vertex_type, len(vertices))
            vertex_edges = []
            for result in results:
                if result:
                    vertex_edges.append(-1)
                else:
                    vertex_edges.append(Edge(vertex, -1))

            vertex.edges = vertex_edges
            vertices.append(vertex)
            self.current_vertexes[idx] = vertex
            for vertex_edge in vertex_edges:
                if (type(vertex_edge) == Edge) and (vertex_edge not in edges):
                    edges.append(vertex_edge)
            connect_vertex(self.current_vertexes[idx])
        self.current_edges[idx] = Edge.undefined_edge()

    def get_coords(self, idx):
        position = self.sim.getObjectPosition(self.sim.getObject('/youBot' + f'[{str(idx)}]'), -1)
        return Point(position[0], position[1])

    def get_orientation(self, idx):
        angles = sim.getObjectOrientation(self.sim.getObject('/youBot' + f'[{str(idx)}]'), -1)
        return angles[0], angles[1]

    def _choose_path(self, idx):
        if self.is_in_vertex(idx)[1]:
            present_edges = [edge for edge in self.current_vertexes[idx].edges
                             if edge != -1 and not edge.checked]

            if len(present_edges) > 0:
                # Рандомный выбор
                # выбирается если есть неисследованные рёбра и отмечает выбранное
                chosen_edge = random.choice([i for i in present_edges if i != -1])
                chosen_edge.checked = True

            else:
                # TODO: Отладить движение по Дейкстре(в какой-то момент накапливается ошибка и он цепляется за стену)
                # self.g = nx.Graph()
                self.g.add_weighted_edges_from(adjacency_list)

                path_list = [nx.dijkstra_path(self.g, str(self.current_vertexes[idx].id), str(v.id)) for v in vertices for i in range(4) if
                                   v.id != self.current_vertexes[idx].id and
                                   False in [edge.checked for edge in v.edges if edge != -1] and
                                  (str(v.id), str(self.current_vertexes[idx].id), float(i)) in adjacency_list]

                chosen_edge = min(path_list) if path_list != [] else nx.dijkstra_path(self.g, str(self.current_vertexes[idx].id), str(idx))

                print('##', [edge.checked for edge in vertices[int(chosen_edge[0])].edges if edge != -1])

                if len(chosen_edge) == 1:
                    self.stay(idx)

                v1 = vertices[int(chosen_edge[0])]
                v2 = vertices[int(chosen_edge[1])]
                for edge in v1.edges:
                    if type(edge) == Edge:
                        if v2 in (edge.vert1, edge.vert2):
                            chosen_edge = edge

            chosen_edge.checked = True

            self.movings[idx] = self.current_vertexes[idx].edges.index(chosen_edge)
            self._destination_point[idx] = get_neighbour_cell_center(self.get_coords(idx),
                                                                self.movings[idx])
            self.current_edges[idx] = chosen_edge
            return chosen_edge
        else:
            raise Exception("Выбор пути должен производиться только на ветвлении!")

    def _set_movement(self, forward_back_vel, left_right_vel, rotation_vel, idx):
        wheel_joints = [sim.getObject('/' + self.names[idx] + '/rollingJoint_fl'),
                        sim.getObject('/' + self.names[idx] + '/rollingJoint_rl'),
                        sim.getObject('/' + self.names[idx] + '/rollingJoint_rr'),
                        sim.getObject('/' + self.names[idx] + '/rollingJoint_fr')]
        sim.setJointTargetVelocity(wheel_joints[0], -forward_back_vel - left_right_vel - rotation_vel)
        sim.setJointTargetVelocity(wheel_joints[1], -forward_back_vel + left_right_vel - rotation_vel)
        sim.setJointTargetVelocity(wheel_joints[2], -forward_back_vel - left_right_vel + rotation_vel)
        sim.setJointTargetVelocity(wheel_joints[3], -forward_back_vel + left_right_vel + rotation_vel)

    def _move(self, robot_idx):
        global adjacency_list
        in_vertex, vertex_type = self.is_in_vertex(robot_idx)
        if in_vertex:
            self._add_vertex(vertex_type, robot_idx)
        adjacency_list = [(str(edge.vert1.id), str(edge.vert2.id), edge.length) for edge in edges
                          if (edge.vert1 != -1) and (edge.vert2 != -1)]
        print(adjacency_list)

        if in_vertex:
            print(self.names[robot_idx], [edge.checked for edge in self.current_vertexes[robot_idx].edges if edge != -1])
            self._choose_path(robot_idx)
        else:
            self._destination_point[robot_idx] = get_neighbour_cell_center(self.get_coords(robot_idx), self.movings[robot_idx])

        if (self._destination_point[robot_idx].x != np.NaN) and (self._destination_point[robot_idx].y != np.NaN):
            destination_vector = Vector(self.get_coords(robot_idx), self._destination_point[robot_idx])
            alpha = self.get_orientation(robot_idx)[1] - Vector(Point(0, 0), Point(0, 1)).angle(destination_vector)

            forward_back_vel = abs(destination_vector.length() * np.cos(alpha))
            left_right_vel = abs(destination_vector.length() * np.sin(alpha))

            destination_angle = Vector(Point(0, 0), Point(0, 1)).signed_angle(destination_vector) - \
                                self.get_orientation(robot_idx)[1]

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

            self._set_movement(self.SPEED_BOOST * forward_back_vel, self.SPEED_BOOST * left_right_vel, 0, robot_idx)

    def stay(self, idx):
        while 1:
            self._set_movement(0, 0, 0, idx)

    def start(self):
        while 1:
            for robot in self.names:
                robot_idx = self.names.index(robot)
                self._move(robot_idx)
            while any([self.get_coords(self.names.index(robot)).distance(
                    self._destination_point[self.names.index(robot)]) > 0.2 for robot in self.names]):
                # print(self._destination_point.x, self._destination_point.y)
                for robot in self.names:
                    robot_idx = self.names.index(robot)
                    if self.get_coords(robot_idx).distance(self._destination_point[robot_idx]) <= 0.2:
                        self._set_movement(0, 0, 0, robot_idx)
                continue


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
robots = Robot(sim, ['youBot[0]', 'youBot[1]'])
if sim.getSimulationTime() > 1:
    robots.start()
