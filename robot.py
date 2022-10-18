from graph_structure import *
from zmqRemoteApi import RemoteAPIClient
import random
import networkx as nx

class Moving(enum.IntEnum):
    front = 0
    right = 1
    back = 2
    left = 3
    standing = 5


class Robot:
    _destination_point = (np.NaN, np.NaN)

    def __init__(self, sim, robot_name):
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
        out = self.sim.readProximitySensor(sensor)
        return {sensor_output[i]: sim.readProximitySensor(sensor)[i] for i in range(len(sensor_output))}

    def is_in_vertex(self):
        # TODO: Разнести проверку на вершину и добавление вершины в разные методы ???
        def add_vertex(vertex_type):
            nonlocal results

            add = True
            coords_0 = self.get_coords()
            for vertex in vertices:
                coords_i = vertex.coords
                if coords_0.distance(coords_i) <= 0.25:
                    add = False
                    self.current_vertex = vertex
                    break
            if add:
                vertex = Vertex(self.get_coords(), *([-1] * 4), vertex_type, len(vertices))
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
                    if (type(vertex_edge) == Edge) and (vertex_edge not in edges):
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
        return Point(position[0], position[1])

    def get_orientation(self):
        angles = sim.getObjectOrientation(self.sim.getObject('/youBot'), -1)
        return angles[0], angles[1]

    def _choose_path(self):
        if self.is_in_vertex():
            present_edges = [edge for edge in self.current_vertex.edges
                             if edge != -1]
            status = [e.checked for e in self.current_vertex.edges if e != -1]

            # В первую очередь ошибки искать тут)
            if False in status or self.current_vertex.id == 0:
                # Рандомный выбор
                # выбирается если есть неисследованные рёбра и отмечает выбранное
                chosen_edge = random.choice(present_edges)
                chosen_edge.checked = True
            else:
                # TODO: Сделать занесение вершины раньше этого места, а то в тупике он не видит в графе текущую вершину
                g = nx.Graph()
                g.add_weighted_edges_from(adjacency_list)
                print("**", adjacency_list)
                print(self.current_vertex.edges)
                chosen_edge = min([nx.dijkstra_path(g, str(self.current_vertex.id), v) for v in vertices if v.id != self.current_vertex.id])
            # chosen_edge = present_edges[2]

            if chosen_edge.vert2 != -1:
                self._destination_point = chosen_edge.vert2.coords
            else:
                self.moving = self.current_vertex.edges.index(chosen_edge)
                self._destination_point = get_neighbour_cell_center(self.get_coords(),
                                                                    self.moving)
            chosen_edge.length = 0
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
        adjacency_list = [(edge.vert1.id, edge.vert2.id, edge.length) for edge in edges
                          if (edge.vert1 != -1) and (edge.vert2 != -1)]
        print(adjacency_list)
        if self.is_in_vertex():
            self._choose_path()
        else:
            self._destination_point = get_neighbour_cell_center(self.get_coords(), self.moving)
            self.current_edge.length += 1
        if (self._destination_point.x != np.NaN) and (self._destination_point.y != np.NaN):
            destination_vector = Vector(self.get_coords(), self._destination_point)
            alpha = self.get_orientation()[1] - Vector(Point(0, 0), Point(0, 1)).angle(destination_vector)

            forward_back_vel = destination_vector.length() * np.cos(alpha)
            f_w_x = np.sin(alpha) * forward_back_vel
            f_w_y = abs(np.cos(alpha)) * forward_back_vel
            f_w_vector = Vector(self.get_coords(), self.get_coords() + Point(f_w_x, f_w_y))

            l_r_vector = destination_vector - f_w_vector
            left_right_vel = l_r_vector.length()

            if l_r_vector.signed_angle(Vector(Point(0, 0), Point(0, 1))) - self.get_orientation()[1] < 0:
                left_right_vel = -left_right_vel

            # if ((forward_back_vel < 0) and (l_r_vector.end.is_on_left(f_w_vector))) or \
            #         (forward_back_vel > 0) and (not l_r_vector.end.is_on_left(f_w_vector)):
            #     left_right_vel = -left_right_vel

            # left_right_vel = vec_length(*self.get_coords(), *self._destination_point) * np.sin(alpha)
            self._set_movement(forward_back_vel, left_right_vel, 0)

    def start(self):
        while 1:
            self._move()
            while self.get_coords().distance(self._destination_point) > 0.2:
                #print(self._destination_point.x, self._destination_point.y)
                continue
            self._set_movement(0, 0, 0)

    # def _orient(self):
    #     eps = 0.001
    #     side = 0
    #     while abs(self.get_orientation()[0] * 180 / np.pi - (-90)) > eps:
    #         if self.get_orientation()[0] * 180 / np.pi > -90 and side != 1:
    #             self._set_movement(0, 0, 0.08)
    #             side = 1
    #         elif self.get_orientation()[0] * 180 / np.pi < -90 and side != 2:
    #             self._set_movement(0, 0, -0.08)
    #             side = 2
    #     self._set_movement(0, 0, 0)


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
# print(robot.is_in_vertex())
if sim.getSimulationTime() > 1:
    robot.start()
    # client.step()
    # backSens = sim.getObject('/youBot/BACKSENS')
    # result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(
    #     backSens)
    # print(result, distance, detectedObjectHandle, detectedSurfaceNormalVector)
    # if t > 10:
    #     exit()
# sim.stopSimulation()

