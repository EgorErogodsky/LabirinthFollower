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


class Vertex:
    def __init__(self, x, y, vertex_type):
        self.coords = (x, y)
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

    def get_sensor_data(self, sensor):
        sensor_output = ['result',
                         'distance',
                         'detectedPoint',
                         'detectedObjectHandle',
                         'detectedSurfaceNormalVector']
        out = sim.readProximitySensor(sensor)
        return {sensor_output[i]: sim.readProximitySensor(sensor)[i] for i in range(len(sensor_output))}

    def is_in_vertex(self):
        results = [self.get_sensor_data(sensor)['result'] for sensor in self.sensors.values()]
        if results.count(1) == 3:
            self.add_vertex(VertexType.deadlock)
        elif results.count(1) == 1:
            self.add_vertex(VertexType.triple_fork)
        elif results.count(1) == 4:
            self.add_vertex(VertexType.cross)
        elif (results != [1, 0, 1, 0]) \
                and (results != [0, 1, 0, 1]) \
                and (results.count(1) == 2):
            self.add_vertex(VertexType.corner)
        else:
            return False
        return True

    def add_vertex(self, vertex_type):
        add = True
        coords_0 = self.get_coords()
        for vertex in vertices:
            coords_i = vertex.coords
            if distance(*coords_0, *coords_i) <= 0.25:
                add = False
                break
        if add:
            vertices.append(Vertex(*self.get_coords(), vertex_type))

    def get_coords(self):
        position = self.sim.getObjectPosition(self.sim.getObject('/youBot'), -1)
        return position[0], position[1]


client = RemoteAPIClient()
sim = client.getObject('sim')
vertices = []

client.setStepping(True)

sim.startSimulation()
while (t := sim.getSimulationTime()) < 20:
    # s = f'Simulation time: {t:.2f} [s]'
    # print(s)
    robot = Robot(sim, 'youBot')
    print(robot.is_in_vertex())
    client.step()
    # backSens = sim.getObject('/youBot/BACKSENS')
    # result, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(
    #     backSens)
    # print(result, distance, detectedObjectHandle, detectedSurfaceNormalVector)
    if t > 10:
        exit()
sim.stopSimulation()
