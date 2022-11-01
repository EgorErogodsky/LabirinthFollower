import numpy as np


class Point:
    def __init__(self, x, y):
        self.x, self.y = x, y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def is_on_left(self, vec):
        return ((vec.end.x - vec.start.x) * (self.y - vec.start.y) -
                (vec.end.y - vec.start.y) * (self.x - vec.start.x)) > 0

    def distance(self, p2):
        return np.sqrt((self.x - p2.x) ** 2 + (self.y - p2.y) ** 2)


class Vector:
    def __init__(self, p1, p2):
        self.start = p1
        self.end = p2

    def length(self):
        return np.sqrt((self.end.x - self.start.x) ** 2 + (self.end.y - self.start.y) ** 2)

    def __add__(self, other):
        other = other.to_origin().end
        return Vector(self.start + other.end, self.end + other.end)

    def __sub__(self, other):
        v2 = other.to_origin()
        v1 = self.to_origin()
        v_res_end = v1.end - v2.end
        return Vector(self.start, self.start + v_res_end)

    def angle(self, vec2):
        x_a = self.end.x - self.start.x
        y_a = self.end.y - self.start.y
        x_b = vec2.end.x - vec2.start.x
        y_b = vec2.end.y - vec2.start.y
        alpha = np.arccos((x_a * x_b + y_a * y_b) /
                          (np.sqrt(x_a ** 2 + y_a ** 2) *
                           np.sqrt(x_b ** 2 + y_b ** 2)))
        return alpha

    def signed_angle(self, vec2):
        vec1 = self.to_origin().end
        vec2 = vec2.to_origin().end
        dot = vec1.x * vec2.x + vec1.y * vec2.y
        det = vec1.x * vec2.y - vec1.y * vec2.x
        return -np.arctan2(det, dot)

    def to_origin(self):
        return Vector(Point(0, 0), Point(self.end.x - self.start.x, self.end.y - self.start.y))


def get_neighbour_cell_center(object_coords, side):
    cell_center = get_cell_center(object_coords)
    x_c, y_c = cell_center.x, cell_center.y
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
    return Point(x_n, y_n)


def get_cell_center(object_coords):
    x, y = object_coords.x, object_coords.y
    x1, x2 = np.floor(x), np.ceil(x)
    y1, y2 = np.floor(y), np.ceil(y)
    x_c, y_c = (x2 + x1) / 2, (y2 + y1) / 2
    return Point(x_c, y_c)
