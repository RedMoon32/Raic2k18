import math
import json
import copy

eps = 0.005


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def copy(self):
        return Vector(self.x, self.y, self.z)

    def dot(self, v):
        return v.x * self.x + v.y * self.y + v.z * self.z

    def distance(self, v1):
        if isinstance(v1, Entity):
            return self.distance(v1.position)
        l1 = (v1.x - self.x) ** 2
        l2 = (v1.y - self.y) ** 2
        l3 = (v1.z - self.z) ** 2
        return math.sqrt(l1 + l2 + l3)

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def clamp(self, clamp_length):
        if isinstance(clamp_length, Vector):
            if self.length() == 0:
                return Vector()
            l = clamp_length.length() / self.length()
        else:
            if self.length() == 0:
                return Vector()
            l = clamp_length / self.length()

        if l < 1:
            return Vector(self.x * l, self.y * l, self.z * l)
        else:
            return self

    def normalize(self):
        l = self.length()
        if l == 0:
            return self * 1
        return Vector(self.x / l, self.y / l, self.z / l)

    def to_2d(self, y=1):
        return Vector(self.x, y, self.z)

    def __sub__(self, other):
        return self + other * -1

    def __add__(self, other):
        v = Vector(self.x + other.x, self.y + other.y, self.z + other.z)
        return v

    def __rmul__(self, other):
        return self.__mul__(other)

    def __mul__(self, other):
        return Vector(self.x * other, self.y * other, self.z * other)

    def __str__(self):
        return '<{},{},{}>'.format(round(self.x, 2), round(self.y, 2), round(self.z, 2))

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return abs(self.x - other.x) <= eps and abs(self.y - other.y) <= eps and abs(self.z - other.z) <= eps

    def __len__(self):
        return self.length()

    def rounded(self):
        return Vector(round(self.x), round(self.y), round(self.z))


class EAction:
    target_velocity = Vector()
    jump_speed = 0.0
    use_nitro = False


class Render:
    objects = []
    remove_ticks = []

    def add_line(self, v1: Vector, v2: Vector, remove_ticks=1):
        self.objects.append({"Line": {"x1": v1.x, "y1": v1.y, "z1": v1.z, "x2": v2.x, "y2": v2.y, "z2": v2.z,
                                      "width": 3.0, "r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0}})
        self.remove_ticks.append(remove_ticks)

    def add_sphere(self, v1: Vector, color=[1, 0, 0], remove_ticks=1,radius = 0.5):
        sphere = {
            "Sphere": {"x": v1.x, "y": v1.y, "z": v1.z, "radius": radius, "r": color[0], "g": color[1], "b": color[2],
                       "a": 1.0}}
        self.objects.append(sphere)
        self.remove_ticks.append(remove_ticks)

    def reset(self):
        self.objects = []
        # for i in range(len(self.remove_ticks)):
        #     self.remove_ticks[i] -= 1
        #     if self.remove_ticks == 0:
        #         self.objects.remove(self.objects[i])
        #         #self.remove_ticks.


def get_ball_entity(ball):
    e = Entity()
    e.action = None
    e.radius = ball.radius
    e.radius_change_speed = 0
    e.position = Vector(ball.x, ball.y, ball.z)
    e.velocity = Vector(ball.velocity_x, ball.velocity_y, ball.velocity_z)
    e.mass = Rulez.BALL_MASS
    e.arena_e = Rulez.BALL_ARENA_E

    return e


def get_robot_entity(robot):
    e = Entity()
    e.touch = robot.touch
    e.radius = robot.radius
    e.position = Vector(robot.x, robot.y, robot.z)
    e.velocity = Vector(robot.velocity_x, robot.velocity_y, robot.velocity_z)
    e.arena_e = Rulez.ROBOT_ARENA_E
    e.mass = Rulez.ROBOT_MASS
    e.id = robot.id
    e.teammate = robot.is_teammate
    e.touch_normal = Vector(robot.touch_normal_x, robot.touch_normal_y, robot.velocity_z)
    return e


class Entity:
    id = 0
    action = EAction()
    position = Vector()
    velocity = Vector()
    target_velocity = Vector()
    touch_normal = Vector()
    use_nitro = False
    touch = False
    nitro = False
    radius = 0.0
    arena_e = 0.0
    mass = 1
    nitro_amount = 0.0
    radius_change_speed = 0.0
    teammate = None

    @property
    def normalized_velocity(self):
        return self.velocity.normalize()

    @property
    def x(self):
        return self.position.x

    @property
    def y(self):
        return self.position.y

    @property
    def z(self):
        return self.position.z

    def distance(self, e):
        return self.position.distance(e)


class Arenz:
    width = 60
    height = 20
    depth = 80
    bottom_radius = 3
    top_radius = 7
    corner_radius = 13
    goal_top_radius = 3
    goal_width = 30
    goal_depth = 10
    goal_height = 10
    goal_side_radius = 1


class Rulez:
    arena = Arenz()
    ROBOT_MIN_RADIUS = 1
    ROBOT_MAX_RADIUS = 1.05
    ROBOT_MAX_JUMP_SPEED = 15
    ROBOT_ACCELERATION = 100
    ROBOT_NITRO_ACCELERATION = 30
    ROBOT_MAX_GROUND_SPEED = 30
    ROBOT_ARENA_E = 0
    ROBOT_RADIUS = 1
    ROBOT_MASS = 2
    TICKS_PER_SECOND = 60
    MICROTICKS_PER_TICK = 100
    RESET_TICKS = 2 * TICKS_PER_SECOND
    BALL_ARENA_E = 0.7
    BALL_RADIUS = 2
    BALL_MASS = 1
    MIN_HIT_E = 0.4
    MAX_HIT_E = 0.5
    MAX_ENTITY_SPEED = 100
    MAX_NITRO_AMOUNT = 100
    START_NITRO_AMOUNT = 50
    NITRO_POINT_VELOCITY_CHANGE = 0.6
    NITRO_PACK_X = 20
    NITRO_PACK_Y = 1
    NITRO_PACK_Z = 30
    NITRO_PACK_RADIUS = 0.5
    NITRO_PACK_AMOUNT = 100
    NITRO_PACK_RESPAWN_TICKS = 10 * TICKS_PER_SECOND
    GRAVITY = 30
    gravity = GRAVITY


def normalize(v):
    return v.normalize()


def dot(a, b):
    return a.dot(b)


def length(v):
    return v.length()


def clamp(*args):
    if len(args) == 2:
        return args[0].clamp(args[1])
    else:
        return min(args[2], max(args[0], args[1]))


def min_dan(d1, d2):
    if (d1[0]) < (d2[0]):
        return d1
    else:
        return d2


def clamp_by_position(v: Vector):
    return Vector(clamp(v.x, -Rulez.arena.width / 2, Rulez.arena.width / 2),
                  clamp(v.y, 1, Rulez.arena.height),
                  clamp(v.z, -Rulez.arena.depth / 2, Rulez.arena.depth / 2))
