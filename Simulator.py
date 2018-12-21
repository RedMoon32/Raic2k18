from model import *
from MyStrategy import Vector
import random

arena = None


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


class Simulator:
    def __init__(self, rules, game):
        self.rules, self.game = rules, game

    def move(self, e, delta_time=0.0):
        time = delta_time
        e.velocity = min(e.velocity, self.rules)
        e.position += e.velocity * time
        e.position.y = self.rules.GRAVITY * time ** 2 / 2
        e.velocity.y -= self.rules.GRAVITY * time

    def collide_entities(self, a, b):
        delta_position = b.position - a.position
        distance = delta_position.length()
        penetration = a.radius + b.radius - distance
        if penetration > 0:
            k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass))
            k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass))
            normal = delta_position.normalize()
            a.position -= normal * penetration * k_a
            b.position += normal * penetration * k_b
            delta_velocity = normal.dot(b.velocity - a.velocity) + b.radius_change_speed - a.radius_change_speed
            if delta_velocity < 0:
                impulse = normal * (1 + random.randint(self.rules.MIN_HIT_E, self.rules.MAX_HIT_E)) * delta_velocity
                a.velocity += impulse * k_a
                b.velocity -= impulse * k_b

    def collide_with_arena(self, e):
        distance, normal = self.dan_to_arena(e.position)
        penetration = e.radius - distance
        if penetration > 0:
            e.position += penetration * normal
            velocity = e.velocity.dot(normal) - e.radius_change_speed
            if velocity < 0:
                e.velocity -= (1 + e.arena_e) * velocity * normal
                return normal
        return None

    def update(self, delta_time: float):
        robots = random.shuffle(self.game.robots)
        for robot in robots:
            if robot.touch:
                target_velocity = \
                    robot.action.target_velocity.clamp(self.rules.ROBOT_MAX_GROUND_SPEED)
                target_velocity -= robot.touch_normal * robot.touch_normal.dot(target_velocity)
                target_velocity_change = target_velocity - robot.velocity
                if target_velocity_change.length() > 0:
                    acceleration = self.rules.ROBOT_ACCELERATION * max(0, robot.touch_normal.y)
                    robot.velocity += clamp(target_velocity_change.normalize() * acceleration * delta_time,
                                            target_velocity_change)
            if robot.action.use_nitro:
                target_velocity_change = clamp(robot.action.target_velocity - robot.velocity,
                                               robot.nitro * self.rules.NITRO_POINT_VELOCITY_CHANGE)
                if length(target_velocity_change) > 0:
                    acceleration = normalize(target_velocity_change) * self.rules.ROBOT_NITRO_ACCELERATION
                    velocity_change = clamp(acceleration * delta_time, length(target_velocity_change))
                    robot.velocity += velocity_change
                    robot.nitro -= length(velocity_change) / self.rules.NITRO_POINT_VELOCITY_CHANGE
            self.move(robot, delta_time)
            robot.radius = self.rules.ROBOT_MIN_RADIUS + (
                    self.rules.ROBOT_MAX_RADIUS - self.rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed / self.rules.ROBOT_MAX_JUMP_SPEED
            robot.radius_change_speed = robot.action.jump_speed
        self.move(self.game.ball, delta_time)
        for i in range(len(robots)):
            for j in range(i):
                self.collide_entities(self.game.robots[i], self.game.robots[j])
        for robot in robots:
            self.collide_entities(robot, self.game.ball)
            collision_normal = self.collide_with_arena(robot)
            if collision_normal is None:
                robot.touch = False
            else:
                robot.touch = True
                robot.touch_normal = collision_normal
        self.collide_with_arena(self.game.ball)
        if abs(self.game.ball.position.z) > self.rules.arena.depth / 2 + self.game.ball.radius:
            self.goal_scored()
        for robot in self.game.robots:
            if robot.niro == self.rules.MAX_NITRO_AMOUNT:
                continue
            for pack in self.game.nitro_packs:
                if not pack.alive:
                    continue
                if length(robot.position - pack.position) <= robot.radius + pack.radius:
                    robot.nitro = self.rules.MAX_NITRO_AMOUNT
                    pack.alive = False
                    pack.respawn_ticks = self.rules.NITRO_PACK_RESPAWN_TICKS

    def tick(self):
        self.delta_time = 1 / self.rules.TICKS_PER_SECOND
        for _ in range(self.rules.MICROTICKS_PER_TICK):
            self.update(delta_time=self.delta_time / self.rules.MICROTICKS_PER_TICK)

        for pack in self.game.nitro_packs:
            if pack.alive:
                continue
            pack.respawn_ticks -= 1
            if pack.respawn_ticks == 0:
                pack.alive = True

    def goal_scored(self):
        print('Goal')

    def dan_to_plane(self, point, point_on_plane, plane_normal):
        return {
            'distance': dot(point - point_on_plane, plane_normal),
            'normal': plane_normal
        }['distance']

    def dan_to_sphere_inner(self, point, sphere_center, sphere_radius):
        return {
            'distance': sphere_radius - length(point - sphere_center),
            'normal': normalize(sphere_center - point)
        }['distance']

    def dan_to_sphere_outer(self, point, sphere_center, sphere_radius):
        return {
            'distance': length(point - sphere_center) - sphere_radius,
            'normal': normalize(point - sphere_center)
        }['distance']

    def dan_to_arena_quarter(self, point):
        dan = self.dan_to_plane(point, Vector(0, 0, 0), Vector(0, 1, 0))
        dan = min(dan, self.dan_to_plane(point, Vector(0, self.game.arena.height, 0), Vector(0, -1, 0)))
        dan = min(dan, self.dan_to_plane(point, Vector(arena.width / 2, 0, 0), Vector(-1, 0, 0)))
        dan = min(dan, self.dan_to_plane(
            point,
            Vector(0, 0, (self.game.arena.depth / 2) + self.game.arena.goal_depth),
            Vector(0, 0, -1)))
        v = Vector(point.x - (self.game.arena.goal_width / 2) + self.game.arena.goal_top_radius,
                   point.y - self.game.arena.goal_height + self.game.arena.goal_top_radius, 0)
        if point.x >= (
                self.game.arena.goal_width / 2) + self.game.arena.goal_side_radius or point.y >= self.game.arena.goal_height + self.game.arena.goal_side_radius or (
                v.x > 0
                and v.y > 0
                and length(v) >= self.game.arena.goal_top_radius + self.game.arena.goal_side_radius):
            dan = min(dan, self.dan_to_plane(point, Vector(0, 0, self.game.arena.depth / 2), Vector(0, 0, -1)))

        if point.z >= (self.game.arena.depth / 2) + self.game.arena.goal_side_radius:
            dan = min(dan, self.dan_to_plane(
                point,
                Vector(self.game.arena.goal_width / 2, 0, 0),
                Vector(-1, 0, 0)))
            dan = min(dan, self.dan_to_plane(point, Vector(0, self.game.arena.goal_height, 0), Vector(0, -1, 0)))

        assert self.game.arena.bottom_radius == self.game.arena.goal_top_radius
        if point.z > (self.game.arena.depth / 2) + self.game.arena.goal_depth - self.game.arena.bottom_radius:
            dan = min(dan, self.dan_to_sphere_inner(
                point,
                Vector(
                    clamp(
                        point.x,
                        self.game.arena.bottom_radius - (self.game.arena.goal_width / 2),
                        (self.game.arena.goal_width / 2) - self.game.arena.bottom_radius,
                    ),
                    clamp(
                        point.y,
                        self.game.arena.bottom_radius,
                        self.game.arena.goal_height - self.game.arena.goal_top_radius,
                    ),
                    (self.game.arena.depth / 2) + self.game.arena.goal_depth - self.game.arena.bottom_radius),
                self.game.arena.bottom_radius))
        if point.x > (self.game.arena.width / 2) - self.game.arena.corner_radius and point.z > (
                self.game.arena.depth / 2) - self.game.arena.corner_radius:
            dan = min(dan, self.dan_to_sphere_inner(
                point,
                Vector(
                    (self.game.arena.width / 2) - self.game.arena.corner_radius,
                    point.y,
                    (self.game.arena.depth / 2) - self.game.arena.corner_radius),
                self.game.arena.corner_radius))
        if point.z < (self.game.arena.depth / 2) + self.game.arena.goal_side_radius:

            if point.x < (self.game.aarena.goal_width / 2) + self.game.arena.goal_side_radius:
                dan = min(dan, self.dan_to_sphere_outer(
                    point,
                    Vector(
                        (self.game.arena.goal_width / 2) + self.game.arena.goal_side_radius,
                        point.y,
                        (self.game.arena.depth / 2) + self.game.arena.goal_side_radius
                    ),
                    self.game.arena.goal_side_radius))
            if point.y < arena.goal_height + arena.goal_side_radius:
                dan = min(dan, self.dan_to_sphere_outer(
                    point,
                    Vector(
                        point.x,
                        arena.goal_height + arena.goal_side_radius,
                        (arena.depth / 2) + arena.goal_side_radius
                    ),
                    arena.goal_side_radius))
            o = Vector(
                (arena.goal_width / 2) - arena.goal_top_radius,
                arena.goal_height - arena.goal_top_radius, 0
            )
            v = Vector(point.x, point.y, 0) - o
            if v.x > 0 and v.y > 0:
                o = o + normalize(v) * (arena.goal_top_radius + arena.goal_side_radius)
                dan = min(dan, self.dan_to_sphere_outer(
                    point,
                    (o.x, o.y, (arena.depth / 2) + arena.goal_side_radius),
                    arena.goal_side_radius))
            if point.z > (
                    arena.depth / 2) + arena.goal_side_radius and point.y > arena.goal_height - arena.goal_top_radius:

                if point.x > (arena.goal_width / 2) - arena.goal_top_radius:
                    dan = min(dan, self.dan_to_sphere_inner(
                        point,
                        Vector(
                            (arena.goal_width / 2) - arena.goal_top_radius,
                            arena.goal_height - arena.goal_top_radius,
                            point.z
                        ),
                        arena.goal_top_radius))
                if point.z > (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius:
                    dan = min(dan, self.dan_to_sphere_inner(
                        point,
                        Vector(point.x,
                               arena.goal_height - arena.goal_top_radius,
                               (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius
                               ),
                        arena.goal_top_radius))
        if point.y < arena.bottom_radius:
            if point.x > (arena.width / 2) - arena.bottom_radius:
                dan = min(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        (arena.width / 2) - arena.bottom_radius,
                        arena.bottom_radius,
                        point.z
                    ),
                    arena.bottom_radius))
            if point.z > (arena.depth / 2) - arena.bottom_radius \
                    and point.x >= (arena.goal_width / 2) + arena.goal_side_radius:
                dan = min(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        point.x,
                        arena.bottom_radius,
                        (arena.depth / 2) - arena.bottom_radius
                    ),
                    arena.bottom_radius))
            if point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius:
                dan = min(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        point.x,
                        arena.bottom_radius,
                        (arena.depth / 2) + arena.goal_depth - arena.bottom_radius
                    ),
                    arena.bottom_radius))
            o = Vector(
                (arena.goal_width / 2) + arena.goal_side_radius,
                (arena.depth / 2) + arena.goal_side_radius, 0
            )
            v = Vector(point.x, point.y, 0) - o
            if v.x < 0 and v.y < 0 and length(v) < arena.goal_side_radius + arena.bottom_radius:
                o = o + normalize(v) * (arena.goal_side_radius + arena.bottom_radius)
                dan = min(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(o.x, arena.bottom_radius, o.y),
                    arena.bottom_radius))
                if point.z >= (arena.depth / 2) + arena.goal_side_radius \
                        and point.x > (arena.goal_width / 2) - arena.bottom_radius:
                    dan = min(dan, self.dan_to_sphere_inner(point,
                                                            Vector(
                                                                (arena.goal_width / 2) - arena.bottom_radius,
                                                                arena.bottom_radius,
                                                                point.z
                                                            ),
                                                            arena.bottom_radius))
        if point.y > arena.height - arena.top_radius:
            if point.x > (arena.width / 2) - arena.top_radius:
                dan = min(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        (arena.width / 2) - arena.top_radius,
                        arena.height - arena.top_radius,
                        point.z,
                    ),
                    arena.top_radius))
            if point.z > (arena.depth / 2) - arena.top_radius:
                dan = min(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        point.x,
                        arena.height - arena.top_radius,
                        (arena.depth / 2) - arena.top_radius,
                    ),
                    arena.top_radius))

            if point.x > (arena.width / 2) - arena.corner_radius \
                    and point.z > (arena.depth / 2) - arena.corner_radius:

                corner_o = Vector(
                    (arena.width / 2) - arena.corner_radius,
                    (arena.depth / 2) - arena.corner_radius
                )

                dv = Vector(point.x, point.z) - corner_o
                if length(dv) > arena.corner_radius - arena.top_radius:
                    n = normalize(dv)
                    o2 = corner_o + n * (arena.corner_radius - arena.top_radius)
                    dan = min(dan, self.dan_to_sphere_inner(
                        point,
                        Vector(o2.x, arena.height - arena.top_radius, o2.y),
                        arena.top_radius))
        return dan

    def dan_to_arena(self, point):
        negate_x = point.x < 0
        negate_z = point.z < 0
        if negate_x:
            point.x = -point.x
        if negate_z:
            point.z = -point.z
        result = self.dan_to_arena_quarter(point)
        if negate_x:
            result.normal.x = -result.normal.x
        if negate_z:
            result.normal.z = -result.normal.z
        return result
