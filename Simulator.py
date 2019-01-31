import copy
import json
from model import *
import random
import math
from meta import *

eps = 0.005


class Simulator:
    def __init__(self, robots, ball, rules):
        self.robots = copy.deepcopy(robots)
        self.ball = copy.deepcopy(ball)
        self.rules = rules
        self.arena = rules.arena
        self.nitro_packs = []
        self.collision = False

    @staticmethod
    def move(e: Entity, delta_time):
        time = delta_time
        e.velocity = clamp(e.velocity, Rulez.MAX_ENTITY_SPEED)
        e.position += e.velocity * time
        e.position.y -= Rulez.GRAVITY * time ** 2 / 2
        e.velocity.y -= Rulez.GRAVITY * time

    @staticmethod
    def collide_entities(a: Entity, b: Entity):
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
                impulse = normal * (1 + (Rulez.MIN_HIT_E + Rulez.MAX_HIT_E) / 2) * delta_velocity
                a.velocity += impulse * k_a
                b.velocity -= impulse * k_b
                if a.teammate:
                    return True
        return False

    def collide_with_arena(self, e: Entity):
        distance, normal = self.dan_to_arena(e.position)
        penetration = e.radius - distance
        if penetration > 0:
            e.position += penetration * normal
            velocity = e.velocity.dot(normal) - e.radius_change_speed
            if velocity < 0:
                a = normal * (1 + e.arena_e) * velocity
                e.velocity -= a
                return normal

        return None

    @staticmethod
    def robot_touch(robot: Entity, delta_time):
        if robot.touch:
            target_velocity = \
                robot.action.target_velocity.clamp(Rulez.ROBOT_MAX_GROUND_SPEED)
            target_velocity -= robot.touch_normal * robot.touch_normal.dot(target_velocity)
            target_velocity_change = target_velocity - robot.velocity
            if target_velocity_change.length() > 0:
                acceleration = Rulez.ROBOT_ACCELERATION * max(0, robot.touch_normal.y)
                robot.velocity += clamp(target_velocity_change.normalize() * acceleration * delta_time,
                                        target_velocity_change)
            return target_velocity_change

    def update_robot(self, delta_time=1 / Rulez.TICKS_PER_SECOND):
        for robot in self.robots:
            Simulator.robot_touch(robot, delta_time)
            self.move(robot, delta_time)
            robot.radius = self.rules.ROBOT_MIN_RADIUS + (
                    self.rules.ROBOT_MAX_RADIUS - self.rules.ROBOT_MIN_RADIUS) * robot.action.jump_speed / self.rules.ROBOT_MAX_JUMP_SPEED
            robot.radius_change_speed = robot.action.jump_speed
            collision_normal = self.collide_with_arena(robot)
            if collision_normal is None:
                robot.touch = False
            else:
                robot.touch = True
                robot.touch_normal = collision_normal

    def update(self, delta_time: float):
        robots = self.robots
        for robot in robots:
            Simulator.robot_touch(robot, delta_time)
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
        self.move(self.ball, delta_time)
        for i in range(len(robots)):
            for j in range(i):
                self.collide_entities(self.robots[i], self.robots[j])
        for robot in robots:
            self.collision = self.collide_entities(robot, self.ball)
            # collision_normal = self.collide_with_arena(robot)
            if robot.y == 1:
                collision_normal = Vector(0, 1, 0)
            else:
                collision_normal = self.collide_with_arena(robot)

            if collision_normal is None:
                robot.touch = False
            else:
                robot.touch = True
                robot.touch_normal = collision_normal
        if not (
                self.ball.position.x < self.rules.arena.width / 2 - 2 and self.ball.position.x > -self.rules.arena.width / 2 + 2
                and self.ball.position.y < self.rules.arena.height - 2 and self.ball.position.y > 2
                and self.ball.position.z < self.rules.arena.depth / 2 - 2 and self.ball.position.z > -self.rules.arena.depth / 2 + 2):
            self.collide_with_arena(self.ball)
        if abs(self.ball.position.z) > self.rules.arena.depth / 2 + self.ball.radius:
            self.goal_scored()
        for robot in self.robots:
            if robot.nitro_amount == self.rules.MAX_NITRO_AMOUNT:
                continue
            for pack in self.nitro_packs:
                if not pack.alive:
                    continue
                if length(robot.position - pack.position) <= robot.radius + pack.radius:
                    robot.nitro = self.rules.MAX_NITRO_AMOUNT
                    pack.alive = False
                    pack.respawn_ticks = self.rules.NITRO_PACK_RESPAWN_TICKS

    def tick(self, step=1):
        delta_time = 1 / self.rules.TICKS_PER_SECOND
        self.update(delta_time=delta_time * step)

    def goal_scored(self):
        pass

    def dan_to_plane(self, point, point_on_plane, plane_normal):
        return dot(point - point_on_plane, plane_normal), plane_normal

    def dan_to_sphere_inner(self, point, sphere_center, sphere_radius):
        return sphere_radius - length(point - sphere_center), normalize(sphere_center - point)

    def dan_to_sphere_outer(self, point, sphere_center, sphere_radius):
        return length(point - sphere_center) - sphere_radius, normalize(point - sphere_center)

    def dan_to_arena_quarter(self, point):
        dan = self.dan_to_plane(point, Vector(0, 0, 0), Vector(0, 1, 0))
        dan = min_dan(dan, self.dan_to_plane(point, Vector(0, self.rules.arena.height, 0), Vector(0, -1, 0)))
        dan = min_dan(dan, self.dan_to_plane(point, Vector(self.arena.width / 2, 0, 0), Vector(-1, 0, 0)))
        dan = min_dan(dan, self.dan_to_plane(
            point,
            Vector(0, 0, (self.rules.arena.depth / 2) + self.rules.arena.goal_depth),
            Vector(0, 0, -1)))
        v = Vector(point.x - (self.rules.arena.goal_width / 2) + self.rules.arena.goal_top_radius,
                   point.y - self.rules.arena.goal_height + self.rules.arena.goal_top_radius, 0)
        if point.x >= (
                self.rules.arena.goal_width / 2) + self.rules.arena.goal_side_radius or point.y >= self.rules.arena.goal_height + self.rules.arena.goal_side_radius or (
                v.x > 0
                and v.y > 0
                and length(v) >= self.rules.arena.goal_top_radius + self.rules.arena.goal_side_radius):
            dan = min_dan(dan, self.dan_to_plane(point, Vector(0, 0, self.rules.arena.depth / 2), Vector(0, 0, -1)))

        if point.z >= (self.rules.arena.depth / 2) + self.rules.arena.goal_side_radius:
            dan = min_dan(dan, self.dan_to_plane(
                point,
                Vector(self.rules.arena.goal_width / 2, 0, 0),
                Vector(-1, 0, 0)))
            dan = min_dan(dan, self.dan_to_plane(point, Vector(0, self.rules.arena.goal_height, 0), Vector(0, -1, 0)))

        assert self.rules.arena.bottom_radius == self.rules.arena.goal_top_radius
        if point.z > (self.rules.arena.depth / 2) + self.rules.arena.goal_depth - self.rules.arena.bottom_radius:
            dan = min_dan(dan, self.dan_to_sphere_inner(
                point,
                Vector(
                    clamp(
                        point.x,
                        self.rules.arena.bottom_radius - (self.rules.arena.goal_width / 2),
                        (self.rules.arena.goal_width / 2) - self.rules.arena.bottom_radius,
                    ),
                    clamp(
                        point.y,
                        self.rules.arena.bottom_radius,
                        self.rules.arena.goal_height - self.rules.arena.goal_top_radius,
                    ),
                    (self.rules.arena.depth / 2) + self.rules.arena.goal_depth - self.rules.arena.bottom_radius),
                self.rules.arena.bottom_radius))
        if point.x > (self.rules.arena.width / 2) - self.rules.arena.corner_radius and point.z > (
                self.rules.arena.depth / 2) - self.rules.arena.corner_radius:
            dan = min_dan(dan, self.dan_to_sphere_inner(
                point,
                Vector(
                    (self.rules.arena.width / 2) - self.rules.arena.corner_radius,
                    point.y,
                    (self.rules.arena.depth / 2) - self.rules.arena.corner_radius),
                self.rules.arena.corner_radius))
        if point.z < (self.rules.arena.depth / 2) + self.rules.arena.goal_side_radius:

            if point.x < (self.rules.arena.goal_width / 2) + self.rules.arena.goal_side_radius:
                dan = min_dan(dan, self.dan_to_sphere_outer(
                    point,
                    Vector(
                        (self.rules.arena.goal_width / 2) + self.rules.arena.goal_side_radius,
                        point.y,
                        (self.rules.arena.depth / 2) + self.rules.arena.goal_side_radius
                    ),
                    self.rules.arena.goal_side_radius))
            if point.y < self.arena.goal_height + self.arena.goal_side_radius:
                dan = min_dan(dan, self.dan_to_sphere_outer(
                    point,
                    Vector(
                        point.x,
                        self.arena.goal_height + self.arena.goal_side_radius,
                        (self.arena.depth / 2) + self.arena.goal_side_radius
                    ),
                    self.arena.goal_side_radius))
            o = Vector(
                (self.arena.goal_width / 2) - self.arena.goal_top_radius,
                self.arena.goal_height - self.arena.goal_top_radius, 0
            )
            v = Vector(point.x, point.y, 0) - o
            if v.x > 0 and v.y > 0:
                o = o + normalize(v) * (self.arena.goal_top_radius + self.arena.goal_side_radius)
                dan = min_dan(dan, self.dan_to_sphere_outer(
                    point,
                    Vector(o.x, o.y, (self.arena.depth / 2) + self.arena.goal_side_radius),
                    self.arena.goal_side_radius))
            if point.z > (
                    self.arena.depth / 2) + self.arena.goal_side_radius and point.y > self.arena.goal_height - self.arena.goal_top_radius:

                if point.x > (self.arena.goal_width / 2) - self.arena.goal_top_radius:
                    dan = min_dan(dan, self.dan_to_sphere_inner(
                        point,
                        Vector(
                            (self.arena.goal_width / 2) - self.arena.goal_top_radius,
                            self.arena.goal_height - self.arena.goal_top_radius,
                            point.z
                        ),
                        self.arena.goal_top_radius))
                if point.z > (self.arena.depth / 2) + self.arena.goal_depth - self.arena.goal_top_radius:
                    dan = min_dan(dan, self.dan_to_sphere_inner(
                        point,
                        Vector(point.x,
                               self.arena.goal_height - self.arena.goal_top_radius,
                               (self.arena.depth / 2) + self.arena.goal_depth - self.arena.goal_top_radius
                               ),
                        self.arena.goal_top_radius))
        if point.y < self.arena.bottom_radius:
            if point.x > (self.arena.width / 2) - self.arena.bottom_radius:
                dan = min_dan(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        (self.arena.width / 2) - self.arena.bottom_radius,
                        self.arena.bottom_radius,
                        point.z
                    ),
                    self.arena.bottom_radius))
            if point.z > (self.arena.depth / 2) - self.arena.bottom_radius \
                    and point.x >= (self.arena.goal_width / 2) + self.arena.goal_side_radius:
                dan = min_dan(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        point.x,
                        self.arena.bottom_radius,
                        (self.arena.depth / 2) - self.arena.bottom_radius
                    ),
                    self.arena.bottom_radius))
            if point.z > (self.arena.depth / 2) + self.arena.goal_depth - self.arena.bottom_radius:
                dan = min_dan(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        point.x,
                        self.arena.bottom_radius,
                        (self.arena.depth / 2) + self.arena.goal_depth - self.arena.bottom_radius
                    ),
                    self.arena.bottom_radius))
            o = Vector(
                (self.arena.goal_width / 2) + self.arena.goal_side_radius,
                (self.arena.depth / 2) + self.arena.goal_side_radius, 0
            )
            v = Vector(point.x, point.y, 0) - o
            if v.x < 0 and v.y < 0 and length(v) < self.arena.goal_side_radius + self.arena.bottom_radius:
                o = o + normalize(v) * (self.arena.goal_side_radius + self.arena.bottom_radius)
                dan = min_dan(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(o.x, self.arena.bottom_radius, o.y),
                    self.arena.bottom_radius))
                if point.z >= (self.arena.depth / 2) + self.arena.goal_side_radius \
                        and point.x > (self.arena.goal_width / 2) - self.arena.bottom_radius:
                    dan = min_dan(dan, self.dan_to_sphere_inner(point,
                                                                Vector(
                                                                    (
                                                                            self.arena.goal_width / 2) - self.arena.bottom_radius,
                                                                    self.arena.bottom_radius,
                                                                    point.z
                                                                ),
                                                                self.arena.bottom_radius))
        if point.y > self.arena.height - self.arena.top_radius:
            if point.x > (self.arena.width / 2) - self.arena.top_radius:
                dan = min_dan(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        (self.arena.width / 2) - self.arena.top_radius,
                        self.arena.height - self.arena.top_radius,
                        point.z,
                    ),
                    self.arena.top_radius))
            if point.z > (self.arena.depth / 2) - self.arena.top_radius:
                dan = min_dan(dan, self.dan_to_sphere_inner(
                    point,
                    Vector(
                        point.x,
                        self.arena.height - self.arena.top_radius,
                        (self.arena.depth / 2) - self.arena.top_radius,
                    ),
                    self.arena.top_radius))

            if point.x > (self.arena.width / 2) - self.arena.corner_radius \
                    and point.z > (self.arena.depth / 2) - self.arena.corner_radius:

                corner_o = Vector(
                    (self.arena.width / 2) - self.arena.corner_radius,
                    (self.arena.depth / 2) - self.arena.corner_radius
                )

                dv = Vector(point.x, point.z) - corner_o
                if length(dv) > self.arena.corner_radius - self.arena.top_radius:
                    n = normalize(dv)
                    o2 = corner_o + n * (self.arena.corner_radius - self.arena.top_radius)
                    dan = min_dan(dan, self.dan_to_sphere_inner(
                        point,
                        Vector(o2.x, self.arena.height - self.arena.top_radius, o2.y),
                        self.arena.top_radius))
        return dan

    def dan_to_arena(self, point):
        p = point.copy()
        negate_x = p.x < 0
        negate_z = p.z < 0
        if negate_x:
            p.x = -p.x
        if negate_z:
            p.z = -p.z
        result = self.dan_to_arena_quarter(p)
        if negate_x:
            result[1].x = -result[1].x
        if negate_z:
            result[1].z = -result[1].z
        return result
