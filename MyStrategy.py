import copy
import json
from Simulator import Simulator
from model import *
from meta import *
import json
import random

ENEMY_GOAL = Vector(0, 0, Rulez.arena.depth / 2)
MY_GOAL = -1 * ENEMY_GOAL
MY_PART = Vector(0, 0, -Rulez.arena.depth / 2 + 10)


class MyStrategy:

    def __init__(self):
        self.render = Render()
        self.positions = []
        self.rules = None
        self.ball = None
        self.robots = None
        self.teammate = None
        self.me = None
        self.action = None
        self.game = None
        self.attacker_jump = None
        self.defender_jump = None

    def act(self, me, rules, game, action):
        self.set_game(me, rules, game, action)
        self.predict_ball(100)
        if me.id % 2 == 1:
            self.defender_logic()
        else:
            self.attacker_logic()

    # self.defender_logic()

    def get_line(self, posindex=15):
        dv = self.positions[posindex] + (self.positions[posindex] - Vector(
            clamp(self.ball.x, -self.rules.arena.goal_width / 2 + 1, self.rules.arena.goal_width / 2 - 1), 0,
            self.rules.arena.depth / 2)).normalize() * 2

        return dv, posindex

    def get_jump_speed(self):
        mult = 15
        if self.ball.position.z > 32:
            mult = 12
        return mult

    def defender_logic(self):
        pl = None
        for l in range(10, 40):
            dv, posindex = self.get_line(l)
            if dv.x < 16 and dv.x > -16 and dv.z > -42 and dv.z < -25 and dv.y < 8:
                pl = dv

                break
        dv = pl
        distance_between_velocities = \
            self.me.velocity.distance((self.ball.position - self.me.position))
        self.debug_text += "\n Distance between velocities:" + str(distance_between_velocities)
        self.debug_text += "\n DV vector:" + str(dv)
        self.defender_jump = False

        if self.ball.position.z > 10 and self.positions[50].z > 0:
            for pack in self.game.nitro_packs:

                if pack.z < -10 and pack.respawn_ticks is None:
                    self.set_action(Vector(pack.x, pack.y, pack.z) - self.me.position)
                    return

        if not self.attacker_jump and self.collision_will_be() and self.me.distance(
                self.ball.position.to_2d()) < 9 and self.ball.position.z > self.me.position.z:
            self.set_action(self.ball.position - self.me.position, self.get_jump_speed())
            self.action.use_nitro = True
            self.render.add_sphere(self.ball.position)
            self.debug_text += "Can beet ball!!"
            if self.me.velocity.y == 0:
                self.defender_jump = True
        elif dv is not None and not self.attacker_jump:
            self.set_action(dv - self.me.position)
            self.render.add_sphere(dv)
            self.debug_text += "GO to preditiction dv is NOT near!!"
        else:
            self.set_action(MY_GOAL + Vector(clamp(self.positions[30].x, -11, 11), 0, ) - self.me.position)
            self.render.add_sphere(MY_GOAL)
            self.debug_text += "GO to my GOAL"

    def collision_will_be(self, nitro=True, target=None):
        if target is None:
            target = self.ball.position
        self.me.action.target_velocity = (target - self.me.position).normalize() * Rulez.MAX_ENTITY_SPEED
        self.me.action.jump_speed = Rulez.ROBOT_MAX_JUMP_SPEED
        # self.me.action.use_nitro = nitro
        sim = Simulator([self.me], self.ball, self.rules)
        for l in range(40):
            sim.tick()
            if sim.collision and sim.robots[0].velocity.y > 5:
                sim.tick()
                if sim.ball.z > -41:
                    return True
        return False

    def attacker_logic(self):
        min = 999999
        best_pos = None
        shtraf = 0
        tick = 1 / self.rules.TICKS_PER_SECOND
        for ind, pos in enumerate(self.positions):
            if pos.z < -41:
                shtraf += 99999999999999
            if pos.y < 8 and pos.z > -41:
                nticks_to_get = (pos - self.me.position).length() / self.rules.ROBOT_MAX_GROUND_SPEED / 0.7 / tick

                razn = nticks_to_get - ind
                if razn < min and razn > 0:
                    min = razn
                    best_pos = pos
        if best_pos is not None:
            self.render.add_sphere(best_pos, color=[0, 1, 0], radius=2)
        else:
            best_pos = self.positions[10]
        dv = best_pos + (best_pos - ENEMY_GOAL).normalize() * 2
        self.render.add_line(dv, ENEMY_GOAL)
        distance = self.me.position.distance(dv.to_2d())
        distance_between_velocities = \
            self.me.velocity.distance((self.ball.position - self.me.position))
        self.debug_text += "\n Distance between velocities:" + str(distance_between_velocities)
        self.debug_text += "\n DV vector:" + str(dv)
        self.debug_text += "\n Distance to dv:" + str(distance)
        self.attacker_jump = False
        min = lambda a, b: a if a < b else b
        if not self.defender_jump and distance < (
                5) and self.ball.position.z > self.me.position.z and self.collision_will_be(best_pos
                                                                                            ):
            self.set_action(best_pos - self.me.position, self.get_jump_speed())
            self.render.add_sphere(self.ball.position)
            self.debug_text += "Can beet ball!!"
            if self.me.velocity.y == 0:
                self.attacker_jump = True
        else:
            if self.me.distance(self.ball.position.to_2d()) < 6 and self.collision_will_be(
                    self.ball.position) and self.ball.position.z > self.me.z:
                self.set_action(self.ball.position - self.me.position, self.get_jump_speed())
            elif distance < 8:
                self.set_action(best_pos - self.me.position)
            else:
                self.set_action(dv - self.me.position)
            self.render.add_sphere(dv)
            self.debug_text += "GO to preditiction dv is NOT near!!"

    def set_action(self, target: Vector, speed=0, rspeed=Rulez.MAX_ENTITY_SPEED):
        target = target.normalize() * rspeed
        self.action.target_velocity_x = target.x
        self.action.target_velocity_y = target.y
        self.action.target_velocity_z = target.z
        self.action.jump_speed = speed

    def set_game(self, me: Robot, rules: Rulez, game: Game, action: Action):
        self.debug_text = ""
        self.render.reset()
        self.rules = rules
        self.ball = get_ball_entity(game.ball)
        self.robots = [get_robot_entity(robot) for robot in game.robots]
        self.teammate = [robot for robot in self.robots if robot.teammate][0]
        self.me = self.get_me(self.robots, me.id)
        self.action = action
        self.game = game

    def get_me(self, robots, id):
        return [robot for robot in robots if robot.id == id][0]

    def predict_ball(self, nticks=51):
        self.positions = []
        sim = Simulator([], self.ball, self.rules)
        v1 = self.ball.position
        self.positions.append(v1)
        step = 1
        for i in range(round(nticks / step)):
            sim.tick(step=step)
            self.render.add_line(v1, sim.ball.position)
            v1 = sim.ball.position
            self.positions.append(v1)

    def custom_rendering(self):
        self.debug_text += "\n Ball: vel " + str(self.ball.velocity) + " pos " + str(self.ball.position)
        self.debug_text += "\n Me: vel " + str(self.me.velocity) + " pos " + str(self.me.position)

        self.render.objects.append({"Text": self.debug_text})
        js = json.dumps(self.render.objects)

        return js
