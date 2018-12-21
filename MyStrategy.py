from model import Action
import math

rules = None

BALL_IS_NEAR = 20
HIT_BALL = 4
BALL_IS_DIRECTED_TO_GOAL = 0
BALL_ATTACKER = 30
ANGLE_TO_SAVE_GOAL = 60


class Vector:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def dot(self, v):
        return v.x * self.x + v.y * self.y + v.z * self.z

    def distance(self, v1):
        l1 = (v1.x - self.x) ** 2
        l2 = (v1.y - self.y) ** 2
        l3 = (v1.z - self.z) ** 2
        return math.sqrt(l1 + l2 + l3)

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def clamp(self, clamp_length):
        if clamp_length > self.length():
            l = clamp_length / self.length()
            return Vector(self.x * l, self.y * l, self.z * l)
        else:
            return self

    def normalize(self):
        l = self.length()
        return Vector(self.x / l, self.y / l, self.z / l)

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
        return '<{},{},{}>'.format(self.x, self.y, self.z)

    def __repr__(self):
        return self.__str__()


class MyStrategy:

    def __init__(self):
        self.me = self.rules = self.game = self.action = None
        self.vball2, self.vball, self.vspeed, self.vgravity, self.vplayer = None, None, None, None, None
        self.flog = open('log.txt', 'w')

    def set_vectors(self):
        self.vball = Vector(self.game.ball.x, self.game.ball.y, self.game.ball.z)
        self.vspeed = Vector(self.game.ball.velocity_x / self.rules.TICKS_PER_SECOND,
                             self.game.ball.velocity_y / self.rules.TICKS_PER_SECOND,
                             self.game.ball.velocity_z / self.rules.TICKS_PER_SECOND)
        self.vgravity = Vector(0, - self.rules.GRAVITY / self.rules.TICKS_PER_SECOND ** 2, 0)
        self.vplayer = Vector(self.me.x, self.me.y, self.me.z)

        self.vgloal_mine = Vector(0, 0, -rules.arena.depth / 2)
        self.vgoal_en = Vector(0, 0, rules.arena.depth / 2)

    def get_ball_prediction(self, ticks=1):
        vball2 = self.vball + \
                 self.vspeed * ticks + self.vgravity * (0.5 * ticks ** 2)
        return vball2

    def get_many_ball_predictions(self, ticks=None):
        if ticks is None:
            ticks = self.rules.TICKS_PER_SECOND
        balls = [self.get_ball_prediction(tick) for tick in range(ticks)]
        return balls

    def set_action(self, vtarget, jump_speed=None):
        vtarget *= 10
        self.action.target_velocity_x = vtarget.x
        self.action.target_velocity_y = vtarget.y
        self.action.target_velocity_z = vtarget.z
        if jump_speed is not None:
            self.action.jump_speed = jump_speed

    def defense(self):
        # vfball = self.get_ball_prediction(ticks=round(self.rules.TICKS_PER_SECOND / 2))
        predictions = self.get_many_ball_predictions(self.rules.TICKS_PER_SECOND)
        for vball_pred in predictions:
            distance_to_ball = self.distance(vball_pred, self.vplayer)
            if distance_to_ball < BALL_IS_NEAR and vball_pred.z < 0:
                if distance_to_ball > HIT_BALL:
                    self.log('Ball is Near ', distance_to_ball)
                    target_vector = vball_pred + self.vplayer * -1
                    self.set_action(target_vector)
                else:
                    self.log('Hit time ', distance_to_ball)
                    target_vector = self.vgoal_en + self.vplayer * -1
                    self.set_action(target_vector, jump_speed=rules.ROBOT_MAX_JUMP_SPEED)
                return
        target_vector = self.vgloal_mine + self.vplayer * -1
        self.set_action(target_vector)

    def attack(self):
        predictions = self.get_many_ball_predictions(round(self.rules.TICKS_PER_SECOND / 2))
        for vball_pred in predictions:
            distance_to_ball = self.distance(vball_pred, self.vplayer)
            if distance_to_ball < BALL_ATTACKER:
                if distance_to_ball > HIT_BALL:
                    self.log('Ball is Near - ready to attack ', distance_to_ball)
                    target_vector = vball_pred + self.vplayer * -1
                    self.set_action(target_vector)
                else:
                    self.log('Hit time  - attack', distance_to_ball)
                    self.log(self.vspeed.z)
                    if self.vspeed.z < BALL_IS_DIRECTED_TO_GOAL and self.vball.z < 0:
                        target_vector = self.vgoal_en + self.vplayer * -1
                    else:
                        self.log('Change direction')
                        target_vector = self.vgoal_en + self.vplayer * -1
                    self.set_action(target_vector, jump_speed=rules.ROBOT_MAX_JUMP_SPEED * 0.75)
                return
        target_vector = predictions[-1] + self.vplayer * -1
        self.set_action(target_vector)

    def log(self, *args, **kwargs):
        self.flog.write(' '.join(str(i) for i in args) + '\n')
        print(self.game.current_tick, end=' ')
        print(*args, **kwargs)

    def act(self, Me, Rules, Game, Action):
        global rules
        rules = Rules
        self.me, self.rules, self.game, self.action = Me, Rules, Game, Action
        self.set_vectors()
        if Me.id == 1:
            self.defense()
        else:
            self.attack()
