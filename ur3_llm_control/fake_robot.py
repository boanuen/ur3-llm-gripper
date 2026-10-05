"""Robot + camera gia (khong can ROS/Gazebo) de test LLM, validator, skill, executor.
Fake giu 'truth' = vi tri that cua cac khoi; FakeCam doc truth nhu camera that."""
import math


class Fake:
    def __init__(self, sc, truth, fail=False):
        self.sc = sc
        self.truth = truth     # {obj: (x, y, yaw)} vi tri that tren ban
        self.fail = fail       # True -> moi lenh di chuyen deu PLANNING_FAILED
        self.log = []
        self.q = list(sc.home_q)
        self.tool = None       # (x, y) cua tool
        self.f = 0.04          # do mo 1 ngon tay
        self.hold = None

    def at(self, q, tol=0.05):
        return all(abs(a - b) < tol for a, b in zip(self.q, q))

    def move_q(self, q):
        self.log.append(('move_q',))
        if self.fail:
            return 'PLANNING_FAILED'
        self.q, self.tool = list(q), None
        return 'SUCCESS'

    def move_xyz(self, p, yaw):
        self.log.append(('move_xyz', round(p[0], 3), round(p[1], 3)))
        if self.fail or math.hypot(p[0], p[1]) > 0.5:
            return 'PLANNING_FAILED'
        self.q, self.tool = [9] * 6, (p[0], p[1])
        return 'SUCCESS'

    def move_z(self, z):
        self.log.append(('move_z', round(z, 3)))
        return 'PLANNING_FAILED' if self.fail else 'SUCCESS'

    def fingers(self):
        return [self.f, self.f]

    def grip(self, close):
        self.log.append(('grip', close))
        if not close:
            self.f = 0.04
            return 2 * self.f
        # co khoi nam duoi tool -> ngon tay dung o be rong khoi
        for o, p in self.truth.items():
            if self.tool and math.hypot(p[0] - self.tool[0], p[1] - self.tool[1]) < 0.015:
                self.hold, self.f = o, 0.02
                return 2 * self.f
        self.f = 0.0
        return 0.0

    def attach(self, o):
        self.log.append(('attach', o))
        self.truth.pop(o, None)
        return 'SUCCESS'

    def detach(self, o, p, yaw):
        self.log.append(('detach', o))
        self.truth[o] = (p[0], p[1], yaw)
        self.hold = None
        return 'SUCCESS'

    def set_objects(self):
        return True


class FakeCam:
    def __init__(self, rb):
        self.rb = rb

    def capture(self):
        return dict(self.rb.truth)
