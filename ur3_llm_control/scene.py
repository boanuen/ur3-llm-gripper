"""Doc scene.yaml va luu trang thai cac khoi."""
import copy
import math
import os

import yaml


def cfg_path(n):
    # file config: uu tien thu muc da build, khong co thi lay trong source
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('ur3_llm_control'), 'config', n)
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', n)


class Scene:
    def __init__(self, path):
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        self.frame = d['frame']
        self.group = d['group']
        self.ee = d['ee']
        self.tb = d['table']
        self.cube = d['cube']
        self.zones = d['zones']
        self.objs = d['objects']
        self.spawn = d['spawn']          # chi dung sinh world Gazebo
        self.cam = d['camera']
        self.grip = d['gripper']
        self.home_q = d['home']
        self.seed_q = d['seed']
        self.tool_yaw = d['tool_yaw']
        self.shrink = d['shrink']
        self.gap = d['gap']
        self.vel = d['vel']

        # cac do cao
        h = self.tb['h']
        g = self.grip
        self.z_cube = h + self.cube / 2                  # tam khoi
        self.z_top = h + self.cube                       # mat tren khoi
        self.z_grasp = h + g['tip'] + g['len']           # tool0 khi gap
        self.z_place = self.z_grasp + 0.004              # tool0 khi dat
        self.z_up = self.z_grasp + d['up']               # tool0 khi o tren vat
        self.d_hold = self.z_grasp - self.z_cube         # tool0 -> tam khoi

        # pos[obj] = (x, y, yaw), None = khong thay
        self.pos = {}
        for o in self.objs:
            self.pos[o] = None
        self.held = None

    # ---------------- trang thai
    def update(self, det):
        # cap nhat tu camera, bo qua vat dang cam
        for o in self.objs:
            if o != self.held:
                self.pos[o] = det.get(o)

    def xy(self, zone):
        return tuple(self.zones[zone]['xy'])

    def in_zone(self, p, zone):
        zx, zy = self.xy(zone)
        k = self.zones[zone]['size'] / 2 + 0.01
        return abs(p[0] - zx) < k and abs(p[1] - zy) < k

    def where(self, obj):
        # tra ve zone, 'table', 'gripper' hoac None
        if obj == self.held:
            return 'gripper'
        p = self.pos[obj]
        if p is None:
            return None
        for z in self.zones:
            if self.in_zone(p, z):
                return z
        return 'table'

    def who(self, zone, skip=None):
        # khoi trong zone, None = trong
        for o in self.objs:
            if o != skip and self.where(o) == zone:
                return o
        return None

    def free_pos(self, near):
        # quet luoi tren ban, chon diem trong gan 'near' nhat
        t = self.tb
        best = None
        bd = 1e9
        x = t['x'] - t['sx'] / 2 + 0.05
        while x <= t['x'] + t['sx'] / 2 - 0.05:
            y = t['y'] - t['sy'] / 2 + 0.06
            while y <= t['y'] + t['sy'] / 2 - 0.06:
                if self.free_ok((x, y)):
                    d = math.hypot(x - near[0], y - near[1])
                    if d < bd:
                        best = (round(x, 3), round(y, 3))
                        bd = d
                y += 0.02
            x += 0.02
        return best

    def free_ok(self, p):
        # ngoai tam voi
        r = math.hypot(p[0], p[1])
        if r < 0.20 or r > 0.42:
            return False
        # gan zone
        for z in self.zones:
            zx, zy = self.xy(z)
            if math.hypot(p[0] - zx, p[1] - zy) < self.gap:
                return False
        # gan khoi khac
        for o in self.pos:
            q = self.pos[o]
            if q is None or o == self.held:
                continue
            if math.hypot(p[0] - q[0], p[1] - q[1]) < self.gap:
                return False
        return True

    def state(self):
        # {obj: vi tri} de in / gui LLM
        out = {}
        for o in self.objs:
            w = self.where(o)
            if w is None:
                w = 'not seen'
            out[o] = w
        return out

    def zone_state(self):
        out = {}
        for z in self.zones:
            o = self.who(z)
            if o is None:
                o = 'free'
            out[z] = o
        return out

    def copy(self):
        return copy.deepcopy(self)
