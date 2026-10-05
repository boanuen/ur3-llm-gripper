"""Doc scene.yaml + trang thai the gioi (lay tu camera): khoi nao o dau, zone nao trong."""
import copy
import math
import os

import yaml


def cfg_path(n):
    """Duong dan file trong config: lay trong workspace da build, neu khong trong source."""
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
        self.spawn = d['spawn']          # chi dung de sinh world Gazebo
        self.cam = d['camera']
        self.grip = d['gripper']
        self.home_q = d['home']
        self.seed_q = d['seed']
        self.tool_yaw = d['tool_yaw']
        self.shrink = d['shrink']
        self.gap = d['gap']
        self.vel = d['vel']

        # do cao cua tool0
        h, g = self.tb['h'], self.grip
        self.z_cube = h + self.cube / 2                  # tam khoi tren ban
        self.z_top = h + self.cube                       # mat tren khoi (camera nhin thay)
        self.z_grasp = h + g['tip'] + g['len']           # tool0 khi gap
        self.z_place = self.z_grasp + 0.004              # tool0 khi dat (cao hon 4mm)
        self.z_up = self.z_grasp + d['up']               # tool0 khi o tren vat
        self.d_hold = self.z_grasp - self.z_cube         # tool0 -> tam khoi dang cam

        # trang thai: pos[obj] = (x, y, yaw) do camera thay, None = khong thay
        self.pos = {o: None for o in self.objs}
        self.held = None

    # ----------------------------------------------------------- trang thai
    def update(self, det):
        """Cap nhat tu ket qua camera {obj: (x, y, yaw)}. Vat dang cam giu nguyen."""
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
        """'zone_x', 'table', 'gripper' hoac None (camera khong thay)."""
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
        """Khoi dang nam trong zone (None = zone trong)."""
        for o in self.objs:
            if o != skip and self.where(o) == zone:
                return o
        return None

    def free_pos(self, near):
        """Tim 1 vi tri trong tren ban: khong trong zone, cach cac khoi khac >= gap,
        robot voi toi duoc. Chon diem gan 'near' nhat (de di chuyen ngan)."""
        t = self.tb
        best, bd = None, 1e9
        x = t['x'] - t['sx'] / 2 + 0.05
        while x <= t['x'] + t['sx'] / 2 - 0.05:
            y = t['y'] - t['sy'] / 2 + 0.06
            while y <= t['y'] + t['sy'] / 2 - 0.06:
                if self.free_ok((x, y)):
                    dd = math.hypot(x - near[0], y - near[1])
                    if dd < bd:
                        best, bd = (round(x, 3), round(y, 3)), dd
                y += 0.02
            x += 0.02
        return best

    def free_ok(self, p):
        r = math.hypot(p[0], p[1])
        if r < 0.20 or r > 0.42:                          # ngoai tam voi cua UR3e
            return False
        for z in self.zones:
            zx, zy = self.xy(z)
            if math.hypot(p[0] - zx, p[1] - zy) < self.gap:
                return False
        for o, q in self.pos.items():
            if q is not None and o != self.held and math.hypot(p[0] - q[0], p[1] - q[1]) < self.gap:
                return False
        return True

    def state(self):
        """Trang thai de in ra / gui LLM."""
        out = {}
        for o in self.objs:
            w = self.where(o)
            out[o] = 'not seen' if w is None else w
        return out

    def zone_state(self):
        return {z: (self.who(z) or 'free') for z in self.zones}

    def copy(self):
        return copy.deepcopy(self)
