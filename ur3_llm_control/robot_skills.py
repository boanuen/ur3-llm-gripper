"""Cac skill cua robot, moi skill tra ve 1 trang thai."""
OK = 'SUCCESS'


class Skills:
    def __init__(self, sc, rb, cam, log=print):
        self.sc = sc
        self.rb = rb
        self.cam = cam
        self.log = log

    def say(self, s):
        self.log('    ' + s)

    def home(self):
        return self.rb.move_q(self.sc.home_q)

    # ---------------- camera
    def detect_objects(self):
        if self.sc.held:
            return 'FAILED'
        # ve home de tay khong che camera
        if not self.rb.at(self.sc.home_q):
            st = self.home()
            if st != OK:
                return st
        # chup anh
        det = self.cam.capture()
        if det is None:
            self.say('khong nhan duoc anh tu camera')
            return 'FAILED'
        self.sc.update(det)
        self.rb.set_objects()
        # in cac khoi thay duoc
        seen = []
        st = self.sc.state()
        for o in st:
            if st[o] != 'not seen':
                seen.append(o + '@' + st[o])
        self.say('camera thay: ' + ', '.join(seen))
        return OK

    def check_zone(self, zone):
        if zone not in self.sc.zones:
            return 'INVALID_ZONE'
        st = self.detect_objects()
        if st != OK:
            return st
        o = self.sc.who(zone)
        if o:
            self.say(f'{zone}: dang co {o}')
        else:
            self.say(f'{zone}: trong')
        return OK

    def find_object(self, obj):
        if obj not in self.sc.objs:
            return 'INVALID_OBJECT'
        if self.sc.pos[obj] is None:
            return 'OBJECT_NOT_FOUND'
        return OK

    def find_free_position(self, obj):
        # cho trong gan khoi nhat
        p = self.sc.pos[obj]
        if p is None:
            p = (self.sc.tb['x'], self.sc.tb['y'])
        q = self.sc.free_pos(p)
        self.say(f'find_free_position({obj}) -> {q}')
        return q

    # ---------------- gripper
    def open_gripper(self):
        gap = self.rb.grip(False)
        if gap > self.sc.grip['open'] - 0.012:
            return OK
        return 'FAILED'

    def close_gripper(self, obj):
        gap = self.rb.grip(True)
        # 2 ngon dong sat -> kep truot
        if gap < self.sc.grip['min_gap']:
            self.say(f'kep truot (khe ho {gap:.3f} m)')
            self.rb.grip(False)
            return 'GRASP_FAILED'
        self.sc.held = obj
        return self.rb.attach(obj)

    # ---------------- pick / place
    def pick(self, obj):
        sc = self.sc
        if obj not in sc.objs:
            return 'INVALID_OBJECT'
        if sc.held:
            return 'FAILED'
        p = sc.pos[obj]
        if p is None:
            return 'OBJECT_NOT_FOUND'
        yaw = sc.tool_yaw + p[2]                # xoay tay theo khoi

        st = self.open_gripper()
        if st == OK:
            st = self.rb.move_xyz((p[0], p[1], sc.z_up), yaw)    # len tren khoi
        if st == OK:
            st = self.rb.move_z(sc.z_grasp)                       # ha xuong
        if st == OK:
            st = self.close_gripper(obj)                          # kep
        if st == OK:
            st = self.rb.move_z(sc.z_up)                          # nhac len
        if st != OK:
            return st

        # kiem tra vat bi roi
        if sum(self.rb.fingers()) < sc.grip['min_gap']:
            self.say(f'{obj} bi roi khi nhac len')
            self.rb.detach(obj, p, p[2])
            sc.held = None
            return 'GRASP_FAILED'
        sc.pos[obj] = None
        return OK

    def place_at(self, obj, xy):
        sc = self.sc
        st = self.rb.move_xyz((xy[0], xy[1], sc.z_up), sc.tool_yaw)   # len tren diem dat
        if st == OK:
            st = self.rb.move_z(sc.z_place)     # ha xuong
        if st == OK:
            st = self.open_gripper()            # tha vat
        if st != OK:
            return st
        sc.held = None
        sc.pos[obj] = (xy[0], xy[1], 0.0)
        self.rb.detach(obj, xy, 0.0)
        return self.rb.move_z(sc.z_up)

    def place(self, obj, zone):
        sc = self.sc
        if obj not in sc.objs:
            return 'INVALID_OBJECT'
        if zone not in sc.zones:
            return 'INVALID_ZONE'
        if sc.held != obj:
            return 'FAILED'
        o = sc.who(zone, skip=obj)
        if o:
            self.say(f'{zone} dang co {o}')
            return 'ZONE_OCCUPIED'
        return self.place_at(obj, sc.xy(zone))

    def place_free(self, obj):
        if self.sc.held != obj:
            return 'FAILED'
        q = self.find_free_position(obj)
        if q is None:
            return 'NO_FREE_SPACE'
        return self.place_at(obj, q)

    def clear_zone(self, zone, keep=None):
        # don vat trong zone ra cho trong (bo qua neu la vat keep)
        st = self.check_zone(zone)
        if st != OK:
            return st
        o = self.sc.who(zone)
        if o is None or o == keep:
            return 'SKIPPED'
        self.say(f'don {zone}: pick({o}) -> place_free({o})')
        st = self.pick(o)
        if st != OK:
            return st
        return self.place_free(o)
