"""Cac robot skill. Moi skill tra ve 1 trang thai (SUCCESS, FAILED, ...).

rb  : MoveIt + gripper (moveit_if.MoveIt) hoac robot gia khi test (fake_robot.Fake)
cam : camera that (vision.Camera) hoac camera gia, cam.capture() -> {ten_khoi: (x, y, yaw)}
"""
OK = 'SUCCESS'


class Skills:
    def __init__(self, sc, rb, cam, log=print):
        self.sc = sc
        self.rb = rb
        self.cam = cam
        self.log = log

    def say(self, s):
        self.log('    ' + s)       # in thut vao de phan biet voi dong ket qua

    def home(self):
        return self.rb.move_q(self.sc.home_q)

    # ---------------- camera
    def detect_objects(self):
        """Ve home (de tay khong che camera), chup anh, cap nhat vi tri cac khoi."""
        if self.sc.held:
            return 'FAILED'
        if not self.rb.at(self.sc.home_q):
            st = self.home()
            if st != OK:
                return st
        det = self.cam.capture()
        if det is None:
            self.say('khong nhan duoc anh tu camera')
            return 'FAILED'
        self.sc.update(det)
        self.rb.set_objects()       # dua cac khoi vao planning scene de MoveIt tranh
        seen = [f'{o}@{w}' for o, w in self.sc.state().items() if w != 'not seen']
        self.say('camera thay: ' + ', '.join(seen))
        return OK

    def check_zone(self, zone):
        if zone not in self.sc.zones:
            return 'INVALID_ZONE'
        st = self.detect_objects()
        if st != OK:
            return st
        o = self.sc.who(zone)
        self.say(f'{zone}: ' + (f'dang co {o}' if o else 'trong'))
        return OK

    def find_object(self, obj):
        if obj not in self.sc.objs:
            return 'INVALID_OBJECT'
        if self.sc.pos[obj] is None:
            return 'OBJECT_NOT_FOUND'
        return OK

    def find_free_position(self, obj):
        """Tim cho trong tren ban, gan khoi nhat (de di chuyen ngan)."""
        p = self.sc.pos[obj] or (self.sc.tb['x'], self.sc.tb['y'])
        q = self.sc.free_pos(p)
        self.say(f'find_free_position({obj}) -> {q}')
        return q

    # ---------------- gripper
    def open_gripper(self):
        gap = self.rb.grip(False)
        return OK if gap > self.sc.grip['open'] - 0.012 else 'FAILED'

    def close_gripper(self, obj):
        gap = self.rb.grip(True)
        # neu 2 ngon dong sat vao nhau tuc la khong kep trung vat
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
            return 'FAILED'                     # dang cam vat khac
        p = sc.pos[obj]
        if p is None:
            return 'OBJECT_NOT_FOUND'
        yaw = sc.tool_yaw + p[2]                # xoay co tay cho ngon tay song song mat khoi

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

        # nhac len xong ma ngon tay dong het -> vat bi roi
        if sum(self.rb.fingers()) < sc.grip['min_gap']:
            self.say(f'{obj} bi roi khi nhac len')
            self.rb.detach(obj, p, p[2])
            sc.held = None
            return 'GRASP_FAILED'
        sc.pos[obj] = None
        return OK

    def place_at(self, obj, xy):
        sc = self.sc
        st = self.rb.move_xyz((xy[0], xy[1], sc.z_up), sc.tool_yaw)
        if st == OK:
            st = self.rb.move_z(sc.z_place)     # ha xuong (cao hon mat ban vai mm)
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
            return 'FAILED'                     # khong cam dung vat
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
        """Zone dang co vat thi dua vat do ra cho trong tren ban.
        keep: vat sap duoc dat vao zone nay, neu no da nam san trong zone thi thoi."""
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
