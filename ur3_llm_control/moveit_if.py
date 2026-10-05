"""Giao tiep MoveIt 2 va gripper."""
import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
import tf2_ros

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
FINGERS = ['finger_l_joint', 'finger_r_joint']


def mk_pose(p, q=(0.0, 0.0, 0.0, 1.0)):
    m = Pose()
    m.position.x = float(p[0])
    m.position.y = float(p[1])
    m.position.z = float(p[2])
    m.orientation.x = float(q[0])
    m.orientation.y = float(q[1])
    m.orientation.z = float(q[2])
    m.orientation.w = float(q[3])
    return m


def mk_box(name, size, p, yaw=0.0, frame='world', op=CollisionObject.ADD):
    b = SolidPrimitive()
    b.type = SolidPrimitive.BOX
    b.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    co.pose = mk_pose(p, (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    co.primitives = [b]
    co.primitive_poses = [mk_pose((0, 0, 0))]
    co.operation = op
    return co


def wait(fut, t):
    # cho future xong (node spin o thread khac)
    end = time.time() + t
    while not fut.done():
        if time.time() > end:
            return None
        time.sleep(0.01)
    return fut.result()


class MoveIt:
    def __init__(self, node, sc):
        self.sc = sc
        cb = ReentrantCallbackGroup()
        self.mv = ActionClient(node, MoveGroup, '/move_action', callback_group=cb)
        self.ex = ActionClient(node, ExecuteTrajectory, '/execute_trajectory', callback_group=cb)
        # chi de kiem tra controller da bat chua
        self.jtc = ActionClient(node, FollowJointTrajectory,
                                '/joint_trajectory_controller/follow_joint_trajectory',
                                callback_group=cb)
        self.cart = node.create_client(GetCartesianPath, '/compute_cartesian_path',
                                       callback_group=cb)
        self.ps = node.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                     callback_group=cb)
        self.ikc = node.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.gp = node.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        node.create_subscription(JointState, '/joint_states', self.on_js, 10, callback_group=cb)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, node)
        self.js = {}
        self.lock = threading.Lock()

    def on_js(self, m):
        with self.lock:
            for i in range(len(m.name)):
                self.js[m.name[i]] = m.position[i]

    def joints(self, names):
        with self.lock:
            return [self.js.get(n) for n in names]

    def fingers(self):
        return self.joints(FINGERS)

    def at(self, q, tol=0.05):
        # robot dang o tu the q?
        cur = self.joints(JOINTS)
        if None in cur:
            return False
        for i in range(6):
            if abs(cur[i] - q[i]) >= tol:
                return False
        return True

    def tool(self):
        # vi tri + huong tool0 tu TF
        try:
            t = self.tfb.lookup_transform(self.sc.frame, self.sc.ee, Time(),
                                          timeout=Duration(seconds=0.5))
        except Exception:
            return None
        a = t.transform.translation
        r = t.transform.rotation
        return (a.x, a.y, a.z), (r.x, r.y, r.z, r.w)

    def wait_ready(self, t=180.0):
        # cho move_group, controller, joint_states
        end = time.time() + t
        while time.time() < end:
            ok = (self.mv.server_is_ready() and self.ex.server_is_ready()
                  and self.jtc.server_is_ready() and self.cart.service_is_ready()
                  and self.ps.service_is_ready() and self.ikc.service_is_ready()
                  and None not in self.fingers() and self.tool() is not None)
            if ok:
                time.sleep(2.0)
                return True
            time.sleep(0.5)
        return False

    # ---------------- planning scene
    def apply(self, ps):
        ps.is_diff = True
        ps.robot_state.is_diff = True
        req = ApplyPlanningScene.Request()
        req.scene = ps
        res = wait(self.ps.call_async(req), 10.0)
        return res is not None and res.success

    def remove(self, o):
        # xoa tung khoi rieng (xoa chung de bi tu choi)
        ps = PlanningScene()
        ps.world.collision_objects = [mk_box(o, (0, 0, 0), (0, 0, 0), op=CollisionObject.REMOVE)]
        self.apply(ps)

    def setup_scene(self):
        # bo khoi dang gan o tool
        ps = PlanningScene()
        for o in self.sc.objs:
            a = AttachedCollisionObject()
            a.link_name = self.sc.ee
            a.object.id = o
            a.object.operation = CollisionObject.REMOVE
            ps.robot_state.attached_collision_objects.append(a)
        self.apply(ps)
        # xoa khoi cu
        for o in self.sc.objs:
            self.remove(o)
        # them ban
        tb = self.sc.tb
        h = tb['h']
        ps = PlanningScene()
        ps.world.collision_objects = [mk_box('table', (tb['sx'], tb['sy'], h),
                                             (tb['x'], tb['y'], h / 2))]
        return self.apply(ps)

    def set_objects(self):
        # cap nhat khoi theo camera
        sc = self.sc
        s = sc.cube
        ps = PlanningScene()
        for o in sc.pos:
            p = sc.pos[o]
            if o == sc.held:
                continue
            if p is None:
                self.remove(o)
            else:
                ps.world.collision_objects.append(mk_box(o, (s, s, s), (p[0], p[1], sc.z_cube), p[2]))
        return self.apply(ps)

    def attach(self, o):
        # gan khoi vao tool0 (hop nho hon 1 chut de khong cham ban)
        self.remove(o)
        s = self.sc.cube - self.sc.shrink
        a = AttachedCollisionObject()
        a.link_name = self.sc.ee
        a.object = mk_box(o, (s, s, s), (0, 0, self.sc.d_hold), frame=self.sc.ee)
        a.touch_links = ['tool0', 'grip_base', 'finger_l', 'finger_r', 'wrist_3_link']
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [a]
        if self.apply(ps):
            return 'SUCCESS'
        return 'FAILED'

    def detach(self, o, p, yaw):
        # bo khoi khoi tool0
        a = AttachedCollisionObject()
        a.link_name = self.sc.ee
        a.object.id = o
        a.object.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [a]
        self.apply(ps)
        # dat lai khoi tai p
        s = self.sc.cube
        ps = PlanningScene()
        ps.world.collision_objects = [mk_box(o, (s, s, s), (p[0], p[1], self.sc.z_cube), yaw)]
        if self.apply(ps):
            return 'SUCCESS'
        return 'FAILED'

    # ---------------- gripper
    def grip(self, close):
        # gui luc, cho 2s, tra ve khe ho 2 ngon
        f = self.sc.grip['force']
        if close:
            f = -f
        self.gp.publish(Float64MultiArray(data=[f, f]))
        time.sleep(2.0)
        return sum(self.fingers())

    # ---------------- chuyen dong
    def send(self, goal):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = self.sc.group
        r.num_planning_attempts = 10
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = float(self.sc.vel)
        r.max_acceleration_scaling_factor = float(self.sc.vel)
        r.start_state.is_diff = True
        r.goal_constraints = [goal]
        g.planning_options.plan_only = False
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        st = 'PLANNING_FAILED'
        for _ in range(2):              # loi thi thu lai 1 lan
            h = wait(self.mv.send_goal_async(g), 10.0)
            if h is None or not h.accepted:
                continue
            res = wait(h.get_result_async(), 120.0)
            if res is None:
                st = 'FAILED'
            elif res.result.error_code.val == MoveItErrorCodes.SUCCESS:
                return 'SUCCESS'
            time.sleep(1.0)
        return st

    def move_q(self, q):
        # di toi 6 goc khop q
        c = Constraints()
        for i in range(6):
            j = JointConstraint()
            j.joint_name = JOINTS[i]
            j.position = float(q[i])
            j.tolerance_above = 0.01
            j.tolerance_below = 0.01
            j.weight = 1.0
            c.joint_constraints.append(j)
        return self.send(c)

    def ik(self, p, yaw):
        # IK: tool0 tai p, huong xuong, xoay yaw (seed co dinh)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.sc.group
        r.ik_link_name = self.sc.ee
        r.avoid_collisions = True
        r.timeout = Duration(seconds=1.0).to_msg()
        r.robot_state.joint_state.name = JOINTS
        r.robot_state.joint_state.position = [float(v) for v in self.sc.seed_q]
        r.pose_stamped.header.frame_id = self.sc.frame
        r.pose_stamped.pose = mk_pose(p, (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0))
        res = wait(self.ikc.call_async(req), 5.0)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        d = {}
        for i in range(len(js.name)):
            d[js.name[i]] = js.position[i]
        return [d[n] for n in JOINTS]

    def move_xyz(self, p, yaw):
        # IK roi lap ke hoach
        q = self.ik(p, yaw)
        if q is None:
            return 'PLANNING_FAILED'
        return self.move_q(q)

    def move_z(self, z):
        # di thang dung toi do cao z
        t = self.tool()
        if t is None:
            return 'FAILED'
        x = t[0][0]
        y = t[0][1]
        q = t[1]
        req = GetCartesianPath.Request()
        req.header.frame_id = self.sc.frame
        req.start_state.is_diff = True
        req.group_name = self.sc.group
        req.link_name = self.sc.ee
        req.waypoints = [mk_pose((x, y, z), q)]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        res = wait(self.cart.call_async(req), 15.0)
        if res is None or res.fraction < 0.98:
            return 'PLANNING_FAILED'
        # chay quy dao
        g = ExecuteTrajectory.Goal()
        g.trajectory = res.solution
        h = wait(self.ex.send_goal_async(g), 10.0)
        if h is None or not h.accepted:
            return 'FAILED'
        r = wait(h.get_result_async(), 60.0)
        if r is None or r.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return 'FAILED'
        return 'SUCCESS'
