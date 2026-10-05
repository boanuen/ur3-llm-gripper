"""Node ROS 2 dieu khien UR3/UR3e bang ngon ngu tu nhien.

  ros2 run ur3_llm_control llm_robot_node                        # go lenh truc tiep
  ros2 run ur3_llm_control llm_robot_node --ros-args -p command:="Put the red cube in zone B."
  ros2 run ur3_llm_control send_command "Move the blue cube to zone C."   # gui qua topic

Topic:  /llm_robot/command (sub, String)   /llm_robot/plan, /llm_robot/result (pub, String)
"""
import json
import queue
import readline  # noqa: F401  (go/xoa tieng Viet dung trong Command>)
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from .fake_robot import Fake, FakeCam
from .llm_planner import Planner
from .robot_skills import Skills
from .scene import Scene, cfg_path
from .skill_executor import run_cmd, show_state
from .student import info


def out(s=''):
    print(s, flush=True)


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_robot_node')
        self.declare_parameter('command', '')        # chay 1 lenh roi thoat
        self.declare_parameter('interactive', True)  # go lenh tu ban phim
        self.declare_parameter('fake', False)        # True: khong dung MoveIt
        self.declare_parameter('model', '')          # doi model LLM

        self.sc = Scene(cfg_path('scene.yaml'))
        with open(cfg_path('student_config.yaml'), encoding='utf-8') as f:
            st = yaml.safe_load(f)
        self.name, self.sid = st['student_name'], st['student_id']
        llm = st['llm']
        if self.get_parameter('model').value:
            llm['model'] = self.get_parameter('model').value

        if self.get_parameter('fake').value:     # robot + camera gia (test khong can Gazebo)
            truth = {o: (x, y, 0.0) for o, (x, y) in self.sc.spawn.items()}
            self.rb = Fake(self.sc, truth)
            self.cam = FakeCam(self.rb)
        else:
            from .moveit_if import MoveIt
            from .vision import Camera
            self.rb = MoveIt(self, self.sc)
            self.cam = Camera(self, self.sc)
        self.sk = Skills(self.sc, self.rb, self.cam, out)
        self.pl = Planner(llm, self.sc, self.name, self.sid)
        self.llm = llm

        self.q = queue.Queue()
        self.create_subscription(String, '/llm_robot/command', lambda m: self.q.put(m.data), 10)
        self.plan_pub = self.create_publisher(String, '/llm_robot/plan', 10)
        self.res_pub = self.create_publisher(String, '/llm_robot/result', 10)

    def setup(self):
        if isinstance(self.rb, Fake):
            return self.sk.detect_objects() == 'SUCCESS'
        out('[setup] cho MoveIt 2 (move_group) va controller ...')
        if not self.rb.wait_ready():
            out('[setup] LOI: khong thay move_group / controller. Da chay sim.launch.py chua?')
            return False
        if not self.rb.setup_scene():
            out('[setup] LOI: khong cap nhat duoc planning scene')
            return False
        out('[setup] da them ban vao planning scene')
        out(f'[setup] open_gripper() ... {self.sk.open_gripper()}')
        for i in range(3):                 # thu lai neu Gazebo con dang khoi dong
            st = self.sk.home()
            out(f'[setup] home() ... {st}')
            if st == 'SUCCESS':
                break
            time.sleep(3.0)
        if st != 'SUCCESS':
            return False
        st = self.sk.detect_objects()
        out(f'[setup] detect_objects() ... {st}')
        return st == 'SUCCESS'

    def do_cmd(self, cmd):
        rep = run_cmd(cmd, self.pl, self.sk, out)
        if rep['plan']:
            self.plan_pub.publish(String(data=json.dumps({'plan': rep['plan']})))
        self.res_pub.publish(String(data=json.dumps(rep)))
        return rep


def main(args=None):
    rclpy.init(args=args)
    nd = LLMNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(nd)
    th = threading.Thread(target=ex.spin, daemon=True)     # spin o thread rieng
    th.start()

    out('=' * 50)
    out('UR3 LLM CONTROL')
    out(info(nd.name, nd.sid))
    out(f"LLM: {nd.llm['model']} @ {nd.llm['base_url']}")
    out('=' * 50)
    code = 0
    try:
        if not nd.setup():
            code = 1
        elif nd.get_parameter('command').value:
            rep = nd.do_cmd(nd.get_parameter('command').value)
            code = 0 if rep['status'] == 'TASK SUCCESS' else 2
        elif nd.get_parameter('interactive').value and sys.stdin.isatty():
            out("Nhap lenh ('state' xem trang thai, 'q' de thoat)")
            while True:
                cmd = input('\nCommand> ').strip()
                if cmd in ('q', 'quit', 'exit'):
                    break
                if cmd == 'state':              # chup lai va in trang thai
                    nd.sk.detect_objects()
                    show_state(nd.sc, out)
                elif cmd:
                    nd.do_cmd(cmd)
        else:
            out('Cho lenh tren topic /llm_robot/command ...')
            while rclpy.ok():
                try:
                    cmd = nd.q.get(timeout=0.5)
                except queue.Empty:
                    continue
                nd.do_cmd(cmd)
    except (KeyboardInterrupt, EOFError):
        pass
    if rclpy.ok():
        rclpy.shutdown()      # dung spin truoc roi moi huy node
    th.join(timeout=3)
    ex.shutdown()
    nd.destroy_node()
    return code


if __name__ == '__main__':
    sys.exit(main())
