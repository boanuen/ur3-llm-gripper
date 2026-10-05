"""Thu LLM khong can ROS (robot gia).

    python3 -m ur3_llm_control.offline_cli "Move the blue cube to zone C."
"""
import readline  # noqa: F401  (go tieng Viet trong Command>)
import sys

import yaml

from .fake_robot import Fake, FakeCam
from .llm_planner import Planner
from .robot_skills import Skills
from .scene import Scene, cfg_path
from .skill_executor import run_cmd
from .student import info


def main():
    sc = Scene(cfg_path('scene.yaml'))
    with open(cfg_path('student_config.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    # khoi bat dau o vi tri spawn
    truth = {}
    for o in sc.spawn:
        x, y = sc.spawn[o]
        truth[o] = (x, y, 0.0)
    rb = Fake(sc, truth)
    sk = Skills(sc, rb, FakeCam(rb))
    pl = Planner(cfg['llm'], sc, cfg['student_name'], cfg['student_id'])
    print(info(cfg['student_name'], cfg['student_id']))

    # lenh tu tham so
    if len(sys.argv) > 1:
        run_cmd(' '.join(sys.argv[1:]), pl, sk)
        return 0
    # go lenh
    while True:
        try:
            cmd = input('\nCommand> ').strip()
        except (EOFError, KeyboardInterrupt):
            return 0
        if cmd in ('q', 'quit', 'exit'):
            return 0
        if cmd:
            run_cmd(cmd, pl, sk)


if __name__ == '__main__':
    sys.exit(main())
