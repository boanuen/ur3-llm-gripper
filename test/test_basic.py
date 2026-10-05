"""Test khong can ROS / Gazebo / 9Router:  python3 -m pytest test -q"""
import json
import math
import os

import numpy as np

from ur3_llm_control.fake_robot import Fake, FakeCam
from ur3_llm_control.llm_planner import Planner, get_json, make_prompt
from ur3_llm_control.robot_skills import Skills
from ur3_llm_control.scene import Scene
from ur3_llm_control.skill_executor import fix_text, run_cmd
from ur3_llm_control.student import assign, get_p
from ur3_llm_control.task_validator import check
from ur3_llm_control.vision import detect, make_K, pix2world, world2pix

SCENE = os.path.join(os.path.dirname(__file__), '..', 'config', 'scene.yaml')
CFG = {'base_url': 'x', 'model': 'x', 'tries': 2}


def setup(truth=None):
    """Scene + robot gia + camera gia. Mac dinh khoi o vi tri spawn (blue trong zone_b)."""
    sc = Scene(SCENE)
    if truth is None:
        truth = {o: (x, y, 0.0) for o, (x, y) in sc.spawn.items()}
    rb = Fake(sc, truth)
    sk = Skills(sc, rb, FakeCam(rb), log=lambda s: None)
    sk.detect_objects()
    return sc, rb, sk


def P(*steps):
    """P(('pick','red_cube'), ('place','red_cube','zone_b')) -> {'plan': [...]}"""
    out = []
    for s in steps:
        d = {'skill': s[0]}
        if s[0] in ('pick', 'place_free', 'find_object'):
            d['object'] = s[1]
        if s[0] == 'place':
            d['object'], d['zone'] = s[1], s[2]
        if s[0] in ('check_zone', 'clear_zone'):
            d['zone'] = s[1]
        out.append(d)
    return {'plan': out}


def llm(*answers):
    """LLM gia: lan luot tra ve cac plan cho truoc."""
    a = [json.dumps(x) for x in answers]
    return lambda cfg, msgs: a.pop(0)


def quiet(s):
    pass


# ---------------- MSSV
def test_p():
    assert get_p('23020772') == 0
    assert assign('23020772') == {'zone_a': 'red_cube', 'zone_b': 'yellow_cube',
                                  'zone_c': 'blue_cube'}
    assert get_p('23020123') == 5


def test_prompt():
    s = make_prompt(Scene(SCENE), 'A', '23020772')
    for w in ['clear_zone(zone)', 'place_free(object)', 'green_cube', 'purple_cube',
              'red_cube -> zone_a']:
        assert w in s


# ---------------- camera
def test_pixel_world_roundtrip():
    sc = Scene(SCENE)
    K = make_K(sc.cam)
    for x, y in [(0.22, -0.22), (0.38, 0.0), (0.3, 0.3)]:
        u, v = world2pix(x, y, sc.z_top, K, sc.cam)
        x2, y2 = pix2world(u, v, K, sc.cam, sc.z_top)
        assert abs(x - x2) < 1e-6 and abs(y - y2) < 1e-6


def test_detect_synthetic_image():
    """Ve 5 o vuong mau len anh xam, camera phai tim dung vi tri."""
    import cv2
    sc = Scene(SCENE)
    K = make_K(sc.cam)
    img = np.full((sc.cam['height'], sc.cam['width'], 3), 200, np.uint8)
    for o, (x, y) in sc.spawn.items():
        r, g, b, _ = sc.objs[o]['rgba']
        pts = [world2pix(x + dx, y + dy, sc.z_top, K, sc.cam)
               for dx, dy in [(-.02, -.02), (-.02, .02), (.02, .02), (.02, -.02)]]
        cv2.fillPoly(img, [np.int32(pts)], (int(r * 255), int(g * 255), int(b * 255)))
    det = detect(img, K, sc)
    assert set(det) == set(sc.spawn)
    for o, (x, y) in sc.spawn.items():
        assert math.hypot(det[o][0] - x, det[o][1] - y) < 0.005


# ---------------- trang thai, zone, cho trong
def test_zone_state():
    sc, rb, sk = setup()
    assert sc.zone_state() == {'zone_a': 'free', 'zone_b': 'blue_cube', 'zone_c': 'green_cube'}
    assert sc.where('red_cube') == 'table'


def test_free_pos_is_free():
    sc, rb, sk = setup()
    p = sc.free_pos((0.38, 0.0))
    assert p is not None and sc.free_ok(p)
    for z in sc.zones:
        assert not sc.in_zone(p, z)


# ---------------- validator
def test_validator_adds_safety_steps():
    sc, rb, sk = setup()
    plan, errs = check(P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_b')), sc)
    assert errs == []
    names = [s['skill'] for s in plan]
    assert names == ['detect_objects', 'clear_zone', 'pick', 'place', 'home']


def test_validator_rejects():
    sc, rb, sk = setup()
    assert check({'plan': [{'skill': 'fly'}]}, sc)[1]
    assert 'INVALID_OBJECT' in check(P(('pick', 'orange_cube')), sc)[1][0]
    assert 'INVALID_ZONE' in check(P(('pick', 'red_cube'), ('place', 'red_cube', 'zone_d')), sc)[1][0]
    assert check({'plan': [{'skill': 'home', 'joints': [0] * 6}]}, sc)[1]        # lenh khop
    assert check(P(('pick', 'red_cube'), ('pick', 'blue_cube')), sc)[1]          # cam 2 vat
    assert check(P(('pick', 'red_cube'), ('check_zone', 'zone_a')), sc)[1]       # camera khi dang cam


def test_validator_object_not_seen():
    sc, rb, sk = setup({'red_cube': (0.22, -0.22, 0.0)})     # camera chi thay khoi do
    assert 'OBJECT_NOT_FOUND' in check(P(('pick', 'blue_cube')), sc)[1][0]


def test_get_json():
    assert get_json('```json\n{"plan": []}\n```') == {'plan': []}
    assert get_json('<think>x {y}</think> {"plan": [1]}') == {'plan': [1]}


def test_fix_text():
    assert fix_text('l\udce1\udcba\udca5y') == 'lấy'


# ---------------- chay ca luong
def test_occupied_zone_demo():
    """Zone B dang co blue_cube, lenh: dat red_cube vao zone B."""
    sc, rb, sk = setup()
    pl = Planner(CFG, sc, 'A', '23020772', ask_fn=llm(P(
        ('detect_objects',), ('check_zone', 'zone_b'), ('clear_zone', 'zone_b'),
        ('pick', 'red_cube'), ('place', 'red_cube', 'zone_b'), ('home',))))
    rep = run_cmd('Put the red cube in Zone B.', pl, sk, quiet)
    assert rep['status'] == 'TASK SUCCESS'
    assert sc.who('zone_b') == 'red_cube'
    assert sc.where('blue_cube') == 'table'               # blue da duoc dua ra ngoai


def test_arrange_by_student_id():
    sc, rb, sk = setup()
    steps = [('detect_objects',)]
    for z, o in assign('23020772').items():
        steps += [('check_zone', z), ('clear_zone', z), ('pick', o), ('place', o, z)]
    pl = Planner(CFG, sc, 'A', '23020772', ask_fn=llm(P(*steps, ('home',))))
    rep = run_cmd('Arrange all objects according to my student ID.', pl, sk, quiet)
    assert rep['status'] == 'TASK SUCCESS'
    assert sc.zone_state() == {'zone_a': 'red_cube', 'zone_b': 'yellow_cube', 'zone_c': 'blue_cube'}


def test_already_in_zone_is_skipped():
    sc, rb, sk = setup()
    pl = Planner(CFG, sc, 'A', '23020772', ask_fn=llm(P(
        ('pick', 'blue_cube'), ('place', 'blue_cube', 'zone_b'))))
    rep = run_cmd('Put the blue cube in zone B', pl, sk, quiet)
    assert rep['status'] == 'TASK SUCCESS'
    assert [s for _, s in rep['steps']].count('SKIPPED') >= 2


def test_rejected_plan_not_executed():
    sc, rb, sk = setup()
    n = len(rb.log)
    pl = Planner(CFG, sc, 'A', '23020772', ask_fn=llm({'plan': [], 'error': 'no orange cube'}))
    rep = run_cmd('Move the orange cube', pl, sk, quiet)
    assert rep['status'] == 'TASK REJECTED'
    assert not any(a[0] in ('move_xyz', 'grip') for a in rb.log[n:])   # robot khong gap gi


def test_grasp_failed_stops():
    sc, rb, sk = setup()
    rb.truth['red_cube'] = (0.30, 0.30, 0.0)    # camera thay sai cho -> kep truot
    sc.pos['red_cube'] = (0.22, -0.22, 0.0)
    assert sk.pick('red_cube') == 'GRASP_FAILED'
    assert sc.held is None
