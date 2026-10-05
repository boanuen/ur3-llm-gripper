"""Camera: tim khoi theo mau (OpenCV, HSV) roi doi pixel -> toa do (x, y) tren ban.

Camera co dinh, biet vi tri (scene.yaml) va K (camera_info). Moi pixel ung voi 1 tia,
cat tia voi mat phang z = mat tren cua khoi la ra (x, y).
"""
import math
import threading
import time

import cv2
import numpy as np


def rot(r, p, y):
    """Ma tran quay tu roll, pitch, yaw."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pix2world(u, v, K, cam, z):
    """Pixel (u, v) -> (x, y) tren mat phang do cao z.
    Chu y: truc anh la (x phai, y xuong, z truoc), con link camera trong Gazebo
    la (x truoc, y trai, z len) nen phai doi truc truoc khi quay."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    d_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    d = rot(*cam['rpy']) @ np.array([d_opt[2], -d_opt[0], -d_opt[1]])
    o = np.array(cam['xyz'], dtype=float)
    t = (z - o[2]) / d[2]
    p = o + t * d
    return float(p[0]), float(p[1])


def world2pix(x, y, z, K, cam):
    """Nguoc lai, diem 3D -> pixel (chi dung trong test)."""
    d = np.array([x, y, z]) - np.array(cam['xyz'], dtype=float)
    dl = rot(*cam['rpy']).T @ d
    d_opt = np.array([-dl[1], -dl[2], dl[0]])
    return (K[0] * d_opt[0] / d_opt[2] + K[2], K[4] * d_opt[1] / d_opt[2] + K[5])


def make_K(cam):
    """Tinh K tu goc nhin hfov (dung tam khi chua nhan duoc camera_info)."""
    f = cam['width'] / 2 / math.tan(cam['hfov'] / 2)
    return [f, 0, cam['width'] / 2, 0, f, cam['height'] / 2, 0, 0, 1]


def detect(img, K, sc):
    """Tim cac khoi trong anh RGB. Tra ve {ten: (x, y, yaw)}, yaw trong khoang +-45 do."""
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    t = sc.tb
    out = {}
    for name, o in sc.objs.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for r in o['hsv']:
            mask |= cv2.inRange(hsv, np.array(r[:3]), np.array(r[3:]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, n):
            a = st[i, cv2.CC_STAT_AREA]
            if 150 < a < 4000 and (best is None or a > st[best, cv2.CC_STAT_AREA]):
                best = i
        if best is None:
            continue
        pts = np.column_stack(np.where(lab == best))[:, ::-1].astype(np.float32)   # (u, v)
        (u, v), _, _ = cv2.minAreaRect(pts)
        x, y = pix2world(u, v, K, sc.cam, sc.z_top)
        # bo qua neu nam ngoai mat ban (vd mau tren than robot)
        if abs(x - t['x']) > t['sx'] / 2 or abs(y - t['y']) > t['sy'] / 2:
            continue
        # goc xoay cua khoi: lay 1 canh cua hinh chu nhat bao quanh
        box = cv2.boxPoints(cv2.minAreaRect(pts))
        p0 = pix2world(box[0][0], box[0][1], K, sc.cam, sc.z_top)
        p1 = pix2world(box[1][0], box[1][1], K, sc.cam, sc.z_top)
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
        out[name] = (round(x, 4), round(y, 4), round(yaw, 3))
    return out


class Camera:
    """Nhan anh tu camera trong Gazebo."""

    def __init__(self, node, sc, cb=None):
        from sensor_msgs.msg import CameraInfo, Image
        self.sc = sc
        self.K = make_K(sc.cam)
        self.img = None
        self.n = 0
        self.lock = threading.Lock()
        node.create_subscription(Image, sc.cam['topic'], self.on_img, 2, callback_group=cb)
        node.create_subscription(CameraInfo, sc.cam['info_topic'], self.on_info, 2,
                                 callback_group=cb)

    def on_info(self, m):
        self.K = list(m.k)

    def on_img(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        if m.encoding == 'bgr8':
            a = a[:, :, ::-1]
        with self.lock:
            self.img = a.copy()
            self.n += 1

    def grab(self, t=10.0):
        """Bo qua anh cu, doi 2 anh moi (chup luc robot da dung yen)."""
        with self.lock:
            n0 = self.n
        end = time.time() + t
        while time.time() < end:
            with self.lock:
                if self.n >= n0 + 2:
                    return self.img
            time.sleep(0.05)
        return None

    def capture(self):
        """Chup 1 anh va tim khoi. None neu khong co anh."""
        img = self.grab()
        if img is None:
            return None
        return detect(img, self.K, self.sc)
