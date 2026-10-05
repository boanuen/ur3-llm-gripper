"""Camera: tim khoi theo mau (HSV) roi doi pixel -> toa do (x, y) tren ban."""
import math
import threading
import time

import cv2
import numpy as np


def rot(r, p, y):
    # ma tran quay tu roll, pitch, yaw
    cr = math.cos(r)
    sr = math.sin(r)
    cp = math.cos(p)
    sp = math.sin(p)
    cy = math.cos(y)
    sy = math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def pix2world(u, v, K, cam, z):
    # pixel (u, v) -> (x, y) tai do cao z
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    a = [(u - cx) / fx, (v - cy) / fy, 1.0]          # tia trong truc anh
    d = rot(*cam['rpy']) @ np.array([a[2], -a[0], -a[1]])   # doi truc anh -> truc link
    o = np.array(cam['xyz'], dtype=float)
    t = (z - o[2]) / d[2]
    p = o + t * d
    return float(p[0]), float(p[1])


def world2pix(x, y, z, K, cam):
    # diem 3D -> pixel (dung trong test)
    d = np.array([x, y, z]) - np.array(cam['xyz'], dtype=float)
    dl = rot(*cam['rpy']).T @ d
    a = [-dl[1], -dl[2], dl[0]]
    u = K[0] * a[0] / a[2] + K[2]
    v = K[4] * a[1] / a[2] + K[5]
    return u, v


def make_K(cam):
    # K tam tinh tu hfov (khi chua co camera_info)
    f = cam['width'] / 2 / math.tan(cam['hfov'] / 2)
    return [f, 0, cam['width'] / 2, 0, f, cam['height'] / 2, 0, 0, 1]


def detect(img, K, sc):
    """Tim khoi trong anh RGB -> {ten: (x, y, yaw)}."""
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    t = sc.tb
    out = {}
    for name in sc.objs:
        # loc mau
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for r in sc.objs[name]['hsv']:
            mask = mask | cv2.inRange(hsv, np.array(r[:3]), np.array(r[3:]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # chon vung lon nhat
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, n):
            a = st[i, cv2.CC_STAT_AREA]
            if 150 < a < 4000 and (best is None or a > st[best, cv2.CC_STAT_AREA]):
                best = i
        if best is None:
            continue

        # tam khoi -> toa do ban
        pts = cv2.findNonZero((lab == best).astype(np.uint8))
        rect = cv2.minAreaRect(pts)
        u, v = rect[0]
        x, y = pix2world(u, v, K, sc.cam, sc.z_top)
        if abs(x - t['x']) > t['sx'] / 2 or abs(y - t['y']) > t['sy'] / 2:
            continue                # ngoai ban

        # goc xoay tu 1 canh, dua ve +-45 do
        box = cv2.boxPoints(rect)
        p0 = pix2world(box[0][0], box[0][1], K, sc.cam, sc.z_top)
        p1 = pix2world(box[1][0], box[1][1], K, sc.cam, sc.z_top)
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
        out[name] = (round(x, 4), round(y, 4), round(yaw, 3))
    return out


class Camera:
    """Nhan anh tu camera Gazebo."""

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
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        a = a[:, :, :3]
        if m.encoding == 'bgr8':
            a = a[:, :, ::-1]
        with self.lock:
            self.img = a.copy()
            self.n += 1

    def grab(self, t=10.0):
        # doi 2 anh moi (luc robot da dung yen)
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
        img = self.grab()
        if img is None:
            return None
        return detect(img, self.K, self.sc)
