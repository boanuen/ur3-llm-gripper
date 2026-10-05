"""Sinh file world Gazebo (SDF) tu scene.yaml: ban, 3 zone, 5 khoi (vat ly that), camera.
Ban va zone mau xam/trang de khong lan voi mau cac khoi khi nhan dang."""


def color(rgba):
    c = ' '.join(str(v) for v in rgba)
    return f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'


def static_box(name, p, size, rgba, coll=True):
    g = f'<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'
    col = f'<collision name="c">{g}</collision>' if coll else ''
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose>
      <link name="link">{col}<visual name="v">{g}{color(rgba)}</visual></link>
    </model>"""


def cube(name, p, s, rgba):
    """Khoi lap phuong dong (co trong luc, ma sat) -> gripper phai kep that."""
    m = 0.05
    i = m * s * s / 6
    g = f'<geometry><box><size>{s} {s} {s}</size></box></geometry>'
    return f"""
    <model name="{name}">
      <pose>{p[0]} {p[1]} {p[2]} 0 0 0</pose>
      <link name="link">
        <inertial><mass>{m}</mass>
          <inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">{g}
          <surface>
            <friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction>
            <contact><ode><kp>1000000</kp><kd>100</kd><max_vel>0.0</max_vel><min_depth>0.001</min_depth></ode></contact>
          </surface>
        </collision>
        <visual name="v">{g}{color(rgba)}</visual>
      </link>
    </model>"""


def camera(c):
    x, y, z = c['xyz']
    r, p, w = c['rpy']
    return f"""
    <model name="top_camera">
      <static>true</static>
      <pose>{x} {y} {z} {r} {p} {w}</pose>
      <link name="link">
        <visual name="v"><geometry><box><size>0.04 0.06 0.04</size></box></geometry>{color([0.2, 0.2, 0.2, 1])}</visual>
        <sensor name="cam" type="camera">
          <update_rate>5</update_rate>
          <camera>
            <horizontal_fov>{c['hfov']}</horizontal_fov>
            <image><width>{c['width']}</width><height>{c['height']}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>5</far></clip>
          </camera>
          <plugin name="cam_plugin" filename="libgazebo_ros_camera.so">
            <ros><namespace>/camera</namespace></ros>
            <camera_name>top</camera_name>
            <frame_name>camera_link</frame_name>
          </plugin>
        </sensor>
      </link>
    </model>"""


def make_world(sc):
    t, h, s = sc.tb, sc.tb['h'], sc.cube
    ms = [static_box('work_table', (t['x'], t['y'], h / 2), (t['sx'], t['sy'], h),
                     (0.75, 0.75, 0.75, 1))]
    for n, z in sc.zones.items():
        x, y = z['xy']
        k = z['size']
        ms.append(static_box(n + '_border', (x, y, h + 0.0005), (k + 0.012, k + 0.012, 0.001),
                             (0.15, 0.15, 0.15, 1), coll=False))
        ms.append(static_box(n, (x, y, h + 0.001), (k, k, 0.001), (1, 1, 1, 1), coll=False))
    for n, (x, y) in sc.spawn.items():
        ms.append(cube(n, (x, y, h + s / 2 + 0.002), s, sc.objs[n]['rgba']))
    ms.append(camera(sc.cam))
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="ur3_llm_world">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver><iters>50</iters></solver>
        <constraints><contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0005</contact_surface_layer></constraints>
      </ode>
    </physics>
    <gui><camera name="user_camera"><pose>1.1 -0.8 0.8 0 0.45 2.45</pose></camera></gui>
    {''.join(ms)}
  </world>
</sdf>
"""
