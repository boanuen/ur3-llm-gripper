- Nhiệm vụ: P = 72 mod 6 = **0** → Zone A = red, Zone B = yellow, Zone C = blue
- Video demo: https://drive.google.com/file/d/1FGfWvSGGuMDtMpVdWTMnC-y5MIgk3V_K/view?usp=sharing

Bài 03 phát triển tiếp từ Bài 02: thêm gripper, camera đang ở đâu, 5 khối nhưng chỉ có 3 zone nên robot phải tự xử lý khi zone đích đang bị chiếm.

## 1. Kiến trúc

```
Câu lệnh (tiếng Việt / tiếng Anh)
        ↓
Camera             vision.py          chụp ảnh → tìm 5 khối theo màu → khối nào ở đâu, zone nào trống
        ↓
LLM Planner        llm_planner.py     prompt (config/prompt.txt) + trạng thái camera → 9Router → JSON
        ↓
Plan Validator     task_validator.py  kiểm tra skill / object / zone / logic, thêm bước an toàn
        ↓
Skill Executor     skill_executor.py  chạy từng skill, dừng khi lỗi, camera kiểm tra lại kết quả
        ↓
Robot Skills       robot_skills.py    detect_objects, check_zone, clear_zone, pick, place, ...
        ↓
MoveIt 2           moveit_if.py       IK, lập kế hoạch, tránh va chạm, điều khiển gripper
        ↓
UR3e + gripper (Gazebo)
```

LLM chỉ chọn skill, tham số và thứ tự. LLM không sinh góc khớp hay quỹ đạo.
Vị trí các khối không khai báo cứng mà lấy từ camera.

## 2. Môi trường mô phỏng

| Thành phần | Mô tả |
|---|---|
| Robot | UR3e, home quay sang bên (ngoài tầm nhìn camera) |
| Gripper | 2 ngón tự xây (`urdf/ur_gripper.urdf.xacro`), gắn vào `tool0` |
| Camera | RGB 640×480, đặt cao 1 m nhìn thẳng xuống bàn, topic `/camera/top/image_raw` |
| Bàn | 34 × 80 cm, màu xám (không trùng màu khối) |
| Zone | `zone_a`, `zone_b`, `zone_c`: ô trắng viền đen |
| Khối | `red_cube`, `yellow_cube`, `green_cube`, `blue_cube`, `purple_cube`, cạnh 4 cm, nặng 50 g |

Lúc đầu `blue_cube` nằm trong zone B, `green_cube` nằm trong zone C, 3 khối còn lại nằm ngoài.
World Gazebo được sinh tự động từ `config/scene.yaml` (`world_gen.py`).

### Gripper

- Mỗi ngón là một khớp trượt, điều khiển bằng lực (`effort_controllers/JointGroupEffortController`): gửi +15 N là mở, −15 N là kẹp.
- Vật được giữ hoàn toàn bằng lực ma sát giữa ngón tay và khối. Chương trình không đặt pose cho vật.
- Biết kẹp được hay chưa dựa vào khe hở giữa 2 ngón (đọc từ `/joint_states`):
  khe ≈ 4 cm là đang kẹp khối, khe gần 0 là kẹp trượt (`GRASP_FAILED`). Sau khi nhấc lên cũng kiểm tra lại một lần để biết vật có bị rơi không.
- Cánh tay dùng `joint_trajectory_controller` với giao diện velocity (có PID bám vị trí).
  Lúc đầu dùng position khối bị tuột: với position, Gazebo dịch các link tức thời, link không có vận tốc nên ma sát không kéo được khối lên. Chuyển sang velocity thì gắp được.

### Camera

1. Đổi ảnh sang HSV, lọc theo ngưỡng màu của từng khối (khai báo trong `scene.yaml`).
2. Tìm vùng màu lớn nhất, lấy tâm và góc xoay (`cv2.minAreaRect`).
3. Đổi pixel → tọa độ trên bàn: từ vị trí camera (đã biết) và ma trận K (`camera_info`), kéo một tia qua pixel rồi cắt với mặt phẳng mặt trên của khối.
4. Kết quả nằm ngoài mặt bàn thì bỏ qua (tránh nhận nhầm màu trên thân robot).

Sai số đo được so với vị trí thật trong Gazebo: dưới 7 mm.
Khi chụp, robot luôn về home trước để cánh tay không che camera.

## 3. Cấu trúc package

```
ur3_llm_control/
├── config/
│   ├── scene.yaml              bàn, zone, màu + ngưỡng HSV các khối, camera, gripper, home
│   ├── student_config.yaml     họ tên, MSSV, cấu hình 9Router
│   ├── prompt.txt              system prompt gửi cho LLM
│   ├── controllers.yaml        controller cánh tay (velocity) + gripper (effort)
│   └── initial_positions.yaml  tư thế lúc spawn
├── urdf/ur_gripper.urdf.xacro  UR3e + gripper 2 ngón
├── srdf/ur_gripper.srdf.xacro  SRDF cho MoveIt (bỏ check va chạm giữa các link gripper sát nhau)
├── launch/
│   ├── sim.launch.py           Gazebo + robot + camera + controller + MoveIt 2 + RViz
│   └── llm_robot.launch.py     sim.launch.py + node LLM (nhận lệnh qua topic)
├── ur3_llm_control/
│   ├── llm_robot_node.py       node ROS 2 chính
│   ├── vision.py               nhận dạng khối từ ảnh camera
│   ├── llm_planner.py          tạo prompt, gọi 9Router, lấy JSON
│   ├── task_validator.py       danh sách skill + kiểm tra plan
│   ├── skill_executor.py       chạy plan, in kết quả
│   ├── robot_skills.py         các robot skill
│   ├── moveit_if.py            giao tiếp MoveIt 2 + gripper
│   ├── scene.py                đọc scene.yaml, trạng thái (khối ở đâu, zone nào trống, chỗ trống)
│   ├── student.py              P = XX mod 6 và bảng màu → zone
│   ├── world_gen.py            sinh world Gazebo
│   ├── send_command.py         gửi 1 lệnh lên topic /llm_robot/command
│   ├── offline_cli.py          thử LLM không cần Gazebo (robot + camera giả)
│   └── fake_robot.py           robot + camera giả dùng cho test
└── test/test_basic.py          16 unit test
```

## 4. Robot skills

LLM được dùng các skill sau:

| Skill | Làm gì |
|---|---|
| `detect_objects()` | về home, chụp ảnh, cập nhật vị trí 5 khối, đưa các khối vào planning scene |
| `check_zone(zone)` | chụp lại, cho biết zone trống hay đang có khối nào |
| `find_object(object)` | kiểm tra camera có thấy khối không |
| `clear_zone(zone)` | zone đang có khối khác → `pick` khối đó rồi `place_free` ra chỗ trống |
| `pick(object)` | mở gripper → lên trên khối → hạ thẳng → kẹp (kiểm tra) → nhấc lên (kiểm tra rơi) |
| `place(object, zone)` | zone phải trống → lên trên zone → hạ → mở gripper → nhấc lên |
| `place_free(object)` | `find_free_position` → đặt khối đang cầm ra chỗ trống trên bàn |
| `home()` | về tư thế home |

Các skill dùng bên trong: `find_free_position`, `open_gripper`, `close_gripper`.

`find_free_position`: quét lưới 2 cm trên bàn, chọn điểm không nằm trong zone, cách các khối khác
và tâm các zone ít nhất 10 cm, nằm trong tầm với của UR3e, và gần khối cần dời nhất.

Trạng thái trả về: `SUCCESS`, `SKIPPED`, `FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`,
`OBJECT_NOT_FOUND`, `ZONE_OCCUPIED`, `GRASP_FAILED`, `NO_FREE_SPACE`, `PLANNING_FAILED`.

Về chuyển động:
- Di chuyển tới một điểm: gọi `/compute_ik` với seed cố định để ra góc khớp, rồi MoveIt lập kế hoạch
  trong không gian khớp. Seed cố định giúp robot không bị xoắn khớp sau nhiều lần di chuyển.
- Hạ và nhấc: đi thẳng đứng bằng `/compute_cartesian_path` (`avoid_collisions=True`).
- An toàn: URDF bật `safety_limits`. MoveIt dùng URDF/SRDF **có gripper** nên kiểm tra va chạm cả
  gripper. Bàn và các khối camera thấy nằm trong planning scene; khối đang cầm được gắn vào `tool0`.

## 5. Plan Validator

1. Kết quả phải có dạng `{"plan": [...]}`.
2. Skill nằm trong danh sách trên, đúng tham số. Có tham số lạ (vd `joints`, `trajectory`) → từ chối.
3. Object / zone phải tồn tại, object phải được camera nhìn thấy (`OBJECT_NOT_FOUND`).
4. Chạy thử logic: không `pick` khi đang cầm, chỉ `place` vật đang cầm, không gọi camera khi đang cầm,
   cuối plan tay phải trống.
5. Thêm bước an toàn nếu LLM quên (in kèm `[auto]`): `detect_objects()` ở đầu, `clear_zone(zone)`
   trước `pick` nếu sau đó có `place(..., zone)`, `home()` ở cuối.

Sai thì gửi lỗi lại cho LLM sửa 1 lần, vẫn sai thì `TASK REJECTED` và robot không chạy.

## 6. Xử lý zone bị chiếm

Ví dụ: zone B đang có `blue_cube`, lệnh "Put the red cube in Zone B.":

```
camera → zone_b đang có blue_cube
check_zone(zone_b)
clear_zone(zone_b)  → find_free_position(blue_cube) → pick(blue_cube) → place_free(blue_cube)
pick(red_cube) → place(red_cube, zone_b)
home()
camera chụp lại → zone_b = red_cube
```

Nếu khối cần đặt đã nằm sẵn trong zone đích thì bỏ qua (`SKIPPED`).
Ngoài ra `place()` cũng tự kiểm tra lại, zone không trống thì trả `ZONE_OCCUPIED` chứ không đặt chồng.

## 7. Cài đặt

```bash
sudo apt update
sudo apt install -y ros-humble-ur ros-humble-moveit ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-cv-bridge ros-humble-rqt-image-view python3-opencv python3-pytest

mkdir -p ~/ur3_ws/src && cd ~/ur3_ws/src
git clone https://github.com/boanuen/ur3-llm-gripper.git ur3_llm_control
cd ~/ur3_ws && colcon build --symlink-install
source install/setup.bash
```

9Router (Node.js ≥ 18):
```bash
npm install -g 9router
9router                          # dashboard: http://localhost:20128
```
Trong dashboard kết nối provider (vd Gemini bằng API key của Google AI Studio), tạo API key của
9Router rồi đặt vào biến môi trường (không ghi key vào file):
```bash
echo 'export NINE_KEY="sk-..."' >> ~/.bashrc && source ~/.bashrc
```
Tên model đặt trong `config/student_config.yaml` (`model`, `backup_model`).

## 8. Chạy

Terminal 1 – 9Router: `9router`

Terminal 2 – mô phỏng (tắt tiến trình cũ trước):
```bash
pkill -9 -x gzserver; pkill -9 -x gzclient; pkill -9 -x move_group; pkill -9 -x rviz2; pkill -9 -x robot_state_pub
ros2 launch ur3_llm_control sim.launch.py
```
Chờ đến khi log có `gripper_controller` và `joint_trajectory_controller` đã activated.

Terminal 3 – node LLM:
```bash
ros2 run ur3_llm_control llm_robot_node
```
```
Command> Put red cube in Zone B.
Command> Lấy khốI tím và đặt nó vào ô A.
Command> Arrange all objects according to my student ID.
Command> Put the orange cube in zone D.
Command> state
```

Xem hình camera: `ros2 run rqt_image_view rqt_image_view /camera/top/image_raw`

Muốn các khối về chỗ cũ: tắt terminal 2, 3 rồi chạy lại.

Test không cần Gazebo:
```bash
cd ~/ur3_ws/src/ur3_llm_control && python3 -m pytest test -q
ros2 run ur3_llm_control offline_cli "Put red cube in Zone B."
```

Trên WSL2: nếu cửa sổ Gazebo / RViz trống thì thêm `LIBGL_ALWAYS_SOFTWARE=1` trước lệnh launch.

## 9. Kết quả

Lệnh "Put the red cube in Zone B." (zone B đang có `blue_cube`):

```
CAMERA:
  red_cube     table (0.22, -0.21)
  yellow_cube  table (0.22, 0.06)
  green_cube   zone_c (0.38, -0.16)
  blue_cube    zone_b (0.38, 0.00)
  purple_cube  table (0.24, 0.26)
  zones: zone_a=free, zone_b=blue_cube, zone_c=green_cube

LLM PLAN:
1. detect_objects()
2. check_zone(zone_b)
3. clear_zone(zone_b)
4. pick(red_cube)
5. place(red_cube, zone_b)
6. home()

EXECUTION:
detect_objects() .................. SUCCESS
    zone_b: dang co blue_cube
check_zone(zone_b) ................ SUCCESS
    don zone_b: pick(blue_cube) -> place_free(blue_cube)
    find_free_position(blue_cube) -> (0.28, -0.04)
clear_zone(zone_b) ................ SUCCESS
pick(red_cube) .................... SUCCESS
place(red_cube, zone_b) ........... SUCCESS
home() ............................ SUCCESS

TASK SUCCESS
CAMERA (sau khi lam):
  zones: zone_a=free, zone_b=red_cube, zone_c=green_cube
```

Lệnh "Arrange all objects according to my student ID." (14 bước, dọn 2 zone bị chiếm):

```
clear_zone(zone_a) ................ SKIPPED
pick(red_cube) .................... SUCCESS
place(red_cube, zone_a) ........... SUCCESS
clear_zone(zone_b) ................ SUCCESS     (blue_cube -> (0.28, -0.04))
pick(yellow_cube) ................. SUCCESS
place(yellow_cube, zone_b) ........ SUCCESS
clear_zone(zone_c) ................ SUCCESS     (green_cube -> (0.28, 0.08))
pick(blue_cube) ................... SUCCESS
place(blue_cube, zone_c) .......... SUCCESS
home() ............................ SUCCESS

TASK SUCCESS
  zones: zone_a=red_cube, zone_b=yellow_cube, zone_c=blue_cube
```

## 10. Lỗi thường gặp

| Hiện tượng | Cách xử lý |
|---|---|
| Node đứng ở `cho MoveIt 2 ...` | controller chưa lên, tắt hết tiến trình cũ (lệnh `pkill` ở mục 8) rồi chạy lại |
| `[LLM] loi ... 503` / `noi dung rong` | model đang quá tải, chương trình tự thử lại rồi chuyển sang `backup_model` |
| `GRASP_FAILED` | kẹp trượt, chạy lại lệnh (camera sẽ chụp lại vị trí mới) |
| `NO_FREE_SPACE` | bàn hết chỗ trống thỏa điều kiện, giảm `gap` trong `scene.yaml` |
