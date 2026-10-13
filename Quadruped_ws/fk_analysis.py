import rosbag
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# ============================================================
# CẤU HÌNH - chỉnh thông số chân robot ở đây
# ============================================================
# Đo từ URDF hoặc bản vẽ cơ khí (đơn vị: mét)
L1 = 0.05162024721   # chiều dài hip link (theta1)
L2 = 0.1300    # chiều dài thigh link (theta2)
L3 = 0.13813664159    # chiều dài calf link (theta3)

BAG_FILE = "dog_experiment.bag"
TOPIC    = "/quadruped_gazebo/joint_states"

# Thứ tự khớp trong message
LEGS = {
    "FR": [0, 1, 2],
    "FL": [3, 4, 5],
    "RR": [6, 7, 8],
    "RL": [9, 10, 11],
}

# ============================================================
# 1. ĐỌC ROSBAG
# ============================================================
print("Đang đọc bag file...")
timestamps = []
positions  = []   # shape: (N, 12)

with rosbag.Bag(BAG_FILE) as bag:
    for _, msg, t in bag.read_messages(topics=[TOPIC]):
        timestamps.append(t.to_sec())
        positions.append(list(msg.position))

timestamps = np.array(timestamps)
timestamps -= timestamps[0]          # bắt đầu từ 0
positions  = np.array(positions)     # (N, 12)
print(f"Đọc xong: {len(timestamps)} mẫu, duration = {timestamps[-1]:.2f}s")

# ============================================================
# 2. TÍNH FORWARD KINEMATICS (FK) - 3DOF mỗi chân
# ============================================================
# Mô hình: hip (lateral) → thigh (sagittal) → calf (sagittal)
# Trả về vị trí bàn chân [x, y, z] trong frame gốc của chân

def fk_leg(t1, t2, t3, L1, L2, L3):
    """
    t1: góc hip  (xoay lateral - trục Z)
    t2: góc thigh(xoay sagittal - trục Y)
    t3: góc calf (xoay sagittal - trục Y)
    Trả về: [x, y, z] vị trí bàn chân
    """
    # Hip: dịch chuyển theo trục Y (lateral)
    x_hip = 0
    y_hip = L1 * np.cos(t1)
    z_hip = -L1 * np.sin(t1)

    # Thigh: từ hip tính tiếp
    x_thigh = L2 * np.sin(t2)
    y_thigh = 0
    z_thigh = -L2 * np.cos(t2)

    # Calf: từ thigh tính tiếp
    x_calf = L3 * np.sin(t2 + t3)
    y_calf = 0
    z_calf = -L3 * np.cos(t2 + t3)

    x = x_hip + x_thigh + x_calf
    y = y_hip + y_thigh + y_calf
    z = z_hip + z_thigh + z_calf

    return np.array([x, y, z])

# Tính FK cho cả 4 chân
foot_pos = {}   # {leg_name: array (N,3)}
for leg, idx in LEGS.items():
    pos = []
    for i in range(len(timestamps)):
        t1 = positions[i, idx[0]]
        t2 = positions[i, idx[1]]
        t3 = positions[i, idx[2]]
        pos.append(fk_leg(t1, t2, t3, L1, L2, L3))
    foot_pos[leg] = np.array(pos)
    print(f"{leg} foot - X: [{foot_pos[leg][:,0].min():.3f}, {foot_pos[leg][:,0].max():.3f}] "
          f"Z: [{foot_pos[leg][:,2].min():.3f}, {foot_pos[leg][:,2].max():.3f}]")

# ============================================================
# 3. TÍNH INVERSE KINEMATICS (IK) - kiểm tra tại 1 điểm
# ============================================================
def ik_leg(x, y, z, L1, L2, L3):
    """
    Cho vị trí bàn chân [x, y, z], tính ngược ra [t1, t2, t3]
    """
    # t1: góc hip từ y, z
    t1 = np.arctan2(-z, y)

    # Chiều dài từ hip joint đến foot (trong mặt phẳng sagittal)
    y_prime = np.sqrt(y**2 + z**2) - L1
    r = np.sqrt(x**2 + y_prime**2)

    # Kiểm tra workspace
    if r > L2 + L3:
        return None, "Ngoài workspace!"

    # t3: góc calf (law of cosines)
    cos_t3 = (r**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_t3 = np.clip(cos_t3, -1, 1)
    t3 = -np.arccos(cos_t3)   # âm = gập vào trong

    # t2: góc thigh
    alpha = np.arctan2(x, y_prime)
    beta  = np.arctan2(L3 * np.sin(-t3), L2 + L3 * np.cos(-t3))
    t2 = alpha - beta

    return np.array([t1, t2, t3]), "OK"

# Test IK tại vị trí trung bình của chân FL
fl_mean = foot_pos["FL"].mean(axis=0)
print(f"\n--- Kiểm tra IK cho chân FL ---")
print(f"Vị trí trung bình bàn chân FL: x={fl_mean[0]:.4f}, y={fl_mean[1]:.4f}, z={fl_mean[2]:.4f}")

q_ik, status = ik_leg(fl_mean[0], fl_mean[1], fl_mean[2], L1, L2, L3)
if q_ik is not None:
    print(f"IK kết quả: theta1={np.degrees(q_ik[0]):.2f}°, "
          f"theta2={np.degrees(q_ik[1]):.2f}°, theta3={np.degrees(q_ik[2]):.2f}°")
    # Verify bằng FK
    check = fk_leg(q_ik[0], q_ik[1], q_ik[2], L1, L2, L3)
    err = np.linalg.norm(check - fl_mean)
    print(f"FK verify: x={check[0]:.4f}, y={check[1]:.4f}, z={check[2]:.4f}")
    print(f"Sai số IK→FK: {err*1000:.4f} mm")
else:
    print("IK thất bại:", status)

# ============================================================
# 4. TÍNH VẬN TỐC KHỚP
# ============================================================
dt = np.diff(timestamps)
vel = {}
for leg, idx in LEGS.items():
    vel[leg] = np.diff(positions[:, idx], axis=0) / dt[:, None]

# ============================================================
# 5. LƯU CSV
# ============================================================
rows = []
for i in range(len(timestamps)):
    row = {"time": timestamps[i]}
    for leg, idx in LEGS.items():
        row[f"{leg}_t1"] = np.degrees(positions[i, idx[0]])
        row[f"{leg}_t2"] = np.degrees(positions[i, idx[1]])
        row[f"{leg}_t3"] = np.degrees(positions[i, idx[2]])
        row[f"{leg}_x"]  = foot_pos[leg][i, 0]
        row[f"{leg}_y"]  = foot_pos[leg][i, 1]
        row[f"{leg}_z"]  = foot_pos[leg][i, 2]
    rows.append(row)

df = pd.DataFrame(rows)
df.to_csv("kinematic_results.csv", index=False)
print(f"\nĐã lưu kinematic_results.csv ({len(df)} dòng)")

# ============================================================
# 6. PLOT
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 9))
fig.suptitle("Phân tích động học - Quadruped Robot 12 DOF", fontsize=14, fontweight='bold')

colors = {"FR": "blue", "FL": "orange", "RR": "green", "RL": "red"}

# Plot 1: Quỹ đạo bàn chân X-Z (sagittal)
for leg in LEGS:
    axes[0, 0].plot(foot_pos[leg][:, 0], foot_pos[leg][:, 2],
                    color=colors[leg], label=leg, linewidth=1.5)
axes[0, 0].set_xlabel("X (m)"); axes[0, 0].set_ylabel("Z (m)")
axes[0, 0].set_title("Quỹ đạo bàn chân (mặt phẳng X-Z)")
axes[0, 0].legend(); axes[0, 0].grid(True)

# Plot 2: Z theo thời gian (nhấc chân)
for leg in LEGS:
    axes[0, 1].plot(timestamps, foot_pos[leg][:, 2],
                    color=colors[leg], label=leg, linewidth=1)
axes[0, 1].set_xlabel("Thời gian (s)"); axes[0, 1].set_ylabel("Z (m)")
axes[0, 1].set_title("Chiều cao bàn chân theo thời gian")
axes[0, 1].legend(); axes[0, 1].grid(True)

# Plot 3: Góc khớp FL theo thời gian
leg_names = ["theta1 (hip)", "theta2 (thigh)", "theta3 (calf)"]
c3 = ["red", "green", "blue"]
for i, name in enumerate(leg_names):
    axes[1, 0].plot(timestamps, np.degrees(positions[:, LEGS["FL"][i]]),
                    color=c3[i], label=name, linewidth=1)
axes[1, 0].set_xlabel("Thời gian (s)"); axes[1, 0].set_ylabel("Góc (độ)")
axes[1, 0].set_title("Góc khớp chân FL theo thời gian")
axes[1, 0].legend(); axes[1, 0].grid(True)

# Plot 4: Vận tốc khớp FL
for i, name in enumerate(leg_names):
    axes[1, 1].plot(timestamps[1:], np.degrees(vel["FL"][:, i]),
                    color=c3[i], label=name, linewidth=1)
axes[1, 1].set_xlabel("Thời gian (s)"); axes[1, 1].set_ylabel("Vận tốc (°/s)")
axes[1, 1].set_title("Vận tốc khớp chân FL theo thời gian")
axes[1, 1].legend(); axes[1, 1].grid(True)

plt.tight_layout()
plt.savefig("kinematic_analysis.png", dpi=150, bbox_inches='tight')
plt.show()
print("Đã lưu kinematic_analysis.png")
