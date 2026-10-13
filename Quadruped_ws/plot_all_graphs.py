import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_quaternion

# =========================================================================
# 1. CẤU HÌNH TOPIC VÀ FILE BAG
# =========================================================================
bag_file = 'home_data.bag'

TOPIC_JOINT   = '/quadruped_gazebo/joint_states'
TOPIC_IMU     = '/quadruped_imu/base_link_orientation'
TOPIC_TASK_SPACE = '/task_space_goals'
TOPIC_PID     = '/pid_debug'

plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'axes.grid': True,
    'grid.linestyle': '--',
    'grid.alpha': 0.5
})

# =========================================================================
# 2. KHỞI TẠO BIẾN CHỨA DỮ LIỆU
# =========================================================================
data = {
    'time_joint': [], 'lf_j1': [], 'lf_j2': [], 'lf_j3': [],
    'time_foot_lf': [], 'lf_x': [], 'lf_y': [], 'lf_z': [],
    'time_pid':  [],
    'roll_err':  [], 'pitch_err': [],
    'u_roll':    [], 'u_pitch':   []
}

start_time = None

# =========================================================================
# 3. ĐỌC DỮ LIỆU TỪ ROS BAG
# =========================================================================
print(f"Đang đọc data từ {bag_file}...")

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if start_time is None:
            start_time = t.to_sec()
        current_t = t.to_sec() - start_time

        if topic == TOPIC_JOINT:
            data['time_joint'].append(current_t)
            data['lf_j1'].append(msg.position[0])
            data['lf_j2'].append(msg.position[1])
            data['lf_j3'].append(msg.position[2])

        elif topic == TOPIC_TASK_SPACE:
            data['time_foot_lf'].append(current_t)
            data['lf_x'].append(msg.FL_foot.x * 1000)
            data['lf_y'].append(msg.FL_foot.y * 1000)
            data['lf_z'].append(msg.FL_foot.z * 1000)

        elif topic == TOPIC_PID:
            data['time_pid'].append(current_t)
            if len(msg.data) >= 6:
                data['roll_err'].append(msg.data[2])  
                data['pitch_err'].append(msg.data[3]) 
                data['u_roll'].append(msg.data[4])    
                data['u_pitch'].append(msg.data[5])   
            elif len(msg.data) == 4:
                data['roll_err'].append(msg.data[0])
                data['pitch_err'].append(msg.data[1])
                data['u_roll'].append(msg.data[2])
                data['u_pitch'].append(msg.data[3])

print("Đọc xong! Đang vẽ đồ thị...")

# =========================================================================
# 4. HÀM HỖ TRỢ LÀM MƯỢT
# =========================================================================
def _ma(series, window=15):
    """Làm mượt mảng dữ liệu đầu vào. Window=15 sẽ bo tròn các đỉnh nhọn."""
    return pd.Series(series).rolling(window=window, min_periods=1).mean().to_numpy()

# =========================================================================
# 5. CÁC HÀM VẼ 4 ĐỒ THỊ CỐT LÕI
# =========================================================================

def draw_5_19():
    fig, ax = plt.subplots(figsize=(8, 4), dpi=150)
    if data['lf_j1']:
        ax.plot(data['time_joint'], data['lf_j1'], linewidth=1.2, label='Joint 1 – Hip Ab/Ad')
        ax.plot(data['time_joint'], data['lf_j2'], linewidth=1.2, label='Joint 2 – Hip Flex')
        ax.plot(data['time_joint'], data['lf_j3'], linewidth=1.2, label='Joint 3 – Knee')
    ax.set(title='Hình 5-19: Góc khớp 3 bậc tự do (IK)',
           xlabel='Time (s)', ylabel='Joint Angle (rad)')
    ax.legend(fontsize=9)
    fig.tight_layout()

def draw_5_20():
    fig = plt.figure(figsize=(7, 6), dpi=150)
    ax = fig.add_subplot(111, projection='3d')
    if data['lf_x']:
        ax.plot(data['lf_x'], data['lf_y'], data['lf_z'],
                color='royalblue', linewidth=2, label='LF Foot Trajectory')
        ax.scatter(data['lf_x'][0],  data['lf_y'][0],  data['lf_z'][0],
                   color='green', s=60, zorder=5, label='Start')
        ax.scatter(data['lf_x'][-1], data['lf_y'][-1], data['lf_z'][-1],
                   color='red',   s=60, zorder=5, label='End')
    ax.set(title='Hình 5-20: Quỹ đạo 3D bàn chân (IK)',
           xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
    ax.legend(fontsize=9)
    fig.tight_layout()

def draw_5_21():
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), dpi=150, sharex=True)
    if data['roll_err']:
        ax1.plot(data['time_pid'], _ma(data['roll_err'], 15), 'r-', linewidth=1.2, label='Roll error')
        ax2.plot(data['time_pid'], _ma(data['pitch_err'], 15), 'g-', linewidth=1.2, label='Pitch error')
    
    ax1.axhline(0, color='k', linewidth=0.8, linestyle='--')
    ax2.axhline(0, color='k', linewidth=0.8, linestyle='--')
    
    ax1.set(title='Hình 5-21: Sai lệch tư thế – e(t)', ylabel='Roll Error (rad)')
    ax2.set(xlabel='Time (s)', ylabel='Pitch Error (rad)')
    
    ax1.legend(fontsize=9)
    ax2.legend(fontsize=9)
    fig.tight_layout()

def draw_5_25():
    fig, ax1 = plt.subplots(figsize=(8, 4), dpi=150)
    if data['roll_err']:
        ax1.plot(data['time_pid'], _ma(data['roll_err'], 15),
                 'r-', linewidth=1.2, label='Error e(t)')
        ax1.set_ylabel('Error (rad)', color='r')
        ax1.tick_params(axis='y', labelcolor='r')
        
        ax2 = ax1.twinx()
        ax2.plot(data['time_pid'], _ma(data['u_roll'], 15),
                 color='royalblue', linewidth=1.5,
                 linestyle='--', label='Output u(t)')
        ax2.set_ylabel('Control Output (rad)', color='royalblue')
        ax2.tick_params(axis='y', labelcolor='royalblue')
        
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=9, loc='upper right')
        
    ax1.set(title='Hình 5-25: Roll – So sánh e(t) và u(t)', xlabel='Time (s)')
    fig.tight_layout()

# =========================================================================
# 6. GỌI HÀM VẼ
# =========================================================================
draw_5_19()
draw_5_20()
draw_5_21()
draw_5_25()

plt.show()