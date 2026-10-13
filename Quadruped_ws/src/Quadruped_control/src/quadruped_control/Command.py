import numpy as np


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0]) # Vận tốc [x, y] của body frame
        self.yaw_rate = 0.0 # Tốc độ quay quanh trục z
        self.height = -0.20 # Chiều cao của robot  
        self.pitch = 0.0 # Cúi/ngẩn người
        self.roll = 0.0 # Xoay ngang    
        self.joystick_control_active = 0 # Trạng thái điều khiển
        self.trotting_active = 0 # Trạng thái chạy (có đang đi Trot hay không)

        self.height_movement = 0 # Nâng/hạ thân
        self.roll_movement = 0 # Di chuyển ngang
        
        self.hop_event = False 
        self.trot_event = False
        self.joystick_control_event = False