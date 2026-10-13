import numpy as np
# from quadruped_servo_interfacing.ServoCalibration import MICROS_PER_RAD, NEUTRAL_ANGLE_DEGREES
from quadruped_input_interfacing.HardwareConfig import PS4_COLOR, PS4_DEACTIVATED_COLOR
from enum import Enum
import math as m



class Configuration:
    def __init__(self):
        ################# CONTROLLER BASE COLOR ##############
        self.ps4_color = PS4_COLOR    
        self.ps4_deactivated_color = PS4_DEACTIVATED_COLOR    

        #################### COMMANDS ####################
        self.max_x_velocity = 1.2 # tốc độ tiến/lùi
        self.max_y_velocity = 0.5 # tốc độ sang trái/phải
        self.max_yaw_rate = 2.0 # tốc độ quay quanh trục z
        self.max_pitch = 30.0 * np.pi / 180.0 # góc cúi tối đa

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02 # thời gian hằng số cho chuyển động theo trục z
        self.z_speed = 0.06  # tốc độ thay đổi chiều cao thân tối đa
        self.pitch_deadband = 0.05 # vùng chết cho góc cúi
        self.pitch_time_constant = 0.25 # thời gian hằng số cho chuyển động cúi
        self.max_pitch_rate = 0.3 # tốc độ cúi tối đa
        self.roll_speed = 0.1  # tốc độ thay đổi góc nghiêng tối đa
        self.yaw_time_constant = 0.3 # thời gian hằng số cho chuyển động quay
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 1

        #################### STANCE ####################
        self.delta_x = 0.117 # tọa độ x của chân so với trọng tâm ở tư thế đứng

        #Dịch nhẹ các chân
        self.rear_leg_x_shift = -0.04 
        self.front_leg_x_shift = 0.00

        self.delta_y = 0.1106 #tọa độ y của chân so với trọng tâm ở tư thế đứng
        self.default_z_ref = -0.20 #tọa độ z của chân so với trọng tâm ở tư thế đứng

        #################### SWING ######################
        self.z_clearance = 0.07
        self.alpha = (
            0.5  
        )
        self.beta = (
            0.5  
        )

        #################### GAIT #######################
        self.dt = 0.01 #100Hz
        self.num_phases = 4
        #1: chạm đất, 0: không chạm đất
        #Thứ tự: [FR, FL, BR, BL]
        self.contact_phases = np.array(
            [[1, 1, 1, 0], 
             [1, 0, 1, 1], 
             [1, 0, 1, 1], 
             [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.04  # 4 chân chạm đất
        )
        self.swing_time = (
            0.07  # 2 chân chạm đất
        )

        ######################## GEOMETRY ######################
        self.LEG_FB = 0.11165 # Khoảng cách trước-sau 
        self.LEG_LR = 0.061  # khoảng cách trái-phải
        self.LEG_ORIGINS = np.array( 
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        #Các chân 
        self.L1 = 0.05162024721
        self.L2 = 0.130
        self.L3 = 0.13813664159
        self.phi = m.radians(73.91738698) # góc giữa L2 và L3
        
        ################### INERTIAL ####################
        self.MASS = 3.0 #kg
        # Mô-men quán tính của thân robot (kg*m^2)
        self.FRAME_INERTIA = tuple(
            map(lambda x: 3.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)

        leg_z = 1e-6
        leg_mass = 0.010
        leg_x = 1 / 12 * self.L2 ** 2 * leg_mass
        leg_y = leg_x
        self.LEG_INERTIA = (leg_x, leg_y, leg_z)

    @property
    def default_stance(self):
        return np.array(
            [
                [
                    self.delta_x + self.front_leg_x_shift, #Front Right
                    self.delta_x + self.front_leg_x_shift, #Front Left
                    -self.delta_x + self.rear_leg_x_shift, #Back Right
                    -self.delta_x + self.rear_leg_x_shift, #Back Left
                ],
                [-self.delta_y, 
                 self.delta_y, 
                 -self.delta_y, 
                 self.delta_y],
                [0, 0, 0, 0],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z


    ########################### GAIT ####################
    @property
    def overlap_ticks(self): # Số ticks của thời gian overlap
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self): # Số ticks của thời gian swing
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self): # Số ticks của thời gian pha stance
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self): # Tổng số ticks của một gait cycle
        return np.array(
            [self.overlap_ticks, 
             self.swing_ticks, 
             self.overlap_ticks, 
             self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks



# Cơ cấu các khớp nối
class Leg_linkage:
    def __init__(self,configuration):
        self.a = 35.12 #mm
        self.b = 37.6 #mm
        self.c = 43 #mm
        self.d = 35.23  #mm
        self.e = 67.1 #mm
        self.f = 130 #mm 
        self.g = 37 #mm
        self.h = 43 #mm
        self.upper_leg_length = configuration.L2*1000
        self.lower_leg_length = configuration.L3*1000
        self.lower_leg_bend_angle = m.radians(0) 
        self.i = self.upper_leg_length
        self.hip_width = configuration.L1 * 1000
        self.gamma = m.atan(28.80/20.20)
        self.EDC = m.acos((self.c**2+self.h**2-self.e**2)/(2*self.c*self.h))