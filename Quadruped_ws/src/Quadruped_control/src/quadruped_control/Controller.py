from quadruped_control.Gaits import GaitController
from quadruped_control.StanceController import StanceController
from quadruped_control.SwingLegController import SwingController
from quadruped_utilities.Utilities import clipped_first_order_filter
from quadruped_control.State import BehaviorState, State
from quadruped_control.msg import TaskSpace, JointSpace, Angle

import numpy as np
from transforms3d.euler import euler2mat, quat2euler
from transforms3d.quaternions import qconjugate, quat2axangle
from transforms3d.axangles import axangle2mat
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from math import degrees


class Controller:
    """Controller and planner object"""

    def __init__(self, config, inverse_kinematics):
        self.config = config

        # ── ROS Publishers ────────────────────────────────────────────────
        self.task_space_pub  = rospy.Publisher('task_space_goals',  TaskSpace,         queue_size=10)
        self.joint_space_pub = rospy.Publisher('joint_space_goals', JointSpace,        queue_size=10)
        self.pid_debug_pub   = rospy.Publisher('/pid_debug',        Float32MultiArray, queue_size=10)

        self.smoothed_yaw    = 0.0
        self.inverse_kinematics = inverse_kinematics

        self.contact_modes     = np.zeros(4)
        self.gait_controller   = GaitController(self.config)
        self.swing_controller  = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        # ── EMA Filter ───────────────────────────────────────────────────
        # alpha = 0.2 → trễ ~40ms @ 100Hz: cân bằng giữa mượt và phản ứng nhanh
        # Tăng alpha (→1.0) nếu cần phản ứng nhanh hơn (địa hình gồ ghề)
        # Giảm alpha (→0.0) nếu cần mượt hơn (địa hình phẳng)
        self.ema_alpha  = 0.05
        self.ema_roll   = 0.0
        self.ema_pitch  = 0.0

        # ── State machine mappings ────────────────────────────────────────
        self.hop_transition_mapping = {
            BehaviorState.REST:      BehaviorState.HOP,
            BehaviorState.HOP:       BehaviorState.FINISHHOP,
            BehaviorState.FINISHHOP: BehaviorState.REST,
            BehaviorState.TROT:      BehaviorState.HOP
        }
        self.trot_transition_mapping = {
            BehaviorState.REST:      BehaviorState.TROT,
            BehaviorState.TROT:      BehaviorState.REST,
            BehaviorState.HOP:       BehaviorState.TROT,
            BehaviorState.FINISHHOP: BehaviorState.TROT
        }
        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST,
            BehaviorState.REST:        BehaviorState.DEACTIVATED
        }

    # ─────────────────────────────────────────────────────────────────────
    def _filter_imu(self, roll_raw, pitch_raw):
        """
        EMA 1 tầng — đủ mượt cho chuyển động thật, không lag nhiều.

        Công thức:
            ema[k] = alpha * x[k] + (1 - alpha) * ema[k-1]

        Độ trễ xấp xỉ: tau ≈ (1/alpha - 1) * Ts
            alpha=0.2, Ts=0.01s → tau ≈ 40ms   ✅ cân bằng
            alpha=0.4, Ts=0.01s → tau ≈ 15ms   ⚡ nhanh hơn
            alpha=0.05,Ts=0.01s → tau ≈ 190ms  ❌ quá chậm
        """
        self.ema_roll  = self.ema_alpha * roll_raw  + (1.0 - self.ema_alpha) * self.ema_roll
        self.ema_pitch = self.ema_alpha * pitch_raw + (1.0 - self.ema_alpha) * self.ema_pitch
        return self.ema_roll, self.ema_pitch

    # ─────────────────────────────────────────────────────────────────────
    def step_gait(self, state, command):
        """Calculate the desired foot locations for the next timestep"""
        contact_modes      = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            contact_mode  = contact_modes[leg_index]
            foot_location = state.foot_locations[:, leg_index]

            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(
                    leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks)
                    / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command)

            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations, contact_modes

    # ─────────────────────────────────────────────────────────────────────
    def publish_task_space_command(self, rotated_foot_locations):
        msg = TaskSpace()
        msg.FR_foot = Point(
            rotated_foot_locations[0, 0] - self.config.LEG_ORIGINS[0, 0],
            rotated_foot_locations[1, 0] - self.config.LEG_ORIGINS[1, 0],
            rotated_foot_locations[2, 0] - self.config.LEG_ORIGINS[2, 0])
        msg.FL_foot = Point(
            rotated_foot_locations[0, 1] - self.config.LEG_ORIGINS[0, 1],
            rotated_foot_locations[1, 1] - self.config.LEG_ORIGINS[1, 1],
            rotated_foot_locations[2, 1] - self.config.LEG_ORIGINS[2, 1])
        msg.RR_foot = Point(
            rotated_foot_locations[0, 2] - self.config.LEG_ORIGINS[0, 2],
            rotated_foot_locations[1, 2] - self.config.LEG_ORIGINS[1, 2],
            rotated_foot_locations[2, 2] - self.config.LEG_ORIGINS[2, 2])
        msg.RL_foot = Point(
            rotated_foot_locations[0, 3] - self.config.LEG_ORIGINS[0, 3],
            rotated_foot_locations[1, 3] - self.config.LEG_ORIGINS[1, 3],
            rotated_foot_locations[2, 3] - self.config.LEG_ORIGINS[2, 3])
        msg.header = Header(stamp=rospy.Time.now())
        self.task_space_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────
    def publish_joint_space_command(self, angle_matrix):
        msg = JointSpace()
        msg.FR_foot = Angle(degrees(angle_matrix[0, 0]), degrees(angle_matrix[1, 0]), degrees(angle_matrix[2, 0]))
        msg.FL_foot = Angle(degrees(angle_matrix[0, 1]), degrees(angle_matrix[1, 1]), degrees(angle_matrix[2, 1]))
        msg.RR_foot = Angle(degrees(angle_matrix[0, 2]), degrees(angle_matrix[1, 2]), degrees(angle_matrix[2, 2]))
        msg.RL_foot = Angle(degrees(angle_matrix[0, 3]), degrees(angle_matrix[1, 3]), degrees(angle_matrix[2, 3]))
        msg.header = Header(stamp=rospy.Time.now())
        self.joint_space_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────
    def stabilise_with_IMU(self, foot_locations, orientation):
        """
        Bù trừ nghiêng thân bằng IMU.

        Pipeline:
            IMU raw → EMA(alpha=0.2) → clip(±0.4) → Kp*e → xoay chân

        Tham số:
            Kp       = 0.5   (correction_factor)
            max_tilt = 0.4   rad ≈ 23°
            alpha    = 0.2   (EMA — chỉnh trong __init__)
        """
        yaw, pitch_raw, roll_raw = orientation

        # Lọc EMA
        roll_f, pitch_f = self._filter_imu(roll_raw, pitch_raw)

        correction_factor = 0.5
        max_tilt          = 0.4

        roll_compensation  = correction_factor * np.clip(-roll_f,  -max_tilt, max_tilt)
        pitch_compensation = correction_factor * np.clip(-pitch_f, -max_tilt, max_tilt)

        rmat = euler2mat(roll_compensation, pitch_compensation, 0)
        return rmat.T @ foot_locations

    # ─────────────────────────────────────────────────────────────────────
    def _publish_pid_debug(self, state):
        """
        Publish dữ liệu ra /pid_debug để vẽ đồ thị luận văn.

        Layout msg.data (6 phần tử):
            [0] roll_raw      — góc Roll  thô từ IMU (rad)
            [1] pitch_raw     — góc Pitch thô từ IMU (rad)
            [2] roll_filtered — sau EMA filter (rad)
            [3] pitch_filtered— sau EMA filter (rad)
            [4] u_roll        — lượng bù Roll  = Kp * clip(-roll_f,  ±max_tilt)
            [5] u_pitch       — lượng bù Pitch = Kp * clip(-pitch_f, ±max_tilt)

        Ghi chú: _filter_imu() được gọi trong stabilise_with_IMU() trước đó
        nên self.ema_roll/ema_pitch đã được cập nhật — không gọi lại ở đây
        để tránh double-filter.
        """
        yaw, pitch_raw, roll_raw = state.euler_orientation

        Kp       = 0.5
        max_tilt = 0.4

        # Dùng giá trị EMA đã cập nhật từ stabilise_with_IMU()
        roll_f  = self.ema_roll
        pitch_f = self.ema_pitch

        u_roll  = Kp * float(np.clip(-roll_f,  -max_tilt, max_tilt))
        u_pitch = Kp * float(np.clip(-pitch_f, -max_tilt, max_tilt))

        pid_msg      = Float32MultiArray()
        pid_msg.data = [
            float(roll_raw),  float(pitch_raw),
            float(roll_f),    float(pitch_f),
            u_roll,           u_pitch
        ]
        self.pid_debug_pub.publish(pid_msg)

    # ─────────────────────────────────────────────────────────────────────
    def run(self, state, command):
        """Steps the controller forward one timestep"""

        previous_state = state.behavior_state

        # Cập nhật trạng thái từ joystick
        if command.joystick_control_event:
            state.behavior_state = self.activate_transition_mapping[state.behavior_state]
        elif command.trot_event:
            state.behavior_state = self.trot_transition_mapping[state.behavior_state]
        elif command.hop_event:
            state.behavior_state = self.hop_transition_mapping[state.behavior_state]

        if previous_state != state.behavior_state:
            rospy.loginfo("State changed from %s to %s",
                          str(previous_state), str(state.behavior_state))

        # ── TROT mode ─────────────────────────────────────────────────────
        if state.behavior_state == BehaviorState.TROT:
            state.foot_locations, contact_modes = self.step_gait(state, command)

            # Xoay theo lệnh người dùng
            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, 0.0)
                @ state.foot_locations
            )

            # Bù nghiêng thân bằng IMU (EMA + P controller)
            rotated_foot_locations = self.stabilise_with_IMU(
                rotated_foot_locations, state.euler_orientation)

            state.joint_angles           = self.inverse_kinematics(rotated_foot_locations, self.config)
            state.rotated_foot_locations = rotated_foot_locations

        # ── REST mode ─────────────────────────────────────────────────────
        elif state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )

            state.foot_locations = (
                self.config.default_stance
                + np.array([0, 0, command.height])[:, np.newaxis]
            )

            rotated_foot_locations = (
                euler2mat(command.roll, command.pitch, self.smoothed_yaw)
                @ state.foot_locations
            )

            # Bù nghiêng thân bằng IMU (EMA + P controller)
            rotated_foot_locations = self.stabilise_with_IMU(
                rotated_foot_locations, state.euler_orientation)

            state.joint_angles           = self.inverse_kinematics(rotated_foot_locations, self.config)
            state.rotated_foot_locations = rotated_foot_locations

        # ── Cập nhật state & publish ──────────────────────────────────────
        state.ticks  += 1
        state.pitch   = command.pitch
        state.roll    = command.roll
        state.height  = command.height

        # Publish dữ liệu ra /pid_debug để vẽ đồ thị luận văn
        self._publish_pid_debug(state)

    # ─────────────────────────────────────────────────────────────────────
    def set_pose_to_default(self, state):
        state.foot_locations = (
            self.config.default_stance
            + np.array([0, 0, self.config.default_z_ref])[:, np.newaxis]
        )
        print(state.foot_locations)
        state.joint_angles = self.inverse_kinematics(
            state.foot_locations, self.config)
        return state.joint_angles