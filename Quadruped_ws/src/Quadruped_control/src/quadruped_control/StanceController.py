import numpy as np
from transforms3d.euler import euler2mat

class StanceController:
    def __init__(self, config):
        self.config = config


    def position_delta(self, leg_index, state, command):
        # Trong 1 timestep dt, bàn chân cần thay đổi vị trí và góc bao nhiêu để robot di chuyển theo command
        z = state.foot_locations[2, leg_index] #tọa độ z của bàn chân hiện tại
        v_xy = np.array(
            [
                -command.horizontal_velocity[0], #vận tốc theo x
                -command.horizontal_velocity[1], #vận tốc theo y
                1.0/ self.config.z_time_constant* (state.height - z), # Bộ điều khiển P cho trục z, để bàn chân di chuyển về vị trí z_ref
            ]
        )
        delta_p = v_xy * self.config.dt #thay đổi vị trí
        delta_R = euler2mat(0, 0, -command.yaw_rate * self.config.dt) # thay đổi góc
        return (delta_p, delta_R)

    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command)
        incremented_location = delta_R @ foot_location + delta_p

        return incremented_location
