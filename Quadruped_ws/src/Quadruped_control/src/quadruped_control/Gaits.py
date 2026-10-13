#!/usr/bin/env python3
class GaitController:
    def __init__(self, config):
        self.config = config


    def phase_index(self, ticks): 
        # Robot đang ở pha nào trong gait cycle
        # Chân nào đang ở pha stance, chân nào đang ở pha swing
        # self.config.phase_ticks = [4, 7, 4, 7]
        # self.config.phase_length = 22
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False


    def subphase_ticks(self, ticks):
        # Robot đã đi được bao nhiêu ticks trong pha hiện tại
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        subphase_ticks = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + self.config.phase_ticks[i]
                return subphase_ticks
        assert False


    def contacts(self, ticks):
        # Trả về mảng 4 phần tử, mỗi phần tử là 0 hoặc 1
        # 0: chân đang ở pha swing, 1: chân đang ở pha stance
        return self.config.contact_phases[:, self.phase_index(ticks)]
