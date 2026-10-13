#!/usr/bin/env python3
import rospy
import actionlib
import csv
import os
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolController:
    def __init__(self):
        rospy.init_node('patrol_controller')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("Đang chờ kết nối với move_base...")
        self.client.wait_for_server()

        self.csv_file = os.path.expanduser('~/Quadruped_II/Quadruped_ws/src/Quadruped_gazebo/saved_points/my_room_patrol.csv')
        
        # Biến mục tiêu và trạng thái
        self.goal_reached = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        
        self.run_patrol()

    # Hàm dịch Quaternion sang góc Yaw (để check hướng mặt chó)
    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def feedback_cb(self, feedback):
        curr_p = feedback.base_position.pose.position
        curr_q = feedback.base_position.pose.orientation
        
        # 1. Tính khoảng cách X, Y
        dist = math.sqrt((curr_p.x - self.target_x)**2 + (curr_p.y - self.target_y)**2)
        
        # 2. Tính độ lệch góc xoay (Yaw)
        curr_yaw = self.get_yaw_from_quat(curr_q)
        yaw_diff = abs(self.target_yaw - curr_yaw)
        # Chuẩn hóa góc lệch về khoảng [0, pi]
        if yaw_diff > math.pi: yaw_diff = abs(yaw_diff - 2*math.pi)

        # ĐIỀU KIỆN CHỐT: Cách đích < 30cm VÀ mặt lệch < 20 độ (0.35 rad)
        if dist < 0.30 and yaw_diff < 0.35 and not self.goal_reached:
            print(f"-> Đạt mục tiêu! Khoảng cách: {dist:.2f}m, Lệch góc: {math.degrees(yaw_diff):.1f}°")
            self.goal_reached = True
            self.client.cancel_goal() # Ép dừng khi đã đủ gần và đúng hướng

    def run_patrol(self):
        try:
            with open(self.csv_file, 'r') as file:
                csv_reader = csv.reader(file)
                next(csv_reader) 

                for row in csv_reader:
                    if len(row) < 8: continue
                    point_id = row[0]
                    self.target_x, self.target_y = float(row[1]), float(row[2])
                    z = float(row[3])
                    # Lưu Quaternion mục tiêu để tính Yaw
                    from geometry_msgs.msg import Quaternion
                    q_target = Quaternion(float(row[4]), float(row[5]), float(row[6]), float(row[7]))
                    self.target_yaw = self.get_yaw_from_quat(q_target)

                    print(f"\n[DI CHUYỂN] Điểm {point_id}...")
                    
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = self.target_x
                    goal.target_pose.pose.position.y = self.target_y
                    goal.target_pose.pose.orientation = q_target

                    self.goal_reached = False
                    self.client.send_goal(goal, feedback_cb=self.feedback_cb)
                    
                    # Chờ tối đa 60s cho mỗi điểm
                    self.client.wait_for_result(rospy.Duration(60.0))
                    
                    # SAU KHI DỪNG: Bất kể do Move_base báo hay do mình tự Cancel
                    if self.goal_reached or self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        print(f"==> Đã đậu đúng thế tại ID {point_id}!")
                    else:
                        print(f"==> Bỏ qua ID {point_id} (Timeout/Lỗi)")

                    # ĐÂY RỒI: Lệnh nghỉ 3 giây nằm ngoài mọi điều kiện để chắc chắn nó dừng
                    print("... Đang nghỉ ngơi 3 giây ...")
                    rospy.sleep(3.0) 
                    
            print("\n🎉 HOÀN THÀNH TUẦN TRA!")
        except Exception as e:
            print(f"Lỗi rồi kĩ sư: {e}")

if __name__ == '__main__':
    try: PatrolController()
    except rospy.ROSInterruptException: pass
