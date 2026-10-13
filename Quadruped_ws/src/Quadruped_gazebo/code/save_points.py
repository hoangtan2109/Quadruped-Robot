#!/usr/bin/env python3
import rospy
import csv
import os
from geometry_msgs.msg import PoseStamped

class PointRecorder:
    def __init__(self):
        rospy.init_node('point_recorder')
        
# Đã sửa: Trỏ thẳng về nhà mới của con Quadruped
        self.save_dir = os.path.expanduser('~/Quadruped_II/Quadruped_ws/src/Quadruped_gazebo/saved_points')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # Tạo file CSV của riêng bạn
        self.csv_file = os.path.join(self.save_dir, 'my_room_patrol.csv')
        self.point_id = 0
        
        # Mở file và viết dòng Tiêu đề (Header)
        with open(self.csv_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['ID', 'X', 'Y', 'Z', 'Qx', 'Qy', 'Qz', 'Qw'])
            
        print(f"Đã tạo file lưu tọa độ mới tại: {self.csv_file}")
        print("\n>>> HƯỚNG DẪN: Lên RViz và dùng nút '2D Nav Goal' cắm cờ để lưu điểm. <<<")
        
        # Lắng nghe mỗi khi bạn cắm cờ trên RViz
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        
    def goal_cb(self, msg):
        # Lấy tọa độ từ cú click chuột của bạn
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Ghi thêm vào file CSV
        with open(self.csv_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.point_id, x, y, z, qx, qy, qz, qw])
            
        print(f"[LƯU THÀNH CÔNG] Điểm ID: {self.point_id} | Tọa độ X: {x:.2f}, Y: {y:.2f}")
        self.point_id += 1

if __name__ == '__main__':
    try:
        PointRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
