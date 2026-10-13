#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class AutoDriver:
    def __init__(self):
        rospy.init_node('auto_driver')
        self.pub = rospy.Publisher('/joy', Joy, queue_size=10)
        
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 12
        
        # Biến quản lý trạng thái
        self.state = 'INIT'
        self.cmd_active = False
        self.last_cmd_time = rospy.Time.now()
        
        self.transition_counter = 0
        self.target_button = -1
        
        print("Đang đánh thức chó robot (Bơm lệnh 30Hz)...")
        rospy.sleep(1.0)
        rate = rospy.Rate(30)
        
        # 1. Khởi động: Bấm Phím 2 (Stand)
        print("-> Khởi động: Chó gồng chân lên (STAND)")
        for _ in range(15):
            self.joy_msg.buttons[0] = 1
            self.publish_joy()
            rate.sleep()
        self.joy_msg.buttons[0] = 0
        self.state = 'STAND'
        
        print("HỆ THỐNG SẴN SÀNG! Đang đứng im chờ cờ 2D Nav Goal hoặc lệnh từ patrol.py...")
        
        # Lắng nghe não TEB
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        
        # Vòng lặp liên tục bơm lệnh
        while not rospy.is_shutdown():
            if self.transition_counter > 0:
                # Đang trong quá trình giữ nút chuyển trạng thái (mô phỏng bấm phím)
                self.joy_msg.buttons[self.target_button] = 1
                self.transition_counter -= 1
            else:
                self.joy_msg.buttons = [0] * 12 # Nhả hết nút
                
                # LOGIC CỦA MÁY TRẠNG THÁI (STATE MACHINE):
                
                # TH1: Đang đứng im mà có lệnh chạy -> Chuyển sang Trot
                if self.cmd_active and self.state == 'STAND':
                    print("-> Nhận lệnh di chuyển! Tự động chuyển sang TROT (Phím 1)")
                    self.target_button = 5  # Vị trí nút Trot
                    self.transition_counter = 10 
                    self.state = 'TROT'
                
                # TH2: Đang Trot mà AI dừng gửi lệnh (sau 2 giây) -> Trở về Stand
                elif not self.cmd_active and self.state == 'TROT':
                    if (rospy.Time.now() - self.last_cmd_time).to_sec() > 2.0:
                        print("-> Đã đến đích! Trở về STAND (Phím 2) để nghỉ ngơi")
                        self.target_button = 0  # Vị trí nút Stand
                        self.transition_counter = 10
                        self.state = 'STAND'
            
            self.publish_joy()
            rate.sleep()
            
    def cmd_cb(self, msg):
        self.joy_msg.axes[1] = msg.linear.x * 1.5 
        self.joy_msg.axes[0] = msg.linear.y * 1.5 
        self.joy_msg.axes[3] = msg.angular.z 
        
        # Đo xem vận tốc có lớn hơn 0 không (có đang bắt đi không)
        if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
            self.cmd_active = True
            self.last_cmd_time = rospy.Time.now() # Cập nhật thời điểm cuối cùng có lệnh
        else:
            self.cmd_active = False

    def publish_joy(self):
        self.joy_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.joy_msg)

if __name__ == '__main__':
    try:
        AutoDriver()
    except rospy.ROSInterruptException:
        pass
