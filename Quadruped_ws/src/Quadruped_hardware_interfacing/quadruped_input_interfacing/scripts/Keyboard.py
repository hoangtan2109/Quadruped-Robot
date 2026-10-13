#!/usr/bin/env python3
import rospy
import sys, signal, os, select, termios, tty, threading
from sensor_msgs.msg import Joy

class Keyboard:
    def __init__(self):
        self.speed_multiplier = 1
        self.joystick_message_pub = rospy.Publisher("joy", Joy, queue_size=10)

        self.current_joy_message = Joy()
        self.current_joy_message.axes    = [0.,0.,0.,0.,0.,0.,0.,0.]
        self.current_joy_message.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        self.tty_fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
        self.old_settings = termios.tcgetattr(self.tty_fd)
        tty.setraw(self.tty_fd)

        self.thread = threading.Thread(target=self._read_keys, daemon=True)
        self.thread.start()

    def _read_keys(self):
        try:
            while True:
                r, _, _ = select.select([self.tty_fd], [], [], 0.1)
                if r:
                    key = os.read(self.tty_fd, 1).decode('utf-8', errors='ignore')
                    if key == '\x1b':
                        r2, _, _ = select.select([self.tty_fd], [], [], 0.05)
                        if r2:
                            key += os.read(self.tty_fd, 2).decode('utf-8', errors='ignore')
                    if key == '\x03':
                        os._exit(0)
                    self.on_press(key)
                else:
                    self.on_release_all()
        finally:
            termios.tcsetattr(self.tty_fd, termios.TCSADRAIN, self.old_settings)

    def on_press(self, key):
        msg = self.current_joy_message

        # Reset buttons trước mỗi lần nhấn — button chỉ = 1 đúng 1 tick
        msg.buttons[5] = 0
        msg.buttons[0] = 0
        msg.buttons[4] = 0

        if key == 'w' or key == 'W':
            msg.axes[1] = 0.3*self.speed_multiplier
        elif key == 's' or key == 'S':
            msg.axes[1] = -0.1*self.speed_multiplier
        elif key == 'a' or key == 'A':
            msg.axes[0] = 0.3*self.speed_multiplier
        elif key == 'd' or key == 'D':
            msg.axes[0] = -0.3*self.speed_multiplier
        elif key == '1':
            msg.buttons[5] = 1
        elif key == '2':
            msg.buttons[0] = 1
        elif key == '\x7f':
            msg.buttons[4] = 1
        elif key == '\x1b[A':
            msg.axes[4] = 0.3*self.speed_multiplier
        elif key == '\x1b[B':
            msg.axes[4] = -0.3*self.speed_multiplier
        elif key == '\x1b[D':
            msg.axes[3] = 0.3*self.speed_multiplier
        elif key == '\x1b[C':
            msg.axes[3] = -0.3*self.speed_multiplier
        elif key == '0':
            msg.axes[7] = 1
        elif key == '9':
            msg.axes[7] = -1
        elif key == '8':
            msg.axes[6] = 1
        elif key == '7':
            msg.axes[6] = -1

        self.current_joy_message = msg

    def on_release_all(self):
        msg = self.current_joy_message
        msg.axes[0] = 0.0
        msg.axes[1] = 0.0
        msg.axes[3] = 0.0
        msg.axes[4] = 0.0
        msg.buttons[5] = 0
        msg.buttons[0] = 0
        msg.buttons[4] = 0
        self.current_joy_message = msg

    def publish_current_command(self):
        self.current_joy_message.header.stamp = rospy.Time.now()
        self.joystick_message_pub.publish(self.current_joy_message)

def signal_handler(sig, frame):
    sys.exit(0)

def main():
    rospy.init_node("keyboard_input_listener")
    rate = rospy.Rate(30)

    print("=" * 45)
    print("  KEYBOARD CONTROLLER")
    print("  W/S/A/D     : Tiến / Lùi / Trái / Phải")
    print("  Mũi tên     : Pitch / Roll")
    print("  1           : Đứng dậy / Nằm xuống")
    print("  2           : Chức năng 2")
    print("  Backspace   : Chức năng Backspace")
    print("  7/8/9/0     : Điều chỉnh dáng")
    print("  Ctrl+C      : Thoát")
    print("=" * 45)

    signal.signal(signal.SIGINT, signal_handler)
    kbd = Keyboard()

    while not rospy.is_shutdown():
        kbd.publish_current_command()
        rate.sleep()

main()