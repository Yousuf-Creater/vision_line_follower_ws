import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from pyzbar import pyzbar
import cv2
import numpy as np
import time

class LineFollowerWithQR(Node):
    def __init__(self):
        super().__init__('line_follower_with_qr')
        self.bridge = CvBridge()
        self.last_qr_command = None
        self.qr_time = time.time()
        self.command_valid_duration = 10.0
        self.intersection_cooldown = 0
        self.turning_left = False
        self.turning_right = False
        self.stop_active = False
        self.permanent_stop = False
        self.turn_start_time = None
        self.turn_timer = None
        self.right_turn_timer = None
        self.stop_timer = None
        self.prev_error = 0.0
        self.kp = 0.004
        self.kd = 0.002

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        twist = Twist()
        if self.permanent_stop or self.turning_left or self.turning_right or self.stop_active:
            return
 
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        height, width, _ = frame.shape
        crop = frame[height - 100:height, 0:width]

        qr_codes = pyzbar.decode(frame)
        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8').strip().upper()
            self.get_logger().info(f'QR Code Detected: {qr_data}')
            self.last_qr_command = qr_data
            self.qr_time = time.time()

        if time.time() - self.qr_time > self.command_valid_duration:
            self.last_qr_command = None

        blur = cv2.GaussianBlur(crop, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                area = cv2.contourArea(largest_contour)

                if area > 20000 and self.intersection_cooldown == 0:
                    self.get_logger().info(f"Intersection Detected")
                    if self.last_qr_command == "TURN LEFT" and not self.turning_left:
                        self.get_logger().info("TURNING LEFT")
                        self.turning_left = True
                        self.turn_start_time = time.time()
                        self.turn_timer = self.create_timer(0.1, self.turn_left_timer_callback)

                    elif self.last_qr_command == "TURN RIGHT" and not self.turning_right:
                        self.get_logger().info("TURNING RIGHT")
                        self.turning_right = True
                        self.turn_start_time = time.time()
                        self.right_turn_timer = self.create_timer(0.1, self.turn_right_timer_callback)

                    elif self.last_qr_command == "STOP" and not self.stop_active:
                        self.get_logger().info("STOP")
                        self.stop_active = True
                        self.turn_start_time = time.time()
                        self.stop_timer = self.create_timer(0.1, self.stop_timer_callback)

                    else:
                        twist.linear.x = 0.15
                        twist.angular.z = 0.0

                    self.intersection_cooldown = 20
                    self.last_qr_command = None
                else:
                    error = cx - width // 2
                    derivative = error - self.prev_error
                    twist.linear.x = 0.5
                    twist.angular.z = -(self.kp * error + self.kd * derivative)
                    self.prev_error = error
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.3
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.4

        self.cmd_vel_pub.publish(twist)

        if self.intersection_cooldown > 0:
            self.intersection_cooldown -= 1

    def turn_left_timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.6
        self.cmd_vel_pub.publish(twist)

        if time.time() - self.turn_start_time >= 1.5:
            self.get_logger().info("TURNED LEFT")
            self.turning_left = False
            self.turn_timer.cancel()
            self.turn_timer = None

    def turn_right_timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -0.6
        self.cmd_vel_pub.publish(twist)

        if time.time() - self.turn_start_time >= 1.5:
            self.get_logger().info("TURNED RIGHT")
            self.turning_right = False
            self.right_turn_timer.cancel()
            self.right_turn_timer = None

    def stop_timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.permanent_stop = True
        self.cmd_vel_pub.publish(twist)

        if time.time() - self.turn_start_time >= 5.0:
            self.get_logger().info("Finished STOP")
            self.stop_active = False
            self.stop_timer.cancel()
            self.stop_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerWithQR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
