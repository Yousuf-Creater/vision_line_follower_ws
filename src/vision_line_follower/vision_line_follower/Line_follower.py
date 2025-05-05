import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your camera topic if different
            self.image_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Line Follower Node Initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        height, width, _ = frame.shape

        # Crop the lower part of the image (region of interest)
        crop_height = 100
        crop = frame[height - crop_height:height, 0:width]

        # Convert to HSV and mask black color
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Find contours of the line
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        if contours:
            # Largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])  # Contour center x
                cy = int(M['m01'] / M['m00'])  # Contour center y

                # Draw center
                cv2.circle(crop, (cx, cy), 5, (255, 0, 0), -1)

                # Proportional control
                error = cx - width // 2
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 200.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.2  # Rotate to search
        else:
            # No line found
            twist.linear.x = 0.0
            twist.angular.z = 0.3

        # Publish velocity
        self.cmd_vel_pub.publish(twist)

        # Optional: show debug window
        # cv2.imshow("Line View", crop)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
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
