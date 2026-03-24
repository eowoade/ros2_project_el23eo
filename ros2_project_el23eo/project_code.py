import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from math import sin, cos
from rclpy.exceptions import ROSInterruptException
from action_msgs.msg import GoalStatus

class Robot(Node):
    def __init__(self):
        super().__init__('robot',)
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.red_found = False
        self.green_found = False
        self.blue_found = False
        self.at_blue = False
        
        self.goal_finished = True 
        self.goal_accepted = False

        self.sensitivity = 10
        self.minimum_detection = 500
        self.stop_threshold = 320000
        
        self.going_blue = False
        self.last_blue_time = 0.0

        


        self.waypoints = [
            (-0.5, -6.4, 2.5),
            (6.8, -10.4, 1.5), 
            (-2.64, -12.1, 0.0)
        ]
        self.current_wp = 0
        self.goal_handle = None

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        
    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            return

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        self.red_found = False
        self.green_found = False
        self.blue_found = False

        lower_red = np.array([0 - self.sensitivity, 100, 100])
        upper_red = np.array([0 + self.sensitivity, 255, 255])
        
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        
        contour_r, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_r) > 0:
            c_red = max(contour_r, key=cv2.contourArea)
            if cv2.contourArea(c_red) > self.minimum_detection:
                self.red_found = True
                (x, y), radius = cv2.minEnclosingCircle(c_red)
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 0, 255),2)

        lower_green = np.array([60 - self.sensitivity, 100, 100])
        upper_green = np.array([60 + self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        
        contour_g, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_g) > 0:
            c_green = max(contour_g, key=cv2.contourArea)
            if cv2.contourArea(c_green) > self.minimum_detection:
                self.green_found = True
                (x, y), radius = cv2.minEnclosingCircle(c_green)
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0),2)


        lower_blue = np.array([120 - self.sensitivity, 100, 100])
        upper_blue = np.array([120 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
        contour_b, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_b) > 0:
            c_blue = max(contour_b, key=cv2.contourArea)
            area = cv2.contourArea(c_blue)
            if area > self.minimum_detection:
                self.blue_found = True
                self.blue_area = area
                M = cv2.moments(c_blue)
                if M['m00'] > 0:
                    self.blue_cx = int(M['m10'] / M['m00'])
                (x, y), radius = cv2.minEnclosingCircle(c_blue)
                cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 0),2)
                self.last_blue_time = time.time()

        display_img = cv2.resize(image, (640, 480)) 
        cv2.imshow('camera_Feed', display_img)
        cv2.waitKey(3)
        
    def send_goal(self, x, y, yaw):
        self.goal_finished = False 
        self.goal_accepted = False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        
        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_finished = True
            return
        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        self.goal_accepted = True
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint Reached.')
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Cancelling navigation, moving to blue')
        
        self.goal_handle = None
        self.goal_accepted = False
        self.goal_finished = True
        
    def cancel_nav(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_accepted = False

    def move_robot(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def stop(self):
        self.move_robot(0.0, 0.0)

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    robot = Robot()
    


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            recently_seen_blue = (time.time() - robot.last_blue_time) < 2.0
            
            if recently_seen_blue and not robot.at_blue:
                robot.going_blue = True
                
                if robot.goal_accepted:
                    robot.cancel_nav()
                
                if robot.blue_found:
                    if robot.blue_area > robot.stop_threshold:
                        robot.stop()
                        robot.at_blue = True
                        robot.going_blue = False
                        print("Around 1 Meter from Blue")
                    else:
                        centre_error = robot.blue_cx - (320)
                        
                        if abs(centre_error) < 5:
                            angular_vel = 0.0
                        else:
                            angular_vel = -float(centre_error) / 400.0
                            angular_vel = max(min(angular_vel, 0.3), -0.3)
                            
                        robot.move_robot(0.22, angular_vel)
                else:
                    robot.move_robot(0.0, 0.2)
            
            elif not robot.at_blue:
                robot.going_blue = False
                if robot.goal_finished and not robot.goal_accepted:
                    time.sleep(1.0)
                    wp = robot.waypoints[robot.current_wp]
                    print(f"New Waypoint is: {robot.current_wp}")
                    robot.send_goal(wp[0], wp[1], wp[2])
                    time.sleep(1.0) 
            
            time.sleep(0.1)
    except (ROSInterruptException, KeyboardInterrupt):
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()