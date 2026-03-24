# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 3rd Lab Session
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialise any flags that signal a colour has been detected (default to false)
        self.green_found = False
        self.blue_found = False
        self.move_forward = False
        self.move_backward = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        self.too_close_threshold = 20000 
        self.too_far_threshold = 15000   
        self.minimum_detection = 500

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        
        lower_green = np.array([60 - self.sensitivity, 100, 100])
        upper_green = np.array([60 + self.sensitivity, 255, 255])
        
        lower_blue = np.array([120 - self.sensitivity, 100, 100])
        upper_blue = np.array([120 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        green_mask = cv2.inRange(Hsv_image, lower_green, upper_green)
        blue_mask = cv2.inRange(Hsv_image, lower_blue, upper_blue)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter


        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours_b, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Loop over the contours
        if len(contours_b) > 0:
            c_blue = max(contours_b, key=cv2.contourArea)
            if cv2.contourArea(c_blue) > self.minimum_detection:
                self.blue_found = True
            else:
                self.blue_found = False
        else:
            self.blue_found = False

        contours_g, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        self.move_forward = False
        self.move_backward = False
        self.green_found = False

        if len(contours_g) > 0:
            c_green = max(contours_g, key=cv2.contourArea)
            area = cv2.contourArea(c_green)
            
            if area > self.minimum_detection:
                self.green_found = True
                if area > self.too_close_threshold:
                    self.move_backward = True
                elif area < self.too_far_threshold:
                    self.move_forward = True

        # Debugging View
        cv2.imshow('Robot View', image)
        cv2.waitKey(3)

    def walk_forward(self):
        #Use what you learnt in lab 3 to make the robot move forwards
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.1
        self.publisher.publish(desired_velocity)


    def walk_backward(self):
        # Use what you learnt in lab 3 to make the robot move backwards
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.1
        self.publisher.publish(desired_velocity)


    def stop(self):
        # Use what you learnt in lab 3 to make the robot stop
        desired_velocity = Twist()
        self.publisher.publish(desired_velocity)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        # Loop rate control (approx 10Hz)
        while rclpy.ok():
            if robot.blue_found:
                print("Blue detected! Emergency Stop.")
                robot.stop()
            elif robot.green_found:
                if robot.move_backward:
                    print("Too close to Green! Backing up...")
                    robot.walk_backward()
                elif robot.move_forward:
                    print("Green is far. Moving forward...")
                    robot.walk_forward()
                else:
                    print("Perfect distance. Staying put.")
                    robot.stop()
            else:
                print("Searching...")
                robot.stop()
            
            time.sleep(0.1)

    except (ROSInterruptException, KeyboardInterrupt):
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
