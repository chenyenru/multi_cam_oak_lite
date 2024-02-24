from re import M
import cv2
import depthai as dai
import message_filters

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class MultiCamSubscriber(Node):
        def __init__(self, queue = 1, slop = 0.1):
            super().__init__("multi_cam_sub_node")
            # self.num_cams = len(dai.Device.getAllAvailableDevices())
            # if not self.num_cams:
                # exit("No device found, please connect the camera first")
            self.num_cams = 1
            self.sync_queue = queue 
            self.sync_slop = slop
            self.camera_subs = []
            self.bridge = CvBridge()
            for i in range(self.num_cams):
                self.camera_subs.append(message_filters.Subscriber(self, Image, "camera/image_" + str(i)))
            
            ts = message_filters.ApproximateTimeSynchronizer(self.camera_subs, self.sync_queue, self.sync_slop)
            ts.registerCallback(self.image_callback)
        
        def callback(self, *args):
            print("received callback: " + str(len(args)))
        
        def image_callback(self, msg):
            # Convert ROS Image message to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Display the image (or perform any other processing)
            cv2.imshow("Camera Image", img)
            cv2.waitKey(1)  # Adjust the waitKey value as needed


def main():
    rclpy.init()
 
    cam_sub = MultiCamSubscriber()

    rclpy.spin(cam_sub)

if __name__ == "__main__":
    main()