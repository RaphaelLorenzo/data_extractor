import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import cv2
from cv_bridge import CvBridge
import os
import datetime
import ros2_numpy as rnp
import numpy as np

class MainSubscriber(Node):

    def __init__(self):
        print("Intializing MainSubscriber")
        super().__init__('minimal_subscriber')
        
        # Camera info subscription
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Image subscription
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        # Create CV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Create directory for saving images if it doesn't exist
        self.image_save_dir = 'saved_images' 
        os.makedirs(self.image_save_dir, exist_ok=True) # do not do it here, because in the docker it will be protected
        self.image_counter = 0
        
        # Point cloud subscription
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.point_cloud_callback,
            10)
        
        self.point_cloud_save_dir = 'saved_point_clouds' 
        os.makedirs(self.point_cloud_save_dir, exist_ok=True) # do not do it here, because in the docker it will be protected
        self.point_cloud_counter = 0


    def camera_info_callback(self, msg: CameraInfo):
        self.get_logger().info(f"Got camera info | frame_id: {msg.header.frame_id} | timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

    def image_callback(self, msg: Image):
        self.get_logger().info(f"Got image | timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Generate a filename with timestamp
            timestamp = "ts_{}_{}".format(msg.header.stamp.sec, msg.header.stamp.nanosec)
            filename = os.path.join(self.image_save_dir, "{:06d}_{}.png".format(self.image_counter, timestamp))
            
            # Save the image
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image to {filename}')
            
            self.image_counter += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def point_cloud_callback(self, msg: PointCloud2):
        self.get_logger().info(f"Got point cloud | timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        
        pcl_array = rnp.numpify(msg) # dict
        
        # print("Got keys : ", pcl_array.keys())
        
        xyz_array = pcl_array['xyz'] # array of shape (N, 3) / float32
        intensity_array = pcl_array['intensity'] # array of shape (N, 1) / uint16
        intensity_array = intensity_array.astype(np.float32) / 1000.0
        
        # print("Got xyz : ", xyz_array.shape, xyz_array.dtype)
        # print("Got intensity : ", intensity_array.shape, intensity_array.dtype)
        
        full_array = np.concatenate([xyz_array, intensity_array], axis=1)
        # print("Got full array : ", full_array.shape, full_array.dtype)
        
        # Save the point cloud
        timestamp = "ts_{}_{}".format(msg.header.stamp.sec, msg.header.stamp.nanosec) # do not use, prefer counter
    
        filename = os.path.join(self.point_cloud_save_dir, "{:06d}_{}.bin".format(self.point_cloud_counter, timestamp))
        full_array.tofile(filename)
        self.get_logger().info(f'Saved point cloud to {filename}')
        
        self.point_cloud_counter += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MainSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()