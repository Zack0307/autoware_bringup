import rclpy
import cv2 as cv 
import os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pcl2
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header

DATA_PATH = 'src/autoware_bringup/image/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync'

class PublisherImageNode(Node):
    def __init__(self):
            super().__init__('publisher_image')
            self.frame = 0
            self.cv_bridge = CvBridge()
            self.cam_pub = self.create_publisher(Image, '/kitti_image', 10)
            self.PCL = self.create_publisher(PointCloud2, '/kitti_pcl', 10)
            self.timer = self.create_timer(0.1, self.publish_image)  # 每 0.1 秒呼叫一次

    def publish_image(self):
        data_number = len(os.listdir(os.path.join(DATA_PATH, 'image_02/data')))
        img = cv.imread(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%self.frame))
        pcl =np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%self.frame), dtype=np.float32).reshape(-1, 4)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        self.PCL.publish(pcl2.create_cloud_xyz32(header, pcl[:, :3]))
        self.get_logger().info(f'Publishing point cloud for frame')
        if img is not None:
            self.cam_pub.publish(self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8'))
            self.get_logger().info('Publishing image')
            self.frame += 1
            if self.frame % data_number == 0:
                self.frame = 0
            
        else:
             self.get_logger().warn('Image not found at frame')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherImageNode()
    rclpy.spin(node)                                
    rclpy.shutdown()

if __name__ == "__main__":
    main()




