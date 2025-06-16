import rclpy
import os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, Imu
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from autoware_bringup import publish_utils 


class PublisherImageNode(Node):
    def __init__(self):
            super().__init__('publisher_data_node')
            self.frame = 0
            self.cam_pub = self.create_publisher(Image, '/kitti_image', 10)
            self.PCL = self.create_publisher(PointCloud2, '/kitti_pcl', 10)
            self.marker = self.create_publisher(Marker, '/marker_gaze', 10)
            self.imu = self.create_publisher(Imu, '/kitti_imu', 10)
            self.timer = self.create_timer(0.1, self.publish_data)  # 每 0.1 秒呼叫一次

    def publish_data(self):
        publish_utils.publish_image(self.frame, self.cam_pub)
        publish_utils.publish_pcl(self.frame, self.PCL, self.get_clock())
        publish_utils.publish_marker_array(self.marker, self.get_clock())
        publish_utils.publish_imu(self.frame, self.imu, self.get_clock())
        self.frame += 1
        if self.frame >= publish_utils.data_number:
            self.frame = 0
        self.get_logger().info(f'Publishing')


def main(args=None):
    rclpy.init(args=args)
    node = PublisherImageNode()
    rclpy.spin(node)                                
    rclpy.shutdown()

if __name__ == "__main__":
    main()




