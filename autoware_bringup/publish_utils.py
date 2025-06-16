import cv2 as cv
import os
import rclpy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pcl2
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge



DATA_PATH = 'src/autoware_bringup/image/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync'
cv_bridge  = CvBridge()
data_number = len(os.listdir(os.path.join(DATA_PATH, 'image_02/data')))

def publish_image(frame, cam_pub):
    img = cv.imread(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
    cam_pub.publish(cv_bridge.cv2_to_imgmsg(img, encoding='bgr8'))
    return data_number

def publish_pcl(frame, PCL, clock):
    pcl = np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin' % frame), dtype=np.float32).reshape(-1, 4)
    header = Header()
    header.stamp = clock.now().to_msg()
    header.frame_id = 'map'
    PCL.publish(pcl2.create_cloud_xyz32(header, pcl[:, :3]))

def publish_marker_array(frame, marker_pub):
    marker_array = MarkerArray()
    marker = marker_array.markers[0] if marker_array.markers else None
    if marker is None:
        marker = marker_array.markers.append()
        marker.header.frame_id = 'map'
        marker.id = 0
        marker.type = 1  # CUBE
        marker.action = 0  # ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

    header = Header()
    header.stamp = rclpy.time.Time().now().to_msg()
    header.frame_id = 'map'
    marker.header = header

    marker_pub.publish(marker_array)



