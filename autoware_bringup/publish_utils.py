import cv2 as cv
import os
import rclpy
import numpy as np
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pcl2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

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

def publish_marker_array(marker_pub, clock):
    marker = Marker()
    marker.header.frame_id = "map"  # 依你的座標框架而定
    marker.header.stamp = clock.now().to_msg()
    marker.ns = "basic_shapes"
    marker.id = 0
    marker.type = Marker.LINE_STRIP  
    marker.action = Marker.ADD
    marker.lifetime = Duration()  # 設定生命週期
    
    # 位置與方向
    # marker.pose.position.x = 1.0
    # marker.pose.position.y = 2.0
    # marker.pose.position.z = 0.5
    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    # marker.pose.orientation.w = 1.0

    # 顏色 (RGBA)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # 透明度
    # 尺寸
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5

    marker.points = []
    marker.points.append(Point(x=10.0, y=10.0, z=0.0))
    marker.points.append(Point(x=0.0, y=0.0, z=1.73))
    marker.points.append(Point(x=10.0, y=-10.0, z=0.0))

    marker_pub.publish(marker)




