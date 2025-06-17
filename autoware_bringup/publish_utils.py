import cv2 as cv
import os
import numpy as np
import pandas as pd
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pcl2
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

Image_DATA_PATH = 'src/autoware_bringup/image/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync'
OXT_DATA_PATH = 'src/autoware_bringup/image/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync'
cv_bridge  = CvBridge()
data_number = len(os.listdir(os.path.join(Image_DATA_PATH, 'image_02/data')))
FRAME_ID = 'map'  # 依你的座標框架而定
IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 'ax', 'ay', 'az', 'af', 'al', 'au',
                    'wx', 'wy', 'wz', 'wf', 'wl', 'wu','posacc', 'velacc', 'navstat', 'numsats', 'posmode', 'velmode', 'orimode']

def publish_image(frame, cam_pub):
    img = cv.imread(os.path.join(Image_DATA_PATH, 'image_02/data/%010d.png'%frame))
    cam_pub.publish(cv_bridge.cv2_to_imgmsg(img, encoding='bgr8'))

def publish_pcl(frame, PCL, clock):
    pcl = np.fromfile(os.path.join(Image_DATA_PATH, 'velodyne_points/data/%010d.bin' % frame), dtype=np.float32).reshape(-1, 4)
    header = Header()
    header.stamp = clock.now().to_msg()
    header.frame_id = FRAME_ID
    PCL.publish(pcl2.create_cloud_xyz32(header, pcl[:, :3]))

def publish_marker_array(marker_pub, clock):
    marker = Marker()
    marker.header.frame_id = FRAME_ID  # 依你的座標框架而定
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

def publish_imu(frame, imu_pub, clock):
    imu = Imu()
    Imu_data = os.path.join(OXT_DATA_PATH, 'oxts/data/%010d.txt'%frame)
    df = pd.read_csv(Imu_data, sep=' ', header=None)
    df.columns = IMU_COLUMN_NAMES
    imu_row = df.iloc[0]
    imu.header.stamp = clock.now().to_msg()
    imu.header.frame_id = FRAME_ID  # 依你的座標框架而定

    q = quaternion_from_euler(float(imu_row['roll']), float(imu_row['pitch']), float(imu_row['yaw']))
    
    # IMU數據
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    
    imu.angular_velocity.x = float(imu_row['wf'])
    imu.angular_velocity.y = float(imu_row['wl'])
    imu.angular_velocity.z = float(imu_row['wu'])
    
    imu.linear_acceleration.x = float(imu_row['af'])
    imu.linear_acceleration.y = float(imu_row['al'])
    imu.linear_acceleration.z = float(imu_row['au'])
    
    imu_pub.publish(imu)
