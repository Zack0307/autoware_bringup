import cv2 as cv
import os
import numpy as np
import pandas as pd
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pcl2
from sensor_msgs.msg import Imu, NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from autoware_bringup.kitti_utils import *


Image_DATA_PATH = '/home/zack/kitti/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync'
OXT_DATA_PATH = '/home/zack/kitti/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync'
cv_bridge  = CvBridge()
data_number = len(os.listdir(os.path.join(Image_DATA_PATH, 'image_02/data')))
FRAME_ID = 'map'  # 依你的座標框架而定
IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 'ax', 'ay', 'az', 'af', 'al', 'au',
                    'wx', 'wy', 'wz', 'wf', 'wl', 'wu','posacc', 'velacc', 'navstat', 'numsats', 'posmode', 'velmode', 'orimode']

TRACKING_COLUMN_NAMES = ['frame', 'track id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom', 
              'height', 'width', 'length', 'loc_x', 'loc_y', 'loc_z', 'rot_y' ]
    
DETECTION = {'Car':(0,255,200), 'Pedestrian':(0,200,35), 'Cyclist':(116,100,200)}
LIFETIME = 0.1 # Marker的生命週期，單位為秒

#LINE POINT
LINES = [[0,1],[1,2],[2,3],[3,0]]
LINES += [[4,5],[5,6],[6,7],[7,4]]
LINES += [[0,4],[1,5],[2,6],[3,7]] # 連接上面和下面的點
LINES += [[0,5],[1,4]]
#data
df = pd.read_csv('/home/zack/kitti/data_tracking_label_2/training/label_02/0000.txt', header=None, sep=' ')
df.columns = TRACKING_COLUMN_NAMES 
df.loc[df.type.isin(['Van','Truck','Tram']), 'type'] = 'Car'
df = df[df.type.isin(['Car','Pedestrian','Cyclist'])]

def publish_image(frame, cam_pub):
    img = cv.imread(os.path.join(Image_DATA_PATH, 'image_02/data/%010d.png'%frame))

    boxes = np.array(df[df.frame==frame][['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
    types = np.array(df[df.frame==frame]['type'])
 
    for typ, box in zip(types, boxes):
        top_left = int(box[0]), int(box[1])
        bottom_right = int(box[2]), int(box[3])
        cv.rectangle(img, top_left, bottom_right, DETECTION[typ], 2)
        cv.putText(img, typ, (int(box[0]), int(box[1]) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cam_pub.publish(cv_bridge.cv2_to_imgmsg(img, encoding='bgr8'))

def publish_pcl(frame, PCL, clock):
    pcl = np.fromfile(os.path.join(Image_DATA_PATH, 'velodyne_points/data/%010d.bin' % frame), dtype=np.float32).reshape(-1, 4)
    header = Header()
    header.stamp = clock.now().to_msg()
    header.frame_id = FRAME_ID
    PCL.publish(pcl2.create_cloud_xyz32(header, pcl[:, :3]))

def compute_3d_box(h, w, l, x, y, z, yaw):
        R = np.array([[np.cos(yaw),0,np.sin(yaw)],
                    [0,1,0],
                    [-np.sin(yaw),0,np.cos(yaw)]])
        x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]   #局部座標系->x:車頭到車尾(length)、y:上下、z:左右(width)
        y_corners = [0,0,0,0,-h,-h,-h,-h]
        z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
        corner_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners])) #世界座標系->z:length、x:width
        corner_3d += np.vstack([x, y, z]) 
        return corner_3d
            
def publish_3d_box(frame, box_pub, clock):
    bbox = MarkerArray()
    camera_pm = np.array(df[df.frame==frame][['height','width','length','loc_x','loc_y','loc_z','rot_y']])
    types = np.array(df[df.frame==frame]['type'])

    calib_list = []
    for box in camera_pm:
        cam_box = compute_3d_box(*box)
        cam_to_velodyne = Calibration('/home/zack/kitti/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync/calib', from_video = True)
        calib = cam_to_velodyne.project_rect_to_velo(cam_box.T)  #[8,3]
        calib_list.append(calib)

    for i, (calib, typ) in enumerate(zip(calib_list, types)):
        marker = Marker()
        marker.header.frame_id = FRAME_ID  # 依你的座標框架而定
        marker.header.stamp = clock.now().to_msg()
        marker.ns = "basic_shapes"
        marker.id = i
        marker.type = Marker.LINE_LIST  
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=int(LIFETIME), nanosec=int((LIFETIME % 1) * 1e9))  # 設定生命週期
        
        # 顏色 (RGBA)
        marker.color.r = DETECTION[typ][0]/255.0
        marker.color.g = DETECTION[typ][1]/255.0
        marker.color.b = DETECTION[typ][2]/255.0
        marker.color.a = 1.0  # 透明度q
        # 尺寸
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.points = []
        for line in LINES:
            pl = calib[line[0]]
            pr = calib[line[1]]
            marker.points.append(Point(x=pl[0], y=pl[1], z=pl[2]))
            marker.points.append(Point(x=pr[0], y=pr[1], z=pr[2]))
        bbox.markers.append(marker)
    box_pub.publish(bbox)

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

    # marker_array.markers.append(marker)

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

def publish_gps(frame, gps_pub, clock):
    gps = NavSatFix()
    Imu_data = os.path.join(OXT_DATA_PATH, 'oxts/data/%010d.txt'%frame)
    df = pd.read_csv(Imu_data, sep=' ', header=None)
    df.columns = IMU_COLUMN_NAMES
    gps_row = df.iloc[0]
    gps.header.stamp = clock.now().to_msg()
    gps.header.frame_id = FRAME_ID  # 依你的座標框架而定

    gps.latitude = float(gps_row['lat'])
    gps.longitude = float(gps_row['lon'])
    gps.altitude = float(gps_row['alt'])

    gps_pub.publish(gps)



