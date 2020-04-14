import rospy
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header
import numpy as np
from sensor_msgs.point_cloud2 import create_cloud_xyz32
# https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/src/carla_ros_bridge/lidar.py

# ros-bridge/carla_ros_bridge/src/carla_ros_bridge/lidar.py ??????
# https://github.com/anshulpaigwar/Attentional-PointNet/blob/master/kitti_custom/pcl_helper.py
def publishing(data,pub):
    header=Header()
    header.stamp=rospy.Time.now()
    header.frame_id = "map"

    lidar_data = np.frombuffer(
        data.raw_data, dtype=np.float32)
    lidar_data = np.reshape(
        lidar_data, (int(lidar_data.shape[0] / 3), 3))
    lidar_data = -lidar_data
    lidar_data = lidar_data[..., [1, 0, 2]]
    point_cloud_msg = create_cloud_xyz32(header, lidar_data)
    pub.publish(point_cloud_msg)