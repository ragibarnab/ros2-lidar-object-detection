import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
import os
import glob
import numpy as np
from numpy.lib.recfunctions import unstructured_to_structured
from .utils.io import read_points



class KittiLidarPublisherNode(Node):

    def __init__(self, point_cloud_bin_files):
        super().__init__('lidar_publisher_node')

        self.counter = 0
        self.num_files = len(point_cloud_bin_files)
        self.point_cloud_bin_files = point_cloud_bin_files

        self.publisher_ = self.create_publisher(PointCloud2, 'lidar', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):

        point_cloud_bin_file = self.point_cloud_bin_files[self.counter]
        point_cloud_numpy = read_points(point_cloud_bin_file)

        dtypes = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('r', np.float32)])
        point_cloud_numpy = unstructured_to_structured(arr=point_cloud_numpy, dtype=dtypes)

        msg: PointCloud2 = rnp.msgify(PointCloud2, point_cloud_numpy)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published lidar scan: {os.path.basename(point_cloud_bin_file)}")

        self.counter += 1
        if self.counter == self.num_files:
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)

    package_name = 'lidar_object_detection'
    share_dir = os.path.dirname(get_package_share_directory(package_name))

    point_cloud_bin_files = sorted(glob.glob(os.path.join(share_dir, package_name) + '/data/*.bin'))
    lidar_publisher = KittiLidarPublisherNode(point_cloud_bin_files)

    rclpy.spin(lidar_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()