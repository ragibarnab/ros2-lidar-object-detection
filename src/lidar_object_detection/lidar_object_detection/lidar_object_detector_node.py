import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import PointCloud2
import torch
from .model import PointPillars
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from .utils import bbox3d2corners
from object_detection_msgs.msg import Object3d, Object3dArray
from geometry_msgs.msg import Point

CLASSES = {
    'Pedestrian': 0, 
    'Cyclist': 1, 
    'Car': 2        
}


def point_range_filter(pts, point_range=[0, -39.68, -3, 69.12, 39.68, 1]):
    '''
    data_dict: dict(pts, gt_bboxes_3d, gt_labels, gt_names, difficulty)
    point_range: [x1, y1, z1, x2, y2, z2]
    '''
    flag_x_low = pts[:, 0] > point_range[0]
    flag_y_low = pts[:, 1] > point_range[1]
    flag_z_low = pts[:, 2] > point_range[2]
    flag_x_high = pts[:, 0] < point_range[3]
    flag_y_high = pts[:, 1] < point_range[4]
    flag_z_high = pts[:, 2] < point_range[5]
    keep_mask = flag_x_low & flag_y_low & flag_z_low & flag_x_high & flag_y_high & flag_z_high
    pts = pts[keep_mask]
    return pts 


class LidarObjectDetectorNode(Node):

    def __init__(self, weights, device=torch.device('cuda')):
        super().__init__('lidar_object_detector_node')

        self.lidar_subscription = self.create_subscription(
            msg_type = PointCloud2,
            topic = 'lidar',
            callback = self.lidar_point_cloud_callback,
            qos_profile = 10
        )

        self.detections_publisher = self.create_publisher(Object3dArray, 'object_detections_3d', 10)
        
        # setup model
        self.device = device
        self.model = PointPillars(nclasses=len(CLASSES)).to(self.device)
        self.model.load_state_dict(torch.load(weights))
        self.model.eval()        


    def lidar_point_cloud_callback(self, lidar_msg: PointCloud2):
        
        point_cloud_numpy = rnp.numpify(lidar_msg)
        point_cloud_numpy = point_cloud_numpy.view((np.float32, len(point_cloud_numpy.dtype.names)))    # to unstructured array
        point_cloud_numpy = point_range_filter(point_cloud_numpy)
        point_cloud_tensor = torch.from_numpy(point_cloud_numpy).to(self.device)

        # inference
        with torch.no_grad():
            results = self.model(batched_pts=[point_cloud_tensor])[0]

        
        bboxes = bbox3d2corners(results['lidar_bboxes'])
        labels = results['labels']
        confidence_scores = results['scores']

        if len(bboxes) == 0:
            self.get_logger().info("empty")
            return

        detection_array = Object3dArray()
        for bbox, label, confidence_score in zip(bboxes, labels, confidence_scores):
            detection = Object3d()
            detection.label = int(label)
            detection.confidence_score = float(confidence_score)
            for i in range(len(bbox)):
                corner = Point()
                corner.x = float(bbox[i][0])
                corner.y = float(bbox[i][1])
                corner.z = float(bbox[i][2])
                if corner.x == 0.0:
                    self.get_logger().info(f"{bbox}")
                detection.bounding_box.corners[i] = corner
            detection_array.objects.append(detection)

        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'map'
        self.detections_publisher.publish(detection_array)
        self.get_logger().info("Successfully ran inference on lidar scan")


def main(args=None):
    rclpy.init(args=args)

    package_name = 'lidar_object_detection'
    share_dir = os.path.dirname(get_package_share_directory(package_name))
    weights = os.path.join(share_dir, package_name, 'weights/epoch_160.pth')

    lidar_object_detector_node = LidarObjectDetectorNode(weights)

    rclpy.spin(lidar_object_detector_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_object_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()