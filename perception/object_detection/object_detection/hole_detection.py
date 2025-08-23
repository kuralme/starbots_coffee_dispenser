import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import pcl
import math
import numpy as np
import tf2_ros
from tf2_ros import TransformException, ConnectivityException
from custom_msgs.msg import DetectedSurfaces, DetectedHoles
from typing import List, Tuple, Union


class HoleDetection(Node):
    def __init__(self) -> None:
        super().__init__('hole_detection_node')
        self.pc_sub = self.create_subscription(PointCloud2, '/wrist_rgbd_depth_sensor/points', self.callback, 10)
        self.tray_marker_pub = self.create_publisher(MarkerArray, '/table_markers', 10)
        self.hole_marker_pub = self.create_publisher(MarkerArray, '/hole_markers', 10)
        self.surface_detected_pub = self.create_publisher(DetectedSurfaces, '/surface_detected', 10)
        self.hole_detected_pub = self.create_publisher(DetectedHoles, '/hole_detected', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, msg: PointCloud2) -> None:
        try:
            cloud = self.from_ros_msg(msg)
            # Filtered cloud for tray surface detection
            filtered_cloud_plane = self.filter_cloud(cloud, min_x=-0.6, max_x=-0.2, min_y=-0.2, max_y=0.2, min_z=-0.65, max_z=-0.54)
            # Filtered cloud for hole detection
            filtered_cloud_holes = self.filter_cloud(cloud, min_x=-0.55, max_x=-0.25, min_y=-0.15, max_y=0.15, min_z=-0.63, max_z=-0.55)

            # Plane segmentations using RANSAC: tray and holes
            plane_indices, plane_coefficients, tray_cloud = self.extract_plane(filtered_cloud_plane)
            hole_indices, hole_coefficients, hole_cloud = self.extract_cylinder(filtered_cloud_holes)

            # Clustering methods
            table_clusters, surface_centroids, surface_dimensions = self.extract_clusters(tray_cloud, "Tray Surface")
            hole_clusters, hole_centroids, hole_dimensions = self.extract_clusters(hole_cloud, "Hole cloud")

            # Publish detected markers
            self.pub_surface_marker(surface_centroids, surface_dimensions)
            self.pub_hole_markers(hole_centroids, hole_dimensions)

            # Publish detected info
            self.pub_surface_detected(surface_centroids, surface_dimensions)
            self.pub_hole_detected(hole_centroids, hole_dimensions)

        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

    def from_ros_msg(self, msg: PointCloud2) -> Union[pcl.PointCloud, None]:
        """Converts a ROS PointCloud2 message to a PCL point cloud"""
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                        msg.header.frame_id,
                                                        rclpy.time.Time(),
                                                        timeout=rclpy.time.Duration(seconds=1.0))
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation_quaternion = np.array([transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w])

            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(rotation_quaternion)

            # Convert PointCloud2 msg to numpy array
            point_step = msg.point_step
            num_points = len(msg.data) // point_step
            points = []
            for i in range(num_points):
                start_index = i * point_step
                x_bytes = msg.data[start_index:start_index + 4]
                y_bytes = msg.data[start_index + 4:start_index + 8]
                z_bytes = msg.data[start_index + 8:start_index + 12]
                x = np.frombuffer(x_bytes, dtype=np.float32)[0]
                y = np.frombuffer(y_bytes, dtype=np.float32)[0]
                z = np.frombuffer(z_bytes, dtype=np.float32)[0]
                point = np.array([x, y, z])

                # Apply the rotation to the point
                rotated_point = np.dot(rotation_matrix, point)

                # Apply the translation to the rotated point to get its position relative to the base_link frame
                relative_point = rotated_point + translation

                points.append(relative_point)

            data = np.array(points, dtype=np.float32)
            assert data.shape[1] == 3, "Number of fields must be 3"
            cloud = pcl.PointCloud()
            cloud.from_array(data)
            return cloud

        except (TransformException, ConnectivityException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in from_ros_msg: {e}")
            return None

    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Converts a quaternion to a rotation matrix"""
        x, y, z, w = q
        rotation_matrix = np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                                    [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                                    [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])
        return rotation_matrix

    def filter_cloud(self, cloud, min_x, max_x, min_y, max_y, min_z, max_z):
        indices = []
        for i in range(cloud.size):
            pt = cloud[i]
            if min_x <= pt[0] <= max_x and min_y <= pt[1] <= max_y and min_z <= pt[2] <= max_z:
                indices.append(i)
        return cloud.extract(indices)

    def extract_plane(self, cloud: pcl.PointCloud) -> Tuple[np.ndarray, np.ndarray, pcl.PointCloud]:
        """Segmentation: Extracts a plane from the point cloud."""
        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)
        indices, coefficients = seg.segment()

        # Extract points belonging to the plane
        plane_cloud = cloud.extract(indices)

        return indices, coefficients, plane_cloud

    def extract_cylinder(self, cloud, max_holes=4, min_distance=0.05):
        """Segmentation: Extracts cylindrical holes from the point cloud."""
        holes_indices = []
        holes_coeffs = []
        hole_centroids = []
        working_cloud = cloud

        for _ in range(max_holes):
            seg = working_cloud.make_segmenter_normals(ksearch=50)
            seg.set_optimize_coefficients(True)
            seg.set_model_type(pcl.SACMODEL_CYLINDER)
            seg.set_normal_distance_weight(0.1)
            seg.set_method_type(pcl.SAC_RANSAC)
            seg.set_max_iterations(10000)
            seg.set_distance_threshold(0.05)
            seg.set_radius_limits(0.01, 0.06)  # Adjust as needed

            indices, coefficients = seg.segment()

            # Accept only cylinders with enough points and reasonable radius
            if len(indices) < 30 or coefficients[6] > 0.06 or coefficients[6] < 0.01:
                break

            holes_indices.append(indices)
            holes_coeffs.append(coefficients)

            # Mask out detected hole points for the next iteration
            mask = np.ones(working_cloud.size, dtype=bool)
            mask[indices] = False
            working_cloud = working_cloud.extract(np.where(mask)[0])

            # Calculate centroid of detected hole
            hole_points = cloud.extract(indices)
            centroid = np.mean(hole_points, axis=0)
            hole_centroids.append(centroid)

        # After all holes are detected, filter out holes that are too close
        hole_centroids, hole_indices = self.filter_close_holes(hole_centroids, holes_indices, min_distance)

        # For visualization, you can merge all detected hole clouds:
        all_hole_points = []
        for indices in hole_indices:
            hole_cloud = cloud.extract(indices)
            all_hole_points.append(hole_cloud.to_array())

        if all_hole_points:
            merged_points = np.vstack(all_hole_points)
            all_hole_cloud = pcl.PointCloud()
            all_hole_cloud.from_array(merged_points)
        else:
            all_hole_cloud = pcl.PointCloud()

        return hole_indices, holes_coeffs, all_hole_cloud

    def filter_close_holes(self, centroids, indices, min_distance):
        """Filters out holes that are too close to each other based on a minimum distance."""
        filtered_centroids = []
        filtered_indices = []

        for i, centroid in enumerate(centroids):
            # Check distance with already selected holes
            too_close = False
            for j, existing_centroid in enumerate(filtered_centroids):
                distance = np.linalg.norm(np.array(centroid) - np.array(existing_centroid))
                if distance < min_distance:
                    too_close = True
                    break

            if not too_close:
                filtered_centroids.append(centroid)
                filtered_indices.append(indices[i])

        return filtered_centroids, filtered_indices


    def extract_clusters(self, cloud: pcl.PointCloud, cluster_type: str) -> Tuple[List[pcl.PointCloud], List[List[float]], List[List[float]]]:
        """Extracts clusters corresponding to tray from the point cloud"""
        tree = cloud.make_kdtree()
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.04)
        ec.set_MinClusterSize(30)
        ec.set_MaxClusterSize(100000)
        ec.set_SearchMethod(tree)

        # Extract clusters
        cluster_indices = ec.Extract()

        # Initialize lists to store table clusters, centroids, and dimensions
        object_clusters = []
        cluster_centroids = []
        cluster_dimensions = []

        # Process each cluster
        for idx, indices in enumerate(cluster_indices):
            self.get_logger().info(f"Processing {cluster_type} cluster {idx + 1}...")

            # Extract points belonging to the cluster
            cluster = cloud.extract(indices)

            # Calculate centroid
            centroid = np.mean(cluster, axis=0)

            # Computes the min and max coordinates along each axis 
            min_coords = np.min(cluster, axis=0)
            max_coords = np.max(cluster, axis=0)
            dimensions = max_coords - min_coords

            # Append clusters, centroids and dimensions to lists
            object_clusters.append(cluster)
            cluster_centroids.append(centroid.tolist())
            cluster_dimensions.append(dimensions.tolist())

            # Log cluster information
            num_points = len(indices)
            self.get_logger().info(f"{cluster_type} cluster {idx + 1} has {num_points} points.")
            self.get_logger().info(f"Centroid of {cluster_type} cluster {idx + 1}: {centroid}")
            # self.get_logger().info(f"Dimensions of {cluster_type} cluster {idx + 1}: {dimensions}")

        # Check if any clusters have been extracted
        if not object_clusters:
            self.get_logger().warning(f"No {cluster_type} clusters extracted...")

        # Return the filtered table clusters, centroids and cluster dimensions
        return object_clusters, cluster_centroids, cluster_dimensions

    def pub_surface_marker(self, surface_centroids: List[List[float]], surface_dimensions: List[List[float]]) -> None:
        """Publishes the detected cylindrical surface (coffee tray) as cylinder markers"""
        marker_array = MarkerArray()

        for idx, (centroid, dimensions) in enumerate(zip(surface_centroids, surface_dimensions)):
            radius = float(dimensions[0]) / 2
            height = 0.09

            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "base_link"
            cylinder_marker.id = idx
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = centroid[0]
            cylinder_marker.pose.position.y = centroid[1]
            cylinder_marker.pose.position.z = centroid[2] - height / 2  # Adjust height to center the cylinder
            cylinder_marker.pose.orientation.w = 1.0

            cylinder_marker.scale.x = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.y = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.z = height  # Height of the cylinder

            cylinder_marker.color.r = 0.0
            cylinder_marker.color.g = 1.0
            cylinder_marker.color.b = 0.0
            cylinder_marker.color.a = 0.4  # Semi-transparent

            marker_array.markers.append(cylinder_marker)

        if marker_array.markers:
            self.tray_marker_pub.publish(marker_array)

    def pub_hole_markers(self, hole_centroids: List[List[float]], hole_dimensions: List[List[float]]) -> None:
        """Publishes the detected cylindrical holes as markers."""
        marker_array = MarkerArray()
        
        for idx, (centroid, dimensions) in enumerate(zip(hole_centroids, hole_dimensions)):
            radius = float(dimensions[0]) / 2
            height = 0.035

            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "base_link"
            cylinder_marker.id = idx
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = centroid[0]
            cylinder_marker.pose.position.y = centroid[1]
            cylinder_marker.pose.position.z = centroid[2]
            cylinder_marker.pose.orientation.w = 1.0

            cylinder_marker.scale.x = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.y = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.z = height      # Height of the cylinder

            cylinder_marker.color.r = 0.0
            cylinder_marker.color.g = 0.0
            cylinder_marker.color.b = 1.0
            cylinder_marker.color.a = 0.9

            marker_array.markers.append(cylinder_marker)

        if marker_array.markers:
            self.hole_marker_pub.publish(marker_array)

    def pub_surface_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        """Publishes the detected surface information"""
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            surface_msg = DetectedSurfaces()
            surface_msg.surface_id = idx
            surface_msg.position.x = centroid[0]
            surface_msg.position.y = centroid[1]
            surface_msg.position.z = centroid[2]
            surface_msg.height = dimension[0]
            surface_msg.width = dimension[1]
            self.surface_detected_pub.publish(surface_msg)

    def pub_hole_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        """Publishes the detected hole information similar to pub_surface_detected."""
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            hole_msg = DetectedHoles()
            hole_msg.hole_id = idx
            hole_msg.position.x = centroid[0]
            hole_msg.position.y = centroid[1]
            hole_msg.position.z = centroid[2]
            hole_msg.radius = float(dimension[0]) / 2
            hole_msg.height = 0.035  # Fixed hole depth of 3cm
            self.hole_detected_pub.publish(hole_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    hole_detection = HoleDetection()
    rclpy.spin(hole_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()