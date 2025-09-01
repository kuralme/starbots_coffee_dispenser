import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import pcl
import numpy as np
import tf2_ros
from tf2_ros import TransformException, ConnectivityException
from starbots_detection_msgs.msg import DetectedSurfaces, DetectedCupholder, DetectedCupholders
from typing import List, Tuple, Union


class CupHolderDetection(Node):
    def __init__(self) -> None:
        super().__init__('cup_holder_detection_node')
        self.pc_sub = self.create_subscription(PointCloud2, '/wrist_rgbd_depth_sensor/points_filtered', self.callback, 10)
        self.tray_marker_pub = self.create_publisher(MarkerArray, '/tray_marker', 10)
        self.cupholder_marker_pub = self.create_publisher(MarkerArray, '/cup_holder_markers', 10)
        self.tray_detected_pub = self.create_publisher(DetectedSurfaces, '/tray_detected', 10)
        self.cupholder_detected_pub = self.create_publisher(DetectedCupholders, '/cup_holder_detected', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, msg: PointCloud2) -> None:
        try:
            cloud = self.from_ros_msg(msg)
            # Filtered cloud for tray surface detection
            filtered_cloud_plane = self.filter_cloud(cloud, min_x=-0.6, max_x=-0.2, min_y=-0.2, max_y=0.2, min_z=-0.65, max_z=-0.54)
            # Filtered cloud for cup holder detection
            filtered_cloud_cupholder = self.filter_cloud(cloud, min_x=-0.55, max_x=-0.25, min_y=-0.15, max_y=0.15, min_z=-0.63, max_z=-0.55)

            # Plane segmentations using RANSAC: tray plane and cupholders
            plane_indices, plane_coefficients, tray_cloud = self.extract_plane(filtered_cloud_plane)
            cupholder_indices, cupholder_coefficients, cupholder_cloud = self.extract_cylinder(filtered_cloud_cupholder)

            # Clustering methods
            table_clusters, surface_centroids, surface_dimensions = self.extract_clusters(tray_cloud, "Tray Cloud")
            cup_holder_clusters, cup_holder_centroids, cup_holder_dimensions = self.extract_clusters(cupholder_cloud, "Cupholder cloud")

            # Publish detected markers
            self.pub_surface_marker(surface_centroids, surface_dimensions)
            self.pub_cup_holder_markers(cup_holder_centroids, cup_holder_dimensions)

            # Publish detected info
            self.pub_surface_detected(surface_centroids, surface_dimensions)
            self.pub_cup_holder_detected(cup_holder_centroids, cup_holder_dimensions)

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

    def extract_cylinder(self, cloud, max_cupholder=4, min_distance=0.05):
        """Segmentation: Extracts cylindrical cupholder from the point cloud."""
        cupholder_indices = []
        cupholder_coeffs = []
        cupholder_centroids = []
        working_cloud = cloud

        for _ in range(max_cupholder):
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

            cupholder_indices.append(indices)
            cupholder_coeffs.append(coefficients)

            # Mask out detected cup holder points for the next iteration
            mask = np.ones(working_cloud.size, dtype=bool)
            mask[indices] = False
            working_cloud = working_cloud.extract(np.where(mask)[0])

            # Calculate centroid of detected cup holder
            cupholder_points = cloud.extract(indices)
            centroid = np.mean(cupholder_points, axis=0)
            cupholder_centroids.append(centroid)

        # After all cupholder are detected, filter out cupholder that are too close
        cupholder_centroids, cupholder_indices = self.filter_close_cupholder(cupholder_centroids, cupholder_indices, min_distance)

        # For visualization, you can merge all detected cup holder clouds:
        cupholder_points = []
        for indices in cupholder_indices:
            cupholder_cloud = cloud.extract(indices)
            cupholder_points.append(cupholder_cloud.to_array())

        if cupholder_points:
            merged_points = np.vstack(cupholder_points)
            cupholder_cloud = pcl.PointCloud()
            cupholder_cloud.from_array(merged_points)
        else:
            cupholder_cloud = pcl.PointCloud()

        return cupholder_indices, cupholder_coeffs, cupholder_cloud

    def filter_close_cupholder(self, centroids, indices, min_distance):
        """Filters out cupholder that are too close to each other based on a minimum distance."""
        filtered_centroids = []
        filtered_indices = []

        for i, centroid in enumerate(centroids):
            # Check distance with already selected cupholder
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
            # self.get_logger().info(f"Processing {cluster_type} cluster {idx + 1}...")

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
            # num_points = len(indices)
            # self.get_logger().info(f"{cluster_type} cluster {idx + 1} has {num_points} points.")
            # self.get_logger().info(f"Centroid of {cluster_type} cluster {idx + 1}: {centroid}")
            # self.get_logger().info(f"Dimensions of {cluster_type} cluster {idx + 1}: {dimensions}")

        # Check if any clusters have been extracted
        # if not object_clusters:
        #     self.get_logger().warning(f"No {cluster_type} clusters extracted...")

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

    def pub_cup_holder_markers(self, cupholder_centroids: List[List[float]], cupholder_dimensions: List[List[float]]) -> None:
        """Publishes the detected cylindrical cupholder as markers."""
        marker_array = MarkerArray()
        
        for idx, (centroid, dimensions) in enumerate(zip(cupholder_centroids, cupholder_dimensions)):
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
            self.cupholder_marker_pub.publish(marker_array)

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
            self.tray_detected_pub.publish(surface_msg)

    def pub_cup_holder_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        """Publishes detected cupholder information of 4 cupholders."""
        
        if len(centroids) != 4:
            # print("Skipping publishing: Expected 4 cupholders, found", len(centroids))
            return  # Do nothing if there are not 4 cupholders
        
        cupholders_msg = DetectedCupholders()
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            cupholder = DetectedCupholder()
            cupholder.cupholder_id = idx
            cupholder.position = Point(x=centroid[0], y=centroid[1], z=centroid[2])
            cupholder.radius = float(dimension[0]) / 2
            cupholder.height = 0.035  # Fixed height for all cupholders
            cupholders_msg.cup_holders.append(cupholder)

        self.cupholder_detected_pub.publish(cupholders_msg)
        # print("Published cupholder information for 4 cupholders.")

def main(args=None) -> None:
    rclpy.init(args=args)
    cup_holder_detection = CupHolderDetection()
    rclpy.spin(cup_holder_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()