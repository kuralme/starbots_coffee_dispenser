import pcl
import numpy as np
import rclpy
import tf2_ros
from typing import List, Tuple, Union
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from starbots_detection_msgs.msg import DetectedSurfaces, DetectedCupholder, DetectedCupholders

class CupHolderDetection(Node):
    def __init__(self) -> None:
        super().__init__('cup_holder_detection_node')
        pcl_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.pc_sub = self.create_subscription(PointCloud2, '/D415/barista_points', self.callback, pcl_qos_profile)
        self.tray_marker_pub = self.create_publisher(MarkerArray, '/tray_marker', 10)
        self.cupholder_marker_pub = self.create_publisher(MarkerArray, '/cup_holder_markers', 10)
        self.tray_detected_pub = self.create_publisher(DetectedSurfaces, '/tray_detected', 10)
        self.cupholder_detected_pub = self.create_publisher(DetectedCupholders, '/cup_holder_detected', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def callback(self, msg: PointCloud2) -> None:
        try:
            cloud = self.from_ros_msg(msg)
            filtered_cloud = self.filter_cloud(cloud, min_x=-0.7, max_x=-0.2, min_y=-0.2, max_y=0.5, min_z=-0.6, max_z=-0.3)

            # Plane segmentation and clustering
            plane_indices, plane_coefficients, tray_cloud = self.extract_plane(filtered_cloud)
            surface_clusters, surface_centroids, surface_dimensions = self.extract_clusters(tray_cloud, "Tray Cloud")

            # Filter points just below the tray surface and cluster cylinder cup holders
            # surface_centroids[0][1] += 0.018
            cupholder_cloud = self.filter_below_surface(filtered_cloud, surface_centroids[0])
            cup_holder_centroids, cup_holder_dimensions = self.extract_cylinders(cupholder_cloud, filtered_cloud)

            # Publish detected tray markers and info
            self.pub_surface_marker(surface_centroids, surface_dimensions)
            self.pub_surface_detected(surface_centroids, surface_dimensions)

            # Publish detected cup holder markers and info
            tray_height = surface_centroids[0][2]
            self.pub_cup_holder_markers(cup_holder_centroids, cup_holder_dimensions, tray_height)
            self.pub_cup_holder_detected(cup_holder_centroids, cup_holder_dimensions, tray_height)

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
    
    def filter_below_surface(self, cloud: pcl.PointCloud, surface_centroid: List[float], height_threshold=0.06, radius_limit=0.15) -> pcl.PointCloud:
        """Filters points just below the tray surface and within a specified radius."""
        filtered_indices = []
        centroid = surface_centroid

        for i in range(cloud.size):
            point = cloud[i]
            x, y, z = point

            # Check if the point is below the tray surface
            if z < centroid[2]-0.02 and (centroid[2] - z) <= height_threshold:
                distance_from_center = np.sqrt((x - centroid[0])**2 + (y - centroid[1])**2)

                # Ensure the point is within the tray's radius limit
                if distance_from_center <= radius_limit:
                    filtered_indices.append(i)

        return cloud.extract(filtered_indices)

    def filter_close_cupholder(self, centroids, min_distance):
        """Filters out cupholders that are too close to each other based on a minimum distance."""
        filtered_centroids = []

        for centroid in centroids:
            too_close = False
            for existing_centroid in filtered_centroids:
                distance = np.linalg.norm(np.array(centroid) - np.array(existing_centroid))
                if distance < min_distance:
                    too_close = True
                    break

            if not too_close:
                filtered_centroids.append(centroid)

        return filtered_centroids

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
            num_points = len(indices)
            self.get_logger().info(f"{cluster_type} cluster {idx + 1} has {num_points} points.")
            self.get_logger().info(f"Centroid of {cluster_type} cluster {idx + 1}: {centroid}")
            self.get_logger().info(f"Dimensions of {cluster_type} cluster {idx + 1}: {dimensions}")

        # Check if any clusters have been extracted
        if not object_clusters:
            self.get_logger().warning(f"No {cluster_type} clusters extracted...")

        # Return the filtered table clusters, centroids and cluster dimensions
        return object_clusters, cluster_centroids, cluster_dimensions

    def extract_cylinders(self, cupholder_cloud, filtered_cloud, min_distance=0.05, min_height=0.005, max_height=0.04, min_radius=0.01, max_radius=0.04)  -> Tuple[List[List[float]], List[List[float]]]:
        """Segmentation: Extracts cylindrical cupholder from the point cloud"""
        tree = cupholder_cloud.make_kdtree()
        ec = cupholder_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.04)
        ec.set_MinClusterSize(30)
        ec.set_MaxClusterSize(100000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()

        cupholder_centroids = []
        cupholder_dimensions = []

        for idx, indices in enumerate(cluster_indices):
            cluster = cupholder_cloud.extract(indices)
            centroid = np.mean(cluster.to_array(), axis=0)
            min_coords = np.min(cluster.to_array(), axis=0)
            max_coords = np.max(cluster.to_array(), axis=0)
            dimensions = max_coords - min_coords
            radius = dimensions[0] / 2.
            height = dimensions[2]

            # Filter out clusters that have points above the tray surface (likely occupied by a cup)
            points_above = []
            for i in range(filtered_cloud.size):
                pt = filtered_cloud[i]
                xy_dist = np.linalg.norm(pt[:2] - centroid[:2])
                if xy_dist < radius + 0.01 and pt[2] > centroid[2] + height/2. + 0.03:
                    points_above.append(pt)
            if len(points_above) > 10:
                self.get_logger().info(f"Skipping cluster {idx+1}: likely occupied by a cup.")
                continue
                
            # Filter by cupholder dimensions
            if min_radius <= radius <= max_radius and min_height <= height <= max_height:
                cupholder_centroids.append(centroid.tolist())
                cupholder_dimensions.append(dimensions.tolist())

            self.get_logger().info(
                f"\n=============================================\n"
                f"Cluster {idx + 1} has {len(indices)} points:\n"
                f"Centroid of cluster {idx + 1}: {centroid}\n"
                f"Radius of cluster {idx + 1}: {radius}\n"
                f"Height of cluster {idx + 1}: {height}"
            )

        if not cupholder_centroids:
            self.get_logger().warning("No cupholder-like clusters detected!")

        # Filter out centroids that are too close to each other based on minimum distance
        cupholder_centroids = self.filter_close_cupholder(cupholder_centroids, min_distance)

        # Sort cupholders by height (Z value of centroid)
        sorted_results = sorted(
            zip(cupholder_centroids, cupholder_dimensions),
            key=lambda x: x[0][0]
        )
        cupholder_centroids, cupholder_dimensions = map(list, zip(*sorted_results)) if sorted_results else ([], [])

        return cupholder_centroids, cupholder_dimensions

    def pub_surface_marker(self, surface_centroids: List[List[float]], surface_dimensions: List[List[float]]) -> None:
        """Publishes the detected cylindrical surface (coffee tray) as cylinder markers"""
        marker_array = MarkerArray()
        height = 0.085

        for idx, (centroid, dimensions) in enumerate(zip(surface_centroids, surface_dimensions)):
            radius = float(dimensions[0]) / 2
            
            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "base_link"
            cylinder_marker.id = idx
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = centroid[0]
            cylinder_marker.pose.position.y = centroid[1] + 0.015
            cylinder_marker.pose.position.z = centroid[2] - height / 2
            cylinder_marker.pose.orientation.w = 1.0
            cylinder_marker.scale.x = radius * 2  # Diameter of the tray
            cylinder_marker.scale.y = radius * 2  # Diameter of the tray
            cylinder_marker.scale.z = height      # Height of the tray
            cylinder_marker.color.r = 0.0
            cylinder_marker.color.g = 1.0
            cylinder_marker.color.b = 0.0
            cylinder_marker.color.a = 0.4  # Semi-transparent
            marker_array.markers.append(cylinder_marker)

        if marker_array.markers:
            self.tray_marker_pub.publish(marker_array)

    def pub_surface_detected(self, centroids: List[List[float]], dimensions: List[List[float]]) -> None:
        """Publishes the detected surface information"""
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            surface_msg = DetectedSurfaces()
            surface_msg.surface_id = idx
            surface_msg.position.x = centroid[0]
            surface_msg.position.y = centroid[1] + 0.015
            surface_msg.position.z = centroid[2]
            surface_msg.height = dimension[1]
            surface_msg.width = dimension[0]
            self.tray_detected_pub.publish(surface_msg)

    def pub_cup_holder_markers(self, cupholder_centroids: List[List[float]], cupholder_dimensions: List[List[float]], tray_height) -> None:
        """Publishes the detected cylindrical cupholder as markers"""
        marker_array = MarkerArray()
        radius = 0.032
        height = 0.045
        text_height_offset = 0.05
        
        for idx, (centroid, dimensions) in enumerate(zip(cupholder_centroids, cupholder_dimensions)):
            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "base_link"
            cylinder_marker.id = idx
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = centroid[0]
            cylinder_marker.pose.position.y = centroid[1] - 0.005
            cylinder_marker.pose.position.z = tray_height - height / 2.
            cylinder_marker.pose.orientation.w = 1.0
            cylinder_marker.scale.x = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.y = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.z = height      # Height of the cylinder
            cylinder_marker.color.r = 0.0
            cylinder_marker.color.g = 0.0
            cylinder_marker.color.b = 1.0
            cylinder_marker.color.a = 0.9
            marker_array.markers.append(cylinder_marker)

            # Create the ID text marker above the cupholder
            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.id = idx + 1000  # Different ID to avoid conflicts with cylinder markers
            text_marker.text = str(idx) # Set the text as the cup holder ID
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = centroid[0]
            text_marker.pose.position.y = centroid[1] - 0.005
            text_marker.pose.position.z = centroid[2] + text_height_offset
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.x = 0.05
            text_marker.scale.y = 0.05
            text_marker.scale.z = 0.05
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            marker_array.markers.append(text_marker)

        if marker_array.markers:
            self.cupholder_marker_pub.publish(marker_array)
        else:
            self.cupholder_marker_pub.publish(MarkerArray())
            self.get_logger().warning("No cup holder markers to publish.")

    def pub_cup_holder_detected(self, centroids: List[List[float]], dimensions: List[List[float]], tray_height) -> None:
        """Publishes detected cupholder information of the cupholders"""
        cupholders_msg = DetectedCupholders()
        radius = 0.032
        height = 0.045
        
        for idx, (centroid, dimension) in enumerate(zip(centroids, dimensions)):
            if idx == 0:
                hole_offset = 0.015
            else:
                hole_offset = 0.0
            cupholder = DetectedCupholder()
            cupholder.cupholder_id = idx
            cupholder.position = Point(x=(centroid[0] + hole_offset), y=(centroid[1] - 0.005), z=(tray_height - height / 2.))
            cupholder.radius = radius
            cupholder.height = height
            cupholders_msg.cup_holders.append(cupholder)
        self.cupholder_detected_pub.publish(cupholders_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    cup_holder_detection = CupHolderDetection()
    rclpy.spin(cup_holder_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()