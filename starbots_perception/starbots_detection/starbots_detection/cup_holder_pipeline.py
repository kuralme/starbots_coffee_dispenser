#!/usr/bin/env python3

import numpy as np
import pcl
import cv2
import rclpy
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from typing import List, Tuple, Union
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from starbots_detection_msgs.msg import DetectedSurfaces, DetectedCupholder, DetectedCupholders

class CupHolderDetection(Node):
    def __init__(self) -> None:
        super().__init__('cup_holder_detection_node')

        self.annot_pub = self.create_publisher(Image, 'barista_cam_annotated', 10)
        self.tray_detected_pub = self.create_publisher(DetectedSurfaces, 'tray_detected', 10)
        self.tray_marker_pub = self.create_publisher(MarkerArray, 'tray_marker', 10)
        self.cupholder_pub = self.create_publisher(DetectedCupholders, 'cup_holders_detected', 10)
        self.cupholder_marker_pub = self.create_publisher(MarkerArray, 'cup_holder_markers', 10)
        
        self.create_subscription(Image, '/D415/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/D415/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/D415/aligned_depth_to_color/camera_info', self.caminfo_callback, 10)
        self.create_subscription(PointCloud2, '/D415/barista_points', self.pc_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.pc_centroids = []
        self.occupied_pcl_centroids = []  # occupied cupholder centroids
        self.hough_centroids = []
        self.prev_detections = []  # For tracking
        self.K = None
        self.last_depth = None
        self.depth_scale = 0.001
        self.depth_window = 10
        self.min_depth_m = 0.02
        self.max_depth_m = 3.0

        # Fusing settings
        self.weight_hough = 0.4
        self.weight_pcl = 0.6
        self.fuse_threshold = 0.03 # 3 cm

        # Hough params
        self.dp = 1.6
        self.minDist = 20
        self.param1 = 162
        self.param2 = 30
        self.minRadius = 10
        self.maxRadius = 18
        self.gauss_k = 6

        self.get_logger().info("Cupholder Detection Full Pipeline initialized")

    def caminfo_callback(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera intrinsics loaded")

    def depth_callback(self, msg: Image):
        depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth_img is None:
            return
        if depth_img.dtype == np.uint16:
            self.last_depth = depth_img.astype(np.float32) * self.depth_scale
        else:
            self.last_depth = depth_img.astype(np.float32)

    def image_callback(self, msg: Image):
        if self.K is None or self.last_depth is None:
            return

        # Convert to OpenCV
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        annotated = img.copy()

        # Crop right 1/4 part of Gray for processing
        h_img, w_img = gray.shape[:2]
        crop_w = 3 * w_img // 4
        proc_gray = gray[:, :crop_w]

        # Gaussian blur
        k = self.gauss_k if (self.gauss_k % 2 == 1) else self.gauss_k + 1
        gray_blur = cv2.GaussianBlur(proc_gray, (k, k), 0) if k > 1 else proc_gray
        
        # Hough Circles
        circles = cv2.HoughCircles(
            gray_blur, cv2.HOUGH_GRADIENT,
            self.dp, self.minDist,
            param1=self.param1, param2=self.param2,
            minRadius=self.minRadius, maxRadius=self.maxRadius
        )

        self.hough_centroids = []
        cam_frame = msg.header.frame_id
        now_msg = self.get_clock().now().to_msg()

        if circles is not None:
            circles = np.around(circles[0]).astype(float)
            for c in circles:
                u_local, v_local = int(round(c[0])), int(round(c[1]))
                r_px = float(c[2])
                u = u_local
                v = v_local

                # Draw on full image
                cv2.circle(annotated, (u, v), int(round(r_px)), (0, 255, 0), 2)
                cv2.circle(annotated, (u, v), 3, (0, 0, 255), -1)

                # Depth and transform
                xyz = self.pixel_to_3d(u, v)
                if xyz is None:
                    self.get_logger().debug(f"No valid depth at ({u},{v})")
                    continue

                ps = PointStamped()
                ps.header.frame_id = cam_frame
                ps.header.stamp = now_msg
                ps.point.x, ps.point.y, ps.point.z = xyz

                try:
                    tf = self.tf_buffer.lookup_transform('base_link', cam_frame, rclpy.time.Time())
                    ps_out = tf2_geometry_msgs.do_transform_point(ps, tf)
                    self.hough_centroids.append([ps_out.point.x, ps_out.point.y, ps_out.point.z])
                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}")
                    continue
        
        # Remove occupied holes using pcl holes
        self.hough_centroids = [
            h for h in self.hough_centroids
            if not any(np.linalg.norm(np.array(h) - np.array(occ)) < 0.04 for occ in self.occupied_pcl_centroids)
        ]

        # Publish annotated image
        try:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.annot_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")

        # Publish markers + fused
        self.publish_cupholders(self.hough_centroids, method='hough')
        self.fuse_detections()

    def pixel_to_3d(self, u, v):
        if self.K is None or self.last_depth is None:
            return None
        h, w = self.last_depth.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None
        for radius in (self.depth_window, self.depth_window + 2, self.depth_window + 6):
            u0 = max(0, u - radius); u1 = min(w - 1, u + radius)
            v0 = max(0, v - radius); v1 = min(h - 1, v + radius)
            window = self.last_depth[v0:v1+1, u0:u1+1].flatten()
            window = window[np.isfinite(window) & (window > 1e-6)]
            if window.size == 0:
                continue
            Z = float(np.median(window))
            if not (self.min_depth_m <= Z <= self.max_depth_m):
                continue
            fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            return (X, Y, Z)
        return None

    def fuse_detections(self):
        if not self.pc_centroids and not self.hough_centroids:
            return

        fused = []
        if not self.pc_centroids:
            fused = self.hough_centroids
        elif not self.hough_centroids:
            fused = self.pc_centroids
        else:
            for h in self.hough_centroids:
                for p in self.pc_centroids:
                    dist = np.linalg.norm(np.array(h) - np.array(p))
                    if dist < self.fuse_threshold:
                        wavg = [(h_i * self.weight_hough + p_i * self.weight_pcl) for h_i, p_i in zip(h, p)]
                        is_occupied = any(np.linalg.norm(np.array(wavg) - np.array(occ)) < 0.04 for occ in self.occupied_pcl_centroids)
                        if not is_occupied:
                            fused.append(wavg)
                        break

        if fused:
            self.publish_cupholders(fused, method='fused')

    def pc_callback(self, msg: PointCloud2) -> None:
        try:
            self.occupied_pcl_centroids = [] 
            cloud = self.from_ros_msg(msg)
            filtered_cloud = self.filter_cloud(cloud, min_x=-0.7, max_x=-0.2, min_y=-0.2, max_y=0.5, min_z=-0.6, max_z=-0.3)

            # Plane segmentation and clustering
            plane_indices, plane_coefficients, tray_cloud = self.extract_plane(filtered_cloud)
            surface_clusters, surface_centroids, surface_dimensions = self.extract_clusters(tray_cloud, "Tray Cloud")

            # Filter points just below the tray surface and cluster cylinder cup holders
            cupholder_cloud = self.filter_below_surface(filtered_cloud, surface_centroids[0])
            cup_holder_centroids, cup_holder_dimensions = self.extract_cylinders(cupholder_cloud, filtered_cloud)

            # Update the stored point cloud centroids for fusing
            self.pc_centroids = cup_holder_centroids

            # Publish detected tray & cupholders markers and ROS2 msg
            self.pub_tray_detected(surface_centroids, surface_dimensions)
            self.publish_cupholders(cup_holder_centroids, dimensions=cup_holder_dimensions, method='pcl')

        except Exception as e:
            self.get_logger().error(f"Error in pc_callback: {e}")

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
            self.get_logger().info(
                f"\n=============================================\n"
                f"Plane Cluster {idx + 1} has {len(indices)} points:\n"
                f"Centroid of cluster {idx + 1}: {centroid}\n"
                f"Dimensions of {cluster_type} cluster {idx + 1}: {dimensions}"
            )

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

    def pub_tray_detected(self, surface_centroids: List[List[float]], surface_dimensions: List[List[float]]) -> None:
        """Publishes the detected cylindrical surface (coffee tray) as cylinder markers"""
        marker_array = MarkerArray()
        surface_msg = DetectedSurfaces()

        for idx, (centroid, dimensions) in enumerate(zip(surface_centroids, surface_dimensions)):
            radius = float(dimensions[0]) / 2
            height = 0.09

            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "base_link"
            cylinder_marker.id = idx
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = centroid[0]
            cylinder_marker.pose.position.y = centroid[1] + 0.018
            cylinder_marker.pose.position.z = centroid[2] - height / 2 # Adjust height to center the cylinder
            cylinder_marker.pose.orientation.w = 1.0
            cylinder_marker.scale.x = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.y = radius * 2  # Diameter of the cylinder
            cylinder_marker.scale.z = height      # Height of the cylinder
            cylinder_marker.color.r = 0.0
            cylinder_marker.color.g = 1.0
            cylinder_marker.color.b = 0.0
            cylinder_marker.color.a = 0.4  # Semi-transparent
            marker_array.markers.append(cylinder_marker)

            # ROS message for detected tray surface
            surface_msg.surface_id = idx
            surface_msg.position.x = centroid[0]
            surface_msg.position.y = centroid[1] + 0.018
            surface_msg.position.z = centroid[2]
            surface_msg.height = dimensions[0]
            surface_msg.width = dimensions[1]

        self.tray_detected_pub.publish(surface_msg)
        self.tray_marker_pub.publish(marker_array)

    def publish_cupholders(self, centroids: List[List[float]], dimensions: List[List[float]] = None, method: str= 'fused') -> None:
        """
        Unified publisher for all cupholder detections (Hough, PCL, fused). 
        Publishes both visualization markers and DetectedCupholders ROS message for 'fused' method.
        Markers are under different namespaces.
        """
        # Keep track of original indices
        matched_centroids = self.match_detections_to_previous(centroids)

        # General X correction
        x_values = [c[0] for _, c in matched_centroids]
        x_center = sum(x_values) / len(x_values)
        correction_factor = 0.1

        marker_array = MarkerArray()
        cupholders_msg = DetectedCupholders()
        now = self.get_clock().now().to_msg()
        text_height_offset = 0.1


        for assigned_id, centroid in matched_centroids:
            radius = 0.035
            height = 0.05
            if dimensions and assigned_id < len(dimensions):
                radius = float(dimensions[assigned_id][0]) / 2
                height = float(dimensions[assigned_id][1])

            # Natural offset based of the camera pov
            centroid[0] += 0.005
            centroid[1] -= 0.001

            # Hole marker
            marker = Marker()
            marker.ns = method
            marker.header.frame_id = "base_link"
            marker.header.stamp = now
            marker.id = assigned_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = centroid[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = radius * 2
            marker.scale.z = height
            marker.color.a = 0.6

            if method == 'hough':
                marker.pose.position.z += 0.025
                marker.scale.z = 0.003
                marker.color.r, marker.color.g, marker.color.b = (1.0, 1.0, 0.0)
            elif method == 'pcl':
                marker.color.r, marker.color.g, marker.color.b = (0.0, 0.0, 1.0)
            elif method == 'fused':
                marker.color.r, marker.color.g, marker.color.b = (1.0, 0.0, 0.0)

                # Hole ID marker
                text_marker = Marker()
                text_marker.header.frame_id = "base_link"
                text_marker.header.stamp = now
                text_marker.ns = 'enum'
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.id = 1000 + assigned_id  # Offset to avoid conflicts
                text_marker.text = str(assigned_id)
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = centroid[0]
                text_marker.pose.position.y = centroid[1]
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

                # Populate detection ROS2 message
                adjusted_x = centroid[0] - (centroid[0] - x_center) * correction_factor

                obj = DetectedCupholder()
                obj.cupholder_id = assigned_id
                obj.position = Point(x=adjusted_x, y=centroid[1], z=centroid[2])
                obj.radius = radius
                obj.height = height
                cupholders_msg.cup_holders.append(obj)

            marker_array.markers.append(marker)


        self.cupholder_marker_pub.publish(marker_array)
        if method == 'fused': # Only publish cupholders message for 'fused' and sort
            cupholders_msg.cup_holders = sorted(cupholders_msg.cup_holders, key=lambda x: x.cupholder_id)
            cupholders_msg.header = Header(stamp=now, frame_id="base_link")
            self.cupholder_pub.publish(cupholders_msg)
            
    def match_detections_to_previous(self, new_centroids, threshold=0.05, max_ids=4):
        """
        Match new centroids to previous ones using nearest neighbor (within threshold).
        Returns a list of (assigned_id, centroid).
        Limits total number of assigned IDs to `max_ids`.
        """
        matched = []
        unmatched = []
        used_prev_ids = set()
        id_pool = set(range(max_ids))  # Allowed IDs: 0, 1, 2, 3

        # Track which IDs are already used this round
        used_current_ids = set()

        for centroid in new_centroids:
            min_dist = float('inf')
            matched_id = None
            for prev_id, prev_pos in self.prev_detections:
                if prev_id in used_prev_ids:
                    continue
                dist = np.linalg.norm(np.array(centroid) - np.array(prev_pos))
                if dist < threshold and dist < min_dist:
                    matched_id = prev_id
                    min_dist = dist

            if matched_id is not None:
                matched.append((matched_id, centroid))
                used_prev_ids.add(matched_id)
                used_current_ids.add(matched_id)
            else:
                unmatched.append(centroid)

        # Reuse available IDs for unmatched centroids
        available_ids = list(id_pool - used_current_ids)
        for centroid in unmatched:
            if not available_ids:
                break  # Ignore extra detections beyond max_ids
            assigned_id = available_ids.pop(0)
            matched.append((assigned_id, centroid))
            used_current_ids.add(assigned_id)

        # Update internal state with only matched detections
        self.prev_detections = [(id_, centroid) for id_, centroid in matched]

        return matched

def main(args=None) -> None:
    rclpy.init(args=args)
    cup_holder_detection = CupHolderDetection()
    rclpy.spin(cup_holder_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()