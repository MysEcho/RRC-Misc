#!/home/wheelchair/.conda/envs/mmpose/bin/python3

import cv2
import numpy as np
from typing import List
from mmpose.apis import MMPoseInferencer

import rospy
import ros_numpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import ApproximateTimeSynchronizer, Subscriber


class KeypointGen:
    def __init__(self):
        rospy.init_node("keypoint_gen", anonymous=True)
        self.inferencer = MMPoseInferencer(pose2d="rtmo")

        self.rgb_sub = Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.info_sub = Subscriber("/camera/color/camera_info", CameraInfo)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], 10, 0.1
        )
        ts.registerCallback(self.image_callback)

        self.image_pub = rospy.Publisher("prediction_img", Image, queue_size=10)
        self.marker_pub = rospy.Publisher("marker", MarkerArray, queue_size=10)
        self.prev_positions = np.zeros((10, 17, 3))
        self.prev_velocity = np.zeros((10, 17, 3))
        self.dt = 1 / 25

    def odom_callback(self, odom: Odometry):
        self.odom_vel = [
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        ]

    def return_depth(self, x, y) -> float:
        try:
            return self.depth[int(y), int(x)] / 1000  # in meters
        except IndexError:
            return 0.0000001

    def get_transformed_keypoint(self, u, v) -> np.ndarray:
        """Return x,y,z of a keypoint using the camera transform"""

        # Get the depth of the keypoint
        z = self.return_depth(u, v)

        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy

        x_cam = x_norm * z
        y_cam = y_norm * z

        return np.array([x_cam, y_cam, z])

    def publish_image(self, keypoints):
        """Publish image with the keypoints displayed"""
        for keypoint in keypoints:
            cv2.circle(
                self.rgb, (int(keypoint[0][0]), int(keypoint[0][1])), 4, (0, 0, 255), -1
            )
            cv2.circle(
                self.rgb, (int(keypoint[1][0]), int(keypoint[1][1])), 4, (0, 255, 0), -1
            )
            cv2.circle(
                self.rgb, (int(keypoint[2][0]), int(keypoint[2][1])), 4, (255, 0, 0), -1
            )

        self.image_pub.publish(ros_numpy.msgify(Image, self.rgb, encoding="rgb8"))

    def perform_inference(self) -> np.ndarray:
        self.predictions = []
        for inference in self.inferencer(inputs=self.rgb):
            self.predictions = inference["predictions"]

        people = np.array([person["keypoints"] for person in self.predictions[0]])
        self.publish_image(people)

        if people.shape != (0,):
            # print(people, people.shape)

            sum_keypoints = np.sum(people, axis=1)
            selected_keypoints = sum_keypoints / people.shape[1]
            # print(np.array(det_keypoints).shape)
            transformed_keypoints = np.array(
                [self.get_transformed_keypoint(*point) for point in selected_keypoints]
            )
            transformed_keypoints = np.sort(transformed_keypoints, axis=0)
            return transformed_keypoints

    def perform_keypoint_inference(self) -> np.ndarray:
        people = np.array([person["keypoints"] for person in self.predictions[0]])
        self.publish_image(people)
        det_keypoints = []
        if people.shape != (0,):
            # print(people, people.shape)
            for person in people:
                det_keypoints.append(
                    [
                        self.get_transformed_keypoint(*person[0]),
                        self.get_transformed_keypoint(*person[5]),
                        self.get_transformed_keypoint(*person[6]),
                    ]
                )
            det_keypoints = np.sort(det_keypoints, axis=0)
            return np.array(det_keypoints)

    def vector_to_euler(self, vector):
        """Converts a 3D vector to Euler angles (XYZ convention).

        Args:
            vector: A numpy array representing the 3D vector.

        Returns:
            A tuple of Euler angles (yaw, pitch, roll) in radians.
        """

        x, y, z = vector[2], -vector[0], -vector[1]
        length = np.linalg.norm(vector)

        # Normalize vector
        x /= length
        y /= length
        z /= length

        # Calculate pitch
        pitch = np.arcsin(y)

        # Handle special cases for pitch near +/- pi/2
        if np.abs(pitch) < np.pi / 2 - 1e-6:
            yaw = np.arctan2(y, x)
            roll = np.arctan2(z, np.sqrt(x**2 + y**2))
        else:
            # Handle gimbal lock cases
            yaw = 0.0
            roll = np.arctan2(x, -z)

        return yaw, pitch, roll

    def marker_publisher(self, poses):
        """Publish the poses as markers"""
        marker_array = MarkerArray()
        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            x = pose[0]
            y = pose[1]
            z = pose[2]
            marker.pose.position.x = z
            marker.pose.position.y = -x
            marker.pose.position.z = 0.7

            # x, y, z, w = quaternion_from_euler(pose[4], pose[5], -pose[3])
            x, y, z, w = quaternion_from_euler(0, 0, -pose[3])
            # print("==============",pose[4])
            # x, y, z, w = quaternion_from_euler(0, 0, pose[4])
            # print("==============", x, y, z, w)
            marker.pose.orientation.x = x
            marker.pose.orientation.y = y
            marker.pose.orientation.z = z
            marker.pose.orientation.w = w

            marker.scale.x = 0.3
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.frame_locked = True
            marker.lifetime = rospy.Duration(0.1)
            # marker.text = "Person " + str(i)
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def calc_velocity(self, positions):
        """Calculate the velocity of the person"""
        if positions.shape == self.prev_positions.shape:
            velocity = np.subtract(positions, self.prev_positions) / self.dt

            # Account for the robot moving as well which causes the velocity to be
            # wrt the robot, which gives wrong orientation
            for i, vel in enumerate(velocity):
                velocity[i] = vel + self.odom_vel

            if self.prev_velocity.shape != velocity.shape:
                self.prev_velocity = velocity

            # Clamp the velocity so that orientation is stable
            # print(np.linalg.norm(velocity - self.prev_velocity, axis=1))
            # if all(np.linalg.norm(velocity - self.prev_velocity, axis=1) > 3):
            #     print("Clamping")
            #     return self.prev_velocity

            self.prev_positions = positions
            self.prev_velocity = velocity
            return velocity
        else:
            self.prev_positions = positions
            return None

    def get_poses(self, positions, velocities):
        """Get the poses of the person"""
        poses = [
            np.append(positions[i], self.vector_to_euler(velocities[i]))
            for i in range(len(positions))
        ]
        return poses

    def get_keypoint_poses(self, keypoints) -> np.ndarray:
        """Get the orientation of the person"""
        poses = []
        for nose, left, right in keypoints:
            # Get the vector of the shoulders
            shoulder_vector = np.array(left) - np.array(right)
            chest_point = (np.array(left) + np.array(right)) / 2
            chest_to_nose_vector = np.array(nose) - chest_point

            #  Cross Product
            orientation_vector = None
            orientation_vector = np.cross(shoulder_vector, chest_to_nose_vector)

            # if right[0] > left[0]:
            #     print("Someone is facing away from the camera")
            #     orientation_vector = np.cross(-shoulder_vector, chest_to_nose_vector)
            #     orientation_vector = orientation_vector/np.linalg.norm(orientation_vector) # Get unit vector
            # else:
            #     orientation_vector = np.cross(shoulder_vector, chest_to_nose_vector)
            #     orientation_vector = orientation_vector/np.linalg.norm(orientation_vector) # Get unit vector

            poses.append(
                np.concatenate(
                    (
                        chest_point,
                        self.vector_to_euler(orientation_vector),
                    )
                )
            )
        return np.array(poses)

    def image_callback(self, rgb: Image, depth: Image, info: CameraInfo):
        # print("hello")
        self.rgb = ros_numpy.numpify(rgb)
        self.depth = ros_numpy.numpify(depth)

        # Update Camera intrisics
        self.fx = info.K[0]
        self.fy = info.K[4]
        self.cx = info.K[2]
        self.cy = info.K[5]
        try:
            positions = self.perform_inference()
            det_positions = self.perform_keypoint_inference()

            if positions is not None and det_positions is not None:
                print("Number of Humans ", len(positions))
                velocities = self.calc_velocity(positions)

                if velocities is not None:
                    # Keep only 10 humans
                    poses = self.get_poses(positions, velocities)[:10]
                    det_poses = self.get_keypoint_poses(det_positions)[:10]

                    avg_poses = np.add(poses, det_poses) / 2

                    self.marker_publisher(poses)

                    for pose in poses:
                        print(
                            "x",
                            pose[0],
                            "y",
                            pose[1],
                            "z",
                            pose[2],
                            "qx: ",
                            pose[3],
                            "qy: ",
                            pose[4],
                            "qz: ",
                            pose[5],
                        )
                    print(
                        "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                    )
        except Exception as e:
            print(e)


if __name__ == "__main__":
    kpg = KeypointGen()
    rospy.spin()
