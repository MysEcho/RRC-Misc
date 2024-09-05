#!/usr/bin/env python3
import rospy
import numpy as np
from typing import List
import tf2_ros
import tf2_geometry_msgs
from pedsim_msgs.msg import AgentStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class CalculateMetrics:
    def __init__(self, odom_topic="/odom", collision_padding=0.5, collision_cooldown=1.3) -> None:
        rospy.init_node("calculate_metrics")
        self.odom_topic = odom_topic
        self.collision_padding = collision_padding
        self.collision_cooldown = collision_cooldown
        self.odom: Odometry = None
        self.agent_states: AgentStates = None
        self.position_array: List[List[float]] = []
        self.velocity_array: List[List[float]] = []
        self.num_collisions = 0
        self.last_collision_time = rospy.Time(0)
        self.colliding_agents = set()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.agent_states_callback)

    def agent_states_callback(self, msg: AgentStates) -> None:
        self.agent_states = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg
        self.position_array.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.velocity_array.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        self.check_collision()

    def transform_pose(self, pose, from_frame, to_frame):
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = rospy.Time.now()
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return transformed_pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform pose: {e}")
            return None

    def check_collision(self) -> None:
        if self.agent_states is None or self.odom is None:
            return

        current_time = rospy.Time.now()
        if (current_time - self.last_collision_time).to_sec() < self.collision_cooldown:
            return

        robot_position = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])

        new_colliding_agents = set()
        for agent in self.agent_states.agent_states:
            transformed_pose = self.transform_pose(agent.pose, "gazebo", "odom")
            if transformed_pose is None:
                continue

            agent_position = np.array([transformed_pose.position.x, transformed_pose.position.y])
            distance = np.linalg.norm(robot_position - agent_position)

            if distance < self.collision_padding:
                new_colliding_agents.add(agent.id)

        # Filter for collision with same agent
        if new_colliding_agents and not new_colliding_agents.issubset(self.colliding_agents):
            self.num_collisions += 1
            self.last_collision_time = current_time
            print(f"Collision detected with agent(s) {new_colliding_agents}")

        self.colliding_agents = new_colliding_agents

    def get_avg_velocity(self) -> np.ndarray:
        vel_array = np.mean(self.velocity_array, axis=0)
        return vel_array[0]

    def get_path_length(self) -> float:
        path_length = 0.0
        for i in range(1, len(self.position_array)):
            path_length += np.linalg.norm(np.array(self.position_array[i]) - np.array(self.position_array[i - 1]))
        return path_length

    def save_metrics(self, filename="metrics.txt"):
        with open(filename, "w") as f:
            f.write("Metrics for the Current Run\n")
            f.write(f"1. Average Speed: {self.get_avg_velocity()} m/s\n")
            f.write(f"2. Total Distance Travelled: {self.get_path_length()} m\n")
            f.write(
                f"3. Number of Collisions: {self.num_collisions} "
                f"(collisions in {self.collision_padding}m around robot)\n"
            )


def main():
    metrics = CalculateMetrics()
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("=====================================")
        print("Printing Metrics for the Current Run")
        print("1. Average Linear Speed: ", metrics.get_avg_velocity(), "m/s")
        print("2. Total Distance Travelled: ", metrics.get_path_length(), "m")
        print(
            "3. Number of Collisions: ",
            metrics.num_collisions,
            f"(collisions in {metrics.collision_padding}m around robot)",
        )
        print("=====================================")
        metrics.save_metrics()


if __name__ == "__main__":
    main()
