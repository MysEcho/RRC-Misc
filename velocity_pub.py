#!/usr/bin/env python3
import rospy
from leg_tracker.msg import PersonArray, PersonVelocity, PeopleVelocity
from time import perf_counter

previous_data = {}


def calculate_velocity(current_pos, previous_pos, dt):
    dx = current_pos[0] - previous_pos[0]
    dy = current_pos[1] - previous_pos[1]
    velocity_x = dx / dt
    velocity_y = dy / dt
    return velocity_x, velocity_y


def people_callback(msg: PersonArray, velocity_publisher):
    current_time = perf_counter()
    people_list = []
    for person in msg.people:
        id = person.id
        coord_x = person.pose.position.x
        coord_y = person.pose.position.y

        if id in previous_data:
            prev_coord_x, prev_coord_y, prev_time = previous_data[id]
            dt = current_time - prev_time

            if dt > 0:
                velocity_x, velocity_y = calculate_velocity(
                    (coord_x, coord_y), (prev_coord_x, prev_coord_y), dt
                )

                vel_msg = PersonVelocity()
                vel_msg.id = id
                vel_msg.pose.position.x = coord_x
                vel_msg.pose.position.y = coord_y
                vel_msg.velocity_x = velocity_x
                vel_msg.velocity_y = velocity_y
                people_list.append(vel_msg)
        previous_data[id] = (coord_x, coord_y, current_time)
    people_msg = PeopleVelocity()
    people_msg.people = people_list
    velocity_publisher.publish(people_msg)


def main():
    rospy.init_node("people_velocity_node")
    velocity_publisher = rospy.Publisher("vel_pub", PeopleVelocity, queue_size=10)
    rospy.Subscriber(
        "/people_tracked",
        PersonArray,
        people_callback,
        callback_args=velocity_publisher,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
