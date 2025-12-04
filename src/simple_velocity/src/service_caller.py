#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger

rospy.init_node("service_caller")

# Wait for services to be available
rospy.wait_for_service("/change_state")
rospy.wait_for_service("/set_heading_i")
rospy.wait_for_service("/set_heading_j")
rospy.wait_for_service("/set_heading_k")
rospy.wait_for_service("/set_heading_l")

# Proxies
state_srv = rospy.ServiceProxy("/change_state", Trigger)
heading_srvs = {
    'i': rospy.ServiceProxy("/set_heading_i", Trigger),
    'j': rospy.ServiceProxy("/set_heading_j", Trigger),
    'k': rospy.ServiceProxy("/set_heading_k", Trigger),
    'l': rospy.ServiceProxy("/set_heading_l", Trigger)
}

print("Press 'h' to toggle state, 'i/j/k/l' to change heading, 'q' to quit")

while not rospy.is_shutdown():
    cmd = input("Enter command: ").strip().lower()
    if cmd == "h":
        try:
            resp = state_srv()
            print(f"[STATE] {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    elif cmd in ["i","j","k","l"]:
        try:
            resp = heading_srvs[cmd]()
            print(f"[HEADING] {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
    elif cmd == "q":
        break
    else:
        print("Unknown command")
