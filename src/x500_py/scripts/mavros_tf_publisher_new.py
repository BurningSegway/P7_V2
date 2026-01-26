#!/usr/bin/env python
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

br = None
static_br = None
drone_name = None
model_found = False
static_published = False

def model_cb(msg):
    global br, drone_name, model_found

    # Find this drone's index
    try:
        idx = msg.name.index(drone_name)
        model_found = True
    except ValueError:
        if not model_found:
            rospy.logwarn_throttle(5, "[%s] not found in /gazebo/model_states yet (waiting for spawn...)" % drone_name)
        return

    pose = msg.pose[idx]

    # Publish: gazebo_odom -> base_link (the ONLY moving transform)
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = drone_name + "/gazebo_odom"
    t.child_frame_id = drone_name + "/base_link"

    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation

    br.sendTransform(t)


def publish_static_transform():
    """Publish the static map -> gazebo_odom transform"""
    global static_br, drone_name, static_published
    
    static_t = TransformStamped()
    static_t.header.stamp = rospy.Time.now()
    static_t.header.frame_id = drone_name + "_map"
    static_t.child_frame_id = drone_name + "/gazebo_odom"
    
    static_t.transform.translation.x = 0.0
    static_t.transform.translation.y = 0.0
    static_t.transform.translation.z = 0.0
    static_t.transform.rotation.x = 0.0
    static_t.transform.rotation.y = 0.0
    static_t.transform.rotation.z = 0.0
    static_t.transform.rotation.w = 1.0
    
    static_br.sendTransform(static_t)
    static_published = True
    rospy.loginfo("Published static transform: %s_map -> %s/gazebo_odom" % (drone_name, drone_name))


if __name__ == '__main__':
    rospy.init_node('gazebo_tf_publisher')

    # Gets the namespace of this drone (drone1 or drone2)
    ns = rospy.get_namespace().strip("/")
    drone_name = rospy.get_param('~tf_prefix', ns)

    rospy.loginfo("Publishing TF for drone [%s] from /gazebo/model_states", drone_name)

    br = tf2_ros.TransformBroadcaster()
    static_br = tf2_ros.StaticTransformBroadcaster()

    rospy.Subscriber('/gazebo/model_states', ModelStates, model_cb)

    # Wait for model to appear in ModelStates
    rospy.loginfo("Waiting for [%s] to appear in /gazebo/model_states..." % drone_name)
    while not model_found and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    if model_found:
        rospy.loginfo("[%s] found in /gazebo/model_states. Publishing transforms." % drone_name)
        publish_static_transform()
    
    rospy.spin()