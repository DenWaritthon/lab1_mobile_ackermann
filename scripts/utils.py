from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

def pub_odom(odom_pub, tf_broadcaster, t, twist, pos_global, ori_global):
    msg = Odometry()
    msg.header.stamp = t.to_msg()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'

    msg.twist.twist.linear.x = twist[0]
    msg.twist.twist.angular.z = twist[1]

    msg.pose.pose.position.x = pos_global[0]
    msg.pose.pose.position.y = pos_global[1]

    quarternion = Rotation.from_euler('z', ori_global, degrees=False).as_quat()
    msg.pose.pose.orientation.x = quarternion[0]
    msg.pose.pose.orientation.y = quarternion[1]
    msg.pose.pose.orientation.z = quarternion[2]
    msg.pose.pose.orientation.w = quarternion[3]

    odom_pub.publish(msg)

    # Define a static transform
    tf_stamped = TransformStamped()
    tf_stamped.header.stamp = msg.header.stamp
    tf_stamped.header.frame_id = 'odom'  # Parent frame
    tf_stamped.child_frame_id = 'base_link'  # Child frame

    # Set translation (x, y, z)
    tf_stamped.transform.translation.x = msg.pose.pose.position.x
    tf_stamped.transform.translation.y = msg.pose.pose.position.y
    tf_stamped.transform.translation.z = msg.pose.pose.position.z

    # Set rotation (quaternion: x, y, z, w)
    tf_stamped.transform.rotation.x = msg.pose.pose.orientation.x
    tf_stamped.transform.rotation.y = msg.pose.pose.orientation.y
    tf_stamped.transform.rotation.z = msg.pose.pose.orientation.z
    tf_stamped.transform.rotation.w = msg.pose.pose.orientation.w

    # Publish transform
    tf_broadcaster.sendTransform(tf_stamped)