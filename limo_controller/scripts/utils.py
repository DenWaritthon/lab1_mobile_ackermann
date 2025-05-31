from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
import numpy as np
import math

def pub_odom(odom_pub, t, twist, pos_global, ori_global, pos_cov = 0.0, twist_cov = 0.0, tf_broadcaster = None, ref_frame='odom', logger = None):
    msg = Odometry()
    msg.header.stamp = t.to_msg()
    msg.header.frame_id = ref_frame
    msg.child_frame_id = 'base_footprint'

    msg.twist.twist.linear.x = twist[0]
    msg.twist.twist.angular.z = twist[1]

    msg.pose.pose.position.x = pos_global[0]
    msg.pose.pose.position.y = pos_global[1]

    quarternion = Rotation.from_euler('z', ori_global, degrees=False).as_quat()
    msg.pose.pose.orientation.x = quarternion[0]
    msg.pose.pose.orientation.y = quarternion[1]
    msg.pose.pose.orientation.z = quarternion[2]
    msg.pose.pose.orientation.w = quarternion[3]

    msg.pose.covariance =  [pos_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, pos_cov, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, pos_cov, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, pos_cov, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, pos_cov, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, pos_cov]
    
    msg.twist.covariance =  [twist_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, twist_cov, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, twist_cov, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, twist_cov, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, twist_cov, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, twist_cov]

    def is_finite(*args):
        return all([not math.isnan(x) and not math.isinf(x) for x in args])

    if is_finite(
        msg.twist.twist.linear.x,
        msg.twist.twist.angular.z,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.orientation.w,  # catch broken quaternions too
        msg.pose.pose.orientation.z
    ):
        odom_pub.publish(msg)
    else:
        if logger:
            logger.warning("NaN or Inf detected in Odometry, skipping publish.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    if tf_broadcaster is not None:
        # Define a static transform
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = msg.header.stamp
        tf_stamped.header.frame_id = 'base_footprint'  # Parent frame
        tf_stamped.child_frame_id = ref_frame  # Child frame

        # Set translation (x, y, z)
        tf_stamped.transform.translation.x = - (pos_global[0] * np.cos(-ori_global) - pos_global[1] * np.sin(-ori_global))
        tf_stamped.transform.translation.y = - (pos_global[0] * np.sin(-ori_global) + pos_global[1] * np.cos(-ori_global))
        tf_stamped.transform.translation.z = - (msg.pose.pose.position.z)

        # Set rotation (quaternion: x, y, z, w)
        quarternion_inv = Rotation.from_euler('z', -ori_global, degrees=False).as_quat()
        tf_stamped.transform.rotation.x = quarternion_inv[0]
        tf_stamped.transform.rotation.y = quarternion_inv[1]
        tf_stamped.transform.rotation.z = quarternion_inv[2]
        tf_stamped.transform.rotation.w = quarternion_inv[3]

        if tf_broadcaster is not None and is_finite(
            tf_stamped.transform.translation.x,
            tf_stamped.transform.translation.y,
            tf_stamped.transform.translation.z,
            tf_stamped.transform.rotation.w,
            tf_stamped.transform.rotation.z
        ):
            tf_broadcaster.sendTransform(tf_stamped)
        else:
            if logger:
                logger.warning("TF not published due to invalid (NaN/Inf) transform.")

        # # Define a static transform
        # tf_stamped = TransformStamped()
        # tf_stamped.header.stamp = msg.header.stamp
        # tf_stamped.header.frame_id = ref_frame  # Parent frame
        # tf_stamped.child_frame_id = 'base_footprint'  # Child frame

        # # Set translation (x, y, z)
        # tf_stamped.transform.translation.x = pos_global[0]
        # tf_stamped.transform.translation.y = pos_global[1]

        # # Set rotation (quaternion: x, y, z, w)
        # tf_stamped.transform.rotation.x = quarternion[0]
        # tf_stamped.transform.rotation.y = quarternion[1]
        # tf_stamped.transform.rotation.z = quarternion[2]
        # tf_stamped.transform.rotation.w = quarternion[3]