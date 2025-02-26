import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class PathTrackingPID(Node):
    def __init__(self):
        super().__init__('path_tracking_pid')

        path_file = os.path.join(get_package_share_directory('lab1_mobile_ackermann'),'config','path.yaml')
        
        with open(path_file, 'r') as file:
            path = yaml.safe_load(file)

        print(path[0]["x"])
   

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()