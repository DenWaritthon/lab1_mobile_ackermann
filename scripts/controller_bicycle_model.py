import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class BicycleModelController(Node):

    def __init__(self):
        super().__init__('bicycle_model_controller')

        # Subscriber
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)

        # Publisher
        self.velocity_pub= self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)

        # Variable
        self.steering_max = 0.523598767
        self.steering_min = -0.523598767
        self.wheelbase = 0.2
        self.track = 0.13
        self.wheel_radius = 0.045

        self.get_logger().info(f'Node BicycleModelController Start!!!!!')

    def cmd_vel_callback(self, msg:Twist):
        v = msg.linear.x
        omega = float(msg.angular.z)

        # Calculate the steering angle
        delta = np.arctan2(omega * self.wheelbase, v)

        # Limit the steering angle
        delta = float(np.clip(delta, self.steering_min, self.steering_max))

        # Publish the velocity and steering angle
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [v, v, v, v]
        self.velocity_pub.publish(velocity_msg)

        steering_msg = Float64MultiArray()
        steering_msg.data = [delta, delta]
        self.steering_pub.publish(steering_msg)

        self.get_logger().info(f'v: {v}, omega: {omega}, delta: {delta}')

def main(args=None):
    rclpy.init(args=args)
    node = BicycleModelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()