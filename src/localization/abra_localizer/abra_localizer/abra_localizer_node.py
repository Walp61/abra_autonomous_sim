import rclpy
from rclpy.node import Node
from abra_interfaces.msg import AbraState
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

PUBLISH_FREQUENCY = 10

class Localizer(Node):
    def __init__(self):
        super().__init__("abra_localizer_node")
        self.pcl_subscriber = self.create_subscription(PoseWithCovarianceStamped,"/pcl_pose", self.pcl_subscriber_callback, 10)
        self.publisher = self.create_publisher(AbraState, '/abra/localization_state', 10)
        self.timer = self.create_timer(1/PUBLISH_FREQUENCY, self.timer_callback)
        self.msg = AbraState()
    
    def pcl_subscriber_callback(self, data):
        print("pcl pose received")
        self.msg.x = data.pose.pose.position.x
        self.msg.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.msg.yaw = yaw
        self.msg.v = 1.38

    def timer_callback(self):
        self.publisher.publish(self.msg)
    
def main(args=None):
    rclpy.init(args=args)
    localizer = Localizer()
    rclpy.spin(localizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
