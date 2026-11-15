"""Transform fresh detected surface points; never commands robot movement."""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class Bridge(Node):
    def __init__(self):
        super().__init__('ball_to_base')
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(PointStamped,'/vision/ball_surface_point_base',1)
        self.sub=self.create_subscription(PointStamped,'/vision/ball_surface_point',self.receive,1)
    def receive(self,msg):
        age=(self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        if age<0 or age>1.0:
            self.get_logger().warning('Dropping stale camera point',throttle_duration_sec=5.)
            return
        try:
            tf=self.buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            result=do_transform_point(msg,tf)
            result.header.stamp=msg.header.stamp
            self.pub.publish(result)
        except TransformException as e:
            self.get_logger().warning(str(e),throttle_duration_sec=5.)

def main():
    rclpy.init();node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
