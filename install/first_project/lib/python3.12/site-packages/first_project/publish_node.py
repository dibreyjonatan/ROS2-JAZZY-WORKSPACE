import  rclpy
from rclpy.node import Node 

from std_msgs.msg import String

class Publisher_node(Node):
    def __init__(self):
        super().__init__("publishernode")
        self.publisher_=self.create_publisher(String,'/send_msg',10)
        timer_period=1
        self.timer=self.create_timer(timer_period,self.timer_callback)
        self.i=0
    def timer_callback(self):
        msg=String()
        if self.i % 2 == 0 :
            msg.data=f'Hello {self.i} is even'
        else :
            msg.data=f'Hello {self.i} is odd'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing : "%s" '% msg.data)
        self.i+=1


def main(args=None):
    rclpy.init(args=args)

    publisher=Publisher_node()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__== '__main__' :
    main()    